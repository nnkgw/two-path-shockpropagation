#include "Simulation.h"
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <algorithm>

Simulation::Simulation()
    : currentMode(FIG1_UPWARD), currentStep(0), isAutoPlay(false),
      autoPlayTimer(0.0f), autoPlayInterval(1.5f),
      gravity(0.0f, -9.81f, 0.0f), numSubsteps(10), numIterations(10) {
    initializeBoxes();
    contactFlags.assign(boxes.size(), 0);
    updateStep();
}

int Simulation::findBoxIndex(const Box* box) const {
    for (int i = 0; i < (int)boxes.size(); i++) {
        if (&boxes[i] == box) return i;
    }
    return -1;
}

void Simulation::initializeBoxes() {
    boxes.clear();
    // X (blue), Y (orange), Z (yellow)
    boxes.push_back(Box("X", glm::vec3(0.0f, 0.5f, 0.0f), glm::vec3(1.0f), glm::vec3(0.3f, 0.5f, 0.9f), 1.0f));
    boxes.push_back(Box("Y", glm::vec3(0.0f, 1.5f, 0.0f), glm::vec3(1.0f), glm::vec3(0.9f, 0.5f, 0.3f), 1.0f));
    boxes.push_back(Box("Z", glm::vec3(0.0f, 2.5f, 0.0f), glm::vec3(1.0f), glm::vec3(0.9f, 0.9f, 0.3f), 1.0f));
}

void Simulation::setMode(Mode mode) {
    currentMode = mode;
    currentStep = 0;
    reset();
}

void Simulation::reset() {
    for (auto& box : boxes) {
        box.reset();
    }
    currentStep = 0;
    updateStep();
}

void Simulation::nextStep() {
    int totalSteps = getTotalSteps();
    if (currentStep < totalSteps - 1) {
        currentStep++;
        updateStep();
    }
}

void Simulation::prevStep() {
    if (currentStep > 0) {
        currentStep--;
        updateStep();
    }
}

void Simulation::autoPlay(bool enable) {
    isAutoPlay = enable;
    autoPlayTimer = 0.0f;
}

void Simulation::update(float deltaTime) {
    float dt = std::min(deltaTime, 0.033f);

    // For static visualization modes, keep the configuration stable and avoid
    // injecting kinetic energy from positional correction (a common PBD pitfall).
    if (currentMode == FIG1_UPWARD || currentMode == FIG2_DOWNWARD_STATIC) {
        // Project constraints a few times to resolve any deliberate interpenetration.
        for (int i = 0; i < numIterations; i++) {
            solveConstraints(0.0f);
        }

        // Freeze motion completely (these modes are meant to be "diagram" states).
        for (auto& box : boxes) {
            box.prevPosition = box.getPosition();
            box.prevOrientation = box.getOrientation();
            box.setVelocity(glm::vec3(0.0f));
            box.setAngularVelocity(glm::vec3(0.0f));
        }
    } else {
        // Dynamic / real-time modes
        stepPhysics(dt);
    }

    if (isAutoPlay) {
        autoPlayTimer += deltaTime;
        if (autoPlayTimer >= autoPlayInterval) {
            autoPlayTimer = 0.0f;
            nextStep();
            if (currentStep >= getTotalSteps() - 1) {
                currentStep = 0;
                updateStep();
            }
        }
    }
}

void Simulation::stepPhysics(float dt) {
    if (dt <= 0.0f) return;
    float sdt = dt / numSubsteps;

    for (int s = 0; s < numSubsteps; s++) {
        std::fill(contactFlags.begin(), contactFlags.end(), (unsigned char)0);

        // 1. Predict positions (only for dynamic mode or specific steps)
        for (auto& box : boxes) {
            if (box.isInfiniteMass()) continue;
            
            // Apply gravity only in dynamic mode or when specifically needed
            if (currentMode == FIG3_DOWNWARD_DYNAMIC || currentMode == PHYSICS_SIM) {
                box.setVelocity(box.getVelocity() + gravity * sdt);
            }
            
            box.prevPosition = box.getPosition();
            box.setPosition(box.getPosition() + box.getVelocity() * sdt);
            
            box.prevOrientation = box.getOrientation();
            glm::vec3 w = box.getAngularVelocity();
            glm::quat q = box.getOrientation();
            glm::quat dq = 0.5f * glm::quat(0.0f, w.x, w.y, w.z) * q;
            box.setOrientation(glm::normalize(q + dq * sdt));
        }

        // 2. Solve constraints based on current step
        for (int i = 0; i < numIterations; i++) {
            solveConstraints(sdt);
        }

        // 3. Update velocities
        for (int bi = 0; bi < (int)boxes.size(); bi++) {
            auto& box = boxes[bi];
            if (box.isInfiniteMass()) continue;
            box.setVelocity((box.getPosition() - box.prevPosition) / sdt);
            
            glm::quat dq = box.getOrientation() * glm::inverse(box.prevOrientation);
            glm::vec3 w(dq.x, dq.y, dq.z);
            float angle = 2.0f * acos(glm::clamp(dq.w, -1.0f, 1.0f));
            if (angle > 0.001f) {
                box.setAngularVelocity(glm::normalize(w) * (angle / sdt));
            } else {
                box.setAngularVelocity(glm::vec3(0.0f));
            }
            
            // Damping
            glm::vec3 v = box.getVelocity() * 0.98f;
            glm::vec3 wv = box.getAngularVelocity() * 0.95f;

            // Make contacts inelastic/stable by removing any velocity introduced
            // purely by position correction (prevents "shooting up" artifacts).
            if (bi < (int)contactFlags.size()) {
                const unsigned char flags = contactFlags[bi];
                if (flags != 0) {
                    v.y = 0.0f;
                    // Angular motion is not handled by our simple contact model.
                    wv = glm::vec3(0.0f);
                }
            }

            box.setVelocity(v);
            box.setAngularVelocity(wv);
        }
    }
}

void Simulation::solveConstraints(float dt) {
    // Ground constraint (for robustness, apply to all bodies)
    for (auto& box : boxes) {
        solveGround(box);
    }

    if (currentMode == FIG1_UPWARD) {
        // Upward Pass: Bottom bodies are fixed (infinite mass)
        // Note: solveContact(a,b) assumes a is the lower body and b is the upper body.
        // To "fix" the lower body, move only the upper body (weightA=0, weightB=1).
        if (currentStep >= 3) solveContact(boxes[0], boxes[1], {}, 0.0f, 1.0f); // X fixed, move Y
        if (currentStep >= 5) solveContact(boxes[1], boxes[2], {}, 0.0f, 1.0f); // Y fixed, move Z
    } else {
        // Downward Pass: Standard solve (both can move)
        solveContact(boxes[1], boxes[2], {}, 0.5f, 0.5f);
        solveContact(boxes[0], boxes[1], {}, 0.5f, 0.5f);
    }
}

void Simulation::solveGround(Box& box) {
    float halfSize = box.getSize().y * 0.5f;
    float penetration = halfSize - box.getPosition().y;
    if (penetration > 0.0f) {
        int idx = findBoxIndex(&box);
        if (idx >= 0 && idx < (int)contactFlags.size()) {
            contactFlags[idx] |= 1; // ground
        }
        glm::vec3 pos = box.getPosition();
        pos.y += penetration;
        box.setPosition(pos);
    }
}

void Simulation::solveContact(Box& a, Box& b, const Contact& contact, float weightA, float weightB) {
    // Simplified contact solver for stacking
    float dist = b.getPosition().y - a.getPosition().y;
    float minDist = (a.getSize().y + b.getSize().y) * 0.5f;
    float penetration = minDist - dist;
    
    if (penetration > 0.0f) {
        int idxA = findBoxIndex(&a);
        int idxB = findBoxIndex(&b);
        if (idxA >= 0 && idxA < (int)contactFlags.size()) contactFlags[idxA] |= 2; // body-body
        if (idxB >= 0 && idxB < (int)contactFlags.size()) contactFlags[idxB] |= 2;

        float invMassA = a.getInvMass() * weightA;
        float invMassB = b.getInvMass() * weightB;
        float totalInvMass = invMassA + invMassB;
        if (totalInvMass <= 0.0f) return;
        
        float correction = penetration / totalInvMass;
        
        if (invMassA > 0.0f) {
            glm::vec3 posA = a.getPosition();
            posA.y -= correction * invMassA;
            a.setPosition(posA);
        }
        if (invMassB > 0.0f) {
            glm::vec3 posB = b.getPosition();
            posB.y += correction * invMassB;
            b.setPosition(posB);
        }
    }
}

int Simulation::getTotalSteps() const {
    switch (currentMode) {
        case FIG1_UPWARD: return 7;
        case FIG2_DOWNWARD_STATIC: return 8;
        case FIG3_DOWNWARD_DYNAMIC: return 8;
        case PHYSICS_SIM: return 1;
        default: return 7;
    }
}

std::string Simulation::getModeName() const {
    switch (currentMode) {
        case FIG1_UPWARD: return "Fig.1: Upward Pass (Static)";
        case FIG2_DOWNWARD_STATIC: return "Fig.2: Downward Pass (Static)";
        case FIG3_DOWNWARD_DYNAMIC: return "Fig.3: Downward Pass (Dynamic)";
        case PHYSICS_SIM: return "Real-time Physics (Simple Stack)";
        default: return "Unknown";
    }
}

std::string Simulation::getCurrentStepDescription() const {
    switch (currentMode) {
        case FIG1_UPWARD:
            switch (currentStep) {
                case 0: return "(a) Initial";
                case 1: return "(b) Eval: Ground-X";
                case 2: return "(c) Apply to T (X)";
                case 3: return "(d) Eval: X-Y";
                case 4: return "(e) Apply to T (Y)";
                case 5: return "(f) Eval: Y-Z";
                case 6: return "(g) Apply to T (Z)";
                default: return "";
            }
        case FIG2_DOWNWARD_STATIC:
            switch (currentStep) {
                case 0: return "(a) Eval: Z-Y";
                case 1: return "(b) Apply to T (Z)";
                case 2: return "(c) Apply to B (Y)";
                case 3: return "(d) Eval: Y-X";
                case 4: return "(e) Apply to T (Y)";
                case 5: return "(f) Apply to B (X)";
                case 6: return "(g) Eval: X-Ground";
                case 7: return "(h) Apply to T (X)";
                default: return "";
            }
        case FIG3_DOWNWARD_DYNAMIC:
            switch (currentStep) {
                case 0: return "(a) Eval: Z-Y";
                case 1: return "(b) Apply to T (Z)";
                case 2: return "(c) Apply to B (Y)";
                case 3: return "(d) Eval: Y-X";
                case 4: return "(e) Apply to T (Y)";
                case 5: return "(f) Apply to B (X)";
                case 6: return "(g) Eval: X-Ground";
                case 7: return "(h) Apply to T (X)";
                default: return "";
            }
        case PHYSICS_SIM:
            return "(sim)";
        default: return "";
    }
}

void Simulation::updateStep() {
    impulses.clear();
    printStepExplanation();
    
    glm::vec3 yellowImpulse(1.0f, 0.8f, 0.0f);
    glm::vec3 redImpulse(1.0f, 0.1f, 0.1f);
    glm::vec3 blueImpulse(0.2f, 0.4f, 1.0f);
    
    // Reset infinite mass flags
    for (auto& box : boxes) box.setInfiniteMass(false);

    if (currentMode == PHYSICS_SIM) {
        // Real-time physics mode: keep an always-dynamic stack with a small perturbation.
        // No step-dependent impulses are used here.
        boxes[0].setPosition(glm::vec3(0.0f, 0.5f, 0.0f));
        boxes[1].setPosition(glm::vec3(0.0f, 1.5f, 0.0f));
        boxes[2].setPosition(glm::vec3(0.15f, 2.5f, 0.0f));

        // Start from rest (avoid inheriting velocities when switching modes).
        for (auto& box : boxes) {
            box.setVelocity(glm::vec3(0.0f));
            box.setAngularVelocity(glm::vec3(0.0f));
            box.prevPosition = box.getPosition();
            box.prevOrientation = box.getOrientation();
        }
        return;
    }

    if (currentMode == FIG1_UPWARD) {
        switch (currentStep) {
            case 0: boxes[0].setPosition(glm::vec3(0.0f, 0.3f, 0.0f)); break;
            case 2: impulses.push_back(Impulse(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f)); break;
            case 3: boxes[0].setInfiniteMass(true); break;
            case 4: impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f));
                    impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), redImpulse, 0.5f)); break;
            case 5: boxes[1].setInfiniteMass(true); break;
            case 6: impulses.push_back(Impulse(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f));
                    impulses.push_back(Impulse(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), redImpulse, 0.5f));
                    impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), blueImpulse, 0.5f)); break;
        }
    } else if (currentMode == FIG2_DOWNWARD_STATIC) {
        switch (currentStep) {
            case 1: impulses.push_back(Impulse(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.1f)); break;
            case 2: impulses.push_back(Impulse(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), redImpulse, 0.5f)); break;
            case 3: boxes[0].setInfiniteMass(true); break;
            case 4: impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f)); break;
            case 5: impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), blueImpulse, 0.5f)); break;
            case 7: impulses.push_back(Impulse(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f)); break;
        }
    } else if (currentMode == FIG3_DOWNWARD_DYNAMIC) {
        // In dynamic mode, we apply an initial offset to trigger falling
        if (currentStep == 0) {
            boxes[2].setPosition(boxes[2].getPosition() + glm::vec3(0.2f, 0.0f, 0.0f));
        }
        switch (currentStep) {
            case 1: impulses.push_back(Impulse(glm::vec3(0.2f, 2.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.1f)); break;
            case 2: impulses.push_back(Impulse(glm::vec3(0.2f, 2.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), redImpulse, 0.5f)); break;
            case 5: impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), blueImpulse, 0.5f)); break;
            case 7: impulses.push_back(Impulse(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f)); break;
        }
    }
}

void Simulation::printStepExplanation() const {
    std::cout << "\n--- " << getModeName() << " ---" << std::endl;
    std::cout << "Step " << (currentStep + 1) << "/" << getTotalSteps() << ": " << getCurrentStepDescription() << std::endl;
}
