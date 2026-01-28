#include "Simulation.h"
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <algorithm>

Simulation::Simulation()
    : currentMode(FIG1_UPWARD), currentStep(0), isAutoPlay(false),
      autoPlayTimer(0.0f), autoPlayInterval(1.5f),
      gravity(0.0f, -9.81f, 0.0f), numSubsteps(25), numIterations(40) {
    initializeBoxes();
    updateStep();
}

void Simulation::initializeBoxes() {
    boxes.clear();
    // X (blue), Y (orange), Z (yellow)
    // Fig.1(a) initial state: Only X penetrates ground.
    // X center at y=0.3, size=1.0 -> bottom at -0.2 (penetrates ground by 0.2), top at 0.8.
    // Y center at y=1.3, size=1.0 -> bottom at 0.8 (exactly touches X's top).
    // Z center at y=2.3, size=1.0 -> bottom at 1.8 (exactly touches Y's top).
    // Note: To ensure they touch, Y should be at 0.8 + 0.5 = 1.3, Z at 1.3 + 0.5 + 0.5 = 2.3.
    // Wait, if X top is 0.8, Y bottom should be 0.8. Y center = 0.8 + 0.5 = 1.3. Correct.
    // If Y top is 1.3 + 0.5 = 1.8, Z bottom should be 1.8. Z center = 1.8 + 0.5 = 2.3. Correct.
    boxes.push_back(Box("X", glm::vec3(0.0f, 0.3f, 0.0f), glm::vec3(1.0f), glm::vec3(0.3f, 0.5f, 0.9f), 1.0f));
    boxes.push_back(Box("Y", glm::vec3(0.0f, 1.3f, 0.0f), glm::vec3(1.0f), glm::vec3(0.9f, 0.5f, 0.3f), 1.0f));
    boxes.push_back(Box("Z", glm::vec3(0.0f, 2.3f, 0.0f), glm::vec3(1.0f), glm::vec3(0.9f, 0.9f, 0.3f), 1.0f));
}

void Simulation::setMode(Mode mode) {
    currentMode = mode;
    currentStep = 0;
    reset();
}

void Simulation::reset() {
    initializeBoxes();
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
    // Limit deltaTime to prevent instability
    float dt = std::min(deltaTime, 0.016f); 
    stepPhysics(dt);

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
        // 1. Predict positions
        for (auto& box : boxes) {
            if (box.isInfiniteMass()) continue;
            
            // Apply gravity
            if (currentMode == FIG3_DOWNWARD_DYNAMIC || currentMode == PHYSICS_SIM) {
                box.setVelocity(box.getVelocity() + gravity * sdt);
            }
            
            box.prevPosition = box.getPosition();
            box.setPosition(box.getPosition() + box.getVelocity() * sdt);
            
            box.prevOrientation = box.getOrientation();
            if (currentMode == FIG3_DOWNWARD_DYNAMIC) {
                // Fig.3 demo uses axis-aligned boxes; keep rotations disabled to avoid
                // visually inconsistent penetrations with the AABB contact model.
                box.setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
                box.setAngularVelocity(glm::vec3(0.0f));
            } else {
                glm::vec3 w = box.getAngularVelocity();
                glm::quat q = box.getOrientation();
                glm::quat dq = 0.5f * glm::quat(0.0f, w.x, w.y, w.z) * q;
                box.setOrientation(glm::normalize(q + dq * sdt));
            }
        }

        // 2. Solve constraints
        for (int i = 0; i < numIterations; i++) {
            solveConstraints(sdt);
        }

        // 3. Update velocities and apply damping
        for (auto& box : boxes) {
            if (box.isInfiniteMass()) continue;
            
            glm::vec3 newVel = (box.getPosition() - box.prevPosition) / sdt;
            
            // Velocity Clamping to prevent "flying away"
            float maxVel = 10.0f;
            if (glm::length(newVel) > maxVel) {
                newVel = glm::normalize(newVel) * maxVel;
            }
            float velDamping = (currentMode == FIG3_DOWNWARD_DYNAMIC) ? 0.9998f : 0.95f;
            box.setVelocity(newVel * velDamping); // Stronger damping (relaxed for Fig.3 dynamic)
            
            if (currentMode == FIG3_DOWNWARD_DYNAMIC) {
                box.setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
                box.setAngularVelocity(glm::vec3(0.0f));
                continue;
            }
            glm::quat dq = box.getOrientation() * glm::inverse(box.prevOrientation);
            glm::vec3 w(dq.x, dq.y, dq.z);
            float angle = 2.0f * acos(glm::clamp(dq.w, -1.0f, 1.0f));
            if (angle > 0.001f) {
                box.setAngularVelocity(glm::normalize(w) * (angle / sdt) * 0.9f);
            } else {
                box.setAngularVelocity(glm::vec3(0.0f));
            }
        }
    }
}

void Simulation::solveConstraints(float dt) {
    // NOTE:
    //  - FIG1_UPWARD is a pedagogical, step-by-step reproduction of Fig.1 in the paper.
    //    In (a) and (b), we must NOT resolve the ground contact yet; otherwise X is pushed up
    //    immediately and Y starts interpenetrating X, which contradicts Fig.1(a).
    //  - Therefore, in FIG1 we only activate each constraint on the corresponding "Apply" step.
    if (currentMode == FIG1_UPWARD) {
        // Step mapping (Fig.1):
        //  0: (a) Initial
        //  1: (b) Eval: Ground-X          (no position change)
        //  2: (c) Apply to X (Top)        (resolve Ground-X)
        //  3: (d) Eval: X-Y               (no position change)
        //  4: (e) Apply to Y (Top)        (resolve X-Y with X treated as infinite)
        //  5: (f) Eval: Y-Z               (no position change)
        //  6: (g) Apply to Z (Top)        (resolve Y-Z with Y treated as infinite)
        if (currentStep >= 2) solveGround(boxes[0]); // Ground-X apply
        if (currentStep >= 4) solveContact(boxes[0], boxes[1], {}, 0.0f, 1.0f); // X fixed, Y moves
        if (currentStep >= 6) solveContact(boxes[1], boxes[2], {}, 0.0f, 1.0f); // Y fixed, Z moves
        return;
    }

    // 1. Upward Pass (Shock Propagation)
    // Solve from bottom to top, treating the bottom object as static (infinite mass)

    // Ground constraints for all boxes
    for (auto& box : boxes) solveGround(box);

    // In Downward Pass or Physics Sim, we perform the full Upward Pass first
    // Check all pairs to prevent skipping collisions when boxes move horizontally
    solveContact(boxes[0], boxes[1], {}, 0.0f, 1.0f);
    solveContact(boxes[1], boxes[2], {}, 0.0f, 1.0f);
    solveContact(boxes[0], boxes[2], {}, 0.0f, 1.0f); // X-Z direct contact if Y is pushed out

    // 2. Downward Pass (Standard Gauss-Seidel)
    // Solve with normal mass ratios to allow settling
    solveContact(boxes[1], boxes[2], {}, 0.5f, 0.5f);
    solveContact(boxes[0], boxes[1], {}, 0.5f, 0.5f);
    solveContact(boxes[0], boxes[2], {}, 0.5f, 0.5f);

    for (auto& box : boxes) solveGround(box);
}


void Simulation::solveGround(Box& box) {
    float halfSize = box.getSize().y * 0.5f;
    float penetration = halfSize - box.getPosition().y;
    if (penetration > 0.0f) {
        glm::vec3 pos = box.getPosition();
        pos.y += penetration;
        box.setPosition(pos);
    }
}

void Simulation::solveContact(Box& a, Box& b, const Contact& contact, float weightA, float weightB) {
    // 3D Box-Box Collision (Faithful to original paper: resolving all axes)
    glm::vec3 posA = a.getPosition();
    glm::vec3 posB = b.getPosition();
    glm::vec3 sizeA = a.getSize();
    glm::vec3 sizeB = b.getSize();

    glm::vec3 diff = posB - posA;
    glm::vec3 min_dist = (sizeA + sizeB) * 0.5f;
    glm::vec3 overlap = min_dist - glm::abs(diff);

    // If overlapping on all axes, we have a collision
    if (overlap.x > 0 && overlap.y > 0 && overlap.z > 0) {
        // For stacking stability, we prioritize the vertical axis (Y) 
        // if the horizontal overlap is significant. This prevents boxes 
        // from sliding sideways when they should be pushed upwards.
        int axis = 1; // Default to Y-axis for stacking
        
        // Only use X or Z if the overlap in those directions is much smaller,
        // indicating a clear side-to-side collision.
        if (overlap.x < overlap.y * 0.5f && overlap.x < overlap.z) axis = 0;
        else if (overlap.z < overlap.y * 0.5f && overlap.z < overlap.x) axis = 2;

        float penetration = overlap[axis];
        glm::vec3 normal(0.0f);
        normal[axis] = (diff[axis] > 0) ? 1.0f : -1.0f;

        float invMassA = a.getInvMass() * weightA;
        float invMassB = b.getInvMass() * weightB;
        float totalInvMass = invMassA + invMassB;
        if (totalInvMass <= 0.0f) return;

        // Resolve the penetration
        glm::vec3 correction = normal * (penetration / totalInvMass);

        if (invMassA > 0.0f) {
            a.setPosition(a.getPosition() - correction * invMassA);
        }
        if (invMassB > 0.0f) {
            b.setPosition(b.getPosition() + correction * invMassB);
        }
        
        // Apply friction/damping to stabilize the stack
        if (axis == 1 && currentMode != FIG3_DOWNWARD_DYNAMIC) { // Vertical contact
            glm::vec3 vA = a.getVelocity();
            glm::vec3 vB = b.getVelocity();
            // Stronger horizontal damping to keep the stack perfectly vertical
            a.setVelocity(glm::vec3(vA.x * 0.8f, vA.y, vA.z * 0.8f));
            b.setVelocity(glm::vec3(vB.x * 0.8f, vB.y, vB.z * 0.8f));
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
        case PHYSICS_SIM: return "Real-time Physics (Two-Pass)";
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
    
    for (auto& box : boxes) box.setInfiniteMass(false);

    if (currentMode == PHYSICS_SIM) {
        boxes[0].setPosition(glm::vec3(0.0f, 0.3f, 0.0f));
        // Match Fig.1(a): only X penetrates the ground; Y/Z are initially just touching.
        boxes[1].setPosition(glm::vec3(0.0f, 1.3f, 0.0f));
        boxes[2].setPosition(glm::vec3(0.15f, 2.3f, 0.0f));
        return;
    }

    if (currentMode == FIG1_UPWARD) {
        switch (currentStep) {
            case 0: 
                boxes[0].setPosition(glm::vec3(0.0f, 0.3f, 0.0f)); 
                // Match Fig.1(a): only X penetrates the ground; Y/Z are initially just touching.
                boxes[1].setPosition(glm::vec3(0.0f, 1.3f, 0.0f));
                boxes[2].setPosition(glm::vec3(0.0f, 2.3f, 0.0f));
                break;
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
            case 0:
                // Start of the downward pass (Fig.2): this begins from the end of the upward pass,
                // where all penetrations have been resolved assuming the Bottom body is infinite.
                boxes[0].setPosition(glm::vec3(0.0f, 0.5f, 0.0f));
                boxes[1].setPosition(glm::vec3(0.0f, 1.5f, 0.0f));
                boxes[2].setPosition(glm::vec3(0.0f, 2.5f, 0.0f));
                break;
            case 1: impulses.push_back(Impulse(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.1f)); break;
            case 2: impulses.push_back(Impulse(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), redImpulse, 0.5f)); break;
            case 3: boxes[0].setInfiniteMass(true); break;
            case 4: impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f)); break;
            case 5: impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), blueImpulse, 0.5f)); break;
            case 7: impulses.push_back(Impulse(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f)); break;
        }
    } else if (currentMode == FIG3_DOWNWARD_DYNAMIC) {
        if (currentStep == 0) {
            // Start of the dynamic downward pass (Fig.3): begin from the end of the upward pass,
            // and introduce a small horizontal perturbation on Z.
            boxes[0].setPosition(glm::vec3(0.0f, 0.5f, 0.0f));
            boxes[1].setPosition(glm::vec3(0.0f, 1.5f, 0.0f));
            // Overhang + small lateral drift so Z slides off Y and falls.
            boxes[2].setPosition(glm::vec3(0.45f, 2.5f, 0.0f));
            boxes[0].setVelocity(glm::vec3(0.0f));
            boxes[1].setVelocity(glm::vec3(0.0f));
            boxes[2].setVelocity(glm::vec3(1.2f, 0.0f, 0.0f));
            boxes[0].setAngularVelocity(glm::vec3(0.0f));
            boxes[1].setAngularVelocity(glm::vec3(0.0f));
            boxes[2].setAngularVelocity(glm::vec3(0.0f));
        }
        switch (currentStep) {
            case 1: impulses.push_back(Impulse(glm::vec3(0.45f, 2.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.1f)); break;
            case 2: impulses.push_back(Impulse(glm::vec3(0.45f, 2.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), redImpulse, 0.5f)); break;
            case 5: impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), blueImpulse, 0.5f)); break;
            case 7: impulses.push_back(Impulse(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f)); break;
        }
    }
}

void Simulation::printStepExplanation() const {
    std::cout << "\n--- " << getModeName() << " ---" << std::endl;
    std::cout << "Step " << (currentStep + 1) << "/" << getTotalSteps() << ": " << getCurrentStepDescription() << std::endl;
}
