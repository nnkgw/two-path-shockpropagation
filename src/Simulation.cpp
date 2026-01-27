#include "Simulation.h"
#include <glm/gtc/constants.hpp>
#include <iostream>

Simulation::Simulation()
    : currentMode(FIG1_UPWARD), currentStep(0), isAutoPlay(false),
      autoPlayTimer(0.0f), autoPlayInterval(1.5f) {
    initializeBoxes();
    updateStep();
}

void Simulation::initializeBoxes() {
    boxes.clear();
    // X (blue), Y (orange), Z (yellow)
    boxes.push_back(Box("X", glm::vec3(0.0f, 0.5f, 0.0f), glm::vec3(1.0f), glm::vec3(0.3f, 0.5f, 0.9f)));
    boxes.push_back(Box("Y", glm::vec3(0.0f, 1.5f, 0.0f), glm::vec3(1.0f), glm::vec3(0.9f, 0.5f, 0.3f)));
    boxes.push_back(Box("Z", glm::vec3(0.0f, 2.5f, 0.0f), glm::vec3(1.0f), glm::vec3(0.9f, 0.9f, 0.3f)));
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

void Simulation::printStepExplanation() const {
    std::cout << "\n--- " << getModeName() << " ---" << std::endl;
    std::cout << "Step " << (currentStep + 1) << "/" << getTotalSteps() << ": " << getCurrentStepDescription() << std::endl;
    
    if (currentMode == FIG1_UPWARD) {
        switch (currentStep) {
            case 0: std::cout << "Initial state: Box X is penetrating the ground." << std::endl; break;
            case 1: std::cout << "Evaluating Ground-X constraint. Box X is the 'Top' body, Ground is 'Bottom'." << std::endl; break;
            case 2: std::cout << "Applying impulse to X. It moves out of the ground. (Yellow arrow)" << std::endl; break;
            case 3: std::cout << "Evaluating X-Y constraint. X is treated as infinite mass (Bottom body)." << std::endl; break;
            case 4: std::cout << "Applying impulse to Y (Yellow). The reaction on X is stored (Red arrow) but NOT applied yet." << std::endl; break;
            case 5: std::cout << "Evaluating Y-Z constraint. Y is treated as infinite mass (Bottom body)." << std::endl; break;
            case 6: std::cout << "Applying impulse to Z (Yellow). Reaction on Y is stored (Red). Unused impulse on X remains (Blue)." << std::endl; break;
        }
    } else if (currentMode == FIG2_DOWNWARD_STATIC) {
        switch (currentStep) {
            case 0: std::cout << "Starting Downward Pass. Evaluating Z-Y constraint." << std::endl; break;
            case 1: std::cout << "Applying impulse to Z (Yellow). In this static case, it's zero as already solved." << std::endl; break;
            case 2: std::cout << "Applying the STORED impulse to Y (Red arrow). This accounts for the reaction from Z." << std::endl; break;
            case 3: std::cout << "Evaluating Y-X constraint." << std::endl; break;
            case 4: std::cout << "Applying impulse to Y (Yellow)." << std::endl; break;
            case 5: std::cout << "Applying the STORED impulse to X (Blue arrow). This causes X to penetrate the ground again." << std::endl; break;
            case 6: std::cout << "Evaluating X-Ground constraint to fix the new penetration." << std::endl; break;
            case 7: std::cout << "Applying final impulse to X (Yellow). The stack is now stable and all forces are accounted for." << std::endl; break;
        }
    } else if (currentMode == FIG3_DOWNWARD_DYNAMIC) {
        switch (currentStep) {
            case 0: std::cout << "Downward Pass (Dynamic). Evaluating Z-Y." << std::endl; break;
            case 1: std::cout << "Applying impulse to Z (Yellow)." << std::endl; break;
            case 2: std::cout << "Applying stored impulse to Y (Red). Notice Y starts to rotate due to off-center force." << std::endl; break;
            case 3: std::cout << "Evaluating Y-X constraint." << std::endl; break;
            case 4: std::cout << "Applying impulse to Y (Yellow). Rotation increases." << std::endl; break;
            case 5: std::cout << "Applying stored impulse to X (Blue). X also starts to rotate and fall." << std::endl; break;
            case 6: std::cout << "Evaluating X-Ground." << std::endl; break;
            case 7: std::cout << "Applying final impulse to X. The Two-Pass method correctly captures the dynamic falling behavior." << std::endl; break;
        }
    }
}

void Simulation::autoPlay(bool enable) {
    isAutoPlay = enable;
    autoPlayTimer = 0.0f;
}

void Simulation::update(float deltaTime) {
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

int Simulation::getTotalSteps() const {
    switch (currentMode) {
        case FIG1_UPWARD: return 7;
        case FIG2_DOWNWARD_STATIC: return 8;
        case FIG3_DOWNWARD_DYNAMIC: return 8;
        default: return 7;
    }
}

std::string Simulation::getModeName() const {
    switch (currentMode) {
        case FIG1_UPWARD: return "Fig.1: Upward Pass (Static)";
        case FIG2_DOWNWARD_STATIC: return "Fig.2: Downward Pass (Static)";
        case FIG3_DOWNWARD_DYNAMIC: return "Fig.3: Downward Pass (Dynamic)";
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
        default: return "";
    }
}

void Simulation::updateStep() {
    impulses.clear();
    printStepExplanation();
    
    glm::vec3 yellowImpulse(1.0f, 0.8f, 0.0f);
    glm::vec3 redImpulse(1.0f, 0.1f, 0.1f);
    glm::vec3 blueImpulse(0.2f, 0.4f, 1.0f);
    
    for (auto& box : boxes) {
        box.setInfiniteMass(false);
        box.setRotation(glm::vec3(0.0f));
    }
    
    if (currentMode == FIG1_UPWARD) {
        switch (currentStep) {
            case 0: boxes[0].setPosition(glm::vec3(0.0f, 0.3f, 0.0f)); break;
            case 1: boxes[0].setPosition(glm::vec3(0.0f, 0.3f, 0.0f)); break;
            case 2: boxes[0].setPosition(glm::vec3(0.0f, 0.5f, 0.0f));
                    impulses.push_back(Impulse(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f)); break;
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
            case 0: boxes[1].setInfiniteMass(false); break;
            case 1: impulses.push_back(Impulse(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.1f)); break;
            case 2: impulses.push_back(Impulse(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), redImpulse, 0.5f)); break;
            case 3: boxes[0].setInfiniteMass(true); break;
            case 4: impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f)); break;
            case 5: boxes[0].setPosition(glm::vec3(0.0f, 0.3f, 0.0f));
                    impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), blueImpulse, 0.5f)); break;
            case 6: boxes[0].setPosition(glm::vec3(0.0f, 0.3f, 0.0f)); break;
            case 7: boxes[0].setPosition(glm::vec3(0.0f, 0.5f, 0.0f));
                    impulses.push_back(Impulse(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f)); break;
        }
    } else if (currentMode == FIG3_DOWNWARD_DYNAMIC) {
        switch (currentStep) {
            case 1: impulses.push_back(Impulse(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.1f)); break;
            case 2: boxes[1].setRotation(glm::vec3(0.0f, 0.0f, -0.1f));
                    impulses.push_back(Impulse(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), redImpulse, 0.5f)); break;
            case 3: boxes[1].setRotation(glm::vec3(0.0f, 0.0f, -0.1f)); boxes[0].setInfiniteMass(true); break;
            case 4: boxes[1].setRotation(glm::vec3(0.0f, 0.0f, -0.15f));
                    impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f)); break;
            case 5: boxes[1].setRotation(glm::vec3(0.0f, 0.0f, -0.15f)); boxes[0].setRotation(glm::vec3(0.0f, 0.0f, -0.2f));
                    impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), blueImpulse, 0.5f)); break;
            case 6: boxes[1].setRotation(glm::vec3(0.0f, 0.0f, -0.15f)); boxes[0].setRotation(glm::vec3(0.0f, 0.0f, -0.2f)); break;
            case 7: boxes[1].setRotation(glm::vec3(0.0f, 0.0f, -0.15f)); boxes[0].setRotation(glm::vec3(0.0f, 0.0f, -0.25f));
                    impulses.push_back(Impulse(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f)); break;
        }
    }
}
