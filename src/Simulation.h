#ifndef SIMULATION_H
#define SIMULATION_H

#include "Box.h"
#include "Impulse.h"
#include <vector>
#include <string>

class Simulation {
public:
    enum Mode {
        FIG1_UPWARD,
        FIG2_DOWNWARD_STATIC,
        FIG3_DOWNWARD_DYNAMIC,
        PHYSICS_SIM
    };
    
    Simulation();
    
    void setMode(Mode mode);
    void nextStep();
    void prevStep();
    void reset();
    void autoPlay(bool enable);
    void update(float deltaTime);
    
    const std::vector<Box>& getBoxes() const { return boxes; }
    const std::vector<Impulse>& getImpulses() const { return impulses; }
    std::string getCurrentStepDescription() const;
    std::string getModeName() const;
    int getCurrentStep() const { return currentStep; }
    int getTotalSteps() const;
    
private:
    void initializeBoxes();
    void updateStep();
    void printStepExplanation() const;
    
    // Physics Engine Methods (Ten Minute Physics / XPBD style)
    void stepPhysics(float dt);
    void solveConstraints(float dt);
    
    // Detailed Collision Detection (SAT - Separating Axis Theorem)
    struct Contact {
        glm::vec3 normal;
        float penetration;
        glm::vec3 point;
    };
    bool checkCollision(const Box& a, const Box& b, Contact& contact);
    void solveContact(Box& a, Box& b, const Contact& contact, float weightA, float weightB);
    void solveGround(Box& box);
    
    Mode currentMode;
    int currentStep;
    bool isAutoPlay;
    float autoPlayTimer;
    float autoPlayInterval;
    
    std::vector<Box> boxes;
    std::vector<Impulse> impulses;
    
    // Physics constants
    glm::vec3 gravity;
    int numSubsteps;
    int numIterations;
};

#endif // SIMULATION_H
