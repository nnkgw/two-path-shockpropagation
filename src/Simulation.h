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
        FIG3_DOWNWARD_DYNAMIC
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
    
    Mode currentMode;
    int currentStep;
    bool isAutoPlay;
    float autoPlayTimer;
    float autoPlayInterval;
    
    std::vector<Box> boxes;
    std::vector<Impulse> impulses;
};

#endif // SIMULATION_H
