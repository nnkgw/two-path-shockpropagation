#ifndef IMPULSE_H
#define IMPULSE_H

#include <glm/glm.hpp>

class Impulse {
public:
    Impulse(const glm::vec3& pos, const glm::vec3& dir, const glm::vec3& color, float mag);
    
    glm::vec3 getPosition() const { return position; }
    glm::vec3 getDirection() const { return direction; }
    glm::vec3 getColor() const { return color; }
    float getMagnitude() const { return magnitude; }
    
private:
    glm::vec3 position;
    glm::vec3 direction;
    glm::vec3 color;
    float magnitude;
};

#endif // IMPULSE_H
