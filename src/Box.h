#ifndef BOX_H
#define BOX_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>

class Box {
public:
    Box(const std::string& name, const glm::vec3& pos, const glm::vec3& size, const glm::vec3& color, float mass = 1.0f);
    
    void setPosition(const glm::vec3& pos);
    void setRotation(const glm::vec3& euler);
    void setOrientation(const glm::quat& q);
    void setVelocity(const glm::vec3& v);
    void setAngularVelocity(const glm::vec3& w);
    void setInfiniteMass(bool infinite);
    void reset();
    
    glm::vec3 getPosition() const { return position; }
    glm::vec3 getRotation() const { return rotation; } // Euler angles for rendering
    glm::quat getOrientation() const { return orientation; }
    glm::vec3 getSize() const { return size; }
    glm::vec3 getColor() const { return color; }
    bool isInfiniteMass() const { return infiniteMass; }
    std::string getName() const { return name; }
    
    // Physics properties
    float getMass() const { return infiniteMass ? 0.0f : mass; }
    float getInvMass() const { return (infiniteMass || mass <= 0.0f) ? 0.0f : 1.0f / mass; }
    glm::vec3 getVelocity() const { return velocity; }
    glm::vec3 getAngularVelocity() const { return angularVelocity; }
    
    // For PBD/XPBD style updates
    glm::vec3 prevPosition;
    glm::quat prevOrientation;

private:
    std::string name;
    glm::vec3 position;
    glm::vec3 rotation; // Euler angles (cached for rendering)
    glm::quat orientation;
    glm::vec3 size;
    glm::vec3 color;
    
    // Physics
    float mass;
    glm::vec3 velocity;
    glm::vec3 angularVelocity;
    bool infiniteMass;
    
    // Initial state
    glm::vec3 initialPosition;
    glm::quat initialOrientation;
    float initialMass;
};

#endif // BOX_H
