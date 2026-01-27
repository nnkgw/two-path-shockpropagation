#ifndef BOX_H
#define BOX_H

#include <glm/glm.hpp>
#include <string>

class Box {
public:
    Box(const std::string& name, const glm::vec3& pos, const glm::vec3& size, const glm::vec3& color);
    
    void setPosition(const glm::vec3& pos);
    void setRotation(const glm::vec3& rot);
    void setInfiniteMass(bool infinite);
    void reset();
    
    glm::vec3 getPosition() const { return position; }
    glm::vec3 getRotation() const { return rotation; }
    glm::vec3 getSize() const { return size; }
    glm::vec3 getColor() const { return color; }
    bool isInfiniteMass() const { return infiniteMass; }
    std::string getName() const { return name; }
    
private:
    std::string name;
    glm::vec3 position;
    glm::vec3 rotation;
    glm::vec3 size;
    glm::vec3 color;
    glm::vec3 initialPosition;
    glm::vec3 initialRotation;
    bool infiniteMass;
};

#endif // BOX_H
