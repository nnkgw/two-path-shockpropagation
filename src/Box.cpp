#include "Box.h"

Box::Box(const std::string& name, const glm::vec3& pos, const glm::vec3& size, const glm::vec3& color)
    : name(name), 
      position(pos), 
      rotation(0.0f), 
      size(size), 
      color(color), 
      initialPosition(pos), 
      initialRotation(0.0f), 
      infiniteMass(false) {
}

void Box::setPosition(const glm::vec3& pos) {
    position = pos;
}

void Box::setRotation(const glm::vec3& rot) {
    rotation = rot;
}

void Box::setInfiniteMass(bool infinite) {
    infiniteMass = infinite;
}

void Box::reset() {
    position = initialPosition;
    rotation = initialRotation;
    infiniteMass = false;
}
