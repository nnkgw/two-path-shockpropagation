#include "Box.h"
#include <glm/gtc/matrix_transform.hpp>

Box::Box(const std::string& name, const glm::vec3& pos, const glm::vec3& size, const glm::vec3& color, float mass)
    : name(name), 
      position(pos), 
      rotation(0.0f), 
      orientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f)),
      size(size), 
      color(color), 
      mass(mass),
      velocity(0.0f),
      angularVelocity(0.0f),
      infiniteMass(false),
      initialPosition(pos), 
      initialOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f)),
      initialMass(mass) {
    prevPosition = pos;
    prevOrientation = orientation;
}

void Box::setPosition(const glm::vec3& pos) {
    position = pos;
}

void Box::setRotation(const glm::vec3& euler) {
    rotation = euler;
    orientation = glm::quat(euler);
}

void Box::setOrientation(const glm::quat& q) {
    orientation = q;
    rotation = glm::eulerAngles(q);
}

void Box::setVelocity(const glm::vec3& v) {
    velocity = v;
}

void Box::setAngularVelocity(const glm::vec3& w) {
    angularVelocity = w;
}

void Box::setInfiniteMass(bool infinite) {
    infiniteMass = infinite;
}

void Box::reset() {
    position = initialPosition;
    orientation = initialOrientation;
    rotation = glm::eulerAngles(orientation);
    velocity = glm::vec3(0.0f);
    angularVelocity = glm::vec3(0.0f);
    infiniteMass = false;
    prevPosition = position;
    prevOrientation = orientation;
}
