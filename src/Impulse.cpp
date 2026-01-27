#include "Impulse.h"

Impulse::Impulse(const glm::vec3& pos, const glm::vec3& dir, const glm::vec3& color, float mag)
    : position(pos), direction(dir), color(color), magnitude(mag) {
}
