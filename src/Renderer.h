#ifndef RENDERER_H
#define RENDERER_H

#include "Box.h"
#include "Impulse.h"
#include "Simulation.h"
#include <glm/glm.hpp>
#include <string>

class Renderer {
public:
    Renderer();
    
    void initialize(int width, int height);
    void renderScene(const Simulation& sim);
    void reshape(int width, int height);
    
    void rotateCamera(float deltaX, float deltaY);
    void panCamera(float deltaX, float deltaY);
    void zoomCamera(float delta);
    void resetCamera();
    
private:
    void renderBox(const Box& box);
    void renderImpulse(const Impulse& impulse);
    void renderGround();
    void renderArrow(const glm::vec3& start, const glm::vec3& end, const glm::vec3& color, float thickness);
    void renderText(const std::string& text, float x, float y);
    void renderText3D(const std::string& text, const glm::vec3& pos, const glm::vec3& color);
    void renderHUD(const Simulation& sim);
    
    void drawCube(float size);
    void drawCone(float radius, float height);
    
    glm::mat4 viewMatrix;
    glm::mat4 projectionMatrix;
    float cameraAngleX;
    float cameraAngleY;
    float cameraDistance;
    glm::vec3 cameraTarget;
    int windowWidth;
    int windowHeight;
};

#endif // RENDERER_H
