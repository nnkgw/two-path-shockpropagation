#include "Renderer.h"

#if defined(WIN32)
  #pragma warning(disable:4996)
  #include <GL/freeglut.h>
#elif defined(__APPLE__) || defined(MACOSX)
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  #define GL_SILENCE_DEPRECATION
  #include <GLUT/glut.h>
#else
  #include <GL/freeglut.h>
#endif

#ifdef _WIN32
#include <GL/glu.h>
#endif
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <cmath>
#include <sstream>

Renderer::Renderer()
    : cameraAngleX(30.0f), cameraAngleY(45.0f), cameraDistance(10.0f),
      cameraTarget(0.0f, 1.5f, 0.0f),
      windowWidth(800), windowHeight(600) {
}

void Renderer::initialize(int width, int height) {
    windowWidth = width;
    windowHeight = height;
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    
    GLfloat lightPos[] = {5.0f, 10.0f, 5.0f, 1.0f};
    GLfloat lightAmbient[] = {0.3f, 0.3f, 0.3f, 1.0f};
    GLfloat lightDiffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
    
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
    
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
    
    reshape(width, height);
}

void Renderer::reshape(int width, int height) {
    windowWidth = width;
    windowHeight = height;
    glViewport(0, 0, width, height);
}

void Renderer::rotateCamera(float deltaX, float deltaY) {
    cameraAngleY += deltaX;
    cameraAngleX += deltaY;
    
    if (cameraAngleX > 89.0f) cameraAngleX = 89.0f;
    if (cameraAngleX < -89.0f) cameraAngleX = -89.0f;
}

void Renderer::panCamera(float deltaX, float deltaY) {
    float sensitivity = 0.01f * cameraDistance;
    
    float radX = glm::radians(cameraAngleX);
    float radY = glm::radians(cameraAngleY);
    
    glm::vec3 forward(
        -cos(radX) * sin(radY),
        -sin(radX),
        -cos(radX) * cos(radY)
    );
    glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0, 1, 0)));
    glm::vec3 up = glm::normalize(glm::cross(right, forward));
    
    cameraTarget -= right * (deltaX * sensitivity);
    cameraTarget += up * (deltaY * sensitivity);
}

void Renderer::zoomCamera(float delta) {
    cameraDistance += delta;
    if (cameraDistance < 1.0f) cameraDistance = 1.0f;
    if (cameraDistance > 50.0f) cameraDistance = 50.0f;
}

void Renderer::resetCamera() {
    cameraAngleX = 30.0f;
    cameraAngleY = 45.0f;
    cameraDistance = 10.0f;
    cameraTarget = glm::vec3(0.0f, 1.5f, 0.0f);
}

void Renderer::renderScene(const Simulation& sim) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)windowWidth / (double)windowHeight, 0.1, 100.0);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    float camX = cameraTarget.x + cameraDistance * cos(glm::radians(cameraAngleX)) * sin(glm::radians(cameraAngleY));
    float camY = cameraTarget.y + cameraDistance * sin(glm::radians(cameraAngleX));
    float camZ = cameraTarget.z + cameraDistance * cos(glm::radians(cameraAngleX)) * cos(glm::radians(cameraAngleY));
    
    gluLookAt(camX, camY, camZ, cameraTarget.x, cameraTarget.y, cameraTarget.z, 0.0, 1.0, 0.0);
    
    renderGround();
    
    // Render boxes first
    for (const auto& box : sim.getBoxes()) {
        renderBox(box);
    }
    
    // Render all labels after all boxes to ensure they are on top
    for (const auto& box : sim.getBoxes()) {
        renderText3D(box.getName(), box.getPosition() + glm::vec3(-1.2f, 0.0f, 0.0f), glm::vec3(1.0f));
    }
    
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    for (const auto& impulse : sim.getImpulses()) {
        renderImpulse(impulse);
    }
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    
    renderHUD(sim);
    
    glutSwapBuffers();
}

void Renderer::renderBox(const Box& box) {
    glPushMatrix();
    
    glm::vec3 pos = box.getPosition();
    glm::quat q = box.getOrientation();
    glm::vec3 size = box.getSize();
    glm::vec3 color = box.getColor();
    
    glTranslatef(pos.x, pos.y, pos.z);
    
    // Convert quaternion to rotation matrix for OpenGL
    glm::mat4 rotationMatrix = glm::mat4_cast(q);
    glMultMatrixf(glm::value_ptr(rotationMatrix));
    
    glColor3f(color.r, color.g, color.b);
    
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);
    glutSolidCube(size.x);
    glDisable(GL_POLYGON_OFFSET_FILL);
    
    glDisable(GL_LIGHTING);
    if (box.isInfiniteMass()) {
        glLineWidth(4.0f);
        glColor3f(1.0f, 1.0f, 1.0f);
    } else {
        glLineWidth(1.5f);
        glColor3f(0.0f, 0.0f, 0.0f);
    }
    glutWireCube(size.x);
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
    
    glPopMatrix();
}

void Renderer::renderGround() {
    glDisable(GL_LIGHTING);
    glColor3f(0.3f, 0.3f, 0.3f);
    
    glBegin(GL_QUADS);
    glVertex3f(-5.0f, 0.0f, -5.0f);
    glVertex3f(5.0f, 0.0f, -5.0f);
    glVertex3f(5.0f, 0.0f, 5.0f);
    glVertex3f(-5.0f, 0.0f, 5.0f);
    glEnd();
    
    glColor3f(0.4f, 0.4f, 0.4f);
    glBegin(GL_LINES);
    for (int i = -5; i <= 5; i++) {
        glVertex3f((float)i, 0.01f, -5.0f);
        glVertex3f((float)i, 0.01f, 5.0f);
        glVertex3f(-5.0f, 0.01f, (float)i);
        glVertex3f(5.0f, 0.01f, (float)i);
    }
    glEnd();
    
    glEnable(GL_LIGHTING);
}

void Renderer::renderImpulse(const Impulse& impulse) {
    glm::vec3 pos = impulse.getPosition();
    glm::vec3 dir = glm::normalize(impulse.getDirection());
    glm::vec3 color = impulse.getColor();
    float mag = impulse.getMagnitude();
    
    if (mag < 0.01f) return;
    
    glm::vec3 offset(0.6f, 0.0f, 0.0f); 
    glm::vec3 startPos = pos + offset;
    
    float arrowLength = mag * 1.5f;
    glm::vec3 end = startPos + dir * arrowLength;
    renderArrow(startPos, end, color, 0.08f);
}

void Renderer::renderArrow(const glm::vec3& start, const glm::vec3& end, const glm::vec3& color, float thickness) {
    (void)thickness;
    glColor3f(color.r, color.g, color.b);
    glLineWidth(5.0f);
    
    glBegin(GL_LINES);
    glVertex3f(start.x, start.y, start.z);
    glVertex3f(end.x, end.y, end.z);
    glEnd();
    
    glm::vec3 dir = glm::normalize(end - start);
    
    glPushMatrix();
    glTranslatef(end.x, end.y, end.z);
    
    glm::vec3 zAxis(0.0f, 0.0f, 1.0f);
    glm::vec3 axis = glm::cross(zAxis, dir);
    float dot = glm::dot(zAxis, dir);
    float angle = glm::degrees(acos(glm::clamp(dot, -1.0f, 1.0f)));
    
    if (glm::length(axis) > 0.001f) {
        glRotatef(angle, axis.x, axis.y, axis.z);
    } else if (dot < 0.0f) {
        glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
    }
    
    glutSolidCone(0.15, 0.3, 12, 1);
    glPopMatrix();
    
    glLineWidth(1.0f);
}

void Renderer::renderText(const std::string& text, float x, float y) {
    glRasterPos2f(x, y);
    for (char c : text) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
    }
}

void Renderer::renderText3D(const std::string& text, const glm::vec3& pos, const glm::vec3& color) {
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    
    // Draw shadow (black, slightly offset)
    glColor3f(0.0f, 0.0f, 0.0f);
    glRasterPos3f(pos.x + 0.02f, pos.y - 0.02f, pos.z);
    for (char c : text) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
    }
    
    // Draw main text (white)
    glColor3f(color.r, color.g, color.b);
    glRasterPos3f(pos.x, pos.y, pos.z);
    for (char c : text) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
    }
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
}

void Renderer::renderHUD(const Simulation& sim) {
    glDisable(GL_LIGHTING);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, windowWidth, 0, windowHeight);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    glColor3f(1.0f, 1.0f, 1.0f);
    
    std::stringstream ss;
    ss << sim.getModeName();
    renderText(ss.str(), 10.0f, (float)windowHeight - 20.0f);
    
    ss.str("");
    ss << "Step " << (sim.getCurrentStep() + 1) << "/" << sim.getTotalSteps() << ": " << sim.getCurrentStepDescription();
    renderText(ss.str(), 10.0f, (float)windowHeight - 40.0f);
    
    renderText("Controls:", 10.0f, 120.0f);
    renderText("1/2/3/4: Switch mode", 10.0f, 100.0f);
    renderText("Space: Next step", 10.0f, 80.0f);
    renderText("Backspace: Previous step", 10.0f, 60.0f);
    renderText("P: Auto play", 10.0f, 40.0f);
    renderText("R: Reset", 10.0f, 20.0f);
    
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glEnable(GL_LIGHTING);
}
