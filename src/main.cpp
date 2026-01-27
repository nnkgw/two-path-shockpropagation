#include "Simulation.h"
#include "Renderer.h"
#include <GL/freeglut.h>
#include <iostream>
#include <chrono>
#include <cstdlib>

// Global objects
Simulation* g_simulation = nullptr;
Renderer* g_renderer = nullptr;

// Timing
auto g_lastTime = std::chrono::high_resolution_clock::now();

// Mouse control
int g_lastMouseX = 0;
int g_lastMouseY = 0;
bool g_mouseLeftDown = false;
bool g_mouseRightDown = false;

void display() {
    if (g_simulation && g_renderer) {
        g_renderer->renderScene(*g_simulation);
    }
}

void reshape(int width, int height) {
    if (g_renderer) {
        g_renderer->reshape(width, height);
    }
}

void keyboard(unsigned char key, int x, int y) {
    (void)x;
    (void)y;
    if (!g_simulation) return;
    
    switch (key) {
        case '1':
            g_simulation->setMode(Simulation::FIG1_UPWARD);
            break;
        case '2':
            g_simulation->setMode(Simulation::FIG2_DOWNWARD_STATIC);
            break;
        case '3':
            g_simulation->setMode(Simulation::FIG3_DOWNWARD_DYNAMIC);
            break;
        case ' ':
            g_simulation->nextStep();
            break;
        case 8: // Backspace
            g_simulation->prevStep();
            break;
        case 'r':
        case 'R':
            g_simulation->reset();
            if (g_renderer) g_renderer->resetCamera();
            break;
        case 'p':
        case 'P':
            {
                static bool autoPlay = false;
                autoPlay = !autoPlay;
                g_simulation->autoPlay(autoPlay);
            }
            break;
        case '+':
        case '=':
            if (g_renderer) g_renderer->zoomCamera(-0.5f);
            break;
        case '-':
        case '_':
            if (g_renderer) g_renderer->zoomCamera(0.5f);
            break;
        case 27: // ESC
            exit(0);
            break;
    }
    
    glutPostRedisplay();
}

void specialKeys(int key, int x, int y) {
    (void)x;
    (void)y;
    if (!g_renderer) return;
    
    switch (key) {
        case GLUT_KEY_UP:
            g_renderer->rotateCamera(0.0f, 5.0f);
            break;
        case GLUT_KEY_DOWN:
            g_renderer->rotateCamera(0.0f, -5.0f);
            break;
        case GLUT_KEY_LEFT:
            g_renderer->rotateCamera(-5.0f, 0.0f);
            break;
        case GLUT_KEY_RIGHT:
            g_renderer->rotateCamera(5.0f, 0.0f);
            break;
    }
    
    glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            g_mouseLeftDown = true;
            g_lastMouseX = x;
            g_lastMouseY = y;
        } else {
            g_mouseLeftDown = false;
        }
    } else if (button == GLUT_RIGHT_BUTTON) {
        if (state == GLUT_DOWN) {
            g_mouseRightDown = true;
            g_lastMouseX = x;
            g_lastMouseY = y;
        } else {
            g_mouseRightDown = false;
        }
    } else if (button == 3) { // Mouse wheel up
        if (g_renderer) g_renderer->zoomCamera(-0.5f);
    } else if (button == 4) { // Mouse wheel down
        if (g_renderer) g_renderer->zoomCamera(0.5f);
    }
    glutPostRedisplay();
}

void motion(int x, int y) {
    if (g_renderer) {
        int dx = x - g_lastMouseX;
        int dy = y - g_lastMouseY;
        
        if (g_mouseLeftDown) {
            g_renderer->rotateCamera((float)dx * 0.5f, (float)-dy * 0.5f);
        } else if (g_mouseRightDown) {
            g_renderer->panCamera((float)dx, (float)dy);
        }
        
        g_lastMouseX = x;
        g_lastMouseY = y;
        
        glutPostRedisplay();
    }
}

void idle() {
    auto currentTime = std::chrono::high_resolution_clock::now();
    float deltaTime = std::chrono::duration<float>(currentTime - g_lastTime).count();
    g_lastTime = currentTime;
    
    if (g_simulation) {
        g_simulation->update(deltaTime);
    }
    
    glutPostRedisplay();
}

void printUsage() {
    std::cout << "==========================================================" << std::endl;
    std::cout << "        Two-Path-ShockPropagation Visualizer" << std::endl;
    std::cout << "==========================================================" << std::endl;
    std::cout << "Based on: \"Two-Pass Shock Propagation for Stable Stacking\"" << std::endl;
    std::cout << "          by Xiong et al., SIGGRAPH 2025" << std::endl;
    std::cout << "----------------------------------------------------------" << std::endl;
    std::cout << " [CONTROLS]" << std::endl;
    std::cout << "  1, 2, 3      : Switch between Fig.1, Fig.2, Fig.3" << std::endl;
    std::cout << "  SPACE        : Next step" << std::endl;
    std::cout << "  BACKSPACE    : Previous step" << std::endl;
    std::cout << "  P            : Toggle Auto-play" << std::endl;
    std::cout << "  R            : Reset simulation and camera" << std::endl;
    std::cout << "  ESC          : Exit" << std::endl;
    std::cout << "----------------------------------------------------------" << std::endl;
    std::cout << " [MOUSE]" << std::endl;
    std::cout << "  Left Drag    : Rotate camera" << std::endl;
    std::cout << "  Right Drag   : Pan camera" << std::endl;
    std::cout << "  Wheel Up/Dn  : Zoom in/out" << std::endl;
    std::cout << "----------------------------------------------------------" << std::endl;
    std::cout << " [VISUALS]" << std::endl;
    std::cout << "  Yellow Arrow : Impulse applied to Top body" << std::endl;
    std::cout << "  Red Arrow    : Stored reaction impulse on Bottom body" << std::endl;
    std::cout << "  Blue Arrow   : Stored impulse from previous layers" << std::endl;
    std::cout << "  Thick Border : Infinite mass state" << std::endl;
    std::cout << "==========================================================" << std::endl;
}

int main(int argc, char** argv) {
    printUsage();
    
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1024, 768);
    glutCreateWindow("Two-Path-ShockPropagation");
    
    g_simulation = new Simulation();
    g_renderer = new Renderer();
    g_renderer->initialize(1024, 768);
    
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKeys);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutIdleFunc(idle);
    
    glutMainLoop();
    
    delete g_simulation;
    delete g_renderer;
    
    return 0;
}
