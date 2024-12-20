// input.cpp
#pragma once
#include "GameEngine.h"
#include "ui.h"
#include "camera.h"
#include "console.h"
#include "selection.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


bool consoleInputOnly = false;
std::map<int, bool> keyStates; // Map to track key states
extern bool play;
extern bool genSpheres;

// A function to execute another function once when the key is pressed and held down
void press_once(GLFWwindow* window, int key, void(*func)(GLFWwindow* window)) {
    bool isPressed = glfwGetKey(window, key) == GLFW_PRESS;

    // Initialize key state if not already present
    if (keyStates.find(key) == keyStates.end()) {
        keyStates[key] = false;
    }

    // Check if the key was not pressed previously but is pressed now
    if (isPressed && !keyStates[key]) {
        func(window);
    }

    // Update the previous key state
    keyStates[key] = isPressed;
}

void press_once_noargs(GLFWwindow* window, int key, void(*func)()) {
    bool isPressed = glfwGetKey(window, key) == GLFW_PRESS;

    // Initialize key state if not already present
    if (keyStates.find(key) == keyStates.end()) {
        keyStates[key] = false;
    }

    // Check if the key was not pressed previously but is pressed now
    if (isPressed && !keyStates[key]) {
        func();
    }

    // Update the previous key state
    keyStates[key] = isPressed;
}

// Mouse callback function
void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
    static double lastX = 400.0;
    static double lastY = 300.0;
    static bool firstMouse = true;


    // Only allow object selection when camera is NOT in movement mode
    if (!camstate && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        int width, height;
        glfwGetWindowSize(window, &width, &height);

        glm::mat4 projection = glm::perspective(glm::radians(45.0f),
            (float)width / (float)height, 0.1f, 100.0f);
        glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);

        Ray ray = SelectionSystem::screenToWorldRay(xpos, ypos, width, height,
            projection, view);
        selectedRB = SelectionSystem::findIntersectedBody(ray, bodies);
    }


    // Process mouse movement only if camstate is true
    if (camstate) {
        if (firstMouse) {
            lastX = xpos;
            lastY = ypos;
            firstMouse = false;
        }

        float xoffset = xpos - lastX;
        float yoffset = lastY - ypos; // Reversed Y axis
        lastX = xpos;
        lastY = ypos;

        xoffset *= sensitivity;
        yoffset *= sensitivity;

        yaw += xoffset;
        pitch += yoffset;

        // Constrain pitch
        if (pitch > 89.0f) pitch = 89.0f;
        if (pitch < -89.0f) pitch = -89.0f;
    }
    else {
        // If camstate is false, reset lastX and lastY to the current cursor position
        lastX = xpos;
        lastY = ypos;
    }

}

void togglePlay() {
    play = !play;
}

void toggleGenSpheres() {
	genSpheres = !genSpheres;
}

// Add these functions
void toggleConsole() {
    consoleInputOnly = !consoleInputOnly;
    Console::getInstance().toggleVisibility();
}

void handleConsoleInput(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (!Console::getInstance().isVisible()) return;

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        if (key == GLFW_KEY_BACKSPACE) {
            Console::getInstance().removeLastChar();
        }
		else if (key == GLFW_KEY_ESCAPE) {
			toggleConsole();
		}
        else if (key == GLFW_KEY_ENTER) {
            Console::getInstance().executeCommand();
        }
        else if (key >= 32 && key <= 126) {
            char character = static_cast<char>(key);
            // Check if Shift key is pressed
            if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS) {
                // Convert to uppercase if Shift is pressed
                if (character >= 'a' && character <= 'z') {
                    character = character - 'a' + 'A';
                }
            }
            else {
                // Convert to lowercase if Shift is not pressed
                if (character >= 'A' && character <= 'Z') {
                    character = character - 'A' + 'a';
                }
            }
            Console::getInstance().addInput(character);
        }
    }
}

void processInput(GLFWwindow* window) {

    if (Console::getInstance().isVisible()) {
        press_once_noargs(window, GLFW_KEY_GRAVE_ACCENT, toggleConsole);
        return;
    }

    /*if (MenuSystem::getInstance().isMenuOpen()) {
        std::cout << "working" << std::endl;
        // Handle menu input
        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        MenuSystem::getInstance().handleMouseMove(xpos, ypos);

        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
            MenuSystem::getInstance().handleClick(xpos, ypos);
        }

        press_once_noargs(window, GLFW_KEY_P, toggleMenu);
        return;
    }*/

    // Call press_once for the spacebar key
    press_once(window, GLFW_KEY_SPACE, toggleCam);
    press_once_noargs(window, GLFW_KEY_P, toggleMenu);
    press_once_noargs(window, GLFW_KEY_L, togglePlay);
    press_once_noargs(window, GLFW_KEY_GRAVE_ACCENT, toggleConsole);
	press_once_noargs(window, GLFW_KEY_G, toggleGenSpheres);

    // Handle movement controls only if camstate is true
    if (camstate) {
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
            cameraPos += cameraSpeed * cameraFront;
        }
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
            cameraPos -= cameraSpeed * cameraFront;
        }
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
			cameraPos -= cameraSpeed * glm::normalize(glm::cross(cameraFront, cameraUp));
        }
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
            cameraPos += cameraSpeed * glm::normalize(glm::cross(cameraFront, cameraUp));
        }
        // Additional processing for mouse
        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        mouse_callback(window, xpos, ypos);
    }
}

// After the processInput function, add:
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (Console::getInstance().isVisible()) {
        handleConsoleInput(window, key, scancode, action, mods);
    }

}
