// input.h
#pragma once
#include "GameEngine.h"
#include "ui.h"

extern std::map<int, bool> keyStates; // Map to track key states

// A function to execute another function once when the key is pressed and held down
void press_once(GLFWwindow* window, int key, void(*func)(GLFWwindow* window));
void press_once_noargs(GLFWwindow* window, int key, void(*func)());
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void processInput(GLFWwindow* window);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

