// camera.h
#pragma once
#include "GameEngine.h"

// Define initial camera parameters
extern glm::vec3 cameraPos;
extern glm::vec3 cameraFront;
extern glm::vec3 cameraUp;
extern float cameraSpeed; // Speed of camera movement
extern float yaw; // Yaw angle
extern float pitch; // Pitch angle
extern float sensitivity; // Mouse sensitivity
extern bool camstate; // camera can be moved when true

// Define initial camera parameters
class Camera {
public:
	glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 5.0f);
	glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
	glm::vec3 cameraUp = glm::vec3(0.0f, 0.0f, 1.0f);
	float cameraSpeed = 0.01f; // Speed of camera movement
	float yaw = -90.0f;// Yaw angle
	float pitch = 0.0f; // Pitch angle
	float sensitivity = 0.1f; // Mouse sensitivity
	bool camstate; // camera can be moved with controls in input.cpp when true
	// glm::vec3 viewPos; // just a vector of cameraPos (remember to change all instances of this to camera.cameraPos)
};

extern glm::vec3 viewPos; // just a vector of cameraPos

void toggleCam(GLFWwindow* window);
void updateCamera(GLuint shaderProgram);
bool isInViewFrustum(const glm::vec3& point, const glm::mat4& viewProjection);