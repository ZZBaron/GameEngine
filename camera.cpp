// camera.cpp
#pragma once
#include "GameEngine.h"
#include "input.h"


glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 5.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 0.0f, 1.0f);
float cameraSpeed = 0.01f; // Speed of camera movement
float yaw = -90.0f; // Yaw angle, needs to match inital cameraFront and cameraUp
float pitch = 0.0f; // Pitch angle, needs to match inital cameraFront and cameraUp
float sensitivity = 0.1f; // Mouse sensitivity
bool camstate; // camera can be moved when true

glm::vec3 viewPos = cameraPos;


// Function to check if a point is inside the view frustum
bool isInViewFrustum(const glm::vec3& point, const glm::mat4& viewProjection) {
    glm::vec4 clipSpacePosition = viewProjection * glm::vec4(point, 1.0f);
    return std::abs(clipSpacePosition.x) <= std::abs(clipSpacePosition.w) &&
        std::abs(clipSpacePosition.y) <= std::abs(clipSpacePosition.w) &&
        0 <= clipSpacePosition.z && clipSpacePosition.z <= clipSpacePosition.w;
}

void toggleCam(GLFWwindow* window) {
    camstate = !camstate;
    if (camstate) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        // Update cameraFront based on current yaw and pitch
        glm::vec3 front;
        front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
        front.y = sin(glm::radians(pitch));
        front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
        cameraFront = glm::normalize(front);
    }
    else {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        // Optionally, save current yaw and pitch if you want to restore them later
    }
}

void updateCamera(GLuint shaderProgram) {
    // Calculate the front vector based on yaw and pitch angles
    glm::vec3 front;
    front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    front.y = sin(glm::radians(pitch));
    front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    front = glm::normalize(front);

    // Calculate the right and up vectors
    glm::vec3 right = glm::normalize(glm::cross(front, glm::vec3(0.0f, 1.0f, 0.0f)));
    glm::vec3 up = glm::normalize(glm::cross(right, front));

    // Compute the view matrix
    glm::mat4 view = glm::lookAt(
        cameraPos,           // Camera position
        cameraPos + front,  // Look at position
        up                                                      // Up vector
    );

    // update front vector
	cameraFront = front;

	//update up vector
	cameraUp = up;

    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, &view[0][0]);

    // Update viewPos with current camera position
    viewPos = cameraPos;

    // Update the shader with the view position
    glUniform3fv(glGetUniformLocation(shaderProgram, "viewPos"), 1, glm::value_ptr(viewPos));

}

