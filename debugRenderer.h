#pragma once
#include "GameEngine.h"

void renderQuad();
GLuint initDebugDepthShader();
void renderDepthMapToQuad(GLuint depthMap, const glm::mat4& lightSpaceMatrix, float near_plane, float far_plane);



