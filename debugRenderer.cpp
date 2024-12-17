// Add these functions to your code, perhaps in a new file called "debugRenderer.h" and "debugRenderer.cpp"

#pragma once
#include "GameEngine.h"

GLuint quadVAO = 0;
GLuint quadVBO;
GLuint debugDepthShader;

void renderQuad()
{
    if (quadVAO == 0)
    {
        float quadVertices[] = {
            // positions        // texture Coords
            -1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
            -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
             1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
             1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
        };
        // setup plane VAO
        glGenVertexArrays(1, &quadVAO);
        glGenBuffers(1, &quadVBO);
        glBindVertexArray(quadVAO);
        glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    }
    glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);
}

GLuint initDebugDepthShader()
{
    const char* vertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec2 aTexCoords;

        out vec2 TexCoords;
        
        uniform mat4 projection;

        void main()
        {
            TexCoords = aTexCoords;
            gl_Position = projection * vec4(aPos, 1.0);
        }
    )";

    //const char* fragmentShaderSource = R"(
    //    #version 330 core
    //    out vec4 FragColor;

    //    in vec2 TexCoords;
    //    
    //    uniform mat4 lightSpaceMatrix;
    //    uniform sampler2D depthMap;
    //    uniform float near_plane;
    //    uniform float far_plane;

    //    void main()
    //    {             
    //        float depthValue = texture(depthMap, TexCoords).r;
    //
    //        // Transform the depth value back to view space
    //        vec4 clipSpacePosition = vec4(TexCoords * 2.0 - 1.0, depthValue * 2.0 - 1.0, 1.0);
    //        vec4 viewSpacePosition = inverse(lightSpaceMatrix) * clipSpacePosition;
    //
    //        // Perspective divide
    //        viewSpacePosition /= viewSpacePosition.w;
    //  
    //        // Calculate linear depth
    //        float linearDepth = (-viewSpacePosition.z - near_plane) / (far_plane - near_plane);
    //
    //        FragColor = vec4(vec3(linearDepth), 1.0);
    //     }
    //)";

    const char* fragmentShaderSource = R"(
        #version 330 core
        out vec4 FragColor;
  
        in vec2 TexCoords;

        uniform sampler2D depthMap;

        void main()
        {             
            float depthValue = texture(depthMap, TexCoords).r;
            FragColor = vec4(vec3(depthValue), 1.0);
        }  
    )";

    // Compile and link shaders
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    debugDepthShader = glCreateProgram();
    glAttachShader(debugDepthShader, vertexShader);
    glAttachShader(debugDepthShader, fragmentShader);
    glLinkProgram(debugDepthShader);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
	return debugDepthShader;
}

void renderDepthMapToQuad(GLuint depthMap, const glm::mat4& lightSpaceMatrix, float near_plane, float far_plane)
{
    glUseProgram(debugDepthShader);
    // Create an orthographic projection matrix for the quad
    glm::mat4 orthoProjection = glm::ortho(-1.0f, 1.0f, -1.0f, 1.0f);
    //lightProjection =  glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f, near_plane, far_plane);

    // Pass the light space matrix directly to the shader
	glUniformMatrix4fv(glGetUniformLocation(debugDepthShader, "projection"), 1, GL_FALSE, glm::value_ptr(orthoProjection));
    glUniformMatrix4fv(glGetUniformLocation(debugDepthShader, "lightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(lightSpaceMatrix));
    glUniform1i(glGetUniformLocation(debugDepthShader, "depthMap"), 0);
    glUniform1f(glGetUniformLocation(debugDepthShader, "near_plane"), near_plane);
    glUniform1f(glGetUniformLocation(debugDepthShader, "far_plane"), far_plane);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, depthMap);

    renderQuad();
}