#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat4 lightSpaceMatrix;

out vec4 FragPosLightSpace; // Position of the fragment in light space
out vec3 FragPos;           // Position of the fragment in world space

void main()
{
    // Compute world-space fragment position
    FragPos = vec3(model * vec4(aPos, 1.0));

    // Transform fragment position to light space
    FragPosLightSpace = lightSpaceMatrix * vec4(FragPos, 1.0);

    // Compute the screen-space position of the fragment
    gl_Position = projection * view * vec4(FragPos, 1.0);
}
