#version 330 core
out vec4 FragColor;

in vec4 FragPosLightSpace;
in vec3 FragPos;

uniform sampler2D shadowMap;  // Depth map
uniform vec3 lightPos;        // Light source position in world space
uniform vec3 viewPos;         // Camera position

// Material and lighting
uniform vec3 lightColor;
uniform vec3 objectColor;

// Function to calculate shadow factor
float ShadowCalculation(vec4 fragPosLightSpace)
{
    // Perform perspective divide to get normalized device coordinates
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    projCoords = projCoords * 0.5 + 0.5; // Map to [0, 1] range

    // Get the depth value from the shadow map
    float closestDepth = texture(shadowMap, projCoords.xy).r;  
    float currentDepth = projCoords.z;

    // Bias to avoid shadow acne (tweak this if needed)
    float bias = 0.005;
    float shadow = (currentDepth - bias > closestDepth) ? 1.0 : 0.0;

    return shadow;
}

void main()
{
    // Calculate shadow factor
    float shadow = ShadowCalculation(FragPosLightSpace);

    // Diffuse lighting (basic Lambertian shading)
    vec3 normal = normalize(FragPos);  // In a real setup, pass actual normals
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(normal, lightDir), 0.0);

    // Combine lighting and shadows
    vec3 lighting = (1.0 - shadow) * lightColor * diff;
    vec3 color = lighting * objectColor;

    FragColor = vec4(color, 1.0);
}
