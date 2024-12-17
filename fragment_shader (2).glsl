#version 330 core
out vec4 FragColor;

in vec3 FragPos;
in vec3 Normal;

uniform vec3 lightPos;
uniform vec3 lightColor;
uniform vec3 viewPos;
uniform vec3 objectColor;

// Sun's luminous flux (in watts)
uniform float sunLuminousFlux;

// Atmospheric density at sea level (kg/m^3)
uniform float atmosphericDensity;

// Rayleigh scattering coefficient
uniform vec3 rayleighScatteringCoeff;

// Earth's radius (in meters)
uniform float earthRadius;

// Function to calculate atmospheric depth
float calcAtmosphericDepth(vec3 pos, vec3 dir) {
    float a = dot(dir, dir);
    float b = 2.0 * dot(pos, dir);
    float c = dot(pos, pos) - (earthRadius + 100000.0) * (earthRadius + 100000.0);
    float discriminant = b*b - 4.0*a*c;
    return (-b + sqrt(discriminant)) / (2.0 * a);
}

void main() {
    // Normalized light direction
    vec3 lightDir = normalize(lightPos - FragPos);
    
    // Calculate distance to light (in meters)
    float distanceToLight = length(lightPos - FragPos);
    
    // Calculate illuminance using inverse square law
    float illuminance = sunLuminousFlux / (4.0 * 3.14159265 * distanceToLight * distanceToLight);
    
    // Basic diffuse lighting
    vec3 norm = normalize(Normal);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;
    
    // Calculate Rayleigh scattering
    float atmosphericDepth = calcAtmosphericDepth(FragPos, lightDir);
    vec3 rayleighScattering = rayleighScatteringCoeff * atmosphericDensity * atmosphericDepth;
    
    // Apply atmospheric scattering
    vec3 transmittance = exp(-rayleighScattering);
    vec3 scatteredLight = (1.0 - transmittance) * lightColor;
    
    // Combine all lighting components
    vec3 result = (diffuse * transmittance + scatteredLight) * illuminance * objectColor;
    
    // Apply a very simple tone mapping
    result = result / (result + vec3(1.0));
    
    FragColor = vec4(result, 1.0);
}