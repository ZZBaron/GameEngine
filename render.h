#include "GameEngine.h"
#include <fstream> // for cerr i think

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct vec3d
{
    float x, y, z;
};

GLuint loadShader(const char* path, GLenum shaderType) {
    std::ifstream shaderFile(path);
    if (!shaderFile.is_open()) {
        std::cerr << "Error opening file: " << path << std::endl;
        return 0;
    }
    std::stringstream shaderStream;
    shaderStream << shaderFile.rdbuf();
    std::string shaderCode = shaderStream.str();
    const char* shaderCodeCStr = shaderCode.c_str();

    GLuint shader = glCreateShader(shaderType);
    glShaderSource(shader, 1, &shaderCodeCStr, NULL);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, NULL, infoLog);
        std::cerr << "Error compiling shader (" << (shaderType == GL_VERTEX_SHADER ? "vertex" : "fragment") << "): " << infoLog << std::endl;
        glDeleteShader(shader);  // Clean up if shader fails to compile
        return 0;
    }

    return shader;
}

GLuint createShaderProgram(const char* vertexPath, const char* fragmentPath) {
    GLuint vertexShader = loadShader(vertexPath, GL_VERTEX_SHADER);
    GLuint fragmentShader = loadShader(fragmentPath, GL_FRAGMENT_SHADER);

    if (vertexShader == 0 || fragmentShader == 0) {
        std::cerr << "Error: Could not create shader program due to shader compilation issues." << std::endl;
        return 0;
    }

    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    GLint success;
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        std::cerr << "Shader program linking error: " << infoLog << std::endl;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return shaderProgram;
}



void drawAxes(const glm::mat4& view, const glm::mat4& projection) {
    // Set line width for better visibility
    glLineWidth(3.0f);

    // // Disable the shader program to use fixed-function pipeline
    glUseProgram(0);

    /// Set the view and projection matrices
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(glm::value_ptr(projection));
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(view));

    // Draw X axis in red
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f); // Red color
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);
    glEnd();

    // Draw Y axis in green
    glBegin(GL_LINES);
    glColor3f(0.0f, 1.0f, 0.0f); // Green color
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);
    glEnd();

    // Draw Z axis in blue
    glBegin(GL_LINES);
    glColor3f(0.0f, 0.0f, 1.0f); // Blue color
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.0f);
    glEnd();

    // Reset line width and color
    glLineWidth(1.0f);
    glColor3f(1.0f, 1.0f, 1.0f); // Default color

}

void drawPoint(const glm::vec3& point, const glm::mat4& view, const glm::mat4& projection, const glm::vec3& color = glm::vec3(1.0f, 0.0f, 0.0f)) {
    // Enable point smoothing
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

    // Set point size for better visibility
	glPointSize(100.0f);
	// Set point color
	glColor3f(color.r, color.g, color.b);
	// // Disable the shader program to use fixed-function pipeline
	glUseProgram(0);
	/// Set the view and projection matrices
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(glm::value_ptr(projection));
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(glm::value_ptr(view));
	// Draw the point
	glBegin(GL_POINTS);
	glVertex3f(point.x, point.y, point.z);
	glEnd();
	// Reset point size
	glPointSize(1.0f);

    // Disable point smoothing
    glDisable(GL_POINT_SMOOTH);
}