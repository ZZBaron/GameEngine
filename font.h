// font.h
#pragma once
#include <map>
#include <string>
#include <glm/glm.hpp>

// Character structure
struct Character {
    unsigned int TextureID; // ID handle of the glyph texture
    glm::ivec2 Size;        // Size of glyph
    glm::ivec2 Bearing;     // Offset from baseline to left/top of glyph
    unsigned int Advance;   // Offset to advance to next glyph
};

// Extern declarations
extern std::map<char, Character> Characters;
extern unsigned int VAO, VBO;

void initFreeType(const char* fontPath);
void renderText(unsigned int shaderProgram, std::string text, float x, float y, float scale, glm::vec3 color);
