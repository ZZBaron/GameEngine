#pragma once
#include "GameEngine.h"
#include <string>
#include <vector>
#include <map>

struct Material {
    glm::vec3 ambient;
    glm::vec3 diffuse;
    glm::vec3 specular;
    float shininess;
    float alpha;

    Material() :
        ambient(0.2f),
        diffuse(0.8f),
        specular(0.5f),
        shininess(32.0f),
        alpha(1.0f) {}
};

class Texture {
public:
    unsigned int id;
    std::string type;
    int width;
    int height;

    Texture() {
        glGenTextures(1, &id);
    }

    bool loadFromRawData(const std::vector<unsigned char>& data, int width, int height, bool hasAlpha = false) {
        this->width = width;
        this->height = height;

        glBindTexture(GL_TEXTURE_2D, id);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        GLenum format = hasAlpha ? GL_RGBA : GL_RGB;
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data.data());
        glGenerateMipmap(GL_TEXTURE_2D);

        return true;
    }

    void bind(unsigned int slot = 0) const {
        glActiveTexture(GL_TEXTURE0 + slot);
        glBindTexture(GL_TEXTURE_2D, id);
    }

    static void unbind() {
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    ~Texture() {
        glDeleteTextures(1, &id);
    }
};