// blender.h
#pragma once

#include "GameEngine.h"
#include "shape.h"
#include <string>
#include <vector>
#include <memory>

// Still in early development
namespace BlenderImport {

    struct Camera {
        glm::vec3 position;
        glm::vec3 target;
        float fov;
    };

    struct Light {
        glm::vec3 position;
        glm::vec3 color;
        float intensity;
        std::string type; // e.g., "POINT", "SPOT", "SUN"
    };

    struct Mesh {
        std::vector<glm::vec3> vertices;
        std::vector<glm::vec3> normals;
        std::vector<unsigned int> indices;
    };

    class BlenderScene {
    public:
        BlenderScene();
        bool loadFromFile(const std::string& filename);

        const Camera& getCamera() const { return camera; }
        const std::vector<Light>& getLights() const { return lights; }
        std::vector<std::shared_ptr<Shape>> getShapes() const;

    private:
        Camera camera;
        std::vector<Light> lights;
        std::vector<Mesh> meshes;

        std::shared_ptr<Shape> convertMeshToShape(const Mesh& mesh);
    };

} // namespace BlenderImport