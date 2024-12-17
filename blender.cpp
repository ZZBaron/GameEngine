// blender.cpp
#include "blender.h"
#include <fstream>
#include <sstream>
#include <iostream>

namespace BlenderImport {

    BlenderScene::BlenderScene() {}

    bool BlenderScene::loadFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return false;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string type;
            iss >> type;

            if (type == "CAMERA") {
                iss >> camera.position.x >> camera.position.y >> camera.position.z
                    >> camera.target.x >> camera.target.y >> camera.target.z
                    >> camera.fov;
            }
            else if (type == "LIGHT") {
                Light light;
                std::string lightType;
                iss >> light.position.x >> light.position.y >> light.position.z
                    >> light.color.r >> light.color.g >> light.color.b
                    >> light.intensity >> lightType;
                light.type = lightType;
                lights.push_back(light);
            }
            else if (type == "MESH") {
                Mesh mesh;
                int vertexCount, indexCount;
                iss >> vertexCount >> indexCount;

                for (int i = 0; i < vertexCount; ++i) {
                    glm::vec3 vertex, normal;
                    std::getline(file, line);
                    std::istringstream vertexLine(line);
                    vertexLine >> vertex.x >> vertex.y >> vertex.z
                        >> normal.x >> normal.y >> normal.z;
                    mesh.vertices.push_back(vertex);
                    mesh.normals.push_back(normal);
                }

                for (int i = 0; i < indexCount; ++i) {
                    unsigned int index;
                    std::getline(file, line);
                    std::istringstream indexLine(line);
                    indexLine >> index;
                    mesh.indices.push_back(index);
                }

                meshes.push_back(mesh);
            }
        }

        return true;
    }

    std::shared_ptr<Shape> BlenderScene::convertMeshToShape(const Mesh& mesh) {
        auto shape = std::make_shared<Shape>();
        shape->vertices = mesh.vertices;
        shape->normals = mesh.normals;
        shape->indices = mesh.indices;
        shape->centroid = glm::vec3(0.0f); // Calculate the actual centroid if needed
        shape->setup();
        return shape;
    }

	// error with convertMeshToShape
   /* std::vector<std::shared_ptr<Shape>> BlenderScene::getShapes() const {
        std::vector<std::shared_ptr<Shape>> shapes;
        for (const auto& mesh : meshes) {
            shapes.push_back(convertMeshToShape(mesh));
        }
        return shapes;
    }*/

} // namespace BlenderImport

// ... in your main function or setup code
//BlenderImport::BlenderScene scene;
//if (scene.loadFromFile("my_blender_scene.txt")) {
//    auto camera = scene.getCamera();
//    auto lights = scene.getLights();
//    auto shapes = scene.getShapes();
//
//    // Use the imported data to set up your scene
//    // ...
//}