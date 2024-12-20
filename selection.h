// selection.h
#pragma once
#include "GameEngine.h"
#include "camera.h"
#include "rigidBody.h"
#include <limits>

// Forward declarations
extern std::vector<std::shared_ptr<RigidBody>> bodies;
extern std::shared_ptr<RigidBody> selectedRB;

// Ray structure for intersection testing
struct Ray {
    glm::vec3 origin;
    glm::vec3 direction;

    Ray(const glm::vec3& o, const glm::vec3& d) : origin(o), direction(glm::normalize(d)) {}
};

class SelectionSystem {
private:
    static SelectionSystem* instance;

public:
    static SelectionSystem& getInstance();

    // Declare functions but don't define them here
    static Ray screenToWorldRay(double mouseX, double mouseY, int screenWidth, int screenHeight,
        const glm::mat4& projection, const glm::mat4& view);

    static bool raySphereIntersect(const Ray& ray, const Sphere* sphere, float& t);
    static bool rayBoxIntersect(const Ray& ray, const RectPrism* box, float& t);
    static std::shared_ptr<RigidBody> findIntersectedBody(const Ray& ray,
        const std::vector<std::shared_ptr<RigidBody>>& bodies);

private:
    SelectionSystem() {} // Private constructor for singleton
};