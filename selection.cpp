// selection.cpp
#include "selection.h"

SelectionSystem* SelectionSystem::instance = nullptr;

SelectionSystem& SelectionSystem::getInstance() {
    if (!instance) {
        instance = new SelectionSystem();
    }
    return *instance;
}

Ray SelectionSystem::screenToWorldRay(double mouseX, double mouseY, int screenWidth, int screenHeight,
    const glm::mat4& projection, const glm::mat4& view) {
    // Convert to normalized device coordinates
    float x = (2.0f * mouseX) / screenWidth - 1.0f;
    float y = 1.0f - (2.0f * mouseY) / screenHeight;

    // Create ray in clip space
    glm::vec4 rayClip = glm::vec4(x, y, -1.0f, 1.0f);

    // Convert to eye space
    glm::vec4 rayEye = glm::inverse(projection) * rayClip;
    rayEye = glm::vec4(rayEye.x, rayEye.y, -1.0f, 0.0f);

    // Convert to world space
    glm::vec4 rayWorld = glm::inverse(view) * rayEye;
    glm::vec3 rayDirection = glm::normalize(glm::vec3(rayWorld));

    return Ray(cameraPos, rayDirection);
}

bool SelectionSystem::raySphereIntersect(const Ray& ray, const Sphere* sphere, float& t) {
    glm::vec3 sphereCenter = sphere->center;
    float radius = sphere->radius;

    glm::vec3 oc = ray.origin - sphereCenter;
    float a = glm::dot(ray.direction, ray.direction);
    float b = 2.0f * glm::dot(oc, ray.direction);
    float c = glm::dot(oc, oc) - radius * radius;
    float discriminant = b * b - 4 * a * c;

    if (discriminant < 0) return false;

    t = (-b - sqrt(discriminant)) / (2.0f * a);
    return t >= 0;
}

bool SelectionSystem::rayBoxIntersect(const Ray& ray, const RectPrism* box, float& t) {
    glm::mat4 invModel = glm::inverse(box->getModelMatrix());
    glm::vec3 rayOriginLocal = glm::vec3(invModel * glm::vec4(ray.origin, 1.0f));
    glm::vec3 rayDirLocal = glm::normalize(glm::vec3(invModel * glm::vec4(ray.direction, 0.0f)));

    glm::vec3 boxMin = -glm::vec3(box->sideLength_a, box->sideLength_b, box->sideLength_c) * 0.5f;
    glm::vec3 boxMax = glm::vec3(box->sideLength_a, box->sideLength_b, box->sideLength_c) * 0.5f;

    float tMin = (boxMin.x - rayOriginLocal.x) / rayDirLocal.x;
    float tMax = (boxMax.x - rayOriginLocal.x) / rayDirLocal.x;
    if (tMin > tMax) std::swap(tMin, tMax);

    float tyMin = (boxMin.y - rayOriginLocal.y) / rayDirLocal.y;
    float tyMax = (boxMax.y - rayOriginLocal.y) / rayDirLocal.y;
    if (tyMin > tyMax) std::swap(tyMin, tyMax);

    if ((tMin > tyMax) || (tyMin > tMax)) return false;
    tMin = fmax(tMin, tyMin);
    tMax = fmin(tMax, tyMax);

    float tzMin = (boxMin.z - rayOriginLocal.z) / rayDirLocal.z;
    float tzMax = (boxMax.z - rayOriginLocal.z) / rayDirLocal.z;
    if (tzMin > tzMax) std::swap(tzMin, tzMax);

    if ((tMin > tzMax) || (tzMin > tMax)) return false;

    t = tMin;
    return t >= 0;
}

std::shared_ptr<RigidBody> SelectionSystem::findIntersectedBody(const Ray& ray, const std::vector<std::shared_ptr<RigidBody>>& bodies) {
    float closestT = std::numeric_limits<float>::max();
    std::shared_ptr<RigidBody> closestBody = nullptr;

    for (const auto& body : bodies) {
        float t;
        bool hit = false;

        if (auto* sphere = dynamic_cast<Sphere*>(body->shape.get())) {
            hit = raySphereIntersect(ray, sphere, t);
        }
        else if (auto* box = dynamic_cast<RectPrism*>(body->shape.get())) {
            hit = rayBoxIntersect(ray, box, t);
        }

        if (hit && t < closestT) {
            closestT = t;
            closestBody = body;
        }
    }

    return closestBody;
}