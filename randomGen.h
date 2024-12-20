#pragma once
#include "GameEngine.h"
#include "shape.h"
#include "rigidBody.h"

// Function to generate a random float between min and max
float randomFloat(float min, float max) {
    static std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<float> distribution(min, max);
    return distribution(generator);
}

// Function to generate a random color
glm::vec3 randomColor() {
    return glm::vec3(randomFloat(0.0f, 1.0f), randomFloat(0.0f, 1.0f), randomFloat(0.0f, 1.0f));
}

void generateRandomSpheres(std::vector<std::shared_ptr<Shape>>& shapes,
    std::vector<std::shared_ptr<RigidBody>>& bodies,
    const glm::vec3& boxMin,
    const glm::vec3& boxMax,
    float radius,
    int numSlices,
    int numStacks,
    int count,
    float mass = 1.0f) {
    for (int i = 0; i < count; ++i) {
        // Generate random position within the bounding box
        glm::vec3 position(
            randomFloat(boxMin.x + radius, boxMax.x - radius),
            randomFloat(boxMin.y + radius, boxMax.y - radius),
            randomFloat(boxMin.z + radius, boxMax.z - radius)
        );

        // Create sphere
        auto sphere = std::make_shared<Sphere>(position, radius, numSlices, numStacks);

        // Set random color
        sphere->color = randomColor();

        // Create RigidBody for the sphere
        auto sphereRB = std::make_shared<RigidBody>(sphere, mass, position);
        sphereRB->restitution = 0.5; // how bouncy

        // Add to vectors
        shapes.push_back(sphere);
        bodies.push_back(sphereRB);
    }
}

void generateRandomBoxes(std::vector<std::shared_ptr<Shape>>& shapes,
    std::vector<std::shared_ptr<RigidBody>>& bodies,
    const glm::vec3& boundBoxMin,
    const glm::vec3& boundBoxMax,
    const float sideLength_a,
    const float sideLength_b,
    const float sideLength_c,
    int count,
    float mass = 1.0f) {
    RectPrism boundingBox(boundBoxMin,boundBoxMax);

    for (int i = 0; i < count; ++i) {
        // Generate random position within the bounding box
        glm::vec3 position(
            randomFloat(boundBoxMin.x + sideLength_a, boundBoxMax.x - sideLength_a),
            randomFloat(boundBoxMin.y + sideLength_b, boundBoxMax.y - sideLength_b),
            randomFloat(boundBoxMin.z + sideLength_c, boundBoxMax.z - sideLength_c)
        );

        // Create rectprism
        auto rectPrism = std::make_shared<RectPrism>(position, sideLength_a, sideLength_b, sideLength_c);

        // Set random color
        rectPrism->color = randomColor();

        // Create RigidBody for the sphere
        auto rectPrismRB = std::make_shared <RigidBody>(rectPrism, mass, position);
        rectPrismRB->restitution = 0.5; // how bouncy

        // Add to vectors
        shapes.push_back(rectPrism);
        bodies.push_back(rectPrismRB);
    }
}