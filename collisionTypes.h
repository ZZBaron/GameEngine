// collisionTypes.h - Base types used by collision system
#pragma once
#include "GameEngine.h"
#include "shape.h"
#include "rigidBody.h"

// Forward declarations
class RigidBody;
class Shape;

// Common collision structures used by collision detection systems
struct ContactPoint {
    glm::vec3 position;      // World space position of contact
    glm::vec3 normal;        // Contact normal pointing from first to second body
    float penetration;       // Penetration depth
};

struct ContactManifold {
    std::vector<ContactPoint> contacts;
    const RigidBody* bodyA;
    const RigidBody* bodyB;
    float restitution;
    bool isColliding;
};

// Add to collisionTypes.h
struct ContactCache {
    std::vector<ContactPoint> persistentContacts;
    float accumulatedNormalImpulse = 0.0f;
    float accumulatedTangentImpulse = 0.0f;
};