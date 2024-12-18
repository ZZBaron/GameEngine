// specificCollision.h - Specific shape collision detection
#pragma once
#include "collisionTypes.h"

class SpecificCollisionDetector {
public:
    static ContactManifold generateManifold(const RigidBody& bodyA, const RigidBody& bodyB) {
        ContactManifold manifold;
        manifold.bodyA = &bodyA;
        manifold.bodyB = &bodyB;
        manifold.restitution = std::min(bodyA.restitution, bodyB.restitution);
        manifold.isColliding = false;

        // Determine shape types and call appropriate collision detection
        auto* sphereA = dynamic_cast<Sphere*>(bodyA.shape.get());
        auto* sphereB = dynamic_cast<Sphere*>(bodyB.shape.get());
        auto* boxA = dynamic_cast<RectPrism*>(bodyA.shape.get());
        auto* boxB = dynamic_cast<RectPrism*>(bodyB.shape.get());

        if (sphereA && sphereB) {
            manifold.isColliding = detectSphereSphere(sphereA, sphereB, bodyA, bodyB, manifold);
        }
        else if (sphereA && boxB) {
            manifold.isColliding = detectSphereBox(sphereA, boxB, bodyA, bodyB, manifold);
        }
        else if (boxA && sphereB) {
            manifold.isColliding = detectSphereBox(sphereB, boxA, bodyB, bodyA, manifold);
            // Flip normals since bodies were swapped
            for (auto& contact : manifold.contacts) {
                contact.normal = -contact.normal;
            }
        }
        else if (boxA && boxB) {
            manifold.isColliding = detectBoxBox(boxA, boxB, bodyA, bodyB, manifold);
        }

        if (manifold.isColliding) {
            //debugCollision(bodyA, bodyB, manifold);
        }

        return manifold;
    }

    static bool detectSphereSphere(const Sphere* sphereA, const Sphere* sphereB,
        const RigidBody& bodyA, const RigidBody& bodyB,
        ContactManifold& manifold) {

        glm::vec3 posA = sphereA->center;
        glm::vec3 posB = sphereB->center;

        glm::vec3 direction = posA - posB;
        float distance = glm::length(direction);
        float sumRadii = sphereA->radius + sphereB->radius;

        if (distance >= sumRadii) return false;

        // Generate single contact point
        ContactPoint contact;
        contact.normal = glm::normalize(direction); // Normal points from B to A
        contact.penetration = sumRadii - distance;
        // Contact point is at the surface of sphere B along the collision normal
        contact.position = posB + contact.normal * sphereB->radius;

        manifold.contacts.push_back(contact);
        return true;
    }

    static bool detectSphereBox(const Sphere* sphere, const RectPrism* box,
        const RigidBody& sphereBody, const RigidBody& boxBody,
        ContactManifold& manifold) {
        // Transform sphere center to box local space
        glm::mat4 boxWorldToLocal = glm::inverse(box->getModelMatrix());
        glm::vec3 sphereCenter = glm::vec3(boxWorldToLocal * glm::vec4(sphereBody.COM, 1.0f));

        // Find closest point on box to sphere center
        glm::vec3 closestPoint;
        const float halfWidth = box->sideLength_a * 0.5f;
        const float halfHeight = box->sideLength_b * 0.5f;
        const float halfDepth = box->sideLength_c * 0.5f;

        // Clamp sphere center to box bounds
        closestPoint.x = glm::clamp(sphereCenter.x, -halfWidth, halfWidth);
        closestPoint.y = glm::clamp(sphereCenter.y, -halfHeight, halfHeight);
        closestPoint.z = glm::clamp(sphereCenter.z, -halfDepth, halfDepth);

        // Transform points back to world space--------------------------------------
        glm::vec3 worldClosestPoint = glm::vec3(box->getModelMatrix() * glm::vec4(closestPoint, 1.0f));

        // Check for collision
        glm::vec3 direction = sphereBody.COM - worldClosestPoint;
        float distance = glm::length(direction);

        if (distance > sphere->radius) return false;

        // Generate contact point
        ContactPoint contact;
        contact.normal = glm::normalize(direction);
        contact.penetration = sphere->radius - distance;
        contact.position = worldClosestPoint;

        manifold.contacts.push_back(contact);
        return true;
    }

    static bool detectBoxBox(const RectPrism* boxA, const RectPrism* boxB,
        const RigidBody& bodyA, const RigidBody& bodyB,
        ContactManifold& manifold) {
        // Use SAT for initial collision detection
        glm::vec3 normal;
        float penetration;

        if (!checkSATCollision(boxA, boxB, bodyA, bodyB, normal, penetration)) {
            return false;
        }

        // Get transformed vertices
        auto vertsA = getTransformedVertices(boxA);
        auto vertsB = getTransformedVertices(boxB);

        // Ensure normal points from A to B
        glm::vec3 centerA = bodyA.COM;
        glm::vec3 centerB = bodyB.COM;
        if (glm::dot(normal, centerB - centerA) < 0) {
            normal = -normal;
        }

        // Find the most extreme points along the collision normal
        float maxA = -std::numeric_limits<float>::infinity();
        float minB = std::numeric_limits<float>::infinity();
        std::vector<glm::vec3> contactVerts;

        // Find deepest points of box B into box A
        for (const auto& v : vertsB) {
            float proj = glm::dot(v, normal);
            if (std::abs(proj - minB) < 0.01f) { // Within tolerance
                contactVerts.push_back(v);
            }
            else if (proj < minB) {
                minB = proj;
                contactVerts.clear();
                contactVerts.push_back(v);
            }
        }

        // If no contact points found, use closest point
        if (contactVerts.empty()) {
            // Use center point of overlap
            glm::vec3 contactPoint = centerA + (centerB - centerA) * 0.5f;
            ContactPoint contact;
            contact.position = contactPoint;
            contact.normal = normal;
            contact.penetration = penetration;
            manifold.contacts.push_back(contact);
        }
        else {
            // Add all found contact points
            for (const auto& point : contactVerts) {
                ContactPoint contact;
                contact.position = point;
                contact.normal = normal;
                contact.penetration = penetration;
                manifold.contacts.push_back(contact);
            }
        }

        return true;
    }

    static bool detectSpherePlane(const Sphere* sphere, const Plane* plane,
        const RigidBody& bodyA, const RigidBody& bodyB,
        ContactManifold& manifold) {

    }
private:

    static bool checkSATCollision(const RectPrism* boxA, const RectPrism* boxB,
        const RigidBody& bodyA, const RigidBody& bodyB,
        glm::vec3& collisionNormal, float& penetrationDepth) {
        penetrationDepth = std::numeric_limits<float>::infinity();

        // Get face normals from both boxes
        auto normalsA = boxA->getFaceNormals();
        auto normalsB = boxB->getFaceNormals();

        // Test box A's normals
        for (const auto& normal : normalsA) {
            float minA, maxA, minB, maxB;
            projectBox(boxA, normal, minA, maxA);
            projectBox(boxB, normal, minB, maxB);

            if (minA >= maxB || minB >= maxA) {
                return false; // Separation found
            }

            float overlap = std::min(maxA - minB, maxB - minA);
            if (overlap < penetrationDepth) {
                penetrationDepth = overlap;
                collisionNormal = normal;
            }
        }

        // Test box B's normals
        for (const auto& normal : normalsB) {
            float minA, maxA, minB, maxB;
            projectBox(boxA, normal, minA, maxA);
            projectBox(boxB, normal, minB, maxB);

            if (minA >= maxB || minB >= maxA) {
                return false; // Separation found
            }

            float overlap = std::min(maxA - minB, maxB - minA);
            if (overlap < penetrationDepth) {
                penetrationDepth = overlap;
                collisionNormal = normal;
            }
        }

        return true;
    }

    // Helper functions for box collision detection
    static void projectBox(const RectPrism* box, const glm::vec3& axis,
        float& min, float& max) {
        auto verts = getTransformedVertices(box);
        min = max = glm::dot(verts[0], axis);

        for (size_t i = 1; i < verts.size(); i++) {
            float projection = glm::dot(verts[i], axis);
            min = std::min(min, projection);
            max = std::max(max, projection);
        }
    }

    static std::vector<glm::vec3> getTransformedVertices(const RectPrism* box) {
        std::vector<glm::vec3> transformed;
        for (const auto& vertex : box->vertices) {
            transformed.push_back(glm::vec3(box->getModelMatrix() * glm::vec4(vertex, 1.0f)));
        }
        return transformed;
    }
    // ... (other specific collision implementations)
};