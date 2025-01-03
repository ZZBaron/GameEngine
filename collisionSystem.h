// collisionSystem.h
#pragma once
#include "GameEngine.h"
#include "physics.h"
#include "shape.h"
#include <algorithm>
#include "misc_funcs.h"
#include "collisionTypes.h"
#include "convexCollision.h"
#include "specificCollision.h"
#include "AABB.h"
#include "restingContact.h"


class CollisionSystem {
public:

    static ContactManifold generateManifold(const std::shared_ptr<RigidBody>& bodyA, const std::shared_ptr<RigidBody>& bodyB) {
        ContactManifold manifold;
        manifold.bodyA = bodyA;
        manifold.bodyB = bodyB;
        manifold.restitution = std::min(bodyA->restitution, bodyB->restitution);
        manifold.isColliding = false;

        // First check AABB intersection
        if (!bodyA->checkBroadPhaseCollision(*bodyB)) {
            return manifold;
        }

        // If both shapes are convex and/or have bounding convex hulls, use GJK/EPA
        if (bodyA->shape->isConvex && bodyB->shape->isConvex) {
            std::cout << "Using GJK/EPA" << std::endl; // Add this line
            return ConvexCollisionDetector::generateManifold(
                bodyA,
                bodyB
            );
        }


        // Otherwise fall back to specific shape collision detection
        return SpecificCollisionDetector::generateManifold(bodyA, bodyB);
    }

    static void resolveCollision(const ContactManifold& manifold, float dt) {
        if (!manifold.isColliding || manifold.contacts.empty()) return;
        if (manifold.bodyA->clamped && manifold.bodyB->clamped) return;

        //debugCollision(*manifold.bodyA, *manifold.bodyB, manifold);

        if (false) { // check if RestingContactSolver::isRestingContact(manifold) is true
            RestingContactSolver::resolveRestingContacts(manifold, dt);
        }
        else {
            for (const auto& contact : manifold.contacts) {

                // Resolve collision impulses
                resolveContactPoint(*manifold.bodyA, *manifold.bodyB, contact, manifold.restitution, dt);
            }
        }

        //debugCollision(*manifold.bodyA, *manifold.bodyB, manifold);

        
    }

private:
    /*
    static std::vector<glm::vec3> generateBoxBoxContactPoints(const RectPrism_6Quads* boxA,
        const RectPrism_6Quads* boxB,
        const RigidBody& bodyA,
        const RigidBody& bodyB,
        const glm::vec3& normal) {
        std::vector<glm::vec3> contactPoints;
        float contactThreshold = 0.01f;

        // Get transformed vertices
        auto vertsA = getTransformedVertices(boxA);
        auto vertsB = getTransformedVertices(boxB);

        // Find vertices from box A that are in contact with box B
        for (const auto& vert : vertsA) {
            if (isPointNearFace(vert, boxB, normal, contactThreshold)) {
                contactPoints.push_back(vert);
            }
        }

        // And vice versa
        for (const auto& vert : vertsB) {
            if (isPointNearFace(vert, boxA, -normal, contactThreshold)) {
                contactPoints.push_back(vert);
            }
        }

        return contactPoints;
    }
    */

    static void resolveContactPoint(const RigidBody& bodyA, const RigidBody& bodyB,
        const ContactPoint& contact,
        float restitution, float dt) {
        // Calculate relative velocity at contact point
        glm::vec3 rA = contact.position - bodyA.COM;
        glm::vec3 rB = contact.position - bodyB.COM;

        glm::vec3 velA = bodyA.velocity + glm::cross(bodyA.angularVelocity, rA);
        glm::vec3 velB = bodyB.velocity + glm::cross(bodyB.angularVelocity, rB);

        glm::vec3 relativeVel = velA - velB;

        // glm::vec3 relativeVelNoRot = bodyA.velocity - bodyB.velocity;

        // Calculate impulse magnitude
        // watch this sign!! velAlongNormal is < 0 for my sphere-box
        float velAlongNormal = glm::dot(relativeVel, contact.normal);
        if (velAlongNormal > 0.0f) return;


        float totalInvMass = (bodyA.clamped ? 0.0f : 1.0f / bodyA.totalMass) +
            (bodyB.clamped ? 0.0f : 1.0f / bodyB.totalMass);

        float j = -(1.0f + restitution) * velAlongNormal;
        j /= totalInvMass +
            glm::dot(contact.normal,
                glm::cross(bodyA.invInertiaTensor * glm::cross(rA, contact.normal), rA) +
                glm::cross(bodyB.invInertiaTensor * glm::cross(rB, contact.normal), rB));

        // float jNoRot = (1.0f + restitution) * glm::dot(relativeVelNoRot, contact.normal) / totalInvMass;

        // Add positional correction using Baumgarte stabilization
        
        const float baumgarte = 0.15f;    // Moderate correction (not too weak, not too bouncy)
        const float slopb = 0.001f;       // Small but reasonable penetration allowance
        float bias = std::max(0.0f, contact.penetration - slopb) * (baumgarte / dt);
        

        // adjusted Baumgarte
        /*
        float omega = 4.0f*glm::pi<float>() / dt;  // Natural frequency, could instead be sqrt(k/m);  // where k is stiffness and m is mass
        float beta = 1.0f / (dt * omega * omega);
        float gamma = 1.0f / (dt * omega);
        const float slop = 0.001f;  // 1mm tolerance
        float bias = (beta / dt) * std::max(0.0f, contact.penetration - slop) +
            gamma * velAlongNormal;
        */


        // idea:
        // Base slop on the size of the objects involved
        // float characteristicLength = std::min(bodyA->shape->getBoundingRadius(),
        //     bodyB->shape->getBoundingRadius());
        // float slop = characteristicLength * 0.001f;  // 0.1% of object size

        // apply position correction instead of baumgarte
        

        // if using baumgarte
        j += bias * (totalInvMass);

        //The key principles are:
        //baumgarte: Lower values(like 0.01 - 0.02) give smoother but slower correction
        //slop : Higher values(like 0.01 - 0.03) allow more penetration but reduce jitter


        glm::vec3 impulse = contact.normal * j;


        // Apply impulse
        if (!bodyA.clamped) {
            const_cast<RigidBody&>(bodyA).applyImpulse(impulse, contact.position);
        }
        if (!bodyB.clamped) {
            const_cast<RigidBody&>(bodyB).applyImpulse(-impulse, contact.position);
        }
    }

    static void debugCollision(const RigidBody& bodyA, const RigidBody& bodyB, const ContactManifold& manifold) {
        std::cout << "\n=== Collision Debug Info ===\n";

        // Print basic body info
        std::cout << "Body A position: " << vec3_to_string(bodyA.COM) << "\n";
        std::cout << "Body B position: " << vec3_to_string(bodyB.COM) << "\n";

        if (manifold.isColliding) {
            std::cout << "Collision detected!\n";
            std::cout << "Number of contact points: " << manifold.contacts.size() << "\n";

            for (size_t i = 0; i < manifold.contacts.size(); ++i) {
                const auto& contact = manifold.contacts[i];
                std::cout << "Contact " << i << ":\n";
                std::cout << "  Position: " << vec3_to_string(contact.position) << "\n";
                std::cout << "  Normal: " << vec3_to_string(contact.normal) << "\n";
                std::cout << "  Penetration: " << contact.penetration << "\n";
            }
        }
        else {
            std::cout << "No collision detected\n";
        }

        // Print velocities
        std::cout << "Body A velocity: " << vec3_to_string(bodyA.velocity) << "\n";
        std::cout << "Body B velocity: " << vec3_to_string(bodyB.velocity) << "\n";

        std::cout << "Body A angular velocity: " << vec3_to_string(bodyA.angularVelocity) << "\n";
        std::cout << "Body B angular velocity: " << vec3_to_string(bodyB.angularVelocity) << "\n";

        std::cout << "=========================\n\n";
    }
};
