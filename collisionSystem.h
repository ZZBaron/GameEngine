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

    static ContactManifold generateManifold(const RigidBody& bodyA, const RigidBody& bodyB) {
        ContactManifold manifold;
        manifold.bodyA = &bodyA;
        manifold.bodyB = &bodyB;
        manifold.restitution = std::min(bodyA.restitution, bodyB.restitution);
        manifold.isColliding = false;

        // First check AABB intersection
        if (!bodyA.checkBroadPhaseCollision(bodyB)) {
            return manifold;
        }

        // If both shapes are convex and/or have bounding convex hulls, use GJK/EPA
        if (bodyA.shape->isConvex && bodyB.shape->isConvex) {
            return ConvexCollisionDetector::generateManifold(
                bodyA.shape.get(),
                bodyB.shape.get(),
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

        debugCollision(*manifold.bodyA, *manifold.bodyB, manifold);

        if (RestingContactSolver::isRestingContact(manifold)) {
            RestingContactSolver::resolveRestingContacts(manifold, dt);
        }
        else {
            for (const auto& contact : manifold.contacts) {

                // Resolve collision impulses
                resolveContactPoint(*manifold.bodyA, *manifold.bodyB, contact, manifold.restitution, dt);
            }
        }

        
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

        glm::vec3 relativeVel = velB - velA;

        // Calculate impulse magnitude
        // watch this sign!! velAlongNormal is < 0 for my sphere-box
        float velAlongNormal = glm::dot(relativeVel, contact.normal);
        if (velAlongNormal < 0) return;

        float totalInvMass = (bodyA.clamped ? 0.0f : 1.0f / bodyA.totalMass) +
            (bodyB.clamped ? 0.0f : 1.0f / bodyB.totalMass);

        float j = -(1.0f + restitution) * velAlongNormal;
        j /= totalInvMass +
            glm::dot(contact.normal,
                glm::cross(bodyA.invInertiaTensor * glm::cross(rA, contact.normal), rA) +
                glm::cross(bodyB.invInertiaTensor * glm::cross(rB, contact.normal), rB));

        float jNoRot = -(1.0f + restitution) * velAlongNormal / totalInvMass;


        std::cout << "totInvMass = " << totalInvMass << std::endl;
        std::cout << "j = " << j << std::endl;
        std::cout << "jNoRot = " << jNoRot << std::endl;

        glm::vec3 impulse = contact.normal * j;

        std::cout << "total impulse" << vec3_to_string(impulse) << std::endl;

        // Apply impulse
        if (!bodyA.clamped) {
            const_cast<RigidBody&>(bodyA).applyImpulse(-impulse, contact.position);
        }
        if (!bodyB.clamped) {
            const_cast<RigidBody&>(bodyB).applyImpulse(impulse, contact.position);
        }
    }

    static float calculateCollisionImpulse(const RigidBody& bodyA, const RigidBody& bodyB,
        const ContactPoint& contact, float restitution,
        const glm::vec3& rA, const glm::vec3& rB,
        float velAlongNormal) {
        // Calculate denominator for impulse
        float inverseMassSum = 0.0f;
        if (!bodyA.clamped) inverseMassSum += 1.0f / bodyA.totalMass;
        if (!bodyB.clamped) inverseMassSum += 1.0f / bodyB.totalMass;

        // Calculate angular components
        glm::vec3 angularA = glm::vec3(0.0f);
        glm::vec3 angularB = glm::vec3(0.0f);

        if (!bodyA.clamped) {
            angularA = glm::cross(bodyA.invInertiaTensor * glm::cross(rA, contact.normal), rA);
        }
        if (!bodyB.clamped) {
            angularB = glm::cross(bodyB.invInertiaTensor * glm::cross(rB, contact.normal), rB);
        }

        float angularEffect = glm::dot(angularA + angularB, contact.normal);

        // Calculate final impulse magnitude
        float j = -(1.0f + restitution) * velAlongNormal;
        j /= inverseMassSum + angularEffect;

        // Clamp maximum impulse to prevent instability
        const float maxImpulse = 50.0f * std::max(bodyA.totalMass, bodyB.totalMass);
        return glm::clamp(j, -maxImpulse, maxImpulse);
    }

    static void applyPositionalCorrection(RigidBody& bodyA, RigidBody& bodyB, const glm::vec3& normal, float depth) {
        // Baumgarte position correction parameters
        const float percent = 0.4f;     // Usually 20% to 80% (reduced from previous value)
        const float slop = 0.005f;      // Small penetration allowed
        const float maxCorrection = 0.2f; // Maximum correction per frame to prevent tunneling

        // Calculate correction magnitude
        float correction = glm::clamp(
            std::max(depth - slop, 0.0f) * percent,
            0.0f,
            maxCorrection
        );

        // Calculate inverse masses (0 for clamped objects)
        float invMassA = bodyA.clamped ? 0.0f : 1.0f / bodyA.totalMass;
        float invMassB = bodyB.clamped ? 0.0f : 1.0f / bodyB.totalMass;
        float totalInvMass = invMassA + invMassB;

        // Skip correction if both bodies are clamped
        if (totalInvMass <= 0.0f) return;

        // Calculate correction vectors for each body
        glm::vec3 correctionVector = (correction / totalInvMass) * normal;

        // Apply position corrections with mass weighting
        if (!bodyA.clamped) {
            bodyA.translate(-correctionVector * invMassA);

            // Add slight rotation correction for better stability
            float rotationCorrection = 0.01f;
            glm::vec3 rotationAxis = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), normal);
            if (glm::length(rotationAxis) > 0.001f) {
                bodyA.rotate(glm::angleAxis(rotationCorrection * correction, glm::normalize(rotationAxis)));
            }
        }

        if (!bodyB.clamped) {
            bodyB.translate(correctionVector * invMassB);

            // Add slight rotation correction for better stability
            float rotationCorrection = 0.01f;
            glm::vec3 rotationAxis = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), normal);
            if (glm::length(rotationAxis) > 0.001f) {
                bodyB.rotate(glm::angleAxis(-rotationCorrection * correction, glm::normalize(rotationAxis)));
            }
        }
    }

    static float calculateImpulse(RigidBody& bodyA, RigidBody& bodyB,
        const glm::vec3& rA, const glm::vec3& rB,
        const glm::vec3& normal, float velocityAlongNormal,
        float restitution) {
        // Calculate inverse masses
        float invMassA = bodyA.clamped ? 0.0f : 1.0f / bodyA.totalMass;
        float invMassB = bodyB.clamped ? 0.0f : 1.0f / bodyB.totalMass;

        // Calculate angular terms
        glm::vec3 angularTermA(0.0f);
        glm::vec3 angularTermB(0.0f);

        if (!bodyA.clamped) {
            glm::vec3 torquePerUnitImpulse = glm::cross(rA, normal);
            angularTermA = glm::cross(
                bodyA.invInertiaTensor * torquePerUnitImpulse,
                rA
            );
        }

        if (!bodyB.clamped) {
            glm::vec3 torquePerUnitImpulse = glm::cross(rB, normal);
            angularTermB = glm::cross(
                bodyB.invInertiaTensor * torquePerUnitImpulse,
                rB
            );
        }

        // Calculate denominator of impulse formula
        float impulseDenominator = invMassA + invMassB +
            glm::dot(angularTermA + angularTermB, normal);

        // Prevent division by zero
        if (impulseDenominator <= 0.0001f) {
            return 0.0f;
        }

        // Calculate final impulse magnitude
        float j = -(1.0f + restitution) * velocityAlongNormal / impulseDenominator;

        // Add velocity-dependent damping for stability
        const float dampingFactor = 0.1f;
        j *= (1.0f - dampingFactor * std::abs(velocityAlongNormal));

        // Clamp maximum impulse to prevent explosion
        const float maxImpulse = 100.0f * std::max(bodyA.totalMass, bodyB.totalMass);
        j = glm::clamp(j, -maxImpulse, maxImpulse);

        return j;
    }

    static void applyImpulse(RigidBody& bodyA, RigidBody& bodyB,
        const glm::vec3& rA, const glm::vec3& rB,
        const glm::vec3& normal, float impulse) {
        // Skip if impulse is too small
        if (std::abs(impulse) < 0.0001f) return;

        // Calculate impulse vector
        glm::vec3 impulseVector = impulse * normal;

        // Apply linear impulse
        if (!bodyA.clamped) {
            bodyA.velocity -= impulseVector / bodyA.totalMass;
        }

        if (!bodyB.clamped) {
            bodyB.velocity += impulseVector / bodyB.totalMass;
        }

        // Apply angular impulse with improved stability
        if (!bodyA.clamped) {
            glm::vec3 torqueA = glm::cross(rA, -impulseVector);
            // Update angular momentum only, angular velocity will be derived from it
            bodyA.angularMomentum += torqueA;
            bodyA.updateAngularVelocity(); // This calculates angular velocity from momentum
        }

        if (!bodyB.clamped) {
            glm::vec3 torqueB = glm::cross(rB, impulseVector);
            bodyB.angularMomentum += torqueB;
            bodyB.updateAngularVelocity();
        }
    }

    static float calculateRestitution(const RigidBody& bodyA, const RigidBody& bodyB, float relativeVelocity) {
        // Velocity-dependent restitution
        const float velocityThreshold = 1.0f;
        float baseRestitution = std::min(bodyA.restitution, bodyB.restitution);

        // Reduce restitution at low velocities to prevent bouncing artifacts
        if (std::abs(relativeVelocity) < velocityThreshold) {
            return baseRestitution * (std::abs(relativeVelocity) / velocityThreshold);
        }
        return baseRestitution;
    }

    static void handleRestingContact(RigidBody& bodyA, RigidBody& bodyB, const glm::vec3& contact,
        const glm::vec3& normal, float depth, float deltaTime) {
        // Enhanced resting contact parameters
        const float restingThreshold = 0.001f;
        const float stabilizationFactor = 0.3f;
        const float angularStabilizationFactor = 0.4f;

        // Calculate relative velocity at contact point
        glm::vec3 rA = contact - bodyA.shape->center;
        glm::vec3 rB = contact - bodyB.shape->center;
        glm::vec3 relativeVel = (bodyB.velocity + glm::cross(bodyB.angularVelocity, rB)) -
            (bodyA.velocity + glm::cross(bodyA.angularVelocity, rA));

        if (glm::length(relativeVel) < restingThreshold) {
            // Apply stronger stabilization for near-vertical orientations
            float upwardAlignment = glm::abs(glm::dot(normal, glm::vec3(0.0f, 1.0f, 0.0f)));
            float stabilizationStrength = stabilizationFactor * (1.0f + upwardAlignment);

            // Enhanced stabilization impulse
            glm::vec3 stabilizationImpulse = normal * depth * stabilizationStrength;

            if (!bodyA.clamped) {
                // Apply progressive angular damping based on orientation
                float angularDamping = angularStabilizationFactor * (1.0f + upwardAlignment);
                bodyA.angularMomentum *= (1.0f - angularDamping * deltaTime);
                bodyA.velocity -= stabilizationImpulse / bodyA.totalMass;

                // Add slight torque to prevent corner balancing
                if (upwardAlignment < 0.9f) { // If not mostly flat
                    glm::vec3 correctiveTorque = glm::cross(normal, glm::vec3(0.0f, 1.0f, 0.0f));
                    bodyA.angularMomentum += correctiveTorque * 0.1f;
                }
            }

            if (!bodyB.clamped) {
                bodyB.velocity += stabilizationImpulse / bodyB.totalMass;
                bodyB.angularMomentum *= (1.0f - angularStabilizationFactor * deltaTime);
            }
        }
    }

    static void applyFriction(RigidBody& bodyA, RigidBody& bodyB, const glm::vec3& rA, const glm::vec3& rB,
        const glm::vec3& normal, float normalImpulse, const glm::vec3& relativeVelocity,
        float deltaTime) {
        // Calculate tangent vector
        glm::vec3 tangent = relativeVelocity - (glm::dot(relativeVelocity, normal) * normal);
        float tangentLength = glm::length(tangent);

        if (tangentLength > 0.0001f) {
            tangent = tangent / tangentLength;

            // Enhanced friction coefficient based on relative motion
            float dynamicFriction = 0.3f;
            float staticFriction = 0.5f;
            float friction = (tangentLength < 0.1f) ? staticFriction : dynamicFriction;

            // Calculate and apply friction impulse
            float jt = -glm::dot(relativeVelocity, tangent);
            jt = glm::clamp(jt, -friction * normalImpulse, friction * normalImpulse);

            if (!bodyA.clamped) {
                bodyA.applyImpulse(-jt * tangent, rA);
            }
            if (!bodyB.clamped) {
                bodyB.applyImpulse(jt * tangent, rB);
            }
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
