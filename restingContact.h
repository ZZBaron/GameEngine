// restingContact.h
#pragma once
#include "GameEngine.h"
#include "collisionTypes.h"
#include "shape.h"
#include <Eigen/Dense>

class RestingContactSolver {
public:
    // External force that affects all bodies (e.g., gravity)
    static glm::vec3 externalForce;  // Default to (0, -9.81, 0) for gravity

    static bool isRestingContact(const ContactManifold& manifold) {
        const float VELOCITY_THRESHOLD = 0.1f;

        for (const auto& contact : manifold.contacts) {
            // Get relative velocity at contact point
            glm::vec3 relVel = getContactPointVelocity(contact, *manifold.bodyA, *manifold.bodyB);
            float normalVel = glm::dot(relVel, contact.normal);

            // Project external force onto contact normal
            float externalForceAlong = glm::dot(externalForce, contact.normal);

            // Adjust threshold based on external force
            float adjustedThreshold = VELOCITY_THRESHOLD + std::abs(externalForceAlong) * 0.01f;

            if (std::abs(normalVel) > adjustedThreshold) {
                return false;
            }
        }
        return true;
    }

    static void resolveRestingContacts(const ContactManifold& manifold, float dt) {
        if (manifold.contacts.empty()) return;
        if (manifold.bodyA->clamped && manifold.bodyB->clamped) return;

        RigidBody& bodyA = *const_cast<RigidBody*>(manifold.bodyA.get());
        RigidBody& bodyB = *const_cast<RigidBody*>(manifold.bodyB.get());

        // Resolve penetration for all contact points
        for (const auto& contact : manifold.contacts) {
            resolveOverlap(bodyA, bodyB, contact);
        }

        // Resolve velocities and apply external force compensation
        for (const auto& contact : manifold.contacts) {
            resolveContact(bodyA, bodyB, contact, dt);
        }
    }

private:
    static glm::vec3 getContactPointVelocity(const ContactPoint& contact,
        const RigidBody& bodyA, const RigidBody& bodyB) {
        glm::vec3 rA = contact.position - bodyA.COM;
        glm::vec3 rB = contact.position - bodyB.COM;

        glm::vec3 velA = bodyA.velocity + glm::cross(bodyA.angularVelocity, rA);
        glm::vec3 velB = bodyB.velocity + glm::cross(bodyB.angularVelocity, rB);

        return velB - velA;
    }

    static void resolveOverlap(RigidBody& bodyA, RigidBody& bodyB, const ContactPoint& contact) {
        const float CORRECTION_PERCENTAGE = 0.8f;
        const float SLOP = 0.001f;

        if (contact.penetration <= SLOP) return;

        float totalInvMass = 0.0f;
        if (!bodyA.clamped) totalInvMass += 1.0f / bodyA.totalMass;
        if (!bodyB.clamped) totalInvMass += 1.0f / bodyB.totalMass;

        if (totalInvMass <= 0.0f) return;

        float separationMag = (contact.penetration - SLOP) * CORRECTION_PERCENTAGE;
        glm::vec3 separation = contact.normal * separationMag;

        if (!bodyA.clamped) {
            float moveA = (1.0f / bodyA.totalMass) / totalInvMass;
            bodyA.translate(-separation * moveA);
        }

        if (!bodyB.clamped) {
            float moveB = (1.0f / bodyB.totalMass) / totalInvMass;
            bodyB.translate(separation * moveB);
        }
    }

    static void resolveContact(RigidBody& bodyA, RigidBody& bodyB,
        const ContactPoint& contact, float dt) {

        glm::vec3 relVel = getContactPointVelocity(contact, bodyA, bodyB);
        float normalVel = glm::dot(relVel, contact.normal);

        // If objects are separating, just apply force compensation
        if (normalVel > 0) {
            applyExternalForceCompensation(bodyA, bodyB, contact, dt);
            return;
        }

        // Calculate impulse
        float restitution = 0.0f;
        glm::vec3 rA = contact.position - bodyA.COM;
        glm::vec3 rB = contact.position - bodyB.COM;

        float invMassA = bodyA.clamped ? 0.0f : 1.0f / bodyA.totalMass;
        float invMassB = bodyB.clamped ? 0.0f : 1.0f / bodyB.totalMass;

        glm::vec3 angularA = glm::cross(
            bodyA.invInertiaTensor * glm::cross(rA, contact.normal), rA);
        glm::vec3 angularB = glm::cross(
            bodyB.invInertiaTensor * glm::cross(rB, contact.normal), rB);

        float invDenom = invMassA + invMassB +
            glm::dot(angularA + angularB, contact.normal);

        float j = -(1.0f + restitution) * normalVel / invDenom;
        glm::vec3 impulse = contact.normal * j;

        // Apply impulse
        if (!bodyA.clamped) bodyA.applyImpulse(-impulse, contact.position);
        if (!bodyB.clamped) bodyB.applyImpulse(impulse, contact.position);

        // Apply external force compensation
        applyExternalForceCompensation(bodyA, bodyB, contact, dt);
    }

    static void applyExternalForceCompensation(RigidBody& bodyA, RigidBody& bodyB,
        const ContactPoint& contact, float dt) {

        // Project external force onto contact normal
        float forceMagnitude = glm::dot(externalForce, contact.normal);

        // Only compensate if force is pushing bodies together
        if (forceMagnitude >= 0) return;

        // Calculate compensation forces
        if (!bodyA.clamped) {
            float normalForceA = -forceMagnitude * bodyA.totalMass;
            glm::vec3 compensationA = contact.normal * normalForceA * dt;
            bodyA.velocity += compensationA / bodyA.totalMass;
        }

        if (!bodyB.clamped) {
            float normalForceB = -forceMagnitude * bodyB.totalMass;
            glm::vec3 compensationB = contact.normal * normalForceB * dt;
            bodyB.velocity += compensationB / bodyB.totalMass;
        }
    }
};

// Initialize the external force (gravity by default)
glm::vec3 RestingContactSolver::externalForce(0.0f, -9.81f, 0.0f);