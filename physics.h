// physics.h
#pragma once
#include "GameEngine.h"
#include "rigidBody.h"
#include "simulation.h"

// Constants
const float FIXED_TIME_STEP = 1.0f / 60.0f; // 60 Hz fixed time step
const int MAX_SUBSTEPS = 10;
const float MAX_DELTA_TIME = 0.1f;

class PhysicsWorld {
public:
    std::vector<std::shared_ptr<RigidBody>> bodies;
    SimulationData simulationData;
    float simulationPlaybackTime = 0.0f;
    float simulationPlaybackSpeed = 1.0f;
    bool simulationMode = false;
    bool play = false;

	PhysicsWorld() {
		// Initialize physics world
	}

    void updateSimulation(float deltaTime) {
        if (simulationMode) {
            updateSimulationPlayback(deltaTime);
        }
        else {
            updatePhysicsRealtime(deltaTime);
        }
    }

private:

    void updateSimulationPlayback(float deltaTime) {
        if (simulationData.timePoints.empty() || simulationPlaybackTime >= simulationData.timePoints.back()) {
            regenerateSimulationData();
        }

        if (play) {
            simulationPlaybackTime += deltaTime * simulationPlaybackSpeed;
            auto state = simulationData.interpolateState(simulationPlaybackTime);
            updateBodiesFromState(state);
        }
    }

    void updatePhysicsRealtime(float deltaTime) {
        if (!play) return;

        float timeLeft = std::min(deltaTime, MAX_DELTA_TIME);
        int numSubsteps = 0;

        while (timeLeft > FIXED_TIME_STEP && numSubsteps < MAX_SUBSTEPS) {
            integrateAndResolve(FIXED_TIME_STEP);
            timeLeft -= FIXED_TIME_STEP;
            numSubsteps++;
        }

        if (timeLeft > 0.0f) {
            integrateAndResolve(timeLeft);
        }
    }

    
    void integrateAndResolve(float dt) {
        // Broad phase collision detection
        auto pairs = broadPhaseCollisionDetection();

        // Integrate velocities and update positions
        for (auto& body : bodies) {
            if (!body->clamped) {
                body->integrateVelocities(dt);
                body->updatePosition(dt);
            }
        }

        // Narrow phase collision detection and resolution
        for (const auto& pair : pairs) {
            if (!pair.first->collisionEnabled || !pair.second->collisionEnabled) {
                continue;
            }
            

            // Generate contact manifold
            auto manifold = CollisionSystem::generateManifold(*pair.first, *pair.second);

            // Resolve collision if detected
            if (manifold.isColliding) {
                CollisionSystem::resolveCollision(manifold, dt);

                // Wake up bodies if they were sleeping
                pair.first->wake();
                pair.second->wake();
            }

        }

        // Update derived data (e.g., transform matrices)
        for (auto& body : bodies) {
            body->updateDerivedData();
        }
    }
    

    // edit this to include AABB of rigid bodies
    std::vector<std::pair<RigidBody*, RigidBody*>> broadPhaseCollisionDetection() {
        // Implement a space partitioning method like Octree or AABB Tree for better performance
        std::vector<std::pair<RigidBody*, RigidBody*>> potentialPairs;
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                if (bodies[i]->collisionEnabled && bodies[j]->collisionEnabled) {
                    potentialPairs.emplace_back(bodies[i].get(), bodies[j].get());
                }
            }
        }
        return potentialPairs;
    }

    void regenerateSimulationData() {
        float t_start = 0.0f;
        float t_end = 5.0f;
        float simDeltaTime = FIXED_TIME_STEP;
        
        // Create a vector of RigidBody objects
        std::vector<RigidBody> bodyObjects;
        for (const auto& bodyPtr : bodies) {
            bodyObjects.push_back(*bodyPtr);
        }


        simulationData = generateSimulationData(bodyObjects, t_start, t_end, simDeltaTime);
        simulationPlaybackTime = t_start;
    }

    void updateBodiesFromState(const SimulationData::StateData& state) {
        for (size_t i = 0; i < bodies.size(); ++i) {
            if (!bodies[i]->clamped) {
                bodies[i]->updateFromState(state.positions[i], state.velocities[i],
                    state.orientations[i], state.angularVelocities[i]);
            }
        }
    }
};