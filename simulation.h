// simulation.h
#pragma once
#include "GameEngine.h"
#include "physics.h"
#include "collisionSystem.h"

class SimulationData {
public:
    struct StateData {
        std::vector<glm::vec3> positions;
        std::vector<glm::vec3> velocities;
        std::vector<glm::quat> orientations;
        std::vector<glm::vec3> angularVelocities;
        std::vector<glm::vec3> angularMomentums;  
        std::vector<glm::mat3> inertiaTensors;    
    };

    std::vector<StateData> states;
    std::vector<float> timePoints;

    void addState(const StateData& state, float time) {
        states.push_back(state);
        timePoints.push_back(time);
    }

    StateData interpolateState(float time) const {
        if (time <= timePoints.front()) return states.front();
        if (time >= timePoints.back()) return states.back();

        auto it = std::lower_bound(timePoints.begin(), timePoints.end(), time);
        size_t index = std::distance(timePoints.begin(), it);
        float t = (time - timePoints[index - 1]) / (timePoints[index] - timePoints[index - 1]);

        StateData result;
        for (size_t i = 0; i < states[index - 1].positions.size(); ++i) {
            result.positions.push_back(glm::mix(states[index - 1].positions[i], states[index].positions[i], t));
            result.velocities.push_back(glm::mix(states[index - 1].velocities[i], states[index].velocities[i], t));
            result.orientations.push_back(glm::slerp(states[index - 1].orientations[i], states[index].orientations[i], t));
            result.angularVelocities.push_back(glm::mix(states[index - 1].angularVelocities[i], states[index].angularVelocities[i], t));
            result.angularMomentums.push_back(glm::mix(states[index - 1].angularMomentums[i], states[index].angularMomentums[i], t));
            

            // Component-wise interpolation for matrices (THIS SUCKS)
            glm::mat3 mat1 = states[index - 1].inertiaTensors[i];
            glm::mat3 mat2 = states[index].inertiaTensors[i];
            glm::mat3 interpolatedMatrix;
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    interpolatedMatrix[row][col] = mat1[row][col] * (1.0f - t) + mat2[row][col] * t;
                }
            }
            result.inertiaTensors.push_back(interpolatedMatrix);
        }


        return result;
    }
};

class AdaptiveRK4Solver {
public:
    struct State {
        glm::vec3 position;
        glm::vec3 velocity;
        glm::quat orientation;
        glm::vec3 angularMomentum;
        glm::mat3 inertiaTensor;
        bool clamped;
        float mass;
    };

    struct Derivative {
        glm::vec3 dx;     // Change in position
        glm::vec3 dv;     // Change in velocity
        glm::quat dq;     // Change in orientation
        glm::vec3 dL;     // Change in angular momentum
    };

    static std::vector<State> integrate(
        const std::vector<State>& states,
        float t,
        float dt,
        const std::vector<std::shared_ptr<RigidBody>>& bodies,
        float& adaptedDt
    ) {
        const float errorTolerance = 1e-6f;
        const float minDt = 1e-6f;
        const float maxDt = 0.1f;

        // Compute two steps of size dt/2
        std::vector<State> halfStep = singleStep(states, t, dt / 2, bodies);
        halfStep = singleStep(halfStep, t + dt / 2, dt / 2, bodies);

        // Compute one step of size dt
        std::vector<State> fullStep = singleStep(states, t, dt, bodies);

        // Estimate error
        float error = 0.0f;
        for (size_t i = 0; i < states.size(); i++) {
            if (states[i].clamped) continue;
            error = std::max(error, glm::length(halfStep[i].position - fullStep[i].position));
            error = std::max(error, glm::length(halfStep[i].velocity - fullStep[i].velocity));
        }

        // Adjust timestep
        float newDt = dt * std::pow(errorTolerance / error, 0.2f);
        newDt = glm::clamp(newDt, minDt, maxDt);

        adaptedDt = newDt;

        // Return the more accurate result (half steps)
        return halfStep;
    }

private:
    static std::vector<State> singleStep(
        const std::vector<State>& states,
        float t,
        float dt,
        const std::vector<std::shared_ptr<RigidBody>>& bodies
    ) {
        std::vector<Derivative> k1 = evaluate(states, t, 0.0f, std::vector<Derivative>(states.size()), bodies);
        std::vector<Derivative> k2 = evaluate(states, t, dt * 0.5f, k1, bodies);
        std::vector<Derivative> k3 = evaluate(states, t, dt * 0.5f, k2, bodies);
        std::vector<Derivative> k4 = evaluate(states, t, dt, k3, bodies);

        std::vector<State> output(states.size());

        for (size_t i = 0; i < states.size(); i++) {
            if (states[i].clamped) {
                output[i] = states[i];
                continue;
            }

            glm::vec3 dxdt = (k1[i].dx + 2.0f * k2[i].dx + 2.0f * k3[i].dx + k4[i].dx) / 6.0f;
            glm::vec3 dvdt = (k1[i].dv + 2.0f * k2[i].dv + 2.0f * k3[i].dv + k4[i].dv) / 6.0f;
            glm::quat dqdt = (k1[i].dq + 2.0f * k2[i].dq + 2.0f * k3[i].dq + k4[i].dq) / 6.0f;
            glm::vec3 dLdt = (k1[i].dL + 2.0f * k2[i].dL + 2.0f * k3[i].dL + k4[i].dL) / 6.0f;

            output[i].position = states[i].position + dxdt * dt;
            output[i].velocity = states[i].velocity + dvdt * dt;
            output[i].orientation = glm::normalize(states[i].orientation + dqdt * dt);
            output[i].angularMomentum = states[i].angularMomentum + dLdt * dt;
            output[i].inertiaTensor = updateInertiaTensor(states[i].inertiaTensor, output[i].orientation);
            output[i].mass = states[i].mass;
            output[i].clamped = states[i].clamped;
        }

        return output;
    }

    static std::vector<Derivative> evaluate(
        const std::vector<State>& initial,
        float t,
        float dt,
        const std::vector<Derivative>& d,
        const std::vector<std::shared_ptr<RigidBody>>& bodies
    ) {
        std::vector<State> state(initial.size());
        for (size_t i = 0; i < initial.size(); i++) {
            if (initial[i].clamped) {
                state[i] = initial[i];
                continue;
            }

            state[i].position = initial[i].position + d[i].dx * dt;
            state[i].velocity = initial[i].velocity + d[i].dv * dt;
            state[i].orientation = glm::normalize(initial[i].orientation + d[i].dq * dt);
            state[i].angularMomentum = initial[i].angularMomentum + d[i].dL * dt;
            state[i].inertiaTensor = updateInertiaTensor(initial[i].inertiaTensor, state[i].orientation);
            state[i].mass = initial[i].mass;
            state[i].clamped = initial[i].clamped;
        }

        std::vector<Derivative> output(initial.size());

        // Calculate forces and torques considering all body interactions
        for (size_t i = 0; i < initial.size(); i++) {
            if (initial[i].clamped) {
                output[i] = Derivative{};
                continue;
            }

            glm::vec3 force = calculateTotalForce(state, i, t + dt, bodies);
            glm::vec3 torque = calculateTotalTorque(state, i, t + dt, bodies);

            output[i].dx = state[i].velocity;
            output[i].dv = force / state[i].mass;

            glm::vec3 angularVelocity = glm::inverse(state[i].inertiaTensor) * state[i].angularMomentum;
            output[i].dq = 0.5f * glm::quat(0, angularVelocity.x, angularVelocity.y, angularVelocity.z) * state[i].orientation;
            output[i].dL = torque;
        }

        return output;
    }

    static glm::vec3 calculateTotalForce(
        const std::vector<State>& states,
        size_t currentIndex,
        float t,
        const std::vector<std::shared_ptr<RigidBody>>& bodies
    ) {
        glm::vec3 force = glm::vec3(0.0f);
        const State& current = states[currentIndex];

        // Gravity
        if (gravityEnabled) {
            force += glm::vec3(0.0f, -9.81f, 0.0f) * current.mass;
        }

        // Air resistance (quadratic drag)
        float airDensity = 1.225f; // kg/m^3 at sea level
        float dragCoefficient = 0.47f; // Sphere drag coefficient
        float crossSectionalArea = glm::pi<float>() * 0.25f; // Assuming unit radius
        glm::vec3 dragForce = -0.5f * airDensity * dragCoefficient * crossSectionalArea *
            glm::length(current.velocity) * current.velocity;
        force += dragForce;

        return force;
    }

    static glm::vec3 calculateTotalTorque(
        const std::vector<State>& states,
        size_t currentIndex,
        float t,
        const std::vector<std::shared_ptr<RigidBody>>& bodies
    ) {
        glm::vec3 torque = glm::vec3(0.0f);
        const State& current = states[currentIndex];

        // Angular damping (air resistance for rotation)
        glm::vec3 angularVelocity = glm::inverse(current.inertiaTensor) * current.angularMomentum;
        float angularDragCoefficient = 0.1f;
        torque += -angularDragCoefficient * angularVelocity * glm::length(angularVelocity);

        return torque;
    }

    static glm::mat3 updateInertiaTensor(const glm::mat3& initialInertiaTensor, const glm::quat& orientation) {
        glm::mat3 rotationMatrix = glm::mat3_cast(orientation);
        return rotationMatrix * initialInertiaTensor * glm::transpose(rotationMatrix);
    }
};

void resolveCollisions(
    const std::vector<std::shared_ptr<RigidBody>> bodies,
    std::vector<AdaptiveRK4Solver::State>& states,
    float dt
) {

    // only update states not bodies
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            if (!bodies[i]->collisionEnabled || !bodies[j]->collisionEnabled) {
                continue;
            }

            // Create temporary rigid bodies to handle collision detection
            RigidBody tempBodyI = *bodies[i];
            RigidBody tempBodyJ = *bodies[j];

            // Update temp bodies with current state
            // make sure to use a method to change position of body because shape position also needs to be updated
            tempBodyI.updateCOM(states[i].position);
            tempBodyI.velocity = states[i].velocity;
            tempBodyI.shape->setOrientation(states[i].orientation);
            tempBodyI.angularMomentum = states[i].angularMomentum;
            tempBodyI.inertiaTensor = states[i].inertiaTensor;

            tempBodyJ.updateCOM(states[j].position);
            tempBodyJ.velocity = states[j].velocity;
            tempBodyJ.shape->setOrientation(states[j].orientation);
            tempBodyJ.angularMomentum = states[j].angularMomentum;
            tempBodyJ.inertiaTensor = states[j].inertiaTensor;



            auto manifold = CollisionSystem::generateManifold(
                std::make_shared<RigidBody>(tempBodyI),
                std::make_shared<RigidBody>(tempBodyJ)
            );

            if (manifold.isColliding) {
                CollisionSystem::resolveCollision(manifold, dt);

                // Update states from the resolved collision
                if (!states[i].clamped) {
                    states[i].position = manifold.bodyA->COM;
                    states[i].velocity = manifold.bodyA->velocity;
                    states[i].orientation = manifold.bodyA->shape->orientation;
                    states[i].inertiaTensor = manifold.bodyA->inertiaTensor;
                    states[i].angularMomentum = manifold.bodyA->angularMomentum;
                }

                if (!states[j].clamped) {
                    states[j].position = manifold.bodyB->COM;
                    states[j].velocity = manifold.bodyB->velocity;
                    states[j].orientation = manifold.bodyB->shape->orientation;
                    states[j].inertiaTensor = manifold.bodyB->inertiaTensor;
                    states[j].angularMomentum = manifold.bodyB->angularMomentum;
                }
            }
        }
    }
}

AdaptiveRK4Solver::State interpolatedStateToState(const SimulationData::StateData& interpolatedState, size_t bodyIndex, const std::shared_ptr<RigidBody>& body) {
    AdaptiveRK4Solver::State state;
    state.position = interpolatedState.positions[bodyIndex];
    state.velocity = interpolatedState.velocities[bodyIndex];
    state.orientation = interpolatedState.orientations[bodyIndex];
    state.inertiaTensor = body->inertiaTensor;
    state.angularMomentum = state.inertiaTensor * interpolatedState.angularVelocities[bodyIndex];
    state.mass = body->totalMass;
    state.clamped = body->clamped;
    return state;
}

SimulationData generateSimulationData(
    const std::vector<std::shared_ptr<RigidBody>>& bodies,
    float t_start,
    float t_end,
    float initialDeltaTime
) {
    SimulationData simData;
    std::cout << "Generating simulation data..." << std::endl;

    // Initialize states directly from the bodies - no copying needed
    std::vector<AdaptiveRK4Solver::State> states;
    for (const auto& body : bodies) {
        AdaptiveRK4Solver::State state;
        state.position = body->COM;
        state.velocity = body->velocity;
        state.orientation = body->shape->orientation;
        state.angularMomentum = body->angularMomentum;
        state.inertiaTensor = body->inertiaTensor;
        state.mass = body->totalMass;
        state.clamped = body->clamped;
        states.push_back(state);
    }

    float t = t_start;
    float dt = initialDeltaTime;

    while (t < t_end) {

        float adaptedDt;
        states = AdaptiveRK4Solver::integrate(states, t, dt, bodies, adaptedDt);
        dt = adaptedDt;

        // Collision detection and resolution
        if (collisionEnabled) {
            resolveCollisions(bodies, states, dt);
        }

        // Store state
        SimulationData::StateData stateData;
        for (const auto& state : states) {
            stateData.positions.push_back(state.position);
            stateData.velocities.push_back(state.velocity);
            stateData.orientations.push_back(state.orientation);
            stateData.angularVelocities.push_back(glm::inverse(state.inertiaTensor) * state.angularMomentum);
            stateData.angularMomentums.push_back(state.angularMomentum);  
            stateData.inertiaTensors.push_back(state.inertiaTensor);      
        }
        simData.addState(stateData, t);

        t += dt;
    }

    std::cout << "Simulation data generation complete. Final time: " << t << "s" << std::endl;
    return simData;
}




