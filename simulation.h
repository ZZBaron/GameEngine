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
        }


        return result;
    }
};

class RK4Solver {
public:
    struct State {
        glm::vec3 position;
        glm::vec3 velocity;
        glm::quat orientation;
        glm::vec3 angularMomentum;
        glm::mat3 inertiaTensor;
        bool clamped;
    };

    struct Derivative {
        glm::vec3 dx; // Change in position
        glm::vec3 dv; // Change in velocity
        glm::quat dq; // Change in orientation
        glm::vec3 dL; // Change in angular momentum
    };

    static std::vector<State> integrate(const std::vector<State>& states, float t, float dt, const std::vector<RigidBody>& bodies) {
        std::vector<Derivative> a = evaluate(states, t, 0.0f, std::vector<Derivative>(states.size()), bodies);
        std::vector<Derivative> b = evaluate(states, t, dt * 0.5f, a, bodies);
        std::vector<Derivative> c = evaluate(states, t, dt * 0.5f, b, bodies);
        std::vector<Derivative> d = evaluate(states, t, dt, c, bodies);

        std::vector<State> output(states.size());

        for (size_t i = 0; i < states.size(); ++i) {
            if (states[i].clamped) {
                output[i] = states[i];
                continue;
            }

            glm::vec3 dxdt = 1.0f / 6.0f * (a[i].dx + 2.0f * (b[i].dx + c[i].dx) + d[i].dx);
            glm::vec3 dvdt = 1.0f / 6.0f * (a[i].dv + 2.0f * (b[i].dv + c[i].dv) + d[i].dv);
            glm::quat dqdt = 1.0f / 6.0f * (a[i].dq + 2.0f * (b[i].dq + c[i].dq) + d[i].dq);
            glm::vec3 dLdt = 1.0f / 6.0f * (a[i].dL + 2.0f * (b[i].dL + c[i].dL) + d[i].dL);

            output[i].position = states[i].position + dxdt * dt;
            output[i].velocity = states[i].velocity + dvdt * dt;
            output[i].orientation = glm::normalize(states[i].orientation + dqdt * dt);
            output[i].angularMomentum = states[i].angularMomentum + dLdt * dt;
            output[i].inertiaTensor = updateInertiaTensor(states[i].inertiaTensor, output[i].orientation);
            output[i].clamped = states[i].clamped;

        }

        return output;
    }

private:
    static std::vector<Derivative> evaluate(const std::vector<State>& initial, float t, float dt, const std::vector<Derivative>& d, const std::vector<RigidBody>& bodies) {
        std::vector<State> state(initial.size());
        for (size_t i = 0; i < initial.size(); ++i) {
            if (initial[i].clamped) {
                state[i] = initial[i];
                continue;
            }

            state[i].position = initial[i].position + d[i].dx * dt;
            state[i].velocity = initial[i].velocity + d[i].dv * dt;
            state[i].orientation = glm::normalize(initial[i].orientation + d[i].dq * dt);
            state[i].angularMomentum = initial[i].angularMomentum + d[i].dL * dt;
            state[i].inertiaTensor = updateInertiaTensor(initial[i].inertiaTensor, state[i].orientation);
            state[i].clamped = initial[i].clamped;

        }
        

        std::vector<Derivative> output(initial.size());
        for (size_t i = 0; i < initial.size(); ++i) {
            if (initial[i].clamped) {
                output[i] = Derivative{};
                continue;
            }
            output[i].dx = state[i].velocity;
            output[i].dv = acceleration(state[i], t + dt, bodies[i]);
            glm::vec3 angularVelocity = glm::inverse(state[i].inertiaTensor) * state[i].angularMomentum;
            output[i].dq = 0.5f * glm::quat(0, angularVelocity.x, angularVelocity.y, angularVelocity.z) * state[i].orientation;
            output[i].dL = torque(state[i], t + dt, bodies[i]);
        }

        return output;
    }

    static glm::vec3 acceleration(const State& state, float t, const RigidBody& body) {
        glm::vec3 force = glm::vec3(0.0f);

        if (gravityEnabled) {
            force += glm::vec3(0.0f, -9.8f, 0.0f) * body.totalMass;
        }

        // Add drag force
         const float dragCoefficient = 0.5f;
         glm::vec3 dragForce = -dragCoefficient * state.velocity * glm::length(state.velocity);
         force += dragForce;

        return force / body.totalMass;
    }

    static glm::quat differentiate(const glm::quat& q, const glm::vec3& w) {
        return glm::quat(0, w.x, w.y, w.z) * q * 0.5f;
    }

    static glm::vec3 torque(const State& state, float t, const RigidBody& body) {
        glm::vec3 torque = glm::vec3(0.0f);

        // Add rotational damping
        const float rotationalDampingCoefficient = 0.1f;
        glm::vec3 angularVelocity = glm::inverse(state.inertiaTensor) * state.angularMomentum;
        torque -= rotationalDampingCoefficient * angularVelocity;

        // Add other torque calculations here (e.g., from collisions or external forces)

        return torque;
    }

    static glm::mat3 updateInertiaTensor(const glm::mat3& initialInertiaTensor, const glm::quat& orientation) {
        glm::mat3 rotationMatrix = glm::mat3_cast(orientation);
        return rotationMatrix * initialInertiaTensor * glm::transpose(rotationMatrix);
    }
};

void updateRBFromState(RigidBody& body, const RK4Solver::State& state) {
    body.updateCOM(state.position);
    body.velocity = state.velocity;
    body.shape->setOrientation(state.orientation);
    body.angularMomentum = state.angularMomentum;
    body.inertiaTensor = state.inertiaTensor;
	body.invInertiaTensor = glm::inverse(state.inertiaTensor);
    body.angularVelocity = body.invInertiaTensor * body.angularMomentum;
    // body.updateAngularVelocity();
}



SimulationData generateSimulationData(const std::vector<RigidBody>& initialBodies, float t_start, float t_end, float deltaTime) {
    SimulationData simData;
    std::cout << "Generating simulation data..." << std::endl;

    std::vector<RK4Solver::State> states;
    std::vector<RigidBody> currentBodies;  // Create a copy to modify during simulation
    std::vector<std::shared_ptr<Shape>> currentShapes; // need to redfine each shape because they are pointers 

    for (const auto& body : initialBodies) {
        // stupid process for preserving the subclass of Shape (Sphere etc.)
        std::shared_ptr<Shape> newShape;
        if (auto sphere = std::dynamic_pointer_cast<Sphere>(body.shape)) {
            newShape = std::make_shared<Sphere>(*sphere);
        }
        else if (auto rectPrism = std::dynamic_pointer_cast<RectPrism>(body.shape)) {
            newShape = std::make_shared<RectPrism>(*rectPrism);
        }
        else {
            // Add more shape types as needed
            newShape = std::make_shared<Shape>(*body.shape);
        }
        currentShapes.push_back(newShape);

        RigidBody newBody(newShape, body.totalMass, body.COM, body.velocity);
        newBody.angularMomentum = body.angularMomentum;
		newBody.inertiaTensor = body.inertiaTensor;
		newBody.invInertiaTensor = body.invInertiaTensor;
		newBody.angularVelocity = body.angularVelocity; // maybe keep this?
		// i dont think i need to do the shape rotation stuff here
        newBody.clamped = body.clamped;
        newBody.collisionEnabled = body.collisionEnabled;
        newBody.restitution = body.restitution;

        currentBodies.push_back(newBody);

        RK4Solver::State state;
        state.position = newBody.COM;
        state.velocity = newBody.velocity;
        state.orientation = newBody.shape->orientation;
        state.angularMomentum = newBody.angularMomentum;
        state.inertiaTensor = newBody.inertiaTensor;
        state.clamped = newBody.clamped;
        states.push_back(state);
    }

    for (float t = t_start; t <= t_end; t += deltaTime) {
        states = RK4Solver::integrate(states, t, deltaTime, currentBodies);


        // Update bodies with new states
        for (size_t i = 0; i < currentBodies.size(); ++i) {
            if (!currentBodies[i].clamped) {
                updateRBFromState(currentBodies[i], states[i]);
            }
        }


        // Perform collision detection and resolution
        if (collisionEnabled) {
            for (size_t i = 0; i < currentBodies.size(); ++i) {
                for (size_t j = i + 1; j < currentBodies.size(); ++j) {
                    if (!currentBodies[i].collisionEnabled || !currentBodies[j].collisionEnabled) {
                        continue;
                    }

                    // Generate contact manifold
                    auto manifold = CollisionSystem::generateManifold(currentBodies[i], currentBodies[j]);

                    // Resolve collision if detected
                    if (manifold.isColliding) {
                        CollisionSystem::resolveCollision(manifold, deltaTime);

                        // Wake up bodies if they were sleeping
                        //currentBodies[i].wake();
                        //currentBodies[j].wake();

                        // Handle resting contacts
                        /*
                        for (const auto& contact : manifold.contacts) {
                            CollisionSystem::handleRestingContact(
                                currentBodies[i],
                                currentBodies[j],
                                contact.position,
                                contact.normal,
                                contact.penetration,
                                deltaTime
                            );
                        }

                        // Apply stability corrections
                        CollisionSystem::applyStabilityCorrections(
                            currentBodies[i],
                            currentBodies[j],
                            manifold.contacts[0].normal,
                            deltaTime
                        );
                        */

                        // Update states after collision resolution
                        if (!currentBodies[i].clamped) {
                            states[i].position = currentBodies[i].COM;
                            states[i].velocity = currentBodies[i].velocity;
                            states[i].orientation = currentBodies[i].shape->orientation;
                            states[i].inertiaTensor = currentBodies[i].inertiaTensor;
                            states[i].angularMomentum = currentBodies[i].angularMomentum;
                        }

                        if (!currentBodies[j].clamped) {
                            states[j].position = currentBodies[j].COM;
                            states[j].velocity = currentBodies[j].velocity;
                            states[j].orientation = currentBodies[j].shape->orientation;
                            states[j].inertiaTensor = currentBodies[j].inertiaTensor;
                            states[j].angularMomentum = currentBodies[j].angularMomentum;
                        }
                    }
                }
            }
        }


        SimulationData::StateData stateData;
        for (const auto& state : states) {
            stateData.positions.push_back(state.position);
            stateData.velocities.push_back(state.velocity);
            stateData.orientations.push_back(state.orientation);
            stateData.angularVelocities.push_back(glm::inverse(state.inertiaTensor) * state.angularMomentum);
        }
        simData.addState(stateData, t);
    }

    std::cout << "Simulation data generation complete" << std::endl;

    return simData;
}




