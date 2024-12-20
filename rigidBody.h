// rigidBody.h
#pragma once
#include "GameEngine.h"
#include "shape.h"
#include "AABB.h"
// #include "simulation.h" // cant do this
#include <numeric> // For std::accumulate

extern bool gravityEnabled;
extern bool collisionEnabled;

class Particle {
private:
    double mass;
    double charge;
    double spin;
    glm::vec3 position;
    glm::vec3 velocity;
    double time;

    Shape shape; // Particle uses Shape for position

public:
    Particle(double m, double q, double s, const glm::vec3& pos, const glm::vec3& vel, double t)
        : mass(m), charge(q), spin(s), velocity(vel), time(t), shape() {
        shape.vertices.push_back(pos); // The only vertex is the particle's position
    }

    double getMass() const { return mass; }
    double getCharge() const { return charge; }
    double getSpin() const { return spin; }
    glm::vec3 getPosition() const { return position; }
    glm::vec3 getVelocity() const { return velocity; }
    double getTime() const { return time; }

    void setMass(double m) { mass = m; }
    void setCharge(double q) { charge = q; }
    void setSpin(double s) { spin = s; }
    void setPosition(const glm::vec3& pos) { position = pos; }
    void setVelocity(const glm::vec3& vel) { velocity = vel; }
    void setTime(double t) { time = t; }

    void updatePosition(double deltaTime) {
        position += velocity * static_cast<float>(deltaTime);
    }

    void display() const {
        std::cout << "Mass: " << mass << "\n";
        std::cout << "Charge: " << charge << "\n";
        std::cout << "Spin: " << spin << "\n";
        std::cout << "Position: (" << position.x << ", " << position.y << ", " << position.z << ")\n";
        std::cout << "Velocity: (" << velocity.x << ", " << velocity.y << ", " << velocity.z << ")\n";
        std::cout << "Time: " << time << "\n";
    }

    // Getter for the shape (if needed)
    Shape& getShape() { return shape; }
};



// RigidBody class
class RigidBody {
public:
    glm::vec3 COM;               // Center of Mass
    glm::vec3 velocity;          // Linear velocity
    glm::vec3 angularVelocity;   // Angular velocity
    glm::vec3 angularMomentum;   // Angular momentum
    glm::mat3 inertiaTensor;     // Inertia tensor
    glm::mat3 invInertiaTensor;  // Inverse inertia tensor
    float totalMass;             // Total mass
	float restitution;		   // Coefficient of restitution
    std::shared_ptr<Shape> shape;  // Shape of the rigid body (can be Sphere, Cylinder, etc.)
    std::vector<Particle> particles; // List of particles (if using particles)
    bool clamped;                // If true, the object is static and unmovable
    bool collisionEnabled;       // If false, collision detection is disabled for this object
    bool isAsleep; // for collision response
    float sleepTimer; // for collision response
    std::unique_ptr<AABB> boundingBox;  // Add AABB member


    // Default constructor
    RigidBody()
        : totalMass(0.0), restitution(0.5f), clamped(false), collisionEnabled(true) {
    }
    
    // Constructor accepting a shape as an argument
    RigidBody(std::shared_ptr<Shape> shapeInstance, float mass, const glm::vec3& com, const glm::vec3& vel = glm::vec3(0.0f))
        : shape(shapeInstance), totalMass(mass), COM(com), velocity(vel), angularVelocity(0.0f), angularMomentum(0.0f), restitution(0.5f), clamped(false), collisionEnabled(true) {
		calculateInertiaTensor();
        wake();
        boundingBox = std::make_unique<AABB>(*shape);  // Initialize AABB
    }

    // Constructor accepting particles and calculating COM and mass from particles
    RigidBody(const std::vector<Particle>& particleList)
        : shape(nullptr), totalMass(0.0f), COM(glm::vec3(0.0f)), velocity(glm::vec3(0.0f)), angularMomentum(0.0f), particles(particleList), clamped(false), collisionEnabled(true) {
        calculateCOMAndMassFromParticles();
    }

    // Copy constructor
    RigidBody(const RigidBody& other)
        : COM(other.COM),
        velocity(other.velocity),
        angularVelocity(other.angularVelocity),
        angularMomentum(other.angularMomentum),
        inertiaTensor(other.inertiaTensor),
        invInertiaTensor(other.invInertiaTensor),
        totalMass(other.totalMass),
        restitution(other.restitution),
        particles(other.particles),
        clamped(other.clamped),
        collisionEnabled(other.collisionEnabled),
        isAsleep(other.isAsleep),
        sleepTimer(other.sleepTimer) {
        // Deep copy the shape
        if (other.shape) {
            if (auto sphere = dynamic_cast<Sphere*>(other.shape.get())) {
                shape = std::make_shared<Sphere>(*sphere);
            }
            else if (auto rectPrism = dynamic_cast<RectPrism*>(other.shape.get())) {
                shape = std::make_shared<RectPrism>(*rectPrism);
            }
            // Add other shape types as needed
        }

        // Deep copy the bounding box
        if (other.boundingBox) {
            boundingBox = std::make_unique<AABB>(*other.boundingBox);
        }
    }

    // Copy assignment operator
    RigidBody& operator=(RigidBody other) {
        if (this != &other) {
            COM = other.COM;
            velocity = other.velocity;
            angularVelocity = other.angularVelocity;
            angularMomentum = other.angularMomentum;
            inertiaTensor = other.inertiaTensor;
            invInertiaTensor = other.invInertiaTensor;
            totalMass = other.totalMass;
            restitution = other.restitution;
            particles = other.particles;
            clamped = other.clamped;
            collisionEnabled = other.collisionEnabled;
            isAsleep = other.isAsleep;
            sleepTimer = other.sleepTimer;

            // Deep copy the shape
            if (other.shape) {
                if (auto sphere = dynamic_cast<Sphere*>(other.shape.get())) {
                    shape = std::make_shared<Sphere>(*sphere);
                }
                else if (auto rectPrism = dynamic_cast<RectPrism*>(other.shape.get())) {
                    shape = std::make_shared<RectPrism>(*rectPrism);
                }
                // Add other shape types as needed
            }

            // Deep copy the bounding box
            if (other.boundingBox) {
                boundingBox = std::make_unique<AABB>(*other.boundingBox);
            }
        }
        return *this;
    }

    // Calculate inertia tensor based on shape
    void calculateInertiaTensor() {
        if (shape) {
            if (auto* sphere = dynamic_cast<Sphere*>(shape.get())) {
                float radius = sphere->radius;
                float I = 2.0f / 5.0f * totalMass * radius * radius;
                inertiaTensor = glm::mat3(
                    I, 0, 0,
                    0, I, 0,
                    0, 0, I
                );
            }
            else if (auto* rectPrism = dynamic_cast<RectPrism*>(shape.get())) {
                float a = rectPrism->sideLength_a;
                float b = rectPrism->sideLength_b;
                float c = rectPrism->sideLength_c;
                float Ix = 1.0f / 12.0f * totalMass * (b * b + c * c);
                float Iy = 1.0f / 12.0f * totalMass * (a * a + c * c);
                float Iz = 1.0f / 12.0f * totalMass * (a * a + b * b);
                inertiaTensor = glm::mat3(
                    Ix, 0, 0,
                    0, Iy, 0,
                    0, 0, Iz
                );
            }
            else {
                // Default to a unit inertia tensor for unknown shapes
                inertiaTensor = glm::mat3(1.0f);
            }

            // Apply rotation from the shape's model matrix
            glm::mat3 rotation = glm::mat3(shape->getModelMatrix());
            inertiaTensor = rotation * inertiaTensor * glm::transpose(rotation);
        }
        else {
            // Fallback to a unit inertia tensor if no shape is defined
            inertiaTensor = glm::mat3(1.0f);
        }

        invInertiaTensor = glm::inverse(inertiaTensor);
    }

    //void updateInertiaForRotation() {
    //    // Recalculate the entire inertia tensor
    //    calculateInertiaTensor();
    //}

    // Function to calculate the COM and total mass based on particles
    void calculateCOMAndMassFromParticles() {
        totalMass = 0.0f;
        glm::vec3 weightedSum(0.0f);

        for (const Particle& particle : particles) {
            totalMass += particle.getMass();
            weightedSum += particle.getPosition() * static_cast<float>(particle.getMass());
        }

        if (totalMass > 0.0f) {
            COM = weightedSum / totalMass;
        }
        else {
            COM = glm::vec3(0.0f);
        }
    }

    // Draw the shape
    void draw(GLuint shaderProgram, glm::mat4 view, glm::mat4 projection, const glm::mat4& lightSpaceMatrix, GLuint depthMap) {
        if (shape) {
            shape->Shape::draw(shaderProgram, view, projection, lightSpaceMatrix, depthMap);
        }
    }

	void drawShadow(GLuint shaderProgram, const glm::mat4& lightSpaceMatrix) {
		if (shape) {
			shape->drawShadow(shaderProgram, lightSpaceMatrix);
		}
	}

    // Update the AABB whenever the rigid body moves or rotates
    void updateBoundingBox() {
        if (boundingBox && shape) {
            boundingBox->updateFromShape(*shape);
        }
    }

    // Display RigidBody details
    void display() const {
        std::cout << "COM: (" << COM.x << ", " << COM.y << ", " << COM.z << ")\n";
        std::cout << "Total Mass: " << totalMass << "\n";
        if (!particles.empty()) {
            std::cout << "RigidBody with " << particles.size() << " particles:\n";
            for (const Particle& p : particles) {
                p.display();
            }
        }
    }
    
    // Function to print the vertices
    void printVertices() {
        if (shape != nullptr) {
            for (const auto& vertex : shape->vertices) {
                std::cout << "Vertex: ("
                    << vertex.x << ", "
                    << vertex.y << ", "
                    << vertex.z << ")\n";
            }
        }
        else {
            std::cout << "Shape is null!\n";
        }
    }

    
    // Project shape onto an axis
    void projectShape(const glm::vec3& axis, float& min, float& max) const {
        // Initialize min and max
        min = std::numeric_limits<float>::max();
        max = std::numeric_limits<float>::lowest();

        // Loop through each vertex in the shape
        for (const auto& vertex : shape->vertices) {
            // Transform the vertex using the model matrix
            glm::vec4 transformedVertex =  shape->model * glm::vec4(vertex, 1.0f); // Convert vec3 to vec4 for transformation

            // Project the transformed vertex onto the specified axis
            float projection = glm::dot(glm::vec3(transformedVertex), axis);

            // Update min and max based on the projection
            min = std::min(min, projection);
            max = std::max(max, projection);
        }
    }

    // Function to update COM and vertices based on new COM
    void updateCOM(const glm::vec3& newCOM) {
        glm::vec3 translation = newCOM - this->COM;  // Calculate the translation vector

        // Update the vertices of the shape based on the new COM
        translate(translation);
    }

    // Update position based on velocity and deltaTime
    void updateWithSim(float deltaTime) {
        if (!clamped && !isAsleep) {

            // Apply gravity
            if (gravityEnabled) {
				velocity += glm::vec3(0.0f, -9.8f, 0.0f) * deltaTime;
            }

            // Apply drag force
            const float dragCoefficient = 0.00f;
            glm::vec3 dragForce = -dragCoefficient * velocity;
            velocity += dragForce * deltaTime;

            // Apply a velocity threshold
            const float velocityThreshold = 0.01f; // maybe make size dependant?
            if (glm::length(velocity) < velocityThreshold) {
                velocity = glm::vec3(0.0f);
            }


			RigidBody::translate(velocity * deltaTime);

            // Update angular motion
            angularVelocity = invInertiaTensor * angularMomentum;
            glm::quat rotation = glm::quat(glm::vec3(angularVelocity * deltaTime));
            rotate(rotation);

            // Check for sleep condition
            const float sleepVelocityThreshold = 0.05f;
            const float sleepAngularVelocityThreshold = 0.05f;
            const float sleepTime = 1.0f; // Time in seconds before putting object to sleep

            if (glm::length(velocity) < sleepVelocityThreshold &&
                glm::length(angularVelocity) < sleepAngularVelocityThreshold) {
                sleepTimer += deltaTime;
                if (sleepTimer > sleepTime) {
                    isAsleep = true;
                }
            }
            else {
                sleepTimer = 0.0f;
            }

        }
    }

    void applyForce(const glm::vec3& force, const glm::vec3& applicationPoint, float deltaTime) {
        if (!clamped) {
            // Apply linear force
            glm::vec3 acceleration = force / totalMass;
            velocity += acceleration * deltaTime;

            // Calculate torque and apply angular force
            glm::vec3 torque = glm::cross(applicationPoint - shape->center, force); // may change to COM whenever i get that shit working
            glm::vec3 angularAcceleration = invInertiaTensor * torque;
            angularVelocity += angularAcceleration * deltaTime;

            // Update angular momentum
            angularMomentum = inertiaTensor * angularVelocity;
        }
    }

    // Apply impulse at a point
    void applyImpulse(const glm::vec3& impulse, const glm::vec3& contactPoint) {
        if (!clamped) {
            velocity += impulse / totalMass;
            glm::vec3 torque = glm::cross(contactPoint - shape->center, impulse);
            angularMomentum += torque;
            angularVelocity = invInertiaTensor * angularMomentum;
        }
    }

    void translate(const glm::vec3& translation) {
        this->COM += translation;
        if (shape) {
			shape->translate(translation);
            updateBoundingBox();  // Update AABB after translation
        }
    }

	// Rotate the rigid body using quaternion
    void rotate(const glm::quat& rotation) {
        // Update inertia tensor
        glm::mat3 rotationMat = glm::mat3_cast(rotation);
        inertiaTensor = rotationMat * inertiaTensor * glm::transpose(rotationMat);
        invInertiaTensor = glm::inverse(inertiaTensor);

        // Update shape's orientation
        if (shape) {
            shape->rotate(glm::axis(rotation), glm::angle(rotation));
            updateBoundingBox();  // Update AABB after translation
        }
    }

    void setOrientation(const glm::quat& orientation) {
        // change inertiaTensor appropriately
        glm::quat initOrientation = shape->orientation;
        glm::quat rotation = orientation * glm::inverse(initOrientation);

        glm::mat3 rotationMat = glm::mat3_cast(rotation);
        inertiaTensor = rotationMat * inertiaTensor * glm::transpose(rotationMat);
        invInertiaTensor = glm::inverse(inertiaTensor);

        if (shape) {
            shape->setOrientation(orientation);
            updateBoundingBox();  // Update AABB after translation
        }
    }

	void scale(const glm::vec3& scale) {
		if (shape) {
			shape->setScale(scale);
            updateBoundingBox();  // Update AABB after translation
		}
	}

    void wake() {
        isAsleep = false;
        sleepTimer = 0.0f;
    }

    // Method to check for potential collision with another rigid body using AABBs
    bool checkBroadPhaseCollision(const RigidBody& other) const {
        if (!collisionEnabled || !other.collisionEnabled) {
            return false;
        }
        return boundingBox && other.boundingBox &&
            boundingBox->intersects(*other.boundingBox);
    }

    glm::vec3 getCOM() const {
        return this->COM;
    }

    void updateAngularVelocity() {
        angularVelocity = glm::inverse(inertiaTensor) * angularMomentum;
    }


	//For Real Time Physics
    void integrateVelocities(float dt) {
        if (clamped) return;

        // Semi-implicit Euler integration
        velocity += calculateAcceleration() * dt;
        angularVelocity += invInertiaTensor * calculateTorque() * dt;
    }

    void updatePosition(float dt) {
        if (clamped) return;

        updateCOM(COM + velocity * dt);
        glm::quat spin(0, angularVelocity.x * dt, angularVelocity.y * dt, angularVelocity.z * dt);
        shape->setOrientation(glm::normalize(shape->orientation + 0.5f * spin * shape->orientation));
    }
	// not sure this is needed
    void updateDerivedData() {
        // Update shape's center and orientation
		// shape->center = COM; // but i might want to make the COM different than the center
        // shape->setOrientation(orientation);

        // Update inertia tensor in world space
        //glm::mat3 rotMat = glm::mat3_cast(orientation);
        //inertiaTensor = rotMat * inertiaTensorLocal * glm::transpose(rotMat);
        //invInertiaTensor = glm::inverse(inertiaTensor);

        //// Update model matrix
        //shape->updateModelMatrix();
    }

    // or this
    void updateFromState(const glm::vec3& position, const glm::vec3& vel,
        const glm::quat& orient, const glm::vec3& angVel) {
        updateCOM(position);
        velocity = vel;
        setOrientation(orient);
        angularVelocity = angVel;
		angularMomentum = inertiaTensor * angularVelocity;
    }
    

    // Virtual destructor to make the class polymorphic
    virtual ~RigidBody() = default;

private:
    glm::vec3 calculateAcceleration() const {
        glm::vec3 force = glm::vec3(0.0f, -9.8f, 0.0f) * totalMass; // Gravity
        // Add other forces (e.g., drag) here
        return force / totalMass;
    }

    glm::vec3 calculateTorque() const {
        glm::vec3 torque = glm::vec3(0.0f);
        // Add torque calculations here
        return torque;
    }
};

class RectPrismBounds : public RigidBody {
public:
    glm::vec3 center = COM;               // Center of Mass
    bool clamped = true;                // If true, the object is static and unmovable
    bool collisionEnabled;       // If false, collision detection is disabled for this object

    // Constructor accepting a shape as an argument
    RectPrismBounds(const glm::vec3 boundMin, const glm::vec3 boundMax)
        : clamped(true), collisionEnabled(true) {
		shape = std::make_shared<RectPrism>(boundMin, boundMax);
        shape->transparency = 0.01f;
		COM = (boundMin + boundMax) / 2.0f;
		center = COM;
    }
};

