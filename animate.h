// animate.h
#include "GameEngine.h"
#include "physics.h"

class Animator {
public:
    std::shared_ptr<Shape> shape;

    // Constructor accepting a shape as an argument
    Animator(std::shared_ptr<Shape> shapeInstance)
        : shape(shapeInstance) {}

    // Constructor accepting rigidBody instead
    Animator(std::shared_ptr<RigidBody> RBInstance)
        : shape(std::shared_ptr()) {}


    void animate(Shape& shape, std::function<void(Shape&, float)> animationFunction, float deltaTime) {
        animationFunction(shape, deltaTime);
    }
};

// Example animation functions
void rotateShape(Shape& shape, float deltaTime) {
    float angle = glm::radians(90.0f) * deltaTime; // Rotate 90 degrees per second
    shape.model = glm::rotate(shape.model, angle, glm::vec3(0.0f, 1.0f, 0.0f));
}

void scaleShape(Shape& shape, float deltaTime) {
    float scaleFactor = 1.0f + 0.5f * deltaTime; // Scale up by 50% per second
    shape.model = glm::scale(shape.model, glm::vec3(scaleFactor, scaleFactor, scaleFactor));
}

void translateShape(Shape& shape, float deltaTime) {
    glm::vec3 translation(1.0f * deltaTime, 0.0f, 0.0f); // Move right 1 unit per second
    shape.model = glm::translate(shape.model, translation);
}

float f_test(float t) {
	return t;
}
