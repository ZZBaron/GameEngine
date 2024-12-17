// AABB.h
#pragma once
#include "shape.h"
#include <algorithm>

class AABB : public RectPrism {
public:
    glm::vec3 min;
    glm::vec3 max;

    // Default constructor
    AABB() : RectPrism(glm::vec3(0), glm::vec3(1)), min(glm::vec3(0)), max(glm::vec3(1)) {
        transparency = 0.2f; // Make it semi-transparent for visualization
        color = glm::vec3(0.0f, 1.0f, 0.0f); // Green for visibility
    }

    // Construct AABB from min/max points
    AABB(const glm::vec3& min, const glm::vec3& max)
        : RectPrism(min, max), min(min), max(max) {
        transparency = 0.2f;
        color = glm::vec3(0.0f, 1.0f, 0.0f);
    }

    // Construct AABB from shape's vertices
    AABB(const Shape& shape) : RectPrism(
        computeMinPoint(shape),
        computeMaxPoint(shape)
    ) {
        min = computeMinPoint(shape);
        max = computeMaxPoint(shape);
        transparency = 0.2f;
        color = glm::vec3(0.0f, 1.0f, 0.0f);
    }

    // Update AABB from shape's transformed vertices
    void updateFromShape(const Shape& shape) {
        // Compute new min and max points
        min = computeMinPoint(shape);
        max = computeMaxPoint(shape);

        // Update the RectPrism base class
        // This recreates the entire box with new dimensions
        *this = AABB(min, max);

        // Restore visual properties
        transparency = 0.2f;
        color = glm::vec3(0.0f, 1.0f, 0.0f);
    }

    // Check if a point is inside the AABB
    bool contains(const glm::vec3& point) const {
        return point.x >= min.x && point.x <= max.x &&
            point.y >= min.y && point.y <= max.y &&
            point.z >= min.z && point.z <= max.z;
    }

    // Check if this AABB intersects with another AABB
    bool intersects(const AABB& other) const {
        return !(other.min.x > max.x ||
            other.max.x < min.x ||
            other.min.y > max.y ||
            other.max.y < min.y ||
            other.min.z > max.z ||
            other.max.z < min.z);
    }

    // Get the dimensions of the AABB
    glm::vec3 getDimensions() const {
        return max - min;
    }

    // Get the center of the AABB
    glm::vec3 getCenter() const {
        return (min + max) * 0.5f;
    }

private:
    // Helper function to compute minimum point from shape's vertices
    static glm::vec3 computeMinPoint(const Shape& shape) {
        glm::vec3 minPoint(std::numeric_limits<float>::max());

        for (const auto& vertex : shape.vertices) {
            // Transform vertex to world space
            glm::vec4 transformedVertex = shape.model * glm::vec4(vertex, 1.0f);
            glm::vec3 worldPos = glm::vec3(transformedVertex);

            minPoint.x = std::min(minPoint.x, worldPos.x);
            minPoint.y = std::min(minPoint.y, worldPos.y);
            minPoint.z = std::min(minPoint.z, worldPos.z);
        }

        return minPoint;
    }

    // Helper function to compute maximum point from shape's vertices
    static glm::vec3 computeMaxPoint(const Shape& shape) {
        glm::vec3 maxPoint(std::numeric_limits<float>::lowest());

        for (const auto& vertex : shape.vertices) {
            // Transform vertex to world space
            glm::vec4 transformedVertex = shape.model * glm::vec4(vertex, 1.0f);
            glm::vec3 worldPos = glm::vec3(transformedVertex);

            maxPoint.x = std::max(maxPoint.x, worldPos.x);
            maxPoint.y = std::max(maxPoint.y, worldPos.y);
            maxPoint.z = std::max(maxPoint.z, worldPos.z);
        }

        return maxPoint;
    }
};