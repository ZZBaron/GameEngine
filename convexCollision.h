#pragma once
#include "GameEngine.h"
#include "shape.h"
#include "collisionTypes.h"

class ConvexCollisionDetector {
public:
    struct Simplex {
        std::vector<glm::vec3> points;

        void push_front(const glm::vec3& point) {
            points.insert(points.begin(), point);
        }

        glm::vec3& operator[](size_t i) { return points[i]; }
        const glm::vec3& operator[](size_t i) const { return points[i]; }
        size_t size() const { return points.size(); }
        void clear() { points.clear(); }
    };

    struct EPAFace {
        glm::vec3 normal;
        float distance;
        std::array<size_t, 3> vertices;

        EPAFace(size_t a, size_t b, size_t c, const std::vector<glm::vec3>& polytope) {
            vertices = { a, b, c };
            normal = glm::normalize(glm::cross(
                polytope[b] - polytope[a],
                polytope[c] - polytope[a]
            ));
            distance = glm::dot(normal, polytope[a]);
        }
    };

    static ContactManifold generateManifold(const std::shared_ptr<RigidBody>& bodyA, const std::shared_ptr<RigidBody>& bodyB) {

        ContactManifold manifold;
        manifold.bodyA = bodyA;
        manifold.bodyB = bodyB;
        manifold.restitution = std::min(bodyA->restitution, bodyB->restitution);
        manifold.isColliding = false;

        auto shapeA = bodyA->shape.get();
        auto shapeB = bodyB->shape.get();

        // First check intersection using GJK
        Simplex simplex;
        if (!gjkIntersection(shapeA, shapeB, simplex)) {
            return manifold;
        }

        // Use EPA to find penetration depth and contact normal
        auto [normal, penetration] = runEPA(shapeA, shapeB, simplex);

        // Generate contact points
        auto contacts = generateContactPoints(shapeA, shapeB, normal, penetration);

        // Create contact manifold
        manifold.isColliding = true;
        manifold.contacts = contacts;

        return manifold;
    }

private:
    static glm::vec3 support(const Shape* shapeA, const Shape* shapeB, const glm::vec3& direction) {
        return shapeA->getSupport(direction) - shapeB->getSupport(-direction);
    }

    static bool nextSimplex(Simplex& simplex, glm::vec3& direction) {
        switch (simplex.size()) {
        case 2: return line(simplex, direction);
        case 3: return triangle(simplex, direction);
        case 4: return tetrahedron(simplex, direction);
        }
        return false;
    }

    static std::pair<glm::vec3, float> runEPA(
        const Shape* shapeA, const Shape* shapeB, const Simplex& gjkSimplex) {

        std::vector<glm::vec3> polytope(gjkSimplex.points.begin(), gjkSimplex.points.end());
        std::vector<EPAFace> faces;

        // Create initial faces
        faces.emplace_back(0, 1, 2, polytope);
        faces.emplace_back(0, 2, 3, polytope);
        faces.emplace_back(0, 3, 1, polytope);
        faces.emplace_back(1, 3, 2, polytope);

        const float EPSILON = 0.0001f;
        const int MAX_ITERATIONS = 32;
        int iterations = 0;

        while (iterations++ < MAX_ITERATIONS) {
            // Find face closest to origin
            auto closestFace = std::min_element(faces.begin(), faces.end(),
                [](const EPAFace& a, const EPAFace& b) {
                    return std::abs(a.distance) < std::abs(b.distance);
                });

            if (closestFace == faces.end()) {
                return { glm::normalize(polytope[0]), glm::length(polytope[0]) };
            }

            // Get new support point
            glm::vec3 supportPoint = support(shapeA, shapeB, closestFace->normal);
            float distance = glm::dot(supportPoint, closestFace->normal);

            if (std::abs(distance - closestFace->distance) < EPSILON) {
                return { closestFace->normal, closestFace->distance };
            }

            polytope.push_back(supportPoint);
            expandPolytope(faces, polytope, polytope.size() - 1);

            if (faces.empty()) {
                return { glm::normalize(polytope[0]), glm::length(polytope[0]) };
            }
        }

        auto closestFace = std::min_element(faces.begin(), faces.end(),
            [](const EPAFace& a, const EPAFace& b) {
                return std::abs(a.distance) < std::abs(b.distance);
            });

        return { closestFace->normal, closestFace->distance };
    }

    static void expandPolytope(
        std::vector<EPAFace>& faces,
        const std::vector<glm::vec3>& polytope,
        size_t newVertex) {

        std::vector<std::pair<size_t, size_t>> uniqueEdges;

        for (size_t i = 0; i < faces.size();) {
            const auto& face = faces[i];
            if (glm::dot(polytope[newVertex] - polytope[face.vertices[0]], face.normal) > 0) {
                uniqueEdges.push_back({ face.vertices[0], face.vertices[1] });
                uniqueEdges.push_back({ face.vertices[1], face.vertices[2] });
                uniqueEdges.push_back({ face.vertices[2], face.vertices[0] });

                faces[i] = faces.back();
                faces.pop_back();
            }
            else {
                ++i;
            }
        }

        // Process edges
        for (auto& edge : uniqueEdges) {
            if (edge.first > edge.second) {
                std::swap(edge.first, edge.second);
            }
        }
        std::sort(uniqueEdges.begin(), uniqueEdges.end());
        auto newEnd = std::unique(uniqueEdges.begin(), uniqueEdges.end());
        uniqueEdges.erase(newEnd, uniqueEdges.end());

        // Create new faces
        for (const auto& edge : uniqueEdges) {
            faces.emplace_back(edge.first, edge.second, newVertex, polytope);
        }
    }

    static bool gjkIntersection(const Shape* shapeA, const Shape* shapeB, Simplex& simplex) {
        glm::vec3 direction(1.0f, 0.0f, 0.0f);

        simplex.push_front(support(shapeA, shapeB, direction));
        direction = -simplex[0];

        while (true) {
            simplex.push_front(support(shapeA, shapeB, direction));

            if (glm::dot(simplex[0], direction) < 0) {
                return false;
            }

            if (nextSimplex(simplex, direction)) {
                return true;
            }
        }
    }

    static bool line(Simplex& simplex, glm::vec3& direction) {
        const glm::vec3& a = simplex[0];
        const glm::vec3& b = simplex[1];

        glm::vec3 ab = b - a;
        glm::vec3 ao = -a;

        direction = glm::cross(glm::cross(ab, ao), ab);
        return false;
    }

    static bool triangle(Simplex& simplex, glm::vec3& direction) {
        const glm::vec3& a = simplex[0];
        const glm::vec3& b = simplex[1];
        const glm::vec3& c = simplex[2];

        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;
        glm::vec3 ao = -a;
        glm::vec3 abc = glm::cross(ab, ac);

        if (glm::dot(glm::cross(abc, ac), ao) > 0) {
            simplex.points.erase(simplex.points.begin() + 1);
            direction = glm::cross(glm::cross(ac, ao), ac);
        }
        else if (glm::dot(glm::cross(ab, abc), ao) > 0) {
            simplex.points.erase(simplex.points.begin() + 2);
            direction = glm::cross(glm::cross(ab, ao), ab);
        }
        else {
            if (glm::dot(abc, ao) > 0) {
                direction = abc;
            }
            else {
                std::swap(simplex[1], simplex[2]);
                direction = -abc;
            }
        }
        return false;
    }

    static bool tetrahedron(Simplex& simplex, glm::vec3& direction) {
        const glm::vec3& a = simplex[0];
        const glm::vec3& b = simplex[1];
        const glm::vec3& c = simplex[2];
        const glm::vec3& d = simplex[3];

        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;
        glm::vec3 ad = d - a;
        glm::vec3 ao = -a;

        glm::vec3 abc = glm::cross(ab, ac);
        glm::vec3 acd = glm::cross(ac, ad);
        glm::vec3 adb = glm::cross(ad, ab);

        if (glm::dot(abc, ao) > 0) {
            simplex.points.erase(simplex.points.begin() + 3);
            return triangle(simplex, direction);
        }
        if (glm::dot(acd, ao) > 0) {
            simplex.points.erase(simplex.points.begin() + 1);
            return triangle(simplex, direction);
        }
        if (glm::dot(adb, ao) > 0) {
            simplex.points.erase(simplex.points.begin() + 2);
            return triangle(simplex, direction);
        }

        return true;
    }

    static std::vector<ContactPoint> generateContactPoints(
        const Shape* shapeA, const Shape* shapeB,
        const glm::vec3& normal, float penetration) {

        std::vector<ContactPoint> contacts;
        ContactPoint contact;
        contact.normal = normal;
        contact.penetration = penetration;
        contact.position = shapeA->getSupport(normal);
        contacts.push_back(contact);
        return contacts;
    }
};