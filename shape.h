// shape.h
#pragma once
#include "GameEngine.h"
#include "stb_image.h"


class Shape {
public:
    GLuint VAO, VBO, EBO;
    std::vector<glm::vec3> vertices; // Vertices of the shape, in local coords
    std::vector<glm::vec3> normals;  // Normals for each vertex
    std::vector<unsigned int> indices; // Indices to form faces
    glm::vec3 center; // chosen center of shape, in world coords
    glm::vec3 centroid; //  centroid given by vertex postition, COM if all masses are equal, in local coords
    glm::mat4 model; // model matrix for shaders, manipulate to move shape around instead of vertices
    glm::quat orientation; // orientation of shape
    glm::vec3 scale;

    //shape type string
    std::string shapeType = "general";

    // Convex hull that bounds the shape (for collision detection)
    std::vector<glm::vec3> convexVertices; // Vertices of the shape, in local coords
    std::vector<glm::vec3> convexNnormals;  // Normals for each vertex
    std::vector<unsigned int> convexIndices; // Indices to form faces

    // Add texture-related members
    GLuint textureID;
    bool hasTexture; // use color instead if hasTexture=false
    std::vector<glm::vec2> texCoords;  // Texture coordinates for vertices

    glm::vec3 color; // color for every polygon face (uses only if hasTexture=false)
    float transparency = 1.0f; // transparency of shape

    bool isConvex = false; // convexity of shape, used for collision detection (SAT) change to hasConvexHull
    bool hasConvexHull = false;
    bool isEmissive = false; // flag for emissive materials


    Shape() : 
        VAO(0), VBO(0), EBO(0), 
        textureID(0), hasTexture(false),
        center(glm::vec3(0.0f)),
        centroid(calculateCentroid()),
        color(glm::vec3(1.0f, 1.0f, 1.0f)),
        orientation(glm::quat(0.0f, 0.0f, 0.0f, 1.0f)),
        scale(1.0f, 1.0f, 1.0f) {
        updateModelMatrix();
    }

    //// Default constructor
    //Shape() : centroid(glm::vec3(0.0f)), VAO(0), VBO(0), EBO(0) {}

    void setup() {
        // Generate buffers
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        // Bind VAO
        glBindVertexArray(VAO);

        // Calculate total buffer size needed
        size_t vertexSize = vertices.size() * sizeof(glm::vec3);
        size_t normalSize = normals.size() * sizeof(glm::vec3);
        size_t texCoordSize = texCoords.size() * sizeof(glm::vec2);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertexSize + normalSize + texCoordSize, nullptr, GL_STATIC_DRAW);

        // Upload data in portions
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertexSize, vertices.data());
        glBufferSubData(GL_ARRAY_BUFFER, vertexSize, normalSize, normals.data());
        glBufferSubData(GL_ARRAY_BUFFER, vertexSize + normalSize, texCoordSize, texCoords.data());

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

        // Vertex positions
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
        glEnableVertexAttribArray(0);

        // Normals
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)vertexSize);
        glEnableVertexAttribArray(1);

        // Texture coordinates
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void*)(vertexSize + normalSize));
        glEnableVertexAttribArray(2);

        glBindVertexArray(0);
    }

    // Add texture loading method
    void loadTexture(const char* texturePath) {
        // Generate texture
        glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_2D, textureID);

        // Set texture wrapping/filtering options
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // Load and generate the texture
        int width, height, nrChannels;
        stbi_set_flip_vertically_on_load(true); // Flip texture vertically
        unsigned char* data = stbi_load(texturePath, &width, &height, &nrChannels, 0);

        if (data) {
            GLenum format = (nrChannels == 4) ? GL_RGBA : GL_RGB;
            glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
            glGenerateMipmap(GL_TEXTURE_2D);
            hasTexture = true;
        }
        else {
            std::cout << "Failed to load texture: " << texturePath << std::endl;
        }

        stbi_image_free(data);
    }

    // New method for shadow pass
    void drawShadow(GLuint depthShaderProgram, const glm::mat4& lightSpaceMatrix) {
        glUseProgram(depthShaderProgram);
        glUniformMatrix4fv(glGetUniformLocation(depthShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(depthShaderProgram, "lightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(lightSpaceMatrix));

        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }

    // Function to draw the shape
    void draw(GLuint shaderProgram, glm::mat4 view, glm::mat4 projection, const glm::mat4& lightSpaceMatrix, GLuint depthMap) {
        glUseProgram(shaderProgram);

        // Set matrices
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "lightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(lightSpaceMatrix));

        // Set material properties
        glUniform3fv(glGetUniformLocation(shaderProgram, "objectColor"), 1, glm::value_ptr(color));
        glUniform1i(glGetUniformLocation(shaderProgram, "isEmissive"), isEmissive);
        glUniform1f(glGetUniformLocation(shaderProgram, "transparency"), transparency);

        // Bind textures
        if (hasTexture) {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, textureID);
            glUniform1i(glGetUniformLocation(shaderProgram, "textureSampler"), 0);
            glUniform1i(glGetUniformLocation(shaderProgram, "hasTexture"), 1);
        }
        else {
            glUniform1i(glGetUniformLocation(shaderProgram, "hasTexture"), 0);
        }

        // Shadow map
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, depthMap);
        glUniform1i(glGetUniformLocation(shaderProgram, "shadowMap"), 1);

        // Draw
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }

    void drawWireFrame(GLuint shaderProgram, glm::mat4 view, glm::mat4 projection, const glm::mat4& lightSpaceMatrix, GLuint depthMap,
        glm::vec3 wireColor = glm::vec3(1.0f), float lineWidth = 1.0f) {
        // Store previous line width to restore later
        GLfloat previousLineWidth;
        glGetFloatv(GL_LINE_WIDTH, &previousLineWidth);
        glLineWidth(lineWidth);

        // Use the shader program
        glUseProgram(shaderProgram);

        // Set matrices
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "lightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(lightSpaceMatrix));

        // Set wireframe color and other uniforms
        glUniform3fv(glGetUniformLocation(shaderProgram, "objectColor"), 1, glm::value_ptr(wireColor));
        glUniform1i(glGetUniformLocation(shaderProgram, "isEmissive"), true); // Make wireframe emissive to be more visible
        glUniform1f(glGetUniformLocation(shaderProgram, "transparency"), 1.0f);

        // Set depth map
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, depthMap);
        glUniform1i(glGetUniformLocation(shaderProgram, "shadowMap"), 0);

        // Bind VAO
        glBindVertexArray(VAO);

        // Draw wireframe
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, 0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // Unbind VAO
        glBindVertexArray(0);

        // Reset line width
        glLineWidth(previousLineWidth);
    }

    void drawLocalAxes(const glm::mat4& view, const glm::mat4& projection, float axisLength = 1.0f, float width = 3.0f) {
        // Set line width for better visibility
        glLineWidth(width);

        // // Disable the shader program to use fixed-function pipeline
        glUseProgram(0);

        /// Set the view and projection matrices
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(glm::value_ptr(projection));
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(glm::value_ptr(view));

        // Get the position from the model matrix (4th column)
        glm::vec3 origin = glm::vec3(model[3]);  // This gets the translation component

        // Get the local axes directly from the model matrix columns
        glm::vec3 xAxis = glm::normalize(glm::vec3(model[0]));  // First column
        glm::vec3 yAxis = glm::normalize(glm::vec3(model[1]));  // Second column
        glm::vec3 zAxis = glm::normalize(glm::vec3(model[2]));  // Third column

        // Scale the axes to a fixed length (e.g., 1.0)
        glm::vec3 xEnd = origin + xAxis * axisLength;
        glm::vec3 yEnd = origin + yAxis * axisLength;
        glm::vec3 zEnd = origin + zAxis * axisLength;

        // Draw X axis in red
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f); // Red color
        glVertex3f(origin.x, origin.y, origin.z);
        glVertex3f(xEnd.x, xEnd.y, xEnd.z);
        glEnd();

        // Draw Y axis in green
        glBegin(GL_LINES);
        glColor3f(0.0f, 1.0f, 0.0f); // Green color
        glVertex3f(origin.x, origin.y, origin.z);
        glVertex3f(yEnd.x, yEnd.y, yEnd.z);
        glEnd();

        // Draw Z axis in blue
        glBegin(GL_LINES);
        glColor3f(0.0f, 0.0f, 1.0f); // Blue color
        glVertex3f(origin.x, origin.y, origin.z);
        glVertex3f(zEnd.x, zEnd.y, zEnd.z);
        glEnd();

        // Reset line width and color
        glLineWidth(1.0f);
        glColor3f(1.0f, 1.0f, 1.0f); // Default color
    }

    // Function to clean up buffers
    void cleanup() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &EBO);
    }

    // Function to display
    void display() {
        std::cout << "centroid: (" << centroid.x << ", " << centroid.y << ", " << centroid.z << ") " << std::endl;
    }

    const glm::vec3& getCentroid() const { return centroid; }



    virtual void translate(const glm::vec3& translation) { //adding virtual keyword allows overriding
        center += translation;
        updateModelMatrix();
    }

    virtual void rotate(const glm::vec3& axis, float angle) {
        orientation = glm::angleAxis(angle, axis) * orientation;
        updateModelMatrix();
    }

    virtual void setOrientation(const glm::quat& orient) {
        orientation = orient;
        updateModelMatrix();
    }

    virtual void setScale(const glm::vec3& newScale) {
        scale = newScale;
        updateModelMatrix();
    }

    // Support function for GJK - returns furthest point in given direction
    virtual glm::vec3 getSupport(const glm::vec3& direction) const {
        glm::vec3 furthestPoint;
        float maxProjection = -std::numeric_limits<float>::infinity();

        // Transform direction to local space
        glm::mat3 invRotation = glm::transpose(glm::mat3(model));
        glm::vec3 localDir = invRotation * direction;

        // Find furthest point in local space
        for (const auto& vertex : vertices) {
            float projection = glm::dot(vertex, localDir);
            if (projection > maxProjection) {
                maxProjection = projection;
                furthestPoint = vertex;
            }
        }

        // Transform point back to world space
        return glm::vec3(model * glm::vec4(furthestPoint, 1.0f));
    }

    // New method to get all face normals (for SAT)
    /*virtual std::vector<glm::vec3> getFaceNormals() const {
        return normals;
    }*/
    virtual std::vector<glm::vec3> getFaceNormals() const {
        glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(model)));
        std::vector<glm::vec3> transformedNormals;
        for (const auto& normal : normals) {
            transformedNormals.push_back(glm::normalize(normalMatrix * normal));
        }
        return transformedNormals;
    }

    virtual void updatePositionWithCenter(const glm::vec3& newCenter) {
    }

    void updateModelMatrix() {
        model = glm::mat4(1.0f);
        model = glm::translate(model, center);
        model = model * glm::mat4_cast(orientation); // apply orientation
        model = glm::rotate(model, glm::pi<float>(), glm::vec3(0.0f, 0.0f, 1.0f)); // i dont know why this is required,
        // but my shapes spawn rotated by pi about their local z (or world z since they are the same upon initialization)
        model = glm::scale(model, scale);
    }

    glm::mat4 getModelMatrix() const {
        return model;
    }

    void generateConvexHull() {
        // Clear existing convex hull data
        convexVertices.clear();
        convexNnormals.clear();
        convexIndices.clear();

        // Create vector of 3D points with normals and original indices
        std::vector<Point3D> points;
        for (size_t i = 0; i < vertices.size(); ++i) {
            points.emplace_back(vertices[i], normals[i], i);
        }

        // Generate convex hull
        buildConvexHull(points, 0, points.size(),
            convexVertices, convexNnormals, convexIndices);

        // Set up convex hull buffers
        setupConvexHullBuffers();

        // Set convex hull flag
        hasConvexHull = true;
    }

    void drawConvexHull(GLuint shaderProgram, const glm::mat4& view, const glm::mat4& projection,
        const glm::mat4& lightSpaceMatrix, GLuint depthMap,
        const glm::vec3& hullColor = glm::vec3(0.0f, 1.0f, 0.0f),
        float lineWidth = 2.0f, bool drawFaces = false) {
        if (convexVertices.empty() || convexHullVAO == 0) {
            return;
        }

        if (!hasConvexHull) {
            return;
        }

        // Store previous line width and polygon mode
        GLfloat previousLineWidth;
        glGetFloatv(GL_LINE_WIDTH, &previousLineWidth);
        GLint previousPolygonMode[2];
        glGetIntegerv(GL_POLYGON_MODE, previousPolygonMode);

        glUseProgram(shaderProgram);

        // Set matrices
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE,
            glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE,
            glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE,
            glm::value_ptr(projection));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "lightSpaceMatrix"), 1, GL_FALSE,
            glm::value_ptr(lightSpaceMatrix));

        // Set material properties
        glUniform3fv(glGetUniformLocation(shaderProgram, "objectColor"), 1,
            glm::value_ptr(hullColor));
        glUniform1i(glGetUniformLocation(shaderProgram, "hasTexture"), 0);
        glUniform1i(glGetUniformLocation(shaderProgram, "isEmissive"), true);
        glUniform1f(glGetUniformLocation(shaderProgram, "transparency"), 0.5f);

        // Bind shadow map
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, depthMap);
        glUniform1i(glGetUniformLocation(shaderProgram, "shadowMap"), 0);

        glBindVertexArray(convexHullVAO);

        if (drawFaces) {
            // Draw filled faces with transparency
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(convexIndices.size()),
                GL_UNSIGNED_INT, 0);
            glDisable(GL_BLEND);
        }

        // Draw wireframe
        glLineWidth(lineWidth);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(convexIndices.size()),
            GL_UNSIGNED_INT, 0);

        // Restore previous states
        glLineWidth(previousLineWidth);
        glPolygonMode(GL_FRONT_AND_BACK, previousPolygonMode[0]);
        glBindVertexArray(0);
    }

private:
    // Helper struct for points with additional info
    struct Point3D {
        glm::vec3 pos;
        glm::vec3 normal;
        size_t originalIndex;

        Point3D(const glm::vec3& p, const glm::vec3& n, size_t idx)
            : pos(p), normal(n), originalIndex(idx) {}
    };

    // Helper function to compute triangle normal
    glm::vec3 computeTriangleNormal(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3) {
        glm::vec3 v1 = p2 - p1;
        glm::vec3 v2 = p3 - p1;
        return glm::normalize(glm::cross(v1, v2));
    }

    // Helper function to check if a point is above a face
    bool isPointAboveFace(const glm::vec3& point, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3) {
        glm::vec3 normal = computeTriangleNormal(p1, p2, p3);
        return glm::dot(point - p1, normal) > 0;
    }

    // Recursive function to build convex hull
    void buildConvexHull(std::vector<Point3D>& points, size_t start, size_t end,
        std::vector<glm::vec3>& hullVertices,
        std::vector<glm::vec3>& hullNormals,
        std::vector<unsigned int>& hullIndices) {
        if (end - start < 4) {
            // Base case: too few points
            return;
        }

        // Sort points by x-coordinate
        std::sort(points.begin() + start, points.begin() + end,
            [](const Point3D& a, const Point3D& b) { return a.pos.x < b.pos.x; });

        size_t mid = (start + end) / 2;

        // Recursively build left and right hulls
        buildConvexHull(points, start, mid, hullVertices, hullNormals, hullIndices);
        buildConvexHull(points, mid, end, hullVertices, hullNormals, hullIndices);

        // Merge the two hulls
        std::vector<Point3D> mergedHull;

        // Find initial bridge edges
        size_t leftUpper = mid - 1;
        size_t rightUpper = mid;
        size_t leftLower = mid - 1;
        size_t rightLower = mid;

        // Find upper bridge
        bool done = false;
        while (!done) {
            done = true;
            while (isPointAboveFace(points[rightUpper].pos,
                points[leftUpper].pos,
                points[leftUpper - 1].pos,
                points[rightUpper].pos)) {
                leftUpper--;
                done = false;
            }
            while (isPointAboveFace(points[leftUpper].pos,
                points[rightUpper].pos,
                points[rightUpper + 1].pos,
                points[leftUpper].pos)) {
                rightUpper++;
                done = false;
            }
        }

        // Find lower bridge
        done = false;
        while (!done) {
            done = true;
            while (isPointAboveFace(points[rightLower].pos,
                points[leftLower].pos,
                points[leftLower - 1].pos,
                points[rightLower].pos)) {
                leftLower--;
                done = false;
            }
            while (isPointAboveFace(points[leftLower].pos,
                points[rightLower].pos,
                points[rightLower + 1].pos,
                points[leftLower].pos)) {
                rightLower++;
                done = false;
            }
        }

        // Merge the hulls using the bridge edges
        for (size_t i = leftLower; i <= leftUpper; ++i) {
            mergedHull.push_back(points[i]);
        }
        for (size_t i = rightUpper; i <= rightLower; ++i) {
            mergedHull.push_back(points[i]);
        }

        // Update hull data
        for (const auto& point : mergedHull) {
            hullVertices.push_back(point.pos);
            hullNormals.push_back(point.normal);
        }

        // Generate triangles for the merged hull
        for (size_t i = 1; i < mergedHull.size() - 1; ++i) {
            hullIndices.push_back(hullVertices.size() - mergedHull.size());
            hullIndices.push_back(hullVertices.size() - mergedHull.size() + i);
            hullIndices.push_back(hullVertices.size() - mergedHull.size() + i + 1);
        }
    }

    void setupConvexHullBuffers() {
        // Clean up existing buffers if they exist
        if (convexHullVAO != 0) {
            glDeleteVertexArrays(1, &convexHullVAO);
            glDeleteBuffers(1, &convexHullVBO);
            glDeleteBuffers(1, &convexHullEBO);
        }

        // Generate new buffers
        glGenVertexArrays(1, &convexHullVAO);
        glGenBuffers(1, &convexHullVBO);
        glGenBuffers(1, &convexHullEBO);

        glBindVertexArray(convexHullVAO);

        // Calculate buffer sizes
        size_t vertexSize = convexVertices.size() * sizeof(glm::vec3);
        size_t normalSize = convexNnormals.size() * sizeof(glm::vec3);

        // Set up VBO
        glBindBuffer(GL_ARRAY_BUFFER, convexHullVBO);
        glBufferData(GL_ARRAY_BUFFER, vertexSize + normalSize, nullptr, GL_STATIC_DRAW);

        // Upload vertex and normal data
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertexSize, convexVertices.data());
        glBufferSubData(GL_ARRAY_BUFFER, vertexSize, normalSize, convexNnormals.data());

        // Set up EBO
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, convexHullEBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, convexIndices.size() * sizeof(unsigned int),
            convexIndices.data(), GL_STATIC_DRAW);

        // Set up vertex attributes
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3),
            (void*)(vertexSize));
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);
    }

protected:
    // Add protected members for convex hull rendering
    GLuint convexHullVAO = 0;
    GLuint convexHullVBO = 0;
    GLuint convexHullEBO = 0;

    glm::vec3 calculateCentroid() const {
        if (vertices.empty()) return glm::vec3(0.0f); // Handle case with no vertices

        glm::vec3 sum(0.0f);
        for (const auto& vertex : vertices) {
            sum += vertex;
        }
        return sum / static_cast<float>(vertices.size());
    }

};



class Sphere : public Shape {
public:
    float radius;
    int numSlices;
    int numStacks;

    Sphere(glm::vec3 center, float radius, int numSlices, int numStacks)
        : radius(radius), numSlices(numSlices), numStacks(numStacks) {
        generateSphereVertices();
        this->center = center;
        setup();
        shapeType = "sphere";
        //isConvex = true;
        // centroid = center;
        updateModelMatrix();
    }

    // Override getFaceNormals for Sphere
    std::vector<glm::vec3> getFaceNormals() const override {
        // For a sphere, any direction is a valid face normal
        return { glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1) };
    }


private:
    void generateSphereVertices() {
        if (numSlices < 3) numSlices = 3;  // Minimum slices
        if (numStacks < 2) numStacks = 2;  // Minimum stacks

        float sliceStep = 2.0f * glm::pi<float>() / numSlices;
        float stackStep = glm::pi<float>() / numStacks;

        // Generate vertices and normals
        for (int i = 0; i <= numStacks; ++i) {
            float stackAngle = i * stackStep;
            float r = radius * sin(stackAngle);
            float y = radius * cos(stackAngle);

            // Calculate texture coordinates for this stack
            float t = 1.0f - (float)i / numStacks;

            for (int j = 0; j <= numSlices; ++j) {
                float sliceAngle = j * sliceStep;
                float x = r * cos(sliceAngle);
                float z = r * sin(sliceAngle);

                // Calculate texture coordinates for this slice
                float s = (float)j / numSlices;

                // Add vertex position (relative to center)
                vertices.push_back(glm::vec3(x, y, z));

                // Add normal vector (same as normalized position for a sphere)
                normals.push_back(glm::normalize(glm::vec3(x, y, z)));

                // Add texture coordinates
                texCoords.push_back(glm::vec2(s, t));
            }
        }

        // Generate indices (unchanged)
        for (int i = 0; i < numStacks; ++i) {
            for (int j = 0; j < numSlices; ++j) {
                unsigned int first = (i * (numSlices + 1)) + j;
                unsigned int second = first + numSlices + 1;
                indices.insert(indices.end(), { first, second, first + 1 });
                indices.insert(indices.end(), { second, second + 1, first + 1 });
            }
        }
    }
};

class RectPrism : public Shape {
public:
    float sideLength_a; //length along local x axis
    float sideLength_b; //length along local y axis
    float sideLength_c; //length along local z axis

    RectPrism(glm::vec3 center, float sideLength_a, float sideLength_b, float sideLength_c)
        : sideLength_a(sideLength_a), sideLength_b(sideLength_b), sideLength_c(sideLength_c) {
        generateVertices();
        centroid = calculateCentroid();
        this->center = center;
        setup();
        shapeType = "box";
        //isConvex = true;
        updateModelMatrix();
    }

    RectPrism(glm::vec3 min, glm::vec3 max)
        : sideLength_a(max.x - min.x), sideLength_b(max.y - min.y), sideLength_c(max.z - min.z) {
        generateVertices();
        centroid = calculateCentroid();
        this->center = (min + max) * 0.5f;
        setup();
        //isConvex = true;
        updateModelMatrix();
    }

    // Override getFaceNormals for RectPrism
    // Override getFaceNormals for RectPrism
    std::vector<glm::vec3> getFaceNormals() const override {
        std::vector<glm::vec3> faceNormals = {
            glm::vec3(1, 0, 0), glm::vec3(-1, 0, 0),
            glm::vec3(0, 1, 0), glm::vec3(0, -1, 0),
            glm::vec3(0, 0, 1), glm::vec3(0, 0, -1)
        };

        glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(model)));
        for (auto& normal : faceNormals) {
            normal = glm::normalize(normalMatrix * normal);
        }

        return faceNormals;
    }

    std::vector<std::pair<glm::vec3, glm::vec3>> getEdges() const {
        // Define edges based on vertices and transform them to world coordinates
        std::vector<std::pair<glm::vec3, glm::vec3>> edges = {
            {vertices[0], vertices[1]}, {vertices[1], vertices[2]}, {vertices[2], vertices[3]}, {vertices[3], vertices[0]}, // Bottom edges
            {vertices[4], vertices[5]}, {vertices[5], vertices[6]}, {vertices[6], vertices[7]}, {vertices[7], vertices[4]}, // Top edges
            {vertices[0], vertices[4]}, {vertices[1], vertices[5]}, {vertices[2], vertices[6]}, {vertices[3], vertices[7]}  // Side edges
        };

        // Transform edges to world coordinates
        for (auto& edge : edges) {
            edge.first = glm::vec3(model * glm::vec4(edge.first, 1.0f));
            edge.second = glm::vec3(model * glm::vec4(edge.second, 1.0f));
        }

        return edges;
    }

private:
    void generateVertices() {
        // Define the 8 vertices of the rectangular prism relative to local origin
        glm::vec3 halfLengths = glm::vec3(sideLength_a, sideLength_b, sideLength_c) * 0.5f;
        glm::vec3 v0 = glm::vec3(-halfLengths.x, -halfLengths.y, -halfLengths.z);
        glm::vec3 v1 = glm::vec3(halfLengths.x, -halfLengths.y, -halfLengths.z);
        glm::vec3 v2 = glm::vec3(halfLengths.x, halfLengths.y, -halfLengths.z);
        glm::vec3 v3 = glm::vec3(-halfLengths.x, halfLengths.y, -halfLengths.z);
        glm::vec3 v4 = glm::vec3(-halfLengths.x, -halfLengths.y, halfLengths.z);
        glm::vec3 v5 = glm::vec3(halfLengths.x, -halfLengths.y, halfLengths.z);
        glm::vec3 v6 = glm::vec3(halfLengths.x, halfLengths.y, halfLengths.z);
        glm::vec3 v7 = glm::vec3(-halfLengths.x, halfLengths.y, halfLengths.z);

        vertices.clear();
        normals.clear();
        texCoords.clear();


        // Front face (Z-negative)
        // Vertex positions
        vertices.push_back(v0); vertices.push_back(v1); vertices.push_back(v2); vertices.push_back(v3);
        // Normal vectors
        for (int i = 0; i < 4; ++i) normals.push_back(glm::vec3(0.0f, 0.0f, -1.0f));
        // Texture coordinates
        texCoords.push_back(glm::vec2(0.0f, 0.0f)); // bottom-left
        texCoords.push_back(glm::vec2(1.0f, 0.0f)); // bottom-right
        texCoords.push_back(glm::vec2(1.0f, 1.0f)); // top-right
        texCoords.push_back(glm::vec2(0.0f, 1.0f)); // top-left

        // Back face (Z-positive)
        vertices.push_back(v4); vertices.push_back(v5); vertices.push_back(v6); vertices.push_back(v7);
        for (int i = 0; i < 4; ++i) normals.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
        texCoords.push_back(glm::vec2(1.0f, 0.0f));
        texCoords.push_back(glm::vec2(0.0f, 0.0f));
        texCoords.push_back(glm::vec2(0.0f, 1.0f));
        texCoords.push_back(glm::vec2(1.0f, 1.0f));

        // Left face (X-negative)
        vertices.push_back(v0); vertices.push_back(v3); vertices.push_back(v7); vertices.push_back(v4);
        for (int i = 0; i < 4; ++i) normals.push_back(glm::vec3(-1.0f, 0.0f, 0.0f));
        texCoords.push_back(glm::vec2(0.0f, 0.0f));
        texCoords.push_back(glm::vec2(1.0f, 0.0f));
        texCoords.push_back(glm::vec2(1.0f, 1.0f));
        texCoords.push_back(glm::vec2(0.0f, 1.0f));

        // Right face (X-positive)
        vertices.push_back(v1); vertices.push_back(v5); vertices.push_back(v6); vertices.push_back(v2);
        for (int i = 0; i < 4; ++i) normals.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
        texCoords.push_back(glm::vec2(1.0f, 0.0f));
        texCoords.push_back(glm::vec2(0.0f, 0.0f));
        texCoords.push_back(glm::vec2(0.0f, 1.0f));
        texCoords.push_back(glm::vec2(1.0f, 1.0f));

        // Bottom face (Y-negative)
        vertices.push_back(v0); vertices.push_back(v1); vertices.push_back(v5); vertices.push_back(v4);
        for (int i = 0; i < 4; ++i) normals.push_back(glm::vec3(0.0f, -1.0f, 0.0f));
        texCoords.push_back(glm::vec2(0.0f, 1.0f));
        texCoords.push_back(glm::vec2(1.0f, 1.0f));
        texCoords.push_back(glm::vec2(1.0f, 0.0f));
        texCoords.push_back(glm::vec2(0.0f, 0.0f));

        // Top face (Y-positive)
        vertices.push_back(v3); vertices.push_back(v2); vertices.push_back(v6); vertices.push_back(v7);
        for (int i = 0; i < 4; ++i) normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
        texCoords.push_back(glm::vec2(0.0f, 0.0f));
        texCoords.push_back(glm::vec2(1.0f, 0.0f));
        texCoords.push_back(glm::vec2(1.0f, 1.0f));
        texCoords.push_back(glm::vec2(0.0f, 1.0f));


        // Define indices for the 6 faces (2 triangles per face)
        indices = {
            0, 1, 2, 2, 3, 0,   // Front face
            4, 5, 6, 6, 7, 4,   // Back face
            8, 9, 10, 10, 11, 8,   // Left face
            12, 13, 14, 14, 15, 12,  // Right face
            16, 17, 18, 18, 19, 16,  // Bottom face
            20, 21, 22, 22, 23, 20   // Top face
        };
    }
};

class Plane : public Shape {
public:
    float sideLength_a;
    float sideLength_b;

    Plane(glm::vec3 center, float sideLength_a, float sideLength_b)
        : sideLength_a(sideLength_a), sideLength_b(sideLength_b) {
        generateVertices();
        this->center = center;
        setup();
        // isConvex = true; // Plane is 2d is it convex? put no for now
        updateModelMatrix();
    }
private:

    void generateVertices() {
        glm::vec3 halfLengths = glm::vec3(sideLength_a, 0.0f, sideLength_b) * 0.5f;
        glm::vec3 v0 = glm::vec3(-halfLengths.x, 0.0f, -halfLengths.z);
        glm::vec3 v1 = glm::vec3(halfLengths.x, 0.0f, -halfLengths.z);
        glm::vec3 v2 = glm::vec3(halfLengths.x, 0.0f, halfLengths.z);
        glm::vec3 v3 = glm::vec3(-halfLengths.x, 0.0f, halfLengths.z);

        // Define vertices and corresponding normals
        std::vector<std::pair<glm::vec3, glm::vec3>> vertexNormalPairs = {
            { v0, glm::vec3(0.0f, 1.0f, 0.0f) }, { v1, glm::vec3(0.0f, 1.0f, 0.0f) }, { v2, glm::vec3(0.0f, 1.0f, 0.0f) }, { v3, glm::vec3(0.0f, 1.0f, 0.0f) } // Top face

        };

        vertices.clear();
        normals.clear();
        for (const auto& pair : vertexNormalPairs) {
            vertices.push_back(pair.first);
            normals.push_back(pair.second);
        }

        // Define indices for the 6 faces (2 triangles per face)
        indices = {
            0, 1, 2, 2, 3, 0,   // Top face
        };

    }


};