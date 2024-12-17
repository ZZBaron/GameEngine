// GameEngine.cpp : Defines the entry point for the application.
//

#include "GameEngine.h"
#include "render.h"
#include "ui.h"
#include "input.h"
#include "camera.h"
#include "shape.h"
#include "rigidBody.h"
#include "physics.h"
#include "font.h"
#include "collisionSystem.h"
#include "console.h"
#include "randomGen.h"
#include "debugRenderer.h" // for rendering depth map to quad
#include "shader.h"
#include "simulation.h" // for sim mode
#include "misc_funcs.h"

// Specify shader paths

std::string textVertex_shaderPATH_string = getProjectRoot() + "/text_vertex_shader.glsl";
std::string textFragment_shaderPATH_string = getProjectRoot() + "/text_fragment_shader.glsl";
const char* textVertex_shaderPATH = textVertex_shaderPATH_string.c_str();
const char* textFragment_shaderPATH = textFragment_shaderPATH_string.c_str();

std::string depthVertexShaderPATH_string = getProjectRoot() + "/depth_vertex_shader.glsl";
std::string depthFragmentShaderPATH_string = getProjectRoot() + "/depth_fragment_shader.glsl";
const char* depthVertexShaderPATH = depthVertexShaderPATH_string.c_str();
const char* depthFragmentShaderPATH = depthFragmentShaderPATH_string.c_str();

std::string shadowVetexShaderPATH_string = getProjectRoot() + "/shadow_vertex_shader.glsl";
std::string shadowFragmentShaderPATH_string = getProjectRoot() + "/shadow_fragment_shader.glsl";
const char* shadowVetexShaderPATH = shadowVetexShaderPATH_string.c_str();
const char* shadowFragmentShaderPATH = shadowFragmentShaderPATH_string.c_str();


// Screen dimensions
int screenWidth = 1792;
int screenHeight = 1008;

bool consoleActive = false; // for console toggle
float simSpeed = 1.0f; // for controlling simulation time factor
bool play = false; // play sim/animation
bool gravityEnabled = true; // enable gravity
bool collisionEnabled = true; // enable collision detection
bool genSpheres = false; // generate spheres

//simulation params
bool simulationMode = true; // generate physics with set deltaTime and then play like a movie at custom speed
SimulationData simulationData;
float simulationPlaybackTime = 0.0f;
float simulationPlaybackSpeed = 1.0f;

std::chrono::steady_clock::time_point startTime_sys;
float deltaTime_sys = 0.0f; // Time difference between frames (sys because might be different than deltaTime_sim)
float deltaTime_sim = 0.0f; // Time difference between frames for simulation


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//Global var
GLuint depthShaderProgram;
GLuint shadowShaderProgram;
GLuint shaderProgram;
GLuint textShaderProgram;
GLuint testShaderProgram;
GLuint debugDepthShaderProgram; // for rendering depth map to quad
GLuint shaders; // change shaders if needed
bool shadowsEnabled = false; // enable shadows
glm::vec3 lightPos(3.0f, 3.0f, 3.0f); // Example light position, dist from earth to sun is 150.4*10^9 m
glm::vec3 lightColor(1.0f, 1.0f, 1.0f); // White light
glm::mat4 lightProjection, lightView;
glm::mat4 lightSpaceMatrix;
float near_plane = 0.1f, far_plane = 20.0f; // for shadow mapping
float luminousPower = 1.0f; // Luminous flux of the light source

bool trackLight = false; // Make camera same as light view

// store shapes and bodies into vectors
std::vector<std::shared_ptr<Shape>> shapes;
std::vector<RigidBody> bodies;
// Physics world
PhysicsWorld physicsWorld;

// for debugging
glm::vec3 spherePos;
glm::vec3 sphereCOM;
glm::vec3 sphereCentroid;

GLuint depthMap;
GLuint depthMapFBO;
// higher values of SHADOW_WIDTH and SHADOW_HEIGHT will give better quality shadows
const unsigned int SHADOW_WIDTH = 2048, SHADOW_HEIGHT = 2048;

void depthMapSetup() {
    glGenFramebuffers(1, &depthMapFBO);

    glGenTextures(1, &depthMap);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float borderColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void setupScene() {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);

    //Set up depth map and shadows
	depthMapSetup();

	Shader depthShader(depthVertexShaderPATH, depthFragmentShaderPATH);
	depthShaderProgram = depthShader.getShaderProgram();

	debugDepthShaderProgram = initDebugDepthShader(); // for rendering depth map to quad

	Shader shadowShader(shadowVetexShaderPATH, shadowFragmentShaderPATH);
    shadowShaderProgram = shadowShader.getShaderProgram();
    
    // Create and compile shaders for text rendering
    textShaderProgram = createShaderProgram(textVertex_shaderPATH, textFragment_shaderPATH);


    // Set up projection matrix for text rendering
    glm::mat4 textProjection = glm::ortho(0.0f, (float) screenWidth, 0.0f, (float) screenHeight);
    glUseProgram(textShaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(textShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(textProjection));

}

// function definitions before main
void updateSimulation(std::vector<RigidBody>& bodies, float deltaTime) {

    if (simulationMode) {
        if (simulationData.timePoints.empty() || simulationPlaybackTime >= simulationData.timePoints.back()) {
            //std::cout << vec3_to_string(bodies[0].COM) << std::endl;
            //std::cout << vec3_to_string(bodies[0].shape->center) << std::endl;
   //         std::cout << vec3_to_string(bodies[0].shape->rotationAxis) << std::endl;
   //         std::cout << bodies[0].shape->rotationAngle << std::endl;
            // Generate or regenerate simulation data
            float t_start = 0.0f;
            float t_end = 5.0f; // Simulate 10 seconds
            float simDeltaTime = 0.01f; // 10ms time step
            simulationData = generateSimulationData(bodies, t_start, t_end, simDeltaTime);
            //std::cout << vec3_to_string(bodies[0].COM) << std::endl;
            //std::cout << vec3_to_string(bodies[0].shape->center) << std::endl;
   //         std::cout << vec3_to_string(bodies[0].shape->rotationAxis) << std::endl;
   //         std::cout << bodies[0].shape->rotationAngle << std::endl;
            simulationPlaybackTime = t_start;
        }


        if (play) {
            // Update simulation playback time
            simulationPlaybackTime += deltaTime * simulationPlaybackSpeed;

            std::cout << "sphere center: " << vec3_to_string(bodies[1].shape->center) << std::endl;
            std::cout << "box center: " << vec3_to_string(bodies[2].shape->center) << std::endl;


            // Get interpolated state for current playback time
            auto state = simulationData.interpolateState(simulationPlaybackTime);

            // Update bodies with interpolated state
            for (size_t i = 0; i < bodies.size(); ++i) {
                if (!bodies[i].clamped) {
                    bodies[i].updateCOM(state.positions[i]);
                    bodies[i].velocity = state.velocities[i];
                    bodies[i].setOrientation(state.orientations[i]);
                    bodies[i].angularVelocity = state.angularVelocities[i];
                }
            }
        }
    }
    else {
        // Regular physics update
        if (play) {
            // updatePhysics(bodies, deltaTime);
        }
    }
}

void updatePhysicsWorld(float deltaTime) {
    // Replace the existing updateSimulation call with:
    physicsWorld.updateSimulation(deltaTime);

    // After updating the simulation, update your local bodies vector
    for (size_t i = 0; i < bodies.size(); ++i) {
        bodies[i] = *physicsWorld.bodies[i];
    }

    // ... (keep the rest of your rendering code)

    // Update PhysicsWorld parameters if they change in the UI
    physicsWorld.simulationMode = simulationMode;
    physicsWorld.play = play;
    physicsWorld.simulationPlaybackSpeed = simulationPlaybackSpeed;
}


int main() {

    std::cout << "GLM Version: "
        << GLM_VERSION_MAJOR << "."
        << GLM_VERSION_MINOR << "."
        << GLM_VERSION_PATCH << std::endl;

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    //
    GLFWwindow* window = glfwCreateWindow(screenWidth, screenHeight, "OpenGL Game Engine", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    // what is the exe dir?
    std::cout << getProjectRoot() << std::endl;

    glfwMakeContextCurrent(window);

    // Set up viewport
    glViewport(0, 0, screenWidth, screenHeight);

    // Set GLFW input mode to capture the mouse cursor
    glfwSetCursorPosCallback(window, mouse_callback);

    // Set the key callback
    glfwSetKeyCallback(window, key_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Initialize GLEW after creating the context
    if (glewInit() != GLEW_OK) {
        return -1;
    }

    setupScene();
    camstate = false; // enable camera movement, default value is false
	initializeConsole(); // Initialize console for text input
    initImGui(window); 

    // Initialize FreeType for font (after glfw and glew)
    std::string fontPATH_string = getProjectRoot() + "/fonts/Roboto/Roboto-Regular.ttf";
    const char* fontPATH = fontPATH_string.c_str();
    initFreeType(fontPATH);

    startTime_sys = std::chrono::steady_clock::now(); // Initialize start time

    // Create objects

    // Create a flat ground plane
    // maybe set clamped object to have mass std::numeric_limits<float>::max()? Although this causes numerical errors

    float groundThickness = 7.0f;
    auto ground = std::make_shared<RectPrism>(glm::vec3(0.0f, 0.0f - groundThickness/2.0f -2.0f , 0.0f), 10.0f, groundThickness, 10.0f);
	ground->color = glm::vec3(0.0f, 1.0f, 0.0f);
    RigidBody groundRB(ground, 1.0f, ground->center, glm::vec3(0.0f)); // infinite mass
	groundRB.clamped = true; // static object
    groundRB.restitution = 1.0f; // elastic
    std::string grassTexPATH_string = getProjectRoot() + "/textures/grass.jpg";
    const char* grassTexPATH = grassTexPATH_string.c_str();
    ground->loadTexture(grassTexPATH);
    shapes.push_back(ground);
    bodies.push_back(groundRB);

    // Add after creating ground RectPrism
    std::cout << "Ground normals: " << std::endl;
    auto normals = ground->getFaceNormals();
    for (int i = 0; i < normals.size(); i++) {
        std::cout << "Face " << i << ": (" << normals[i].x << ", " << normals[i].y << ", " << normals[i].z << ")" << std::endl;
    }

    auto sphere1 = std::make_shared<Sphere>(glm::vec3(0.0f, 2.0f, 0.0f), 0.5f, 40, 40);
    sphere1->generateConvexHull();
    RigidBody sphereRB1(sphere1, 1.0f, sphere1->center, glm::vec3(0.0f, 0.0f, 0.0f));
    sphereRB1.restitution = 0.5f;
    sphereRB1.velocity = glm::vec3(0.0f, -5.0f, 0.0f);
    std::string metalTexPATH_string = getProjectRoot() + "/textures/rubber.jpg";
    const char* metalTexPATH = metalTexPATH_string.c_str();
    sphere1->loadTexture(metalTexPATH);
    shapes.push_back(sphere1);
    bodies.push_back(sphereRB1);

    /*auto sphere2 = std::make_shared<Sphere>(glm::vec3(3.0f, 2.0f, 0.0f), 0.5f, 40, 40);
    RigidBody sphereRB2(sphere2, 1.0f, sphere2->center, glm::vec3(0.0f, 0.0f, 0.0f));
	sphereRB2.velocity = glm::vec3(-0.5f, 0.0f, 0.0f);
    shapes.push_back(sphere2);
    bodies.push_back(sphereRB2);*/

    /*
	auto box = std::make_shared<RectPrism>(glm::vec3(0.0f, 3.0f, 0.0f), 0.5f, 0.5f, 0.5f);
    box->color = glm::vec3(1.0f, 0.0f, 0.0f);
	RigidBody boxRB(box, 1.0f, box->center, glm::vec3(0.0f, 0.0f, 0.0f));
	boxRB.velocity = glm::vec3(0.1f, 0.0f, 0.0f);
    // boxRB.setOrientation(glm::angleAxis(glm::pi<float>(), glm::vec3(0.0f, ));
	//boxRB.angularMomentum = glm::vec3(1.0f, 1.0f, 1.0f);
	//boxRB.updateAngularVelocity();
	shapes.push_back(box);
	bodies.push_back(boxRB);
    */

    auto lightVisualizer = std::make_shared<Sphere>(lightPos, 0.1f, 10, 10);
    lightVisualizer->color = glm::vec3(1.0f, 1.0f, 0.0f); // Yellow color for visibility
    lightVisualizer->isEmissive = true;
	shapes.push_back(lightVisualizer);

	//try out the plane
	// auto plane = std::make_shared<Plane>(glm::vec3(0.0f, 0.5f, 0.0f), 0.5f,0.5f);
	// shapes.push_back(plane);


	// Define your bounding box for sphere generation
    glm::vec3 boxMin(1.0f, 1.0f, 1.0f);  // Adjust as needed
    glm::vec3 boxMax(2.0f, 2.0f, 2.0f);   // Adjust as needed

    //generateRandomSpheres(shapes, bodies, boxMin, boxMax, 0.1f, 10, 10, 100);
	//generateRandomBoxes(shapes, bodies, boxMin, boxMax, 0.1f, 0.1f, 0.1f, 10);

    // define bounding box
	/*RectPrismBounds bounds(boxMinCollision, boxMaxCollision);
	shapes.push_back(bounds.shape);
	bodies.push_back(bounds);*/

    float shapeGenerationInterval = 0.1f;  // Generate spheres every 1 second

    float timeSinceLastGeneration = 0.0f;

	//set up physics world
    for (auto& body : bodies) {
        physicsWorld.bodies.push_back(std::make_shared<RigidBody>(body));
    }

    // Set initial PhysicsWorld parameters
    physicsWorld.simulationMode = simulationMode;
    physicsWorld.play = play;
    physicsWorld.simulationPlaybackSpeed = simulationPlaybackSpeed;

    // set up inital cam view
    glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)screenWidth / (float)screenHeight, 0.1f, 100.0f);
    glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
    glUniformMatrix4fv(glGetUniformLocation(shadowShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
    glUniformMatrix4fv(glGetUniformLocation(shadowShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniform3fv(glGetUniformLocation(shadowShaderProgram, "viewPos"), 1, glm::value_ptr(cameraPos));
	updateCamera(shadowShaderProgram); // Update the camera view

	// Main loop
    while (!glfwWindowShouldClose(window)) {


        //Manage system time and simulation time
        auto currentTime = std::chrono::steady_clock::now(); // Get current time
        std::chrono::duration<float> elapsed = currentTime - startTime_sys; // Calculate elapsed time
        deltaTime_sys = elapsed.count(); // Store elapsed time
        deltaTime_sim = simSpeed * deltaTime_sys; // is this right? use sys for now
        startTime_sys = currentTime; // Update start time for next frame

        processInput(window);  // Process keyboard input

        // Update light position
        lightPos = glm::vec3(5.0f * cos(0.1f * glfwGetTime()), 10.0f, 5.0f * sin(0.1f * glfwGetTime()));
        lightVisualizer->translate(lightPos - lightVisualizer->center);
        
		//spherePos = sphereRB1.shape->center;
        //sphereCOM = sphereRB1.getCOM();

        // Generate random spheres
        timeSinceLastGeneration += deltaTime_sys;
        if (timeSinceLastGeneration >= shapeGenerationInterval && genSpheres) {
            generateRandomSpheres(shapes, bodies, boxMin, boxMax, 0.1f, 10, 10, 1);  // Generate 1 sphere
            // generateRandomBoxes(shapes, bodies, boxMin, boxMax, 0.1f, 0.1f, 0.1f, 1);
            timeSinceLastGeneration = 0.0f;
        }

        // for shadows and depth map
        glUseProgram(depthShaderProgram);
        lightProjection = glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f, near_plane, far_plane);
        lightView = glm::lookAt(lightPos, glm::vec3(0.0f), glm::vec3(0.0, 1.0, 0.0));
        lightSpaceMatrix = lightProjection * lightView;


        glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
        glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
        glClear(GL_DEPTH_BUFFER_BIT);

        // Use your depth map shader and render scene
        if (shadowsEnabled) {
			for (size_t i = 0; i < shapes.size(); ++i) { // does this need to be done in order of distance from light source?
            shapes[i]->drawShadow(depthShaderProgram, lightSpaceMatrix);
        }
        }
		// draw other objects for shadow map here

        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // render scene as normal using the generated depth/shadow map
        glViewport(0, 0, screenWidth, screenHeight);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        projection = glm::perspective(glm::radians(45.0f), (float)screenWidth / (float)screenHeight, 0.1f, 100.0f);
        view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
        // Use the shadow shader program
		shaders = shadowShaderProgram;
        glUseProgram(shaders);
        glUniformMatrix4fv(glGetUniformLocation(shadowShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
        glUniformMatrix4fv(glGetUniformLocation(shadowShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniform3fv(glGetUniformLocation(shadowShaderProgram, "viewPos"), 1, glm::value_ptr(cameraPos));
        glUniform3fv(glGetUniformLocation(shadowShaderProgram, "lightColor"), 1, glm::value_ptr(lightColor));
        // Update light position uniform
        glUniform3fv(glGetUniformLocation(shadowShaderProgram, "lightPos"), 1, glm::value_ptr(lightPos));
		glUniformMatrix4fv(glGetUniformLocation(shadowShaderProgram, "lightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(lightSpaceMatrix));

        //set light flux
        glUniform1f(glGetUniformLocation(shaders, "luminousPower"), luminousPower);
        
        if (camstate == true) {
            updateCamera(shaders); // Update the camera view

        }

		if (trackLight) {
			view = glm::lookAt(lightPos, glm::vec3(0.0f), glm::vec3(0.0, 1.0, 0.0));
		}
    
   
		// also I want my RigidBody class to have a bool clamped and a bool collisionEnabled, clamped=true treats the object as a static object as unmovable and similfies collision calculation, collisionEnabled=false disables collision detection for the object.
       
        
		updatePhysicsWorld(deltaTime_sys); // Update physics world

        //Animation
        // Create a quaternion for rotation (e.g., degrees around the z-axis)
        // glm::quat groundRotation = glm::angleAxis(glm::radians((float) glfwGetTime()), glm::vec3(0.0f, 1.0f, 0.0f));
        // sphereRB1.rotate(groundRotation);

        // Render objects
         drawAxes(view, projection); // world axes

         

        for (size_t i = 0; i < shapes.size(); ++i) {
            shapes[i]->draw(shaders, view, projection, lightSpaceMatrix, depthMap);
            shapes[i]->drawConvexHull(shaderProgram, view, projection, lightSpaceMatrix, depthMap);
            //local axex
            shapes[i]->drawLocalAxes(view, projection);
        }

        for (size_t i = 0; i < bodies.size(); ++i) {
            bodies[i].boundingBox->drawWireFrame(shaders, view, projection, lightSpaceMatrix, depthMap);
        }

        // glViewport(0, 0, screenWidth / 4, screenHeight / 4); // Render to a quarter of the screen
        // renderDepthMapToQuad(depthMap, lightSpaceMatrix, near_plane, far_plane);


        //draw menu after objects
        // Inside the main loop, just before glfwSwapBuffers(window):
        if (MenuSystem::getInstance().isMenuOpen()) {
            MenuSystem::getInstance().render();
        }

        // render console
        if (Console::getInstance().isVisible()) {
            Console::getInstance().render();
        }

		/*if (glGetError() != GL_NO_ERROR) {
			std::cerr << "OpenGL error: " << glGetError() << std::endl;
		}*/

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    //clean up imgui
    cleanupImGui();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
