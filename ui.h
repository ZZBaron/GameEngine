// ui.h
#pragma once
#include "GameEngine.h"
#include "rigidBody.h"
#include "font.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

// Function and variable declarations
void toggleMenu();
void initImGui(GLFWwindow* window);
void cleanupImGui();
void createSphere();

class MenuSystem {
private:
    static MenuSystem* instance;
    bool isOpen;
    int activeTab;
    ImFont* menuFont;  // Store custom font

    // Input fields for sphere creation
    float sphereRadius;
    glm::vec3 sphereCenter;

public:
    static MenuSystem& getInstance() {
        if (!instance) {
            instance = new MenuSystem();
        }
        return *instance;
    }

    MenuSystem() : isOpen(false), activeTab(0), sphereRadius(1.0f), sphereCenter(0.0f), menuFont(nullptr) {}

    bool initializeFont() {
        ImGuiIO& io = ImGui::GetIO();

        // First try to load custom font
        const char* fontPath = "fonts/Roboto/Roboto-Regular.ttf";
        if (FILE* file = fopen(fontPath, "rb")) {
            fclose(file);
            menuFont = io.Fonts->AddFontFromFileTTF(fontPath, 16.0f);
        }

        // If custom font loading failed, use default font
        if (!menuFont) {
            std::cout << "Warning: Could not load custom font. Using default font instead.\n";
            menuFont = io.Fonts->AddFontDefault();
        }

        // Build font atlas
        unsigned char* pixels;
        int width, height;
        io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);

        return menuFont != nullptr;
    }

    void toggle() {
        isOpen = !isOpen;
    }

    bool isMenuOpen() const { return isOpen; }

    void render() {
        if (!isOpen) return;

        // Begin ImGui window
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Get screen dimensions
        int display_w, display_h;
        GLFWwindow* window = glfwGetCurrentContext();
        glfwGetFramebufferSize(window, &display_w, &display_h);

        // Set fixed menu size (80% of screen width, 90% of screen height)
        float menuWidth = display_w * 0.8f;
        float menuHeight = display_h * 0.9f;

        // Center the menu
        ImGui::SetNextWindowPos(ImVec2((display_w - menuWidth) * 0.5f, (display_h - menuHeight) * 0.5f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(menuWidth, menuHeight), ImGuiCond_Always);

        // Set menu style
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.2f, 0.2f, 0.2f, 0.95f)); // Dark gray background
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f)); // White text

        // Push custom font
        ImGui::PushFont(menuFont);

        ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoCollapse;


        if (ImGui::Begin("Game Menu", &isOpen, window_flags)) {
            // Style the tabs
            ImGui::PushStyleColor(ImGuiCol_Tab, ImVec4(0.3f, 0.3f, 0.3f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_TabHovered, ImVec4(0.4f, 0.4f, 0.4f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_TabActive, ImVec4(0.5f, 0.5f, 0.5f, 1.0f));

            if (ImGui::BeginTabBar("MenuTabs", ImGuiTabBarFlags_None)) {
                if (ImGui::BeginTabItem("Objects")) {
                    // Objects tab content
                    ImGui::BeginChild("ObjectsTab", ImVec2(0, 0), true);

                    ImGui::Text("Object Creation");
                    ImGui::Separator();

                    if (ImGui::TreeNode("Add Sphere")) {
                        ImGui::DragFloat("Radius", &sphereRadius, 0.1f, 0.1f, 10.0f);
                        ImGui::DragFloat3("Position", &sphereCenter.x, 0.1f);

                        if (ImGui::Button("Create Sphere", ImVec2(120, 30))) {
                            createSphere();
                        }
                        ImGui::TreePop();
                    }

                    ImGui::EndChild();
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Simulation")) {
                    ImGui::BeginChild("SimulationTab", ImVec2(0, 0), true);

                    ImGui::Text("Simulation Controls");
                    ImGui::Separator();

                    static bool isPlaying = false;
                    if (ImGui::Button(isPlaying ? "Pause" : "Play", ImVec2(120, 30))) {
                        isPlaying = !isPlaying;
                        // Add your play/pause logic here
                        // For example: togglePlay();
                    }

                    ImGui::SameLine();
                    if (ImGui::Button("Reset", ImVec2(120, 30))) {
                        // Add your reset logic here
                    }

                    static float gravity = -9.81f;
                    ImGui::DragFloat("Gravity", &gravity, 0.1f, -20.0f, 20.0f);

                    static float timeScale = 1.0f;
                    ImGui::DragFloat("Time Scale", &timeScale, 0.1f, 0.1f, 10.0f);

                    ImGui::EndChild();
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Debug")) {
                    ImGui::BeginChild("DebugTab", ImVec2(0, 0), true);

                    ImGui::Text("Debug Information");
                    ImGui::Separator();

                    extern float deltaTime_sys;
                    extern glm::vec3 lightPos;
                    extern glm::vec3 spherePos;
                    extern glm::vec3 sphereCOM;
                    extern glm::vec3 sphereCentroid;


                    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Delta Time: %.3f", deltaTime_sys);
                    ImGui::TextColored(ImVec4(0.0f, 1.0f, 1.0f, 1.0f), "Light Position: (%.2f, %.2f, %.2f)",
                        lightPos.x, lightPos.y, lightPos.z);
                    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "Sphere Position: (%.2f, %.2f, %.2f)",
                        spherePos.x, spherePos.y, spherePos.z);

                    ImGui::EndChild();
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Selected Object")) {
                    ImGui::BeginChild("SelectedObjectTab", ImVec2(0, 0), true);

                    extern std::shared_ptr<RigidBody> selectedRB; // Selected rigid body pointer

                    if (selectedRB) {
                        ImGui::Text("Selected Object Properties");
                        ImGui::Separator();

                        // Position
                        glm::vec3 pos = selectedRB->getCOM();
                        ImGui::Text("Position: (%.2f, %.2f, %.2f)", pos.x, pos.y, pos.z);

                        // Mass
                        ImGui::Text("Mass: %.2f", selectedRB->totalMass);

                        // Velocity
                        ImGui::Text("Velocity: (%.2f, %.2f, %.2f)",
                            selectedRB->velocity.x,
                            selectedRB->velocity.y,
                            selectedRB->velocity.z);

                        // Angular velocity
                        ImGui::Text("Angular Velocity: (%.2f, %.2f, %.2f)",
                            selectedRB->angularVelocity.x,
                            selectedRB->angularVelocity.y,
                            selectedRB->angularVelocity.z);

                        // Linear momentum
                        glm::vec3 linearMomentum = selectedRB->velocity * selectedRB->totalMass;
                        ImGui::Text("Linear Momentum: (%.2f, %.2f, %.2f)",
                            linearMomentum.x, linearMomentum.y, linearMomentum.z);

                        // Angular momentum
                        ImGui::Text("Angular Momentum: (%.2f, %.2f, %.2f)",
                            selectedRB->angularMomentum.x,
                            selectedRB->angularMomentum.y,
                            selectedRB->angularMomentum.z);
                    }
                    else {
                        ImGui::Text("No object selected");
                        ImGui::Text("Click on an object to select it");
                    }

                    ImGui::EndChild();
                    ImGui::EndTabItem();
                }

                ImGui::EndTabBar();
            }
            ImGui::PopStyleColor(3); // Pop tab colors
        }
        ImGui::End();

        // Pop styles and font
        ImGui::PopFont();
        ImGui::PopStyleColor(2); // Pop window background and text colors

        // Render ImGui
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }
};



extern bool isMenuOpen;
