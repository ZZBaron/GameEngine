//ui.cpp
#pragma once
#include "GameEngine.h"
#include "ui.h"
#include "font.h"
#include "camera.h"
#include <sstream>
#include "vender/imgui/imgui.h"
#include "vender/imgui/backends/imgui_impl_glfw.h"
#include "vender/imgui/backends/imgui_impl_opengl3.h"

// maybe use qt?
//#include <QApplication> // for qt5 menu
//#include <QOpenGLWidget>
//#include <QVBoxLayout>
//#include <QLabel>
//#include <QWidget>

bool isMenuOpen = false;
MenuSystem* MenuSystem::instance = nullptr;

void toggleMenu() {
    MenuSystem::getInstance().toggle();
    if (MenuSystem::getInstance().isMenuOpen()) {
        glfwSetInputMode(glfwGetCurrentContext(), GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }
    else {
        glfwSetInputMode(glfwGetCurrentContext(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    }
}

//ImGui

// Initialize ImGui
void initImGui(GLFWwindow* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // Initialize MenuSystem and its font
    MenuSystem& menuSystem = MenuSystem::getInstance();
    if (!menuSystem.initializeFont()) {
        std::cerr << "Failed to initialize fonts\n";
    }

    // Configure ImGui style
    ImGui::StyleColorsDark();

    // Customize style if desired
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 5.0f;
    style.FrameRounding = 4.0f;
    style.PopupRounding = 3.0f;
    style.ScrollbarRounding = 3.0f;
    style.GrabRounding = 3.0f;

    // Set colors
    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4(0.1f, 0.1f, 0.1f, 0.94f);
    colors[ImGuiCol_Header] = ImVec4(0.2f, 0.2f, 0.2f, 0.98f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.3f, 0.3f, 0.3f, 0.99f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
    std::cout << "ImGui Initialized" << std::endl;
}

// Cleanup ImGui
void cleanupImGui() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}



void createSphere() {
    // Your sphere creation logic
    std::cout << "Sphere created!" << std::endl;
}

