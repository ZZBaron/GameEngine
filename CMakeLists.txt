# CMakeList.txt : CMake project for GameEngine, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("GameEngine")



find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glew REQUIRED)
find_package(glm REQUIRED)
find_package(Freetype REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

# Add ImGui source files
set(IMGUI_SOURCES
    "vender/imgui/imgui.cpp"
    "vender/imgui/imgui_demo.cpp"
    "vender/imgui/imgui_draw.cpp"
    "vender/imgui/imgui_tables.cpp"
    "vender/imgui/imgui_widgets.cpp"
    "vender/imgui/backends/imgui_impl_glfw.cpp"
    "vender/imgui/backends/imgui_impl_opengl3.cpp"
)


# Add source to this project's executable.
add_executable (GameEngine "GameEngine.cpp" "GameEngine.h" 
"ui.cpp" "ui.h" "input.cpp" "input.h" "camera.cpp" "camera.h" 
"shape.h" "font.cpp" "font.h" "animate.h" "console.h" "console.cpp" "randomGen.h" 
"debugRenderer.cpp" "debugRenderer.h" "blender.cpp" "blender.h" "shader.h"  "misc_funcs.h" 
"simulation.h" "physics.h" "voxel.h" "vender/imgui/imgui.h" 
"vender/imgui/backends/imgui_impl_glfw.h" "vender/imgui/backends/imgui_impl_opengl3.h"
${IMGUI_SOURCES}
 "AABB.h" "convexCollision.h" "collisionTypes.h" "specificCollision.h" "restingContact.h" "texture.h"
"stb_image.h"  "stb_image_impl.cpp" "contactManager.h" "selection.h" "selection.cpp")


if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET GameEngine PROPERTY CXX_STANDARD 20)
endif()

# Add include directories for ImGui
target_include_directories(GameEngine PRIVATE 
    "vender/imgui"
    "vender/imgui/backends"
)


target_link_libraries(GameEngine PRIVATE
	OpenGL::GL glfw
	GLEW::GLEW
	glm::glm
	Freetype::Freetype
    Eigen3::Eigen
)


# TODO: Add tests and install targets if needed.
