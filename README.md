# GameEngine
A 3D physics engine using OpenGL.

## Requirements
- Git
- CMake (3.8 or higher)
- C++ compiler with C++20 support

## Installing Required Tools

If you don't already have a C++ compiler, CMake, or Git installed, follow these steps:

1. **C++ Compiler**
   - **Windows:** Install [MinGW](https://sourceforge.net/projects/mingw/) or the [Microsoft Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/).
   - **Linux:** Install GCC with your package manager (e.g., `sudo apt install build-essential` on Ubuntu).
   - **MacOS:** Install Xcode Command Line Tools with `xcode-select --install`.

2. **CMake**
   - Download and install CMake from the [official website](https://cmake.org/download/).

3. **Git**
   - Download and install Git from the [official website](https://git-scm.com/).

For more detailed instructions, refer to the official documentation for each tool.

## Setup & Build Instructions

Open Command Prompt/Terminal and run these commands:

1. Clone the repository:
```bash
git clone https://github.com/yourusername/GameEngine-PhysX
cd GameEngine-PhysX
```

2. Get vcpkg:
```bash
git clone https://github.com/Microsoft/vcpkg.git
```

3. Setup vcpkg:
```bash
cd vcpkg
./bootstrap-vcpkg.bat  # For Windows
./bootstrap-vcpkg.sh   # For Linux/MacOS
cd ..
```

4. Build the engine:
```bash
cmake -B build -S .
cmake --build build --config Release
```
The executable will be in `build/Release/GameEnginePhysx.exe`.

## Features
- OpenGL rendering
- Rigid body dynamics and collisions
- 3D graphics (pre-rendered)
- Shadow mapping
- Interactive camera controls

## Controls
- **Space**: Toggle camera movement
- **WASD**: Move camera when enabled
- **Mouse**: Look around when camera movement enabled
- **P**: Toggle menu
- **G**: Generate physics objects
- **L**: Play/pause simulation

## License
This project is licensed under the MIT License - see the LICENSE.txt file for details.

## Third-Party Licenses  
This project uses the following third-party libraries:  

1. **OpenGL and GLFW**: GLFW is licensed under the [zlib/libpng license](https://github.com/glfw/glfw/blob/main/LICENSE.md).  

2. **vcpkg**: Licensed under the [MIT License](https://github.com/microsoft/vcpkg/blob/master/LICENSE.txt).  

3. **Dear ImGui**: Licensed under the [MIT License](https://github.com/ocornut/imgui/blob/master/LICENSE.txt).  

These licenses apply to their respective components only.

## Troubleshooting
If you encounter build errors:

1. Make sure you have all requirements installed.
2. Try deleting the `build` directory and rebuilding.
3. Ensure you ran the vcpkg bootstrap script.
4. Check that your compiler supports C++20.

For other issues, please open a GitHub issue.