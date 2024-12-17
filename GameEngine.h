// CMakeProject2.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <GL/glew.h> // for "modern opengl context", need to include before other gl includes (this was nessecary for me)
#include <GL/gl.h>
#include <GL/glu.h>
#include <GLFW/glfw3.h>  // GLFW for window management and OpenGL context
#include <glm/glm.hpp> // math operations i think
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp> // for glm::value_ptr
#include <vector>
#include <array>
#include <cmath> // For trigonometric functions
#include <chrono>
#include <iostream>
#include <string>
#include <glm/gtc/constants.hpp>
#include <map> //for managing keyStates
#include <memory> // for rigid body stuff
#include <functional> //for arbitrary functions
#include <sstream>
#include <fstream> // for file reading
#include <random> // for random number generation, sphere generation, procedual voxels...

//#include <fmt/core.h>

//#include <GL/glew.h>        // GLEW for managing OpenGL extensions
//#include <GLFW/glfw3.h>     // GLFW for window management and input
//#include <glm/glm.hpp>      // GLM for mathematics
//#include <glm/gtc/matrix_transform.hpp>  // GLM for matrix transformations

// TODO: Reference additional headers your program requires here.

