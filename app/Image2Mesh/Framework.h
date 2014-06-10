#ifndef FRAMEWORK_H
#define FRAMEWORK_H

//// GLEW must be included first, if we need it.
//#ifdef _WIN32
//#define GLEW_STATIC
//#define FRAMEWORK_USE_GLEW
//#include <GL/glew.h>
//#endif
//
//
//// SFML automatically includes SDL headers
//#include <SFML/Window.hpp>
//#include <SFML/Graphics.hpp>

// Open Asset Import Library
#include <assimp.hpp>
#include <aiScene.h>
#include <aiPostProcess.h>

#include <memory>
#include <iostream>

//extern sf::RenderWindow window;

inline void copy(const aiColor3D& color, float out[4]) {
    out[0] = color.r;
    out[1] = color.g;
    out[2] = color.b;
    out[3] = 1.0f;
}

inline void copy(const aiVector3D& vector, float out[4]) {
    out[0] = vector.x;
    out[1] = vector.y;
    out[2] = vector.z;
    out[3] = 1.0f;
}

inline void copy(const aiMatrix4x4& matrix, float out[16]) {
    // Need to convert to column-major order as OpenGL likes

    out[0] = matrix.a1;
    out[1] = matrix.b1;
    out[2] = matrix.c1;
    out[3] = matrix.d1;

    out[4] = matrix.a2;
    out[5] = matrix.b2;
    out[6] = matrix.c2;
    out[7] = matrix.d2;

    out[8] = matrix.a3;
    out[9] = matrix.b3;
    out[10] = matrix.c3;
    out[11] = matrix.d3;

    out[12] = matrix.a4;
    out[13] = matrix.b4;
    out[14] = matrix.c4;
    out[15] = matrix.d4;

}

inline aiVector3D cross(const aiVector3D& a, const aiVector3D& b) {
    
    aiVector3D out;
    out.x = a.y*b.z - a.z*b.y;
    out.y = a.z*b.x - a.x*b.z;
    out.z = a.x*b.y - a.y*b.x;

    return out;
}

inline float dot(const aiVector3D& a, const aiVector3D& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

//inline bool glHasErrors() {
//    GLenum error = glGetError();
//
//    if (GL_NO_ERROR == error) {
//        return false;
//    }
//
//    while (GL_NO_ERROR != error) {
//        std::cerr << gluErrorString(error) << std::endl;
//        error = glGetError();
//    }
//    return true;
//}
//
//#define GL_CHECK() {\
//    if (glHasErrors()) {\
//        std::cerr << "Error: " __FILE__ ":" << __LINE__ << std::endl;\
//        exit(-1);\
//    }\
//}

#endif
