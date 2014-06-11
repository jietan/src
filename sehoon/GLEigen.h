#ifndef SEHOON_GLEIGEN_H
#define SEHOON_GLEIGEN_H

#include <Eigen/Dense>
#include "utils/LoadOpengl.h"

namespace sehoon {
    static void glColor3d(const Eigen::Vector3d& color) {
        ::glColor3d(color(0), color(1), color(2));
    }

    static void glTranslated(const Eigen::Vector3d& pos) {
        ::glTranslated(pos(0), pos(1), pos(2));
    }

    static void glVertex3d(const Eigen::Vector3d& v) {
        ::glVertex3d(v(0), v(1), v(2));
    }

    class GLMatrixManip {
    public:
        GLMatrixManip() : count(0) { ::glPushMatrix(); }
        ~GLMatrixManip() { ::glPopMatrix(); }
        bool exit() { ++count; if (count <= 1) return true; else return false; }
    private:
        int count;
    };
} // namespace sehoon

#define glBlock() for(sehoon::GLMatrixManip m; m.exit(); )


#endif // #ifndef SEHOON_GLEIGEN_H

