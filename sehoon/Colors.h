#ifndef SEHOON_COLORS_H
#define SEHOON_COLORS_H

#include <cstdlib>

namespace sehoon {
    namespace glcolors {
        static Eigen::Vector3d black()   { return Eigen::Vector3d(0.0, 0.0, 0.0) / 255.0; }
        static Eigen::Vector3d grey()    { return Eigen::Vector3d(128.0, 128.0, 128.0) / 255.0; }
        static Eigen::Vector3d silver()  { return Eigen::Vector3d(192.0, 192.0, 192.0) / 255.0; }
        static Eigen::Vector3d white()   { return Eigen::Vector3d(255.0, 255.0, 255.0) / 255.0; }
        static Eigen::Vector3d maroon()  { return Eigen::Vector3d(128.0, 0.0, 0.0) / 255.0; }

        static Eigen::Vector3d red()     { return Eigen::Vector3d(255.0, 0.0, 0.0) / 255.0; }
        static Eigen::Vector3d purple()  { return Eigen::Vector3d(128.0, 0.0, 128.0) / 255.0; }
        static Eigen::Vector3d fuchsia() { return Eigen::Vector3d(255.0, 0.0, 255.0) / 255.0; }
        static Eigen::Vector3d green()   { return Eigen::Vector3d(0.0, 255.0, 0.0) / 255.0; }
        static Eigen::Vector3d lime()    { return Eigen::Vector3d(191.0, 255.0, 0.0) / 255.0; }

        static Eigen::Vector3d olive()   { return Eigen::Vector3d(128.0, 128.0, 0.0) / 255.0; }
        static Eigen::Vector3d yellow()  { return Eigen::Vector3d(255.0, 255.0, 0.0) / 255.0; }
        static Eigen::Vector3d navy()    { return Eigen::Vector3d(0.0, 0.0, 128.0) / 255.0; }
        static Eigen::Vector3d blue()    { return Eigen::Vector3d(0.0, 0.0, 255.0) / 255.0; }
        static Eigen::Vector3d teal()    { return Eigen::Vector3d(0.0, 128.0, 128.0) / 255.0; }

        static Eigen::Vector3d aqua()    { return Eigen::Vector3d(0.0, 255.0, 255.0) / 255.0; }
        
        static Eigen::Vector3d pink()    { return fuchsia(); }

        static int numColors() { return 16; }
        static Eigen::Vector3d getColor(int index) {
            switch (index) {
            case 0: return black();
            case 1: return grey();
            case 2: return silver();
            case 3: return white();
            case 4: return maroon();

            case 5: return red();
            case 6: return purple();
            case 7: return fuchsia();
            case 8: return green();
            case 9: return lime();

            case 10: return olive();
            case 11: return yellow();
            case 12: return navy();
            case 13: return blue();
            case 14: return teal();

            case 15: return aqua();
            }
            return red();
        }
        static Eigen::Vector3d randomColor() {
            return getColor( rand() % numColors() );
        }
        
        
    } // namespace colors
} // namespace sehoon

#endif
