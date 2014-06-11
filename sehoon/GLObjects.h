#ifndef SEHOON_GLOBJECTS_H
#define SEHOON_GLOBJECTS_H

#include <cmath>
#include <Eigen/Dense>
#include "utils/UtilsRotation.h"

namespace sehoon {
    namespace globjects {
        static void renderAxis(double LEN = 10.0);
        static void renderChessBoard(int n, int m, double sx, double sz);
        static void renderArrow(const Eigen::Vector3d& p, const Eigen::Vector3d& q,
                                double headWidth, double headLength);

        static void renderAxis(double LEN) {
            glDisable(GL_LIGHTING);
        
            glPushMatrix();
            // const double LEN = 10.0;

            glColor3d(1.0, 0.0, 0.0);
            glBegin(GL_LINES);
            glVertex3d( LEN, 0.0, 0.0);
            glVertex3d(-LEN, 0.0, 0.0);
            glEnd();

            glColor3d(0.0, 1.0, 0.0);
            glBegin(GL_LINES);
            glVertex3d(0.0,  LEN, 0.0);
            glVertex3d(0.0, -LEN, 0.0);
            glEnd();

            glColor3d(0.0, 0.0, 1.0);
            glBegin(GL_LINES);
            glVertex3d(0.0, 0.0,  LEN);
            glVertex3d(0.0, 0.0, -LEN);
            glEnd();
        
            glPopMatrix();

            glEnable(GL_LIGHTING);
        }

        static void renderChessBoard(int n, int m, double sx, double sz) {
            glDisable(GL_LIGHTING);

            glPushMatrix();

            double step_x = sx / static_cast<double>(n);
            double step_z = sz / static_cast<double>(n);
            double EPS = 0.001;

            int cnt = 0;
            for (double x = -0.5 * sx; x < 0.5 * sx - EPS; x += step_x) {
                for (double z = -0.5 * sz; z < 0.5 * sz - EPS; z += step_z) {
                    double x1 = x;
                    double z1 = z;
                    double x2 = x + step_x;
                    double z2 = z + step_z;
        
                    if (cnt % 2 == 0) {
                        // glColor3d(107.0/255.0, 66.0/255.0, 38.0/255.0);
                        glColor3d(0.4, 0.4, 0.4);
                    } else {
                        // glColor3d(142.0/255.0, 107.0/255.0, 35.0/255.0);
                        glColor3d(0.7, 0.7, 0.7);
                    }

                    glBegin(GL_POLYGON);
                    glVertex3d(x1, 0.00, z1);
                    glVertex3d(x1, 0.00, z2);
                    glVertex3d(x2, 0.00, z2);
                    glVertex3d(x2, 0.00, z1);
                    glEnd();

                    ++cnt;
                } // for (double z ... )
                ++cnt;
            } // for (double x ... )
            glPopMatrix();
        }

        static void renderArrow(const Eigen::Vector3d& p, const Eigen::Vector3d& q,
                                double headWidth, double headLength) {

            glLineWidth(2.0);
            glBegin(GL_LINES);
            glVertex3d(p(0), p(1), p(2));
            glVertex3d(q(0), q(1), q(2));
            glEnd();


            // Calculate the direction vector
            Eigen::Vector3d u(q - p);
            u /= u.norm();

            Eigen::Vector3d P = q - headLength * u;

            // Current up vector
            Eigen::Vector3d z(0.0, 0.0, 1.0);

            // Calculate the rotation axis and matrix
            Eigen::Vector3d axis = z.cross(u);
            axis /= axis.norm();
            double angle = acos(u.dot(z));
            Eigen::Matrix3d M = utils::rotation::expMapRot(angle * axis);
            double m[16] = {
                M(0, 0), M(1, 0), M(2, 0), 0.0,
                M(0, 1), M(1, 1), M(2, 1), 0.0, 
                M(0, 2), M(1, 2), M(2, 2), 0.0,
                P(0),    P(1),    P(2), 1.0 };


            // LOG(INFO) << "axis dot u = " << axis.dot(u);
            // LOG(INFO) << "axis dot z = " << axis.dot(z);
            
            // LOG(INFO) << "axis = " << axis.transpose();
            // LOG(INFO) << "angle = " << angle;

            // LOG(INFO) << endl;
            
            glPushMatrix();
            glMultMatrixd(m);
            glutSolidCone(headWidth, headLength, 10, 3);
            glPopMatrix();
        }

    } // namespace globjects
} // namespace sehoon

#endif
