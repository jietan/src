#ifndef _EIGEN_HELPER_H
#define _EIGEN_HELPER_H

#include "stdafx.h"

bool EigenVectorCompare(const Eigen::VectorXd& a, const Eigen::VectorXd& b);
void EigenVectorBoundingBox(const vector<Eigen::VectorXd>& pts, Eigen::VectorXd& minPt, Eigen::VectorXd& maxPt);
bool operator >= (const Eigen::VectorXd& a, const Eigen::VectorXd& b);
bool operator <= (const Eigen::VectorXd& a, const Eigen::VectorXd& b);

#endif