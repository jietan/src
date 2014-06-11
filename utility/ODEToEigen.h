#ifndef _ODE_TO_EIGEN_H
#define _ODE_TO_EIGEN_H

#ifndef dDOUBLE
#define dDOUBLE
#endif
#include "stdafx.h"
#include <ode/ode.h>



Eigen::Matrix3d FromODERotationToEigenMatrix(const dReal* rotMat);
void FromEigenMatrixToODERotation(const Eigen::Matrix3d& mat, dMatrix3& ret); //return value should be deleted using delete[]

Eigen::Matrix3d FromEigenEulerAngleToEigenMatrix(const Eigen::Vector3d& eulerAngle);
Eigen::Vector3d FromODERotationToEulerAngle(const dReal* rotMat);
void FromEulerAngleToODERotation(const Eigen::Vector3d& eulerAngle, dMatrix3& ret); //return value should be deleted using delete[]

Quaterniond FromODEQuaternionToEigenQuaternion(const dReal* quat);

Affine3d FromODETransformToEigenTransform(const dReal* pos, const dReal* rot);

#endif
