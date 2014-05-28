#ifndef _ODE_TO_EIGEN_H
#define _ODE_TO_EIGEN_H

#ifndef dDOUBLE
#define dDOUBLE
#endif
#include "stdafx.h"
#include <ode/ode.h>



Matrix3d FromODERotationToEigenMatrix(const dReal* rotMat);
void FromEigenMatrixToODERotation(const Matrix3d& mat, dMatrix3& ret); //return value should be deleted using delete[]

Matrix3d FromEigenEulerAngleToEigenMatrix(const Vector3d& eulerAngle);
Vector3d FromODERotationToEulerAngle(const dReal* rotMat);
void FromEulerAngleToODERotation(const Vector3d& eulerAngle, dMatrix3& ret); //return value should be deleted using delete[]

Quaterniond FromODEQuaternionToEigenQuaternion(const dReal* quat);

Affine3d FromODETransformToEigenTransform(const dReal* pos, const dReal* rot);

#endif
