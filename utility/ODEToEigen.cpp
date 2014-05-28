#include "stdafx.h"
#include "ODEToEigen.h"

Matrix3d FromODERotationToEigenMatrix(const dReal* rotMat)
{
	Matrix3d ret;
	ret << rotMat[0], rotMat[1], rotMat[2],
		   rotMat[4], rotMat[5], rotMat[6],
		   rotMat[8], rotMat[9], rotMat[10];
	return ret;
}

void FromEigenMatrixToODERotation(const Matrix3d& mat, dMatrix3& ret)
{
	ret[0] = mat(0, 0);
	ret[1] = mat(0, 1);
	ret[2] = mat(0, 2);
	ret[3] = 0;
	ret[4] = mat(1, 0);
	ret[5] = mat(1, 1);
	ret[6] = mat(1, 2);
	ret[7] = 0;
	ret[8] = mat(2, 0);
	ret[9] = mat(2, 1);
	ret[10] = mat(2, 2);
	ret[11] = 0;
}

Vector3d FromODERotationToEulerAngle(const dReal* rotMat)
{
	Matrix3d mat = FromODERotationToEigenMatrix(rotMat);
	Vector3d eulerAngle = mat.eulerAngles(2, 0, 1);
	return eulerAngle;
}

Matrix3d FromEigenEulerAngleToEigenMatrix(const Vector3d& eulerAngle)
{
	Matrix3d rot;
	rot = AngleAxisd(eulerAngle[0], Vector3d::UnitZ())
		* AngleAxisd(eulerAngle[1], Vector3d::UnitX())	
		* AngleAxisd(eulerAngle[2], Vector3d::UnitY());
	return rot;
}

void FromEulerAngleToODERotation(const Vector3d& eulerAngle, dMatrix3& ret)
{
	Matrix3d rot = FromEigenEulerAngleToEigenMatrix(eulerAngle);
	return FromEigenMatrixToODERotation(rot, ret);
}

Quaterniond FromODEQuaternionToEigenQuaternion(const dReal* quat)
{
	Quaterniond ret(quat[0], quat[1], quat[2], quat[3]);
	return ret;
}

Affine3d FromODETransformToEigenTransform(const dReal* pos, const dReal* rot)
{
	Affine3d ret;
	Vector3d trans;
	trans << pos[0], pos[1], pos[2];
	ret.translation() = trans;

	Matrix3d rotation = FromODERotationToEigenMatrix(rot);
	ret.linear() = rotation;
	return ret;
}

