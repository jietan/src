#include "stdafx.h"
#include "mathlib.h"
#include "polarDecomposition.h"

const static DOUBLE PI = 3.14159265359;
DecoQuaternion::DecoQuaternion(DOUBLE angle, vector3 rotationAxis)
{
	DOUBLE sn = sin(angle / 2);
	DOUBLE cs = cos(angle / 2);
	rotationAxis.normalize();
	m_v = sn * rotationAxis;
	m_w = cs;
}
vector4 DecoQuaternion::Rotate(vector4 vec) const
{
	DecoQuaternion qVec(vec);
	DecoQuaternion inv(*this);
	inv.Inverse();
	DecoQuaternion result = *this * qVec * inv;
	vector4 ret(result.m_v.x, result.m_v.y, result.m_v.z, result.m_w);
	matrix44 out;
	Matrix(out);
	vector4 ret1;
	ret1 = out * vec;
	return ret;
}
void DecoQuaternion::Matrix(matrix44& outMatrix) const
{
	DOUBLE norm = Norm();
	DOUBLE qx = m_v.x;
	DOUBLE qy = m_v.y;
	DOUBLE qz = m_v.z;
	DOUBLE qw = m_w;
	DOUBLE s = 2 / norm;

	outMatrix[0].x = 1 - s * (qy * qy + qz * qz);
	outMatrix[0].y = s * (qx * qy + qw * qz);
	outMatrix[0].z = s * (qx * qz - qw * qy);
	outMatrix[0].w = 0;

	outMatrix[1].x = s * (qx * qy - qw * qz);
	outMatrix[1].y = 1 - s * (qx * qx + qz * qz);
	outMatrix[1].z = s * (qy * qz + qw * qx);
	outMatrix[1].w = 0;

	outMatrix[2].x = s * (qx * qz + qw * qy);
	outMatrix[2].y = s * (qy * qz - qw * qx);
	outMatrix[2].z = 1 - s * (qx * qx + qy * qy);
	outMatrix[2].w = 0;

	outMatrix[3].x = 0;
	outMatrix[3].y = 0;
	outMatrix[3].z = 0;
	outMatrix[3].w = 1;
}

void DecoQuaternion::Inverse()
{
	DOUBLE norm = Norm();
	Conjugate();
	DOUBLE invSqrNorm = 1 / (norm * norm);
	m_v *= invSqrNorm;
	m_w *= invSqrNorm;
}

DecoQuaternion DecoQuaternion::operator* (const DecoQuaternion& rhs) const
{
	DecoQuaternion ret;
	ret.m_v = CrossProduct(m_v, rhs.m_v) + rhs.m_w * m_v + m_w * rhs.m_v;
	ret.m_w = m_w * rhs.m_w - DotProduct(m_v, rhs.m_v);
	return ret;
}



//Box Box::TransformBy (const matrix44& matrix) const
//{
//	vector3 vertices[NUM_VERTICES_IN_BOX];
//	vector3 minPoint(MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE);
//	vector3 maxPoint(MIN_DOUBLE, MIN_DOUBLE, MIN_DOUBLE);
//	AllVerteices(vertices);
//	for (size_t ithVertex = 0; ithVertex < NUM_VERTICES_IN_BOX; ithVertex++)
//	{
//		vector4 homoVertex(vertices[ithVertex].x, vertices[ithVertex].y, vertices[ithVertex].z, 1.f);
//		vector4 transVertex = matrix * homoVertex;
//		vertices[ithVertex] = vector3(transVertex.x, transVertex.y, transVertex.z);		
//	}
//	Box bb = Bounding(vertices, NUM_VERTICES_IN_BOX);
//
//	return bb;
//}
//
//void Box::AllVerteices(vector3* Vertices) const
//{
//	assert(Vertices);
//	vector3 center, extent;
//	GetCenterAndExtents(center, extent);
//	Vertices[0] = vector3(center.x - extent.x, center.y - extent.y, center.z - extent.z);
//	Vertices[1] = vector3(center.x + extent.x, center.y - extent.y, center.z - extent.z);
//	Vertices[2] = vector3(center.x + extent.x, center.y - extent.y, center.z + extent.z);
//	Vertices[3] = vector3(center.x - extent.x, center.y - extent.y, center.z + extent.z);
//	Vertices[4] = vector3(center.x - extent.x, center.y + extent.y, center.z - extent.z);
//	Vertices[5] = vector3(center.x + extent.x, center.y + extent.y, center.z - extent.z);
//	Vertices[6] = vector3(center.x + extent.x, center.y + extent.y, center.z + extent.z);
//	Vertices[7] = vector3(center.x - extent.x, center.y + extent.y, center.z + extent.z);
//}
//
//double Box::RayCheck(const vector3& origin, const vector3& dir) const
//{
//	double t = -1;
//	ConvexVolume box = GetVolume();
//	for (size_t ithFace = 0; ithFace < MAX_FACES_BOX / 2; ithFace++)
//	{
//		vector3 faceNormal = box.BoundingPlanes[ithFace].GetNormal();
//		double testDir = DotProduct(faceNormal, dir);
//		//if (testDir < 0)
//		{
//			double intersectionTime = -box.BoundingPlanes[ithFace].PlaneDot(origin) / testDir;
//			if (intersectionTime > 0)
//			{
//				vector3 intersectionPoint(origin + intersectionTime * dir);
//
//				if (intersectionPoint.x <= Max.x + EPSILON_FLOAT && intersectionPoint.x >= Min.x - EPSILON_FLOAT
//					&& intersectionPoint.y <= Max.y + EPSILON_FLOAT && intersectionPoint.y >= Min.y - EPSILON_FLOAT
//					&& intersectionPoint.z <= Max.z + EPSILON_FLOAT && intersectionPoint.z >= Min.z - EPSILON_FLOAT)
//				{
//					if (t < 0 || t > intersectionTime)
//					{
//						t = intersectionTime;
//					}
//				}
//			}
//		}
//	}
//	return t;
//}
//
//bool checkWithInBox (const vector3& pt, const vector3& lbb, const vector3& ruf)
//{
//	return (pt.x >= lbb.x && pt.y >= lbb.y && pt.z>= lbb.z && pt.x < ruf.x && pt.y < ruf.y && pt.z < ruf.z);
//}
//
//BOOL Box::TriangleInBox (const vector3& a, const vector3& b, const vector3& c) const
//{
//	vector3 pt[3];
//	pt[0] = a;
//	pt[1] = b;
//	pt[2] = c;
//
//	bool bInBox = true;
//	for (int i = 0; i < 3; i++)
//	{
//		bInBox &= checkWithInBox(pt[i], Min, Max);
//	}
//	if (bInBox)
//		return true;
//	for (int i = 0; i < 3; i++) 
//	{
//		vector3 start = pt[i];
//		vector3 end = pt[(i + 1) % 3];
//
//		vector3 dir = end - start;
//		double t = DotProduct(Min - start, vector3(-1, 0, 0)) / DotProduct(dir, vector3(-1, 0, 0)); //left face
//		{
//			if (t >= 0 && t < 1) 
//			{
//				vector3 crossPt = start + dir * t;
//				if (checkWithInBox(crossPt, Min, vector3(Min.x + EPSILON_FLOAT, Max.y, Max.z)))
//					return true;
//			}
//
//		}
//		t = DotProduct(Min - start, vector3(0, -1, 0)) / DotProduct(dir, vector3(0, -1, 0)); //bottom face
//		{
//			if (t >= 0 && t < 1) 
//			{
//				vector3 crossPt = start + dir * t;
//				if (checkWithInBox(crossPt, Min, vector3(Max.x, Min.y + EPSILON_FLOAT, Max.z)))
//					return true;
//			}
//
//		}
//		t = DotProduct(Min - start, vector3(0, 0, -1)) / DotProduct(dir, vector3(0, 0, -1)); //back face
//		{
//			if (t >= 0 && t < 1) 
//			{
//				vector3 crossPt = start + dir * t;
//				if (checkWithInBox(crossPt, Min, vector3(Max.x, Max.y, Min.z + EPSILON_FLOAT)))
//					return true;
//			}
//
//		}
//		t = DotProduct(Min - start, vector3(1, 0, 0)) / DotProduct(dir, vector3(1, 0, 0)); //right face
//		{
//			if (t >= 0 && t < 1) 
//			{
//				vector3 crossPt = start + dir * t;
//				if (checkWithInBox(crossPt, vector3(Max.x - EPSILON_FLOAT, Min.y, Min.z), Max))
//					return true;
//			}
//
//		}
//		t = DotProduct(Min - start, vector3(0, 1, 0)) / DotProduct(dir, vector3(0, 1, 0)); //up face
//		{
//			if (t >= 0 && t < 1) 
//			{
//				vector3 crossPt = start + dir * t;
//				if (checkWithInBox(crossPt, vector3(Min.x, Max.y - EPSILON_FLOAT, Min.z), Max))
//					return true;
//			}
//
//		}
//		t = DotProduct(Min - start, vector3(0, 0, 1)) / DotProduct(dir, vector3(0, 0, 1)); //front face
//		{
//			if (t >= 0 && t < 1) 
//			{
//				vector3 crossPt = start + dir * t;
//				if (checkWithInBox(crossPt, vector3(Min.x, Min.y, Max.z - EPSILON_FLOAT), Max))
//					return true;
//			}
//
//		}
//	}
//	return false;
//}
//
//ConvexVolume Box::GetVolume(void) const
//{
//	vector3 vertices[NUM_VERTICES_IN_BOX];
//	AllVerteices(vertices);
//	ConvexVolume volume;
//	volume.NumPlanes = MAX_FACES_BOX / 2; //calculate by quads instead of triangles, so / 2
//	volume.BoundingPlanes[0] = Plane(vertices[0], vertices[1], vertices[3]);
//	volume.BoundingPlanes[1] = Plane(vertices[4], vertices[7], vertices[6]);
//	volume.BoundingPlanes[2] = Plane(vertices[0], vertices[3], vertices[7]);
//	volume.BoundingPlanes[3] = Plane(vertices[1], vertices[5], vertices[6]);
//	volume.BoundingPlanes[4] = Plane(vertices[6], vertices[7], vertices[3]);
//	volume.BoundingPlanes[5] = Plane(vertices[0], vertices[4], vertices[5]);
//	return volume;
//}
//
//Plane Plane::TransformBy (const matrix44& matrix) const
//{
//	matrix33 M(vector3(matrix[0].x, matrix[0].y, matrix[0].z),
//			   vector3(matrix[1].x, matrix[1].y, matrix[1].z),
//			   vector3(matrix[2].x, matrix[2].y, matrix[2].z));
//	vector3 T(matrix[3].x, matrix[3].y, matrix[3].z);
//	matrix33 invM(M);
//	invM.invert();
//	vector3 invMT = -(invM * T);
//	matrix44 invF(vector4(invM[0].x, invM[0].y, invM[0].z, 0),
//				  vector4(invM[1].x, invM[1].y, invM[1].z, 0),
//				  vector4(invM[2].x, invM[2].y, invM[2].z, 0),
//				  vector4(invMT.x  , invMT.y  , invMT.z  , 1));
//	matrix44 invTransposeF(invF);
//	invTransposeF.transpose();
//	vector4 plane(this->Normal.x, this->Normal.y, this->Normal.z, this->W);
//	Plane transformed(invTransposeF * plane);
//
//	return transformed;
//}
//
//ConvexVolume ConvexVolume::TransformBy(const matrix44& matrix) const
//{
//	ConvexVolume transformed;
//	for (INT ithPlane = 0; ithPlane < NumPlanes; ithPlane++)
//	{
//		transformed.BoundingPlanes[ithPlane] = BoundingPlanes[ithPlane].TransformBy(matrix);
//	}
//	transformed.NumPlanes = NumPlanes;
//	return transformed;
//}
//
//
//BYTE ConvexVolume::BoxCheck(const Box& InBox)
//{
//	BYTE result = 0;
//	//vector3 boxVertices[NUM_VERTICES_IN_BOX];
//	//InBox.AllVerteices(boxVertices);
//	vector3 center = InBox.GetCenter();
//	vector3 extent = InBox.GetExtent();
//	double R = extent.length();
//	INT counter = 0;
//	for (int ithPlane = 0; ithPlane < NumPlanes; ithPlane++)
//	{
//		Plane boundingPlane = BoundingPlanes[ithPlane];
//		{
//			double signedDistance = boundingPlane.PlaneDot(center);
//			if (signedDistance >= 0 || -signedDistance <= R)
//			{
//				counter++;
//			}
//		}
//	}
//	if (counter == NumPlanes)
//		result |= CF_Inside;
//	else 
//		result |= CF_Outside;
//	return result;
//}
//
//BYTE ConvexVolume::BoxCheck(const vector3& Origin, const vector3& Extent)
//{
//	Box testBox(Origin - Extent, Origin + Extent);
//	return BoxCheck(testBox);
//}



Coords::Coords(void)
{
	for(int i = 0;i < 3;i++)
		for(int j = 0;j < 3;j++)
		{
			_base[i][j] = 0;
			_rev[i][j] = 0;
		}
		_base[1][1] = _base[0][0] = _base[2][2] = 1;
		_rev[1][1] = _rev[0][0] = _rev[2][2] = 1;
		_origin = vector3(0, 0, 0);
		_dirty = true;
}
Coords::Coords(const Coords& rhs) : _origin(rhs._origin), _determinant(rhs._determinant), _mat(rhs._mat)
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			_base[i][j] = rhs._base[i][j];
			_rev[i][j] = rhs._rev[i][j];
		}
	}
}

Coords& Coords::operator=(const Coords& rhs)
{
	if (this == &rhs) return *this;
	_origin = rhs._origin;
	_determinant = rhs._determinant;
	_mat = rhs._mat;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			_base[i][j] = rhs._base[i][j];
			_rev[i][j] = rhs._rev[i][j];
		}
	}
	return (*this);
}
Coords::Coords(const vector3& origin)
{
	_origin = origin;
	for(int i = 0;i < 3;i++)
		for(int j = 0;j < 3;j++)
		{
			_base[i][j] = 0;
			_rev[i][j] = 0;
		}
		_base[1][1] = _base[0][0] = _base[2][2] = 1;
		_rev[1][1] = _rev[0][0] = _rev[2][2] = 1;
		_dirty = true;
};

Coords::~Coords(void)
{
}

Coords::Coords(double base[][3], const vector3& origin,bool normalized /*= true*/)
{
	if(normalized)
		for(int i = 0;i < 3;i++)
			for(int j = 0;j < 3;j++)
				_base[i][j] = base[i][j];
	else 
		for(int i = 0;i < 3;i++)
		{
			double sum = sqrt(_base[i][0]*_base[i][0] + _base[i][1]*_base[i][1] + 
				_base[i][2]*_base[i][2]);
			for(int j = 0;j < 3;j++)
				_base[i][j] = base[i][j] / sum;
		}
		_origin = origin;
		_dirty = true;
}

void Coords::reflectXZPlane()
{
	_origin.y = -_origin.y;
	_base[0][1] = -_base[0][1];
	_base[1][1] = -_base[1][1];
	_base[2][1] = -_base[2][1];
	_dirty = true;
}

void Coords::Calculate()
{
	//calculate determinant and rev
	_determinant = _base[0][0]*_base[1][1]*_base[2][2] + _base[2][0]*_base[0][1]*_base[1][2]
	+ _base[0][2]*_base[1][0]*_base[2][1] - _base[0][2]*_base[1][1]*_base[2][0] 
	- _base[0][1]*_base[1][0]*_base[2][2] - _base[0][0]*_base[1][2]*_base[2][1];

	_rev[0][0] = (_base[2][2]*_base[1][1] - _base[1][2]*_base[2][1]) / _determinant;
	_rev[0][1] = -(_base[0][1]*_base[2][2] - _base[2][1]*_base[0][2]) / _determinant;
	_rev[0][2] = (_base[0][1]*_base[1][2] - _base[1][1]*_base[0][2]) / _determinant;

	_rev[1][0] = -(_base[1][0]*_base[2][2] - _base[2][0]*_base[1][2]) / _determinant;
	_rev[1][1] = (_base[0][0]*_base[2][2] - _base[0][2]*_base[2][0]) / _determinant;
	_rev[1][2] = -(_base[0][0]*_base[1][2] - _base[1][0]*_base[0][2]) / _determinant;

	_rev[2][0] = (_base[1][0]*_base[2][1] - _base[2][0]*_base[1][1]) / _determinant;
	_rev[2][1] = -(_base[0][0]*_base[2][1] - _base[2][0]*_base[0][1]) / _determinant;
	_rev[2][2] = (_base[0][0]*_base[1][1] - _base[0][1]*_base[1][0]) / _determinant;

	//matrix
	//double m[] = { _base[0][0], _base[1][0], _base[2][0], 0,
	//	_base[0][1], _base[1][1], _base[2][1], 0,
	//	_base[0][2], _base[1][2], _base[2][2], 0,
	//	_origin.x, _origin.y, _origin.z, 1
	//};
	double m[] = { _base[0][0], _base[0][1], _base[0][2], 0,
				  _base[1][0], _base[1][1], _base[1][2], 0,
				  _base[2][0], _base[2][1], _base[2][2], 0,
				  _origin.x, _origin.y, _origin.z, 1
	};
	_mat.set(m);
	_dirty = false;
}

vector3 Coords::ConvertToParent(const vector3& pt) const
{
	return (vector3(_origin.x + pt.x * _base[0][0] + pt.y * _base[1][0] + pt.z * _base[2][0],
		_origin.y + pt.x * _base[0][1] + pt.y * _base[1][1] + pt.z * _base[2][1],
		_origin.z + pt.x * _base[0][2] + pt.y * _base[1][2] + pt.z * _base[2][2]));
}

vector3 Coords::ConvertToLocal(const vector3& pt)
{
	if(_dirty)
		Calculate();
	vector3 offpt = pt - _origin;
  	return (vector3(offpt.x * _rev[0][0] + offpt.y * _rev[1][0] + offpt.z * _rev[2][0],
		offpt.x * _rev[0][1] + offpt.y * _rev[1][1] + offpt.z * _rev[2][1],
		offpt.x * _rev[0][2] + offpt.y * _rev[1][2] + offpt.z * _rev[2][2]));
	return offpt;
};

Coords Coords::ConvertToLocal(const Coords& frm)
{
	Coords retval;
	const vector3& frmO = frm.origin();
	retval._origin = ConvertToLocal(frm._origin);
	vector3 x(frm._base[0][0] + frmO.x, frm._base[0][1] + frmO.y,
		frm._base[0][2] + frmO.z);
	vector3 y(frm._base[1][0] + frmO.x, frm._base[1][1] + frmO.y, 
		frm._base[1][2] + frmO.z);
	vector3 z(frm._base[2][0] + frmO.x, frm._base[2][1] + frmO.y,
		frm._base[2][2] + frmO.z);
	x = ConvertToLocal(x);
	y = ConvertToLocal(y);
	z = ConvertToLocal(z);
	retval._base[0][0] = x.x;
	retval._base[0][1] = x.y;
	retval._base[0][2] = x.z;
	retval._base[1][0] = y.x;
	retval._base[1][1] = y.y;
	retval._base[1][2] = y.z;
	retval._base[2][0] = z.x;
	retval._base[2][1] = z.y;
	retval._base[2][2] = z.z;
	return retval;
};

Coords Coords::ConvertToParent(const Coords& frm)const
{
	Coords retval;
	
	retval._origin = ConvertToParent(frm._origin);
	const vector3& frmO = retval._origin;

	vector3 x(frm._base[0][0], frm._base[0][1], frm._base[0][2]);
	vector3 y(frm._base[1][0], frm._base[1][1], frm._base[1][2]);
	vector3 z(frm._base[2][0], frm._base[2][1], frm._base[2][2]);
	x = ConvertToParent(x) - retval._origin;
	y = ConvertToParent(y) - retval._origin;
	z = ConvertToParent(z) - retval._origin;
 	retval._base[0][0] = x.x;
	retval._base[0][1] = x.y;
	retval._base[0][2] = x.z;
	retval._base[1][0] = y.x;
	retval._base[1][1] = y.y;
	retval._base[1][2] = y.z;
	retval._base[2][0] = z.x;
	retval._base[2][1] = z.y;
	retval._base[2][2] = z.z;
	return retval;
};


//x base[i][0]
//y base[i][1]
//z base[i][2]
void Coords::RotateAlongGlobalAxis(double angle, AXIS axis)
{
	switch (axis){
	case AXIS_X:
		for(int i = 0;i < 3;i++)
		{
			//old angle
			double alpha = GetAngle(_base[i][1], _base[i][2]);
			double radius = sqrt(_base[i][2]*_base[i][2] + _base[i][1]*_base[i][1]);
			alpha += angle;
			_base[i][1] = radius*cos(alpha);
			_base[i][2] = radius*sin(alpha);
		}
		break;
	case AXIS_Y:
		for(int i = 0;i < 3;i++)
		{
			//old angle
			double alpha = GetAngle(_base[i][2], _base[i][0]);
			double radius = sqrt(_base[i][2]*_base[i][2] + _base[i][0]*_base[i][0]);
			alpha += angle;
			_base[i][2] = radius*cos(alpha);
			_base[i][0] = radius*sin(alpha);
		}
		break;
	case AXIS_Z:
		for(int i = 0;i < 3;i++)
		{
			//old angle
			double alpha = GetAngle(_base[i][0], _base[i][1]);
			double radius = sqrt(_base[i][0]*_base[i][0] + _base[i][1]*_base[i][1]);
			alpha += angle;
			_base[i][0] = radius*cos(alpha);
			_base[i][1] = radius*sin(alpha);
		}
		break;
	}
	_dirty = true;
}

void Coords::RotateAlongLocalAxis(double angle, AXIS axis)
{

	double cosangle = cos(angle);
	double sinangle = sin(angle);

	switch (axis){

	case AXIS_X:
		{
			vector3 vec_y(0.0, cosangle, sinangle);
			vector3 vec_z(0.0, -sinangle, cosangle);
			vec_y = this->ConvertToParent(vec_y) - _origin;
			vec_z = this->ConvertToParent(vec_z) - _origin;
			_base[1][0] = vec_y.x;
			_base[1][1] = vec_y.y;
			_base[1][2] = vec_y.z;
			_base[2][0] = vec_z.x;
			_base[2][1] = vec_z.y;
			_base[2][2] = vec_z.z;
		}
		break;
	case AXIS_Y:
		{
			vector3 vec_x(cosangle, 0.0, -sinangle);
			vector3 vec_z(sinangle, 0.0, cosangle);
			vec_x = this->ConvertToParent(vec_x) - _origin;
			vec_z = this->ConvertToParent(vec_z) - _origin;
			_base[0][0] = vec_x.x;
			_base[0][1] = vec_x.y;
			_base[0][2] = vec_x.z;
			_base[2][0] = vec_z.x;
			_base[2][1] = vec_z.y;
			_base[2][2] = vec_z.z;
		}
		break;
	case AXIS_Z:
		{
			vector3 vec_x(cosangle, sinangle, 0.0);
			vector3 vec_y(-sinangle, cosangle, 0.0);
			vec_x = this->ConvertToParent(vec_x) - _origin;
			vec_y = this->ConvertToParent(vec_y) - _origin;
			_base[0][0] = vec_x.x;
			_base[0][1] = vec_x.y;
			_base[0][2] = vec_x.z;
			_base[1][0] = vec_y.x;
			_base[1][1] = vec_y.y;
			_base[1][2] = vec_y.z;
		}
		break;
	}
	_dirty = true;
};


void Coords::RotateAlongGlobalVector(double angle, vector3 vec)
{
	vector4 baseX(_base[0][0], _base[0][1], _base[0][2], 0);
	vector4 baseY(_base[1][0], _base[1][1], _base[1][2], 0);
	vector4 baseZ(_base[2][0], _base[2][1], _base[2][2], 0);
	DecoQuaternion q(angle, vec);
	baseX = q.Rotate(baseX);
	baseY = q.Rotate(baseY);
	baseZ = q.Rotate(baseZ);
	_base[0][0] = baseX.x;
	_base[0][1] = baseX.y;
	_base[0][2] = baseX.z;
	_base[1][0] = baseY.x;
	_base[1][1] = baseY.y;
	_base[1][2] = baseY.z;
	_base[2][0] = baseZ.x;
	_base[2][1] = baseZ.y;
	_base[2][2] = baseZ.z;
	_dirty = TRUE;
}
vector3 Coords::origin()const
{
	return _origin;
}

void Coords::SetOrigin(const vector3 &origin)
{
	_origin = origin;
	_dirty = true;
}

void Coords::Translate(const vector3 &trans)
{
	SetOrigin(_origin + trans);
}

void Coords::TranslateLocal(const vector3& trans)
{
	SetOrigin(_origin + trans.x * vector3(_base[0][0], _base[0][1], _base[0][2]) + trans.y * vector3(_base[1][0], _base[1][1], _base[1][2]) + trans.z * vector3(_base[2][0], _base[2][1], _base[2][2]));
}

matrix44 Coords::GetMatrix()
{
	if(_dirty)
		Calculate();
	return _mat;
}

double Coords::GetAngle(double x, double y)
{
	if(x == 0 && y == 0)
		return 0;
	if(x == 0)
	{
		if(y > 0)
			return PI/2.0f;
		else
			return PI*1.5f;
	}
	else if(y == 0)
	{
		if(x > 0)
			return 0;
		else
			return PI;
	}
	else
	{
		double theta = atan(y/x);
        if(x > 0)
		{
			if(y > 0)	return theta;
			else        return (2*PI + theta);
		}
		else
		{
			return (PI + theta);			
		}

	}
}

void Coords::Set(const vector3& x, const vector3& y, const vector3& z, const vector3& o, bool normalized/* = true*/)
{
	_base[0][0] = x.x;
	_base[0][1] = x.y;
	_base[0][2] = x.z;
	_base[1][0] = y.x;
	_base[1][1] = y.y;
	_base[1][2] = y.z;
	_base[2][0] = z.x;
	_base[2][1] = z.y;
	_base[2][2] = z.z;
	_origin = o;

	if(!normalized)
	{
		for(int i = 0;i < 3;i++)
		{
			double sum = sqrt(_base[i][0]*_base[i][0] + _base[i][1]*_base[i][1] + 
				_base[i][2]*_base[i][2]);
			for(int j = 0;j < 3;j++)
				_base[i][j] = _base[i][j] / sum;
		}
	}
}

void Coords::GetBase(vector3& x, vector3& y, vector3& z)const
{
	x.set(_base[0][0], _base[0][1], _base[0][2]);
	y.set(_base[1][0], _base[1][1], _base[1][2]);
	z.set(_base[2][0], _base[2][1], _base[2][2]);
}

vector3 Coords::GetYParentProjection()const
{
	double x = -_base[2][0], z = -_base[2][2];
	double mag = sqrt(x*x + z*z);
	
	return (mag == 0) ? vector3(0,0,0) : vector3(x/mag, 0, z/mag);
}

// added by bovine
unsigned short CeilPower2(unsigned short x)
{
	if (x == 0)
		return 1;
	x |= x >> 1;
	x |= x >> 2;
	x |= x >> 4;
	x |= x >> 8;
	return x + 1;
}
//Box Bounding(vector3* array, size_t num)
//{
//	Box bb(vector3(MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE), vector3(MIN_DOUBLE, MIN_DOUBLE, MIN_DOUBLE));
//	for (size_t ithVertex = 0; ithVertex < num; ++ithVertex)
//	{
//		if (array[ithVertex].x > bb[1].x)
//			bb[1].x = array[ithVertex].x;
//		if (array[ithVertex].x < bb[0].x)
//			bb[0].x = array[ithVertex].x;
//
//		if (array[ithVertex].y > bb[1].y)
//			bb[1].y = array[ithVertex].y;
//		if (array[ithVertex].y < bb[0].y)
//			bb[0].y = array[ithVertex].y;
//
//		if (array[ithVertex].z > bb[1].z)
//			bb[1].z = array[ithVertex].z;
//		if (array[ithVertex].z < bb[0].z)
//			bb[0].z = array[ithVertex].z;
//	}
//	return bb;
//}

BOOL TriangleRayIntersection (const vector3& a, const vector3& b, const vector3& c, const vector3& origin,const vector3& dir, vector3 *intersection, double* time)
{
	double A;
	double t, beta, garma;

	matrix33 m;

	m.col[0].x = a.x - b.x; m.col[0].y = a.y - b.y; m.col[0].z = a.z - b.z;
	m.col[1].x = a.x - c.x; m.col[1].y = a.y - c.y; m.col[1].z = a.z - c.z;
	m.col[2].x = dir.x;		m.col[2].y = dir.y;	    m.col[2].z = dir.z;
	A = m.determinant();
	if (!A) return false;


	m.col[0].x = a.x - origin.x; m.col[0].y = a.y - origin.y; m.col[0].z = a.z - origin.z;
	m.col[1].x = a.x - c.x; m.col[1].y = a.y - c.y; m.col[1].z = a.z - c.z;
	m.col[2].x = dir.x;		m.col[2].y = dir.y;	    m.col[2].z = dir.z;
	beta = m.determinant() / A;
	if (beta <= -EPSILON_FLOAT) return false;

	m.col[0].x = a.x - b.x; m.col[0].y = a.y - b.y; m.col[0].z = a.z - b.z;
	m.col[1].x = a.x - origin.x; m.col[1].y = a.y - origin.y; m.col[1].z = a.z - origin.z;
	m.col[2].x = dir.x;		m.col[2].y = dir.y;	    m.col[2].z = dir.z;
	garma = m.determinant() / A;

	if (garma <= -EPSILON_FLOAT || beta + garma >= 1 + EPSILON_FLOAT) return false;

	m.col[0].x = a.x - b.x; m.col[0].y = a.y - b.y; m.col[0].z = a.z - b.z;
	m.col[1].x = a.x - c.x; m.col[1].y = a.y - c.y; m.col[1].z = a.z - c.z;
	m.col[2].x = a.x - origin.x;	m.col[2].y = a.y - origin.y; m.col[2].z = a.z - origin.z;
	t = m.determinant() / A;
	*intersection = origin + t * dir;
	*time = t;
	return true;
} 

BOOL PointInTriangle (const vector3& pt, const vector3& a, const vector3& b, const vector3& c)
{
	vector3 ptToCorner[3];

	ptToCorner[0] = a - pt;
	ptToCorner[1] = b - pt;
	ptToCorner[2] = c - pt;
	
	for (UINT i = 0; i < 3; i++)
	{
		if (ptToCorner[i].lengthSqr() <= EPSILON_FLOAT) 
			return TRUE;
		ptToCorner[i].normalize();
	}

	double theta = 0;

	for (int i = 0; i < 3; i++)
	{
		theta += acos(DotProduct(ptToCorner[i], ptToCorner[(i + 1) % 3]));
	}
	if (signedAbs(theta - 2 * PI) <= EPSILON_FLOAT) return TRUE;
	else return FALSE;

}

BOOL PointInLine (const vector3& pt, const vector3& a, const vector3& b, BOOL bSegment)
{
	vector3 dir1 = pt - a;
	vector3 dir2 = b - a;
	if (NearlyEquals(dir1, vector3(0, 0, 0), EPSILON_FLOAT))
		return TRUE;
	if (NearlyEquals(dir2, vector3(0, 0, 0), EPSILON_FLOAT))
		return FALSE;
	dir1.normalize();
	dir2.normalize();
	if (signedAbs(DotProduct(dir1, dir2) + 1) < EPSILON_FLOAT)
	{
		if (!bSegment)
			return TRUE;
		else
			return FALSE;
	}
	else if (signedAbs(DotProduct(dir1, dir2) - 1) < EPSILON_FLOAT)
	{
		if (!bSegment)
			return TRUE;
		else
		{
			if ((pt - a).lengthSqr() <= (b - a).lengthSqr())
				return TRUE;
			else
				return FALSE;
		}
	}
	else
		return FALSE;
}

double PointRayDistance(const Eigen::Vector3f& pt, const Eigen::Vector3f& origin, const Eigen::Vector3f& dir, double& rayDepth)
{
	vector3 point(pt[0], pt[1], pt[2]);
	vector3 o(origin[0], origin[1], origin[2]);
	vector3 d(dir[0], dir[1], dir[2]);
	return PointRayDistance(point, o, d, rayDepth);
}

DOUBLE PointRayDistance(const vector3& pt, const vector3& origin, const vector3& dir, double& rayDepth)
{
	vector3 OP = pt - origin;
	vector3 normalizeDir = dir;
	normalizeDir.normalize();
	double projDist = CrossProduct(OP, normalizeDir).length();
	rayDepth = DotProduct(OP, normalizeDir);
	if (rayDepth >= 0)
	{
		
		return projDist;
	}
	else
	{
		return OP.length();
	}
}

DOUBLE PointLineDistance (const vector3& pt, const vector3& a, const vector3& b, BOOL bSegment)
{
	vector3 dir = b - a;
	vector3 projPoint = a + DotProduct(pt - a, dir) / DotProduct(dir, dir) * dir;	
	double result = (pt - a).lengthSqr() - (projPoint - a).lengthSqr();

	if (PointInLine(projPoint, a, b))
		return sqrt((pt - a).lengthSqr() - (projPoint - a).lengthSqr());
	else
	{
		return min((pt - a).length(), (pt - b).length());
	}
}

DOUBLE PointPolylineDistance(const vector3& pt, const vector<vector3>& polyline, int& segmentIdx)
{
	int numSegment = static_cast<int>(polyline.size() - 1);
	DOUBLE minDistance = 1e30;
	for (int i = 0; i < numSegment; ++i)
	{
		DOUBLE distance = PointLineDistance(pt, polyline[i], polyline[i + 1], TRUE);
		if (distance < minDistance)
		{
			minDistance = distance;
			segmentIdx = i;
		}
	}
	return minDistance;
}

//DOUBLE PointTriangleDistance(const vector3& pt, const vector3& a, const vector3& b, const vector3& c)
//{
//	Plane p(a, b, c);
//	vector3 Normal = p.GetNormal();
//	DOUBLE Offset = p.GetOffset();
//	vector3 intersectPt = p.RayIntersect(pt, Normal);
//	if (PointInTriangle(intersectPt, a, b, c))
//	{
//		return (pt - intersectPt).length();
//	}
//	else
//	{
//		DOUBLE minDist = min(PointLineDistance(pt, a, b), PointLineDistance(pt, b, c));
//		minDist = min(minDist, PointLineDistance(pt, c, a));
//		return minDist;
//	}
//}

//DOUBLE RayPolylineDistance(const vector3& origin, const vector3& dir, const vector<Eigen::Vector3d>& polyLine, double& depth)
//{
//    int numSegments = static_cast<int>(polyLine.size()) - 1;
//    double minDist = 1e20;
//    for (int i = 0; i < numSegments; ++i)
//    {
//        double rayDepth;
//        double distance = RayLineDistance(origin, dir, vector3(polyLine[i]), vector3(polyLine[i + 1]), rayDepth, true);
//        if (distance < minDist)
//        {
//            minDist = distance;
//            depth = rayDepth;
//        }
//    }
//    return minDist;
//}

//DOUBLE RayLineDistance(const vector3& origin, const vector3& dir, const vector3& a, const vector3& b, double& depth, bool isLineSegmen)
//{
//    vector3 ab = b - a;
//    vector3 n = CrossProduct(dir, ab);
//    n.normalize();
//    vector3 ob = b - origin;
//    double dist = DotProduct(ob, n);
//    Plane p(a, dir);
//    double distanceAlongDirToNearestPoint = p.PlaneDot(origin);
//    vector3 nearestPoint = origin - distanceAlongDirToNearestPoint * dir;
//    //if (abs(dir.x) > EPSILON_FLOAT)
//    //    depth = (nearestPoint - origin).x / dir.x;
//    //else if (abs(dir.y) > EPSILON_FLOAT)
//    //    depth = (nearestPoint - origin).y / dir.y;
//    //else 
//    //    depth = (nearestPoint - origin).z / dir.z;
//    depth = -distanceAlongDirToNearestPoint;
//    if (depth <= 0)
//        return PointLineDistance(origin, a, b, isLineSegmen);
//    else
//    {
//        vector3 planeNormal = ab;
//        planeNormal.normalize();
//        Plane pl(nearestPoint, planeNormal);
//        if (pl.PlaneDot(a) * pl.PlaneDot(b) < 0)
//            return abs(dist);
//        else
//        {
//            double depth1, depth2;
//            double dist1 = PointRayDistance(a, origin, dir, depth1);
//            double dist2 = PointRayDistance(b, origin, dir, depth2);
//            if (dist1 < dist2)
//            {
//                depth = depth1;
//                return (dist1);
//            }
//            else
//            {
//                depth = depth2;
//                return (dist2);
//            }
//        }
//    }
//}

void indexTo2d(int index, int& xIndex, int& yIndex, int numX, int numY)
{
	assert(index >= 0 && index < numX * numY);
	xIndex = index / (numY);
	yIndex = index % numY;
}

int indexTo1d(int xIndex, int yIndex, int numX, int numY)
{
	assert(xIndex >= 0 && xIndex < numX);
	assert(yIndex >= 0 && yIndex < numY);
	return xIndex * numY + yIndex;
}

void indexTo3d (INT index, UINT& xIndex, UINT& yIndex, UINT& zIndex, UINT numX, UINT numY, UINT numZ)
{
	assert (index >= 0 && (UINT)index < numX * numY * numZ);
	xIndex = index / (numY * numZ);
	UINT remainder = index % (numY * numZ);
	yIndex = remainder / numZ;
	zIndex = remainder % numZ;
}

UINT indexTo1d (INT xIndex, INT yIndex, INT zIndex, UINT numX, UINT numY, UINT numZ)
{
	assert(xIndex >= 0 && (UINT)xIndex < numX);
	assert(yIndex >= 0 && (UINT)yIndex < numY);
	assert(zIndex >= 0 && (UINT)zIndex < numZ);
	return xIndex * (numY * numZ) + yIndex * numZ + zIndex;
}

UINT round2Pow2(UINT num)
{
	if (num & (num - 1))
	{
		INT ithBit = 0;
		for (ithBit = 1; ithBit < 8 * sizeof(UINT); ithBit++)
		{
			if ((num >> ithBit) == 1)
				break;
		}
		return 1 << (ithBit + 1);
	}
	else
	{
		return num;
	}
}

vector3 TransformPoint(const matrix44& matrix, const vector3& point)
{
	vector4 homoPoint(point);
	homoPoint.w = 1;
	homoPoint = matrix * homoPoint;
	return vector3(homoPoint.x, homoPoint.y, homoPoint.z);
}

vector3 TransformNormal(const matrix44& matrix, const vector3& norm)
{
	vector4 homoNormal(norm);
	homoNormal.w = 0;

	matrix44 normalTransformMatrix = matrix;
	normalTransformMatrix.invert().transpose();
	vector4 transformedNormal = normalTransformMatrix * homoNormal;
	return vector3(transformedNormal.x, transformedNormal.y, transformedNormal.z);
}

vector3 TransformVector(const matrix44& matrix, const vector3& vec)
{
	vector4 homoPoint(vec);
	homoPoint.w = 0;
	homoPoint = matrix * homoPoint;
	return vector3(homoPoint.x, homoPoint.y, homoPoint.z);
}

vector3 BaryCenter(const vector<vector3>& verts)
{
	INT vertNum = static_cast<INT>(verts.size());
	vector3 sum(0, 0, 0);
	for (INT i = 0; i < vertNum; i++)
	{
		sum += verts[i];
	}
	sum /= vertNum;
	return sum;
}



double RandDouble(double low, double high)
{
	double temp;

	/* swap low & high around if the user makes no sense */
	if (low > high)
	{
		temp = low;
		low = high;
		high = temp;
	}

	/* calculate the random number & return it */
	temp = (rand() / (static_cast<double>(RAND_MAX) + 1.0))
		* (high - low) + low;
	return temp;
}

float TriangleArea(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c)
{
	Eigen::Vector3f ab = b - a;
	Eigen::Vector3f ac = c - a;
	Eigen::Vector3f cross = ab.cross(ac);
	return 0.5f * cross.norm();
}
Eigen::Vector3f TriangleNormal(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c)
{
	Eigen::Vector3f ab = b - a;
	Eigen::Vector3f ac = c - a;
	Eigen::Vector3f cross = ab.cross(ac);
	return cross.normalized();
}
void RandomPointInTriangle(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c, const Eigen::Vector3f& na, const Eigen::Vector3f& nb, const Eigen::Vector3f& nc, Eigen::Vector3f* p, Eigen::Vector3f* n)
{
	float r1 = static_cast<float>(RandDouble(0, 1));
	float r2 = static_cast<float>(RandDouble(0, 1));
	float w1 = (1 - sqrt(r1));
	float w2 = (sqrt(r1) * (1 - r2));
	float w3 = (sqrt(r1) * r2);
	*p = w1 * a + w2 * b + w3 * c;
	*n = w1 * na + w2 * nb + w3 * nc;
	n->normalize();
}

double TriangleArea(const vector3& a, const vector3& b, const vector3& c)
{
	vector3 ab = b - a;
	vector3 ac = c - a;
	vector3 cross = CrossProduct(ab, ac);
	return 0.5 * cross.length();
}
vector3 TriangleNormal(const vector3& a, const vector3& b, const vector3& c)
{
	vector3 ab = b - a;
	vector3 ac = c - a;
	vector3 cross = CrossProduct(ab, ac);
	return cross.normalize();
}

Eigen::MatrixXd GetTangentialDirs(const vector3& normal, int numDirs)
{
	Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(3, numDirs);
	vector3 y = normal;
	y.normalize();
	vector3 x = vector3(1, 0, 0);
	if (CrossProduct(x, y).lengthSqr() < EPSILON_FLOAT)
		x = vector3(0, 0, 1);
	vector3 z = CrossProduct(x, y).normalize();
	x = CrossProduct(y, z).normalize();
	matrix33 transformation(x, y, z);
	double step = 2 * PI / numDirs;
	for (int i = 0; i < numDirs; ++i)
	{
		vector3 dir(cos(i * step), 0, sin(i * step));
		vector3 transformedDir = transformation * dir;
		ret(0, i) = transformedDir.x;
		ret(1, i) = transformedDir.y;
		ret(2, i) = transformedDir.z;
		CHECK_NEAR(DotProduct(normal, transformedDir), 0, 1e-6);
	}
	return ret;
}

matrix33 FormulateSkewSymmetricMatrix(const vector3& vec)
{
    matrix33 ret;
    ret[0][0] = 0;
    ret[1][0] = -vec.z;
    ret[2][0] = vec.y;

    ret[0][1] = vec.z;
    ret[1][1] = 0;
    ret[2][1] = -vec.x;

    ret[0][2] = -vec.y;
    ret[1][2] = vec.x;
    ret[2][2] = 0;

    return ret;
}

Eigen::Matrix3d FormulateSkewSymmetricMatrix(const Eigen::Vector3d& vec)
{
	Eigen::Matrix3d ret;
	ret(0, 0) = 0;
	ret(0, 1) = -vec[2];
	ret(0, 2) = vec[1];
	  
	ret(1, 0) = vec[2];
	ret(1, 1) = 0;
	ret(1, 2) = -vec[0];
	  
	ret(2, 0) = -vec[1];
	ret(2, 1) = vec[0];
	ret(2, 2) = 0;

	return ret;
}

bool NearlyEqual(double a, double b, double threshold)
{
    return (abs(a - b) < threshold);
}


matrix33 ShapeMatch(const vector<vector3>& vertices1, const vector<vector3>& vertices2)
{
	vector3 comRest(0, 0, 0);
	vector3 comContact(0, 0, 0);
	int numVertices = static_cast<int>(vertices1.size());
	CHECK(numVertices == vertices2.size());

	vector<vector3> v1 = vertices1;
	vector<vector3> v2 = vertices2;
	for (int i = 0; i < numVertices; ++i)
	{
		comRest += v1[i];
		comContact += v2[i];
	}
	comRest /= numVertices;
	comContact /= numVertices;

	for (int i = 0; i < numVertices; ++i)
	{
		v1[i] -= comRest;
		v2[i] -= comContact;
	}
	matrix33 ret;
	ret.zero();
	for (int i = 0; i < numVertices; ++i)
	{
		ret += OuterProduct1(v2[i], v1[i]);
	}

	Eigen::Matrix3d R, S, transformation;
	transformation << ret[0][0], ret[1][0], ret[2][0],
		ret[0][1], ret[1][1], ret[2][1],
		ret[0][2], ret[1][2], ret[2][2];
	PolarDecomposition::DoPolarDecomposition(transformation, R, S);
	
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			ret[i][j] = R(j, i);
		}
	}
	return ret;
}
double ComputeWeightFromLWLinearRegression(double dist, double maxDist)
{
	const double tau = 1.0 / 3.0;
	double normalizedDist = dist / maxDist;
	double weight = exp(-normalizedDist * normalizedDist / (2 * tau * tau));
	return weight;
}

//DecoArchive& operator<< (DecoArchive& Ar, const Box& b)
//{
//	return Ar << b.Min << b.Max;
//}
//DecoArchive& operator>> (DecoArchive& Ar, Box& b)
//{
//	return Ar >> b.Min >> b. Max;
//}
DecoArchive& operator<< (DecoArchive& Ar, const DecoQuaternion& q)
{
	return Ar << q.m_v << q.m_w;
}
DecoArchive& operator>> (DecoArchive& Ar, DecoQuaternion& q)
{
	return Ar >> q.m_w >> q.m_w;
}
//DecoArchive& operator<< (DecoArchive& Ar, const Plane& p)
//{
//	return Ar << p.Normal << p.W;
//}
//DecoArchive& operator>> (DecoArchive& Ar, Plane& p)
//{
//	return Ar >> p.Normal >> p.W;
//}
//DecoArchive& operator<< (DecoArchive& Ar, const ConvexVolume& cv)
//{
//	Ar << cv.NumPlanes;
//	for (INT i = 0; i < cv.NumPlanes; i++)
//		Ar << cv.BoundingPlanes[i];
//	return Ar;
//}
//DecoArchive& operator>> (DecoArchive& Ar, ConvexVolume& cv)
//{
//	Ar >> cv.NumPlanes;
//	for (INT i = 0; i < cv.NumPlanes; i++)
//		Ar >> cv.BoundingPlanes[i];
//	return Ar;
//}
DecoArchive& operator<< (DecoArchive& Ar, const Coords& coords)
{
	Ar << coords._origin;
	Ar << coords._determinant;
	Ar << coords._mat;
	for (UINT i = 0; i < 3; i++)
	{
		for (UINT j = 0; j < 3; j++)
		{
			Ar << coords._base[i][j];
			Ar << coords._rev[i][j];
		}
	}
	return Ar;
}
DecoArchive& operator>> (DecoArchive& Ar, Coords& coords)
{
	Ar >> coords._origin;
	Ar >> coords._determinant;
	Ar >> coords._mat;
	for (UINT i = 0; i < 3; i++)
	{
		for (UINT j = 0; j < 3; j++)
		{
			Ar >> coords._base[i][j];
			Ar >> coords._rev[i][j];
		}
	}
	return Ar;
}

vector<int> Index1DToND(int id, const vector<int>& lenPerDim)
{
	int numDim = static_cast<int>(lenPerDim.size());
	vector<int> ret;
	if (numDim == 1)
	{
		ret.push_back(id);
		return ret;
	}
	int divider = 1;
	for (int i = 1; i < numDim; ++i)
	{
		divider *= lenPerDim[i];
	}
	for (int i = 1; i < numDim; ++i)
	{
		int idPerDim = id / divider;
		id %= divider;

		ret.push_back(idPerDim);
		divider /= lenPerDim[i];
	}

	ret.push_back(id);
	return ret;
}
int IndexNDTo1D(const vector<int>& indices, const vector<int>& lenPerDim)
{
	int numDim = static_cast<int>(lenPerDim.size());
	int ret = indices[0];
	for (int i = 1; i < numDim; ++i)
	{
		ret = ret * lenPerDim[i] + indices[i];
	}
	return ret;
}

double sigmod(double x)
{
	return 1.0 / (1.0 + exp(-x));
}

void PCAOnPoints(const vector<Eigen::Vector3f>& points, Eigen::Vector3f& mean, vector<Eigen::Vector3f>& axis)
{
	axis.resize(3);
	int numPoints = static_cast<int>(points.size());
	Eigen::MatrixXf dataMat = Eigen::MatrixXf(numPoints, 3);
	for (int i = 0; i < numPoints; ++i)
	{
		dataMat.row(i) = points[i];
	}
	mean = dataMat.colwise().mean();
	Eigen::MatrixXf centered = dataMat.rowwise() - mean.transpose();
	Eigen::MatrixXf cov = centered.transpose() * centered;

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> es;

	es.compute(cov);
	axis[0] = es.eigenvectors().col(0);
	axis[1] = es.eigenvectors().col(1);
	axis[2] = es.eigenvectors().col(2);
}