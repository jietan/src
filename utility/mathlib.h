#ifndef MATHLIB_H
#define MATHLIB_H

#include "mtxlib.h"
#include <algorithm>
#include <vector>
#include "Eigen/Dense"
using namespace std;

#define LinearInterpolate(value1,value2,alpha) ( ( 1.0 - alpha ) * value1 + alpha * value2 )


const static DOUBLE EPSILON_FLOAT = 1e-6;
const static DOUBLE MAX_DOUBLE = 1e300;
const static DOUBLE MIN_DOUBLE = -1e300;

const static INT MAX_FACES_BOX = 12;
const static INT NUM_VERTICES_PER_FACE = 3;
const static INT NUM_VERTICES_IN_BOX = 8;
const static INT MAX_FACES_PLANE = 2;
const static INT NUM_VERTICES_IN_PLANE = 4;

//  [4/9/2008 HATEVOL] ADDED start
//const static __int32 INV_SIGN_FLOAT = 0x7fffffff;
//const static __int64 INV_SIGN_DOUBLE = 0x7fffffffffffffff;
//  [4/9/2008 HATEVOL] ADDED end

class ConvexVolume;

//  [4/9/2008 HATEVOL] ADDED start
inline FLOAT signedAbs(const FLOAT& _X)
{
//#ifdef USE_ASM
//	T fOut = 0;
//	__asm
//	{
//		MOV EAX, _X;
//		SHL EAX, 1; //set the sign bit to 0 by shift left then right
//		SHR EAX, 1;
//		MOV fOut, EAX;
//	}
//	return fOut;
//#else
//	unsigned int* temp = (unsigned int*)&_X;
//	unsigned int out = *temp;
//
//	out = out << 1;
//	out = out >> 1;
//
//	return *((FLOAT*)&out);
//#endif
	return abs(_X);
/* #ifdef USE_ASM */
/* 	FLOAT fOut; */
/* 	__asm */
/* 	{ */
/* 		MOV EAX, _X; */
/* 		AND EAX, INV_SIGN_FLOAT; //set the sign bit to 0 by AND */
/* 		MOV fOut, EAX; */
/* 	} */
/* 	return fOut; */
/* #else */
/* 	__int32* temp = (__int32*)&_X; */
/* 	__int32 out = *temp & INV_SIGN_FLOAT; */
/* 	return *((FLOAT*)&out); */
/* #endif */

}

inline DOUBLE signedAbs(const DOUBLE& _X)
{
//#ifdef USE_ASM
//	T fOut = 0;
//	__asm
//	{
//		MOV EAX, _X;
//		SHL EAX, 1; //set the sign bit to 0 by shift left then right
//		SHR EAX, 1;
//		MOV fOut, EAX;
//	}
//	return fOut;
//#else
//	unsigned int* temp = (unsigned int*)&_X;
//	unsigned int out = *temp;
//
//	out = out << 1;
//	out = out >> 1;
//	DOUBLE d = *((DOUBLE*)&out);
//
//	return *((DOUBLE*)&out);
//#endif
	return abs(_X);
/* #ifdef USE_ASM */
/* 	DOUBLE dOut; */
/* 	__asm */
/* 	{ */
/* 		MOV EAX, _X; */
/* 		AND EAX, INV_SIGN_DOUBLE; //set the sign bit to 0 by AND */
/* 		MOV fOut, EAX; */
/* 	} */
/* 	return dOut; */
/* #else */
/* 	__int64* temp = (__int64*)&_X; */
/* 	__int64 out = *temp & INV_SIGN_DOUBLE; */
/* DOUBLE d = *((DOUBLE*)&out); */
/* 	return *((DOUBLE*)&out); */
/* #endif */

}

inline INT signedAbs(INT iNum )
{
	return abs(iNum);
#ifdef USE_ASM
	INT iOut = 0;
	__asm
	{
		MOV EAX, iNum;
		MOV EDX, EAX;
		SAR EDX, 31;   //all of edx's bit are eax's sign bit: 000.. or 111
		XOR EAX, EDX; //this interesting algorithm help to avoid "if else" structure
		SUB EAX, EDX;
		MOV iOut, EAX;
	}
	return iOut;
#else

	INT out = iNum;
	INT temp = iNum;
	temp = temp >> 31;

	out = out ^ temp;
	out = out - temp;

	return out;

#endif
}
//  [4/9/2008 HATEVOL] ADDED end

template <typename T>
T Clamp(const T& value, const T& min, const T& max)
{
	if (value > max)
	{
		return max;
	} 
	else if (value < min)
	{
		return min;
	}
	else
	{
		return value;
	}
}

//template <typename T>
//T LinearInterpolate(const T& value1, const T& value2, DOUBLE alpha)
//{
//	assert(alpha >= 0.0 && alpha <= 1.0);
//	//  [4/7/2008 HATEVOL] ANNOTATION start
//	//if (abs(alpha) < EPSILON_FLOAT)
//	//	return value1;
//	//else if (abs(alpha - 1) < EPSILON_FLOAT)
//	//	return value2;
//	//else
//	//	return (1.0 - alpha) * value1 + alpha * value2;
//	//  [4/7/2008 HATEVOL] ANNOTATION end
//
//	//  [4/7/2008 HATEVOL] ADDED start
//	return ( 1.0 - alpha ) * value1 + alpha * value2;
//	//  [4/7/2008 HATEVOL] ADDED end
//}

template <typename T>
T BilinearInterpolate(const T& xValue1, const T& xValue2, const T& yValue1, const T& yValue2, DOUBLE xAlpha, DOUBLE yAlpha)
{
	//assert(xAlpha >= 0.0 && xAlpha <= 1.0 && yAlpha >= 0.0 && yAlpha <= 1.0);
	T xBlend = LinearInterpolate(xValue1, xValue2, xAlpha); //xAlpha * xValue1 + (1.f - xAlpha) * xValue2;
	T yBlend = LinearInterpolate(yValue1, yValue2, xAlpha);//xAlpha * yValue1 + (1.f - xAlpha) * yValue2;
	return LinearInterpolate(xBlend, yBlend, yAlpha);//yAlpha * xBlend + (1.f - yAlpha) * yBlend;
}

template <typename T>
T TrilinearInterpolate(const T& xValue1, const T& xValue2, const T& xValue3, const T& xValue4, 
					   const T& yValue1, const T& yValue2, const T& yValue3, const T& yValue4, 
					   DOUBLE xAlpha, DOUBLE yAlpha, DOUBLE zAlpha)
{
	//assert(xAlpha >= 0.0 && xAlpha <= 1.0 && yAlpha >= 0.0 && yAlpha <= 1.0 && zAlpha >= 0.0 && zAlpha <= 1.0);
	T xBlend = BilinearInterpolate(xValue1, xValue2, xValue3, xValue4, xAlpha, yAlpha);
	T yBlend = BilinearInterpolate(yValue1, yValue2, yValue3, yValue4, xAlpha, yAlpha);
	return LinearInterpolate(xBlend, yBlend, zAlpha);
}

class Index3
{
public:
	SHORT m_i, m_j, m_k;
	//INT m_xRange, m_yRange, m_zRange;
	//Index3(INT i, INT j, INT k, INT maxX, INT maxY, INT maxZ) : m_i(i), m_j(j), m_k(k), m_xRange(maxX), m_yRange(maxY), m_zRange(maxZ)
	//{
	//	assert(i >= 0 && i < maxX && j >= 0 && j < maxY && k >= 0 && k < maxZ);
	//}
	Index3(SHORT i, SHORT j, SHORT k) : m_i(i), m_j(j), m_k(k)
	{
	}
	Index3() : m_i(0), m_j(0), m_k(0)
	{}
	bool operator== (const Index3& rhs)
	{
		return (m_i == rhs.m_i && m_j == rhs.m_j && m_k == rhs.m_k);
	}
	friend DecoArchive& operator<< (DecoArchive& Ar, const Index3& idx);
	friend DecoArchive& operator>> (DecoArchive& Ar, Index3& idx);
};

class Index3INT
{
public:
	INT m_i, m_j, m_k;
	//INT m_xRange, m_yRange, m_zRange;
	//Index3(INT i, INT j, INT k, INT maxX, INT maxY, INT maxZ) : m_i(i), m_j(j), m_k(k), m_xRange(maxX), m_yRange(maxY), m_zRange(maxZ)
	//{
	//	assert(i >= 0 && i < maxX && j >= 0 && j < maxY && k >= 0 && k < maxZ);
	//}
	Index3INT(INT i, INT j, INT k) : m_i(i), m_j(j), m_k(k)
	{
	}
	Index3INT() : m_i(0), m_j(0), m_k(0)
	{}
	//friend DecoArchive& operator<< (DecoArchive& Ar, const Index3INT& idx);
	//friend DecoArchive& operator>> (DecoArchive& Ar, Index3INT& idx);
};

class DecoQuaternion
{
private:
	vector3 m_v;
	DOUBLE m_w;
public:
	DecoQuaternion()
	{
		Identity();
	}
	DecoQuaternion(vector3 v, DOUBLE w) : m_v(v), m_w(w)
	{
	}
	DecoQuaternion(vector4 vec) : m_v(vector3(vec.x, vec.y, vec.z)), m_w(vec.w)
	{
	}
	DecoQuaternion(DOUBLE angle, vector3 rotationAxis);
	vector4 Rotate(vector4 vec) const;
	void Matrix(matrix44& outMatrix) const;
	void Inverse();
	DecoQuaternion operator* (const DecoQuaternion& rhs) const;
	DOUBLE Norm() const
	{
		return sqrt(m_v.lengthSqr() + m_w * m_w);
	}
	void Normalize()
	{
		double len = Norm();
		m_v /= len;
		m_w /= len;

	}
	void Identity()
	{
		m_v = vector3(0, 0, 0);
		m_w = 1;
	}
	void Conjugate()
	{
		m_v = -m_v;
	}

	DecoQuaternion operator+= (const DecoQuaternion& rhs)
	{
		m_v += rhs.m_v;
		m_w += rhs.m_w;
		return *this;
	}
	DecoQuaternion operator-= (const DecoQuaternion& rhs)
	{
		m_v -= rhs.m_v;
		m_w -= rhs.m_w;
		return *this;
	}
	friend bool operator == (const DecoQuaternion &a, const DecoQuaternion &b)
	{
		return(a.m_v == b.m_v && a.m_w == b.m_w);
	}
	// Are these two vector3's not equal?
	friend bool operator != (const DecoQuaternion &a, const DecoQuaternion &b)
	{
		return (!(a == b));
	}

	friend DecoQuaternion operator + (const DecoQuaternion &a, const DecoQuaternion &b)
	{
		DecoQuaternion ret(a);
		ret += b;
		return ret;
	}
	friend DecoQuaternion operator - (const DecoQuaternion &a, const DecoQuaternion &b) 
	{
		DecoQuaternion ret(a);
		ret -= b;
		return ret;
	}
	friend	DecoQuaternion operator* (double fScalar, const DecoQuaternion& rkQ)
	{

		return DecoQuaternion(vector4(fScalar*rkQ.m_v.x,fScalar*rkQ.m_v.y,
			fScalar*rkQ.m_v.z, fScalar*rkQ.m_w));
	}

	friend DecoArchive& operator<< (DecoArchive& Ar, const DecoQuaternion& q);
	friend DecoArchive& operator>> (DecoArchive& Ar, DecoQuaternion& q);
};

//class Box
//{
//private:
//	vector3 Min;
//	vector3 Max;
//public:
//	Box() { Init(); }
//	Box( const vector3& InMin, const vector3& InMax ) : Min(InMin), Max(InMax) {}
//	void Set(const vector3& InMin, const vector3& InMax)
//	{
//		Min = InMin;
//		Max = InMax; 
//	}
//	// Functions.
//	void Init()
//	{ 
//		Min = Max = vector3(0,0,0);
//	}
//	const vector3& GetMin() const
//	{
//		return Min;
//	}
//	const vector3& GetMax() const
//	{
//		return Max;
//	}
//	void AllVerteices(vector3* Vertices) const;
//	Box& operator+=( const vector3 &Other )
//	{
//		Min.x = min( Min.x, Other.x );
//			Min.y = min( Min.y, Other.y );
//			Min.z = min( Min.z, Other.z );
//
//			Max.x = max( Max.x, Other.x );
//			Max.y = max( Max.y, Other.y );
//			Max.z = max( Max.z, Other.z );
//		return *this;
//	}
//	Box operator+( const vector3& Other ) const
//	{
//		return Box(*this) += Other;
//	}
//	Box& operator+=( const Box& Other )
//	{
//			Min.x = min( Min.x, Other.Min.x );
//			Min.y = min( Min.y, Other.Min.y );
//			Min.z = min( Min.z, Other.Min.z );
//
//			Max.x = max( Max.x, Other.Max.x );
//			Max.y = max( Max.y, Other.Max.y );
//			Max.z = max( Max.z, Other.Max.z );
//		return *this;
//	}
//	Box operator+( const Box& Other ) const
//	{
//		return Box(*this) += Other;
//	}
//	vector3& operator[]( INT i )
//	{
//		assert(i>-1);
//		assert(i<2);
//		if( i == 0 )		return Min;
//		else				return Max;
//	}
//    void Move(double x, double y, double z)
//    {
//        Min.x += x;
//        Min.y += y;
//        Min.z += z;
//
//        Max.x += x;
//        Max.y += y;
//        Max.z += z;
//    }
//	Box ExpandBy( double W ) const
//	{
//		return Box( Min - vector3(W,W,W), Max + vector3(W,W,W) );
//	}
//	Box ExpandBy( vector3 V) const
//	{
//		return Box( Min - V, Max + V);
//	}
//	// Returns the midpoint between the min and max points.
//	vector3 GetCenter() const
//	{
//		return vector3( ( Min + Max ) * 0.5f );
//	}
//	// Returns the extent around the center
//	vector3 GetExtent() const
//	{
//		return 0.5f*(Max - Min);
//	}
//
//	void GetCenterAndExtents( vector3 & center, vector3 & Extents ) const
//	{
//		Extents = Max - Min;
//		Extents *= .5f;
//		center = Min + Extents;
//	}
//
//	BOOL Intersect( const Box & other ) const
//	{
//		if( Min.x > other.Max.x || other.Min.x > Max.x )
//			return FALSE;
//		if( Min.y > other.Max.y || other.Min.y > Max.y )
//			return FALSE;
//		if( Min.z > other.Max.z || other.Min.z > Max.z )
//			return FALSE;
//		return TRUE;
//	}
//	Box TransformBy (const matrix44& matrix) const;
//	double RayCheck(const vector3& origin, const vector3& dir) const; //not intersect if return value <= 0, else return value means time t
//	ConvexVolume GetVolume(void) const;
//	BOOL TriangleInBox (const vector3& a, const vector3& b, const vector3& c) const;
//	friend DecoArchive& operator<< (DecoArchive& Ar, const Box& b);
//	friend DecoArchive& operator>> (DecoArchive& Ar, Box& b);
//};

//class Plane
//{
//private:
//	vector3 Normal;
//	double W;
//public:
//	Plane()
//	{}
//	Plane( const Plane& P )
//		:	Normal(P.Normal)
//		,	W(P.W)
//	{}
//	Plane( const vector3& V )
//		:	Normal(V)
//		,	W(0)
//	{}
//	Plane ( const vector4& V )
//		:	Normal(V.x, V.y, V.z)
//		,	W(V.w)
//	{}
//	Plane( double InX, double InY, double InZ, double InW )
//		:	Normal(InX,InY,InZ)
//		,	W(InW)
//	{}
//	Plane( vector3 InNormal, double InW )
//		:	Normal(InNormal), W(InW)
//	{}
//	Plane( vector3 InBase, const vector3 &InNormal )
//		:	Normal(InNormal)
//		,	W(-DotProduct(InBase, InNormal))
//	{}
//	Plane( vector3 A, vector3 B, vector3 C )
//		:	Normal( CrossProduct((B-A), (C-A)))
//		,	W( -DotProduct(A, CrossProduct((B-A), (C-A))))
//	{}
//
//	// Functions.
//	vector3 GetNormal() const
//	{
//		return Normal;
//	}
//	double GetOffset() const
//	{
//		return W;
//	}
//
//	double PlaneDot( const vector3 &P ) const
//	{
//		return Normal.x * P.x + Normal.y * P.y + Normal.z * P.z + W;
//	}
//	Plane Flip() const
//	{
//		return Plane(-Normal.x,-Normal.y,-Normal.z,-W);
//	}
//	BOOL operator==( const Plane& V ) const
//	{
//		return Normal == V.Normal && W == V.W;
//	}
//	BOOL operator!=( const Plane& V ) const
//	{
//		return Normal != V.Normal || W != V.W;
//	}
//	Plane operator+( const Plane& V ) const
//	{
//		return Plane( Normal + V.Normal, W + V.W );
//	}
//	Plane operator-( const Plane& V ) const
//	{
//		return Plane( Normal - V.Normal, W - V.W );
//	}
//	Plane operator/( double Scale ) const
//	{
//		double RScale = 1.f/Scale;
//		return Plane( Normal * RScale, W * RScale );
//	}
//	Plane operator*( double Scale ) const
//	{
//		return Plane( Normal * Scale, W * Scale );
//	}
//	Plane operator*( const Plane& V )
//	{
//		return Plane ( Normal.x * V.Normal.x, Normal.y * V.Normal.y, Normal.z * V.Normal.z, W*V.W );
//	}
//	Plane operator+=( const Plane& V )
//	{
//		Normal += V.Normal;
//		W += V.W;
//		return *this;
//	}
//	Plane operator-=( const Plane& V )
//	{
//		Normal -= V.Normal;
//		W -= V.W;
//		return *this;
//	}
//	Plane operator*=( double Scale )
//	{
//		Normal *= Scale;
//		W *= Scale;
//		return *this;
//	}
//	Plane operator*=( const Plane& V )
//	{
//		Normal.x *= V.Normal.x; 
//		Normal.y *= V.Normal.y;
//		Normal.z *= V.Normal.z;
//		W *= V.W;
//		return *this;
//	}
//	Plane operator/=( double V )
//	{
//		double RV = 1.f/V;
//		Normal *= RV;
//		W *= RV;
//		return *this;
//	}
//	Plane TransformBy (const matrix44& matrix) const;
//	double RayCheck(const vector3& origin, const vector3& dir) const
//	{
//		double intersectionTime = this->PlaneDot(origin) / DotProduct(Normal, dir);
//		return intersectionTime;
//	}
//	vector3 RayIntersect(const vector3& origin, const vector3& dir) const
//	{
//		double intersectionTime = -this->PlaneDot(origin) / DotProduct(Normal, dir);
//		return origin + intersectionTime * dir;
//	}
//	friend DecoArchive& operator<< (DecoArchive& Ar, const Plane& p);
//	friend DecoArchive& operator>> (DecoArchive& Ar, Plane& p);
//};

enum EClippingFlag
{
	CF_Inside = 0x1,
	CF_Outside = 0x2
};
//class ConvexVolume
//{
//public:
//
//	enum { MAX_VOLUME_PLANES = 32 };
//
//	Plane	BoundingPlanes[MAX_VOLUME_PLANES];
//	INT		NumPlanes;
//
//	// Constructor
//	ConvexVolume()
//	{
//		NumPlanes = 0;
//	}
//	ConvexVolume(const ConvexVolume& rhs) : NumPlanes(rhs.NumPlanes)
//	{
//		for (int i = 0; i < MAX_VOLUME_PLANES; i++)
//		{
//			BoundingPlanes[i] = rhs.BoundingPlanes[i];
//		}
//	}
//	ConvexVolume TransformBy(const matrix44& matrix) const;
//	// BoxCheck - Determines whether a box is inside of the volume, outside of the volume, or both.  Returns EClippingFlag.
//	BYTE BoxCheck(const Box& InBox);
//	BYTE BoxCheck(const vector3& Origin, const vector3& Extent);
//	friend DecoArchive& operator<< (DecoArchive& Ar, const ConvexVolume& cv);
//	friend DecoArchive& operator>> (DecoArchive& Ar, ConvexVolume& cv);
//};

enum AXIS
{
	AXIS_X,
	AXIS_Y,
	AXIS_Z,
	AXIS_MAX
};

class Coords
{
public:
	Coords(void);
	Coords(const Coords& rhs);
	explicit Coords(const vector3& origin);
	Coords& operator=(const Coords& rhs);
	
	Coords(double base[][3], const vector3& origin, bool normalized = true);
	virtual ~Coords(void);
	
	vector3 ConvertToLocal(const vector3& pt);
	vector3 ConvertToParent(const vector3& pt)const;

	Coords ConvertToLocal(const Coords& frm);
	Coords ConvertToParent(const Coords& frm)const;
	void RotateAlongGlobalAxis(double angle, AXIS axis);
	void RotateAlongLocalAxis(double angle, AXIS axis);
	void RotateAlongGlobalVector(double angle, vector3 vec);
	vector3 origin()const;
	void SetOrigin(const vector3 &origin);
	void Set(const vector3& x, const vector3& y, const vector3& z, const vector3& o, bool normalized = true);
	void GetBase(vector3& x, vector3& y, vector3& z)const;
	void Translate(const vector3& trans);
	void TranslateLocal(const vector3& trans);
	vector3 GetYParentProjection()const;
	matrix44 GetMatrix();
	void reflectXZPlane();
	friend DecoArchive& operator<< (DecoArchive& Ar, const Coords& coords);
	friend DecoArchive& operator>> (DecoArchive& Ar, Coords& coords);
protected:
	vector3 _origin; 
	double _base[3][3];
	double _determinant;
	double _rev[3][3];
	matrix44 _mat;
private:
	//for lazy evaluation
	bool _dirty;
	//generate _rev, _determinant and _mat
	void Calculate();
	double GetAngle(double x, double y);
};



unsigned short CeilPower2(unsigned short x);
//Box Bounding(vector3* array, size_t num);
BOOL TriangleRayIntersection (const vector3& a, const vector3& b, const vector3& c, const vector3& origin,const vector3& dir, vector3 *intersection, double* t);
BOOL PointInTriangle (const vector3& pt, const vector3& a, const vector3& b, const vector3& c);
BOOL PointInLine (const vector3& pt, const vector3& a, const vector3& b, BOOL bSegment = TRUE);
DOUBLE PointRayDistance(const vector3& pt, const vector3& origin, const vector3& dir, double& rayDepth);
double PointRayDistance(const Eigen::Vector3f& pt, const Eigen::Vector3f& origin, const Eigen::Vector3f& dir, double& rayDepth);
DOUBLE PointLineDistance (const vector3& pt, const vector3& a, const vector3& b, BOOL bSegment = TRUE);
DOUBLE PointPolylineDistance(const vector3& pt, const vector<vector3>& polyline, int& segmentIdx);
DOUBLE PointTriangleDistance(const vector3& pt, const vector3& a, const vector3& b, const vector3& c);
DOUBLE RayPolylineDistance(const vector3& origin, const vector3& dir, const vector<Eigen::Vector3d>& polyLine, double& depth);
DOUBLE RayLineDistance(const vector3& origin, const vector3& dir, const vector3& a, const vector3& b, double& depth, bool isLineSegmen = true);
vector3 TransformPoint(const matrix44& matrix, const vector3& point);
vector3 TransformNormal(const matrix44& matrix, const vector3& norm);
vector3 TransformVector(const matrix44& matrix, const vector3& vec);
bool checkWithInBox (const vector3& pt, const vector3& lbb, const vector3& ruf);
void indexTo3d (INT index, UINT& xIndex, UINT& yIndex, UINT& zIndex, UINT numX, UINT numY, UINT numZ);
UINT indexTo1d (INT xIndex, INT yIndex, INT zIndex, UINT numX, UINT numY, UINT numZ);
void indexTo2d(int index, int& xIndex, int& yIndex, int numX, int numY);
int indexTo1d(int xIndex, int yIndex, int numX, int numY);
UINT round2Pow2(UINT num);
vector3 BaryCenter(const vector<vector3>& verts);
double RandDouble(double low, double high);
float TriangleArea(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c);
Eigen::Vector3f TriangleNormal(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c);
void RandomPointInTriangle(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c, const Eigen::Vector3f& na, const Eigen::Vector3f& nb, const Eigen::Vector3f& nc, Eigen::Vector3f* p, Eigen::Vector3f* n);
double TriangleArea(const vector3& a, const vector3& b, const vector3& c);
vector3 TriangleNormal(const vector3& a, const vector3& b, const vector3& c);
Eigen::MatrixXd GetTangentialDirs(const vector3& normal, int numDirs);
matrix33 FormulateSkewSymmetricMatrix(const vector3& vec);
Eigen::Matrix3d FormulateSkewSymmetricMatrix(const Eigen::Vector3d& vec);
matrix33 ShapeMatch(const vector<vector3>& vertices1, const vector<vector3>& vertices2);
bool NearlyEqual(double a, double b, double threshold = EPSILON_FLOAT);
double ComputeWeightFromLWLinearRegression(double dist, double maxDist);
vector<int> Index1DToND(int id, const vector<int>& lenPerDim);
int IndexNDTo1D(const vector<int>& indices, const vector<int>& lenPerDim);
double sigmod(double x);
#endif
