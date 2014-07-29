#ifndef _BOX_FROM_RECTANGLES_H
#define _BOX_FROM_RECTANGLES_H

#include "Rectangle.h"
#include <Eigen/Dense>
#include <vector>
#include "utility/mathlib.h"
using namespace std;

class ExtentAxisPair
{
public:
	ExtentAxisPair(float extent, const Eigen::Vector3f& axis);
	bool operator< (const ExtentAxisPair& rhs) const;
	pair<float, Eigen::Vector3f> mPair;
};


class BoxFromRectangles
{
public:
	BoxFromRectangles();
	bool Construct(const PartRectangle& rect1, const PartRectangle& rect2, int componentId1, int componentId2, float* score);
	void SavePly(const string& filename);
	void Save(const string& filename);
	void Read(const string& filename);
	float TotalArea() const;
	bool operator<(const BoxFromRectangles &rhs) const
	{
		return mConfidence > rhs.mConfidence;
	}
	int GetNumComponents() const;
	int ComponentId(int ithId) const;
	float Confidence() const;
	BoxFromRectangles MirroredBox(const UtilPlane& pl) const;
	bool IsBoxSimilar(const BoxFromRectangles& rhs) const;
	bool IsPointInBox(const Eigen::Vector3f& pt) const;
	const Eigen::Vector3f& GetCenter() const;
	const vector<Eigen::Vector3f>& GetAxes() const;
	const Eigen::Vector3f& GetExtents() const;
	void GetGeometry(vector<Eigen::Vector3f>& vertices, vector<Eigen::Vector3i>& faces);
	string GetComponentString() const;

	void SetCenter(const Eigen::Vector3f& center);
	void SetAxes(const vector<Eigen::Vector3f>& axes);
	void SetExtent(const Eigen::Vector3f& extent);
	void SetComponentIds(const vector<int>& ids);
	void SetValid(bool isValid);
	void ReassignPoints(const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals);
	vector<Eigen::Vector3f> mPoints;
	vector<Eigen::Vector3f> mNormals;

private:
	vector<PartRectangle> getAllRectangles() const;
	void generateBoxGeometry();
	bool constructBoxOrthogonalRectangles(const PartRectangle& rect1, const PartRectangle& rect2, float* score);
	bool constructBoxParallelRectangles(const PartRectangle& rect1, const PartRectangle& rect2, float* score);
	void computeCenterAndExtent(const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f> axes, Eigen::Vector3f* center, Eigen::Vector3f* extent);
	bool mIsValid;
	Eigen::Vector3f mCenter;
	vector<Eigen::Vector3f> mAxis;
	Eigen::Vector3f mExtent;

	vector<Eigen::Vector3f> mVertices;
	vector<Eigen::Vector3i> mFaces;
	vector<int> mComponentId;
	float mConfidence;

};

#endif