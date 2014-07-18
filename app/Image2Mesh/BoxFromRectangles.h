#ifndef _BOX_FROM_RECTANGLES_H
#define _BOX_FROM_RECTANGLES_H

#include "Rectangle.h"
#include <Eigen/Dense>
#include <vector>
using namespace std;

class BoxFromRectangles
{
public:
	BoxFromRectangles();
	bool Construct(const PartRectangle& rect1, const PartRectangle& rect2, int componentId1, int componentId2, float* score);
	void SavePly(const string& filename);
	float TotalArea() const;
	bool operator<(const BoxFromRectangles &rhs) const
	{
		return mConfidence > rhs.mConfidence;
	}
	int ComponentId(int ithId) const;
	float Confidence() const;
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