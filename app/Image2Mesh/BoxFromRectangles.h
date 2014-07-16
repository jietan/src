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
	bool Construct(const PartRectangle& rect1, const PartRectangle& rect2);
	void SavePly(const string& filename);

private:
	vector<PartRectangle> getAllRectangles() const;
	void generateBoxGeometry();

	bool mIsValid;
	Eigen::Vector3f mCenter;
	Eigen::Vector3f mAxis1;
	Eigen::Vector3f mAxis2;
	Eigen::Vector3f mAxis3;
	Eigen::Vector3f mExtent;

	vector<Eigen::Vector3f> mVertices;
	vector<Eigen::Vector3i> mFaces;
};

#endif