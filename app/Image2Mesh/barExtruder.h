#ifndef _BAR_EXTRUDER
#define _BAR_EXTRUDER


#include <Eigen/Dense>
#include <vector>
using namespace std;

#include "Part.h"
#include "BoxFromRectangles.h"
#include "DepthImage.h"


class BarExtruder
{
public:
	BarExtruder();

	void SetWallAndFloor(PrimitiveShape* wall, PrimitiveShape* floor);
	void SetReferenceDepthImages(const vector<DepthImage*> refImages);
	bool Extrude(const BoxFromRectangles& box, const vector<Part>& parts, vector<Eigen::Vector3f>* extrudedPoints, vector<Eigen::Vector3f>* extrudedNormals);

private:
	int needExtrude(const BoxFromRectangles& box) const;
	bool isBarShape(const BoxFromRectangles& box) const;
	float findExtrudeAmount(const BoxFromRectangles& box, int dir) const;
	void extrude(const BoxFromRectangles& box, const Eigen::Vector3f& extrudeVector, vector<Eigen::Vector3f>* extrudedPoints, vector<Eigen::Vector3f>* extrudedNormals, bool fromOutSideBox = true) const;
	void findBarLongAxisAndExtent(const BoxFromRectangles& box, Eigen::Vector3f* axis, float* extent) const;
	float checkUnseenRatio(const vector<Eigen::Vector3f>& points) const;
	bool isPointVisible(const Eigen::Vector3f& point) const;
	vector<DepthImage*> mRefDepthImages;
	PrimitiveShape* mWall;
	PrimitiveShape* mFloor;
};

#endif