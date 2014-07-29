#ifndef _EXTRUDER_H
#define _EXTRUDER_H


#include <Eigen/Dense>
#include <vector>
using namespace std;

#include "Part.h"
#include "BoxFromRectangles.h"


class Extruder
{
public:
	Extruder();
	bool Extrude(const BoxFromRectangles& box, const vector<Part>& parts, vector<Eigen::Vector3f>* extrudedPoints, vector<Eigen::Vector3f>* extrudedNormals);

private:
	bool needExtrude(const BoxFromRectangles& box, const vector<Part>& parts) const;
	bool isBoardShape(const BoxFromRectangles& box) const;
	Eigen::Vector3f findExtrudeVector(const BoxFromRectangles& box, const vector<Part>& parts, int sourceComponentId, const vector<Eigen::Vector3f>& sourcePoints) const;
	void findExtrudeSource(const BoxFromRectangles& box, const vector<Part>& parts, int* ithComponent, vector<Eigen::Vector3f>* sourcePoints, vector<Eigen::Vector3f>* sourceNormals) const;
	int findExtrudeSourceComponent(const BoxFromRectangles& box, const vector<Part>& parts) const;
	void extrude(const vector<Eigen::Vector3f>& sourcePoints, const vector<Eigen::Vector3f>& sourceNormals, const Eigen::Vector3f& extrudeVector, vector<Eigen::Vector3f>* extrudedPoints, vector<Eigen::Vector3f>* extrudedNormals);
	void findThinBoardAxisAndExtent(const BoxFromRectangles& box, Eigen::Vector3f* axis, float* extent) const;
};

#endif