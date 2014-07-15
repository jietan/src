#ifndef _PART_H
#define _PART_H


#include "pointCloudToPrimitive/PlanePrimitiveShape.h"
#include "pointCloudToPrimitive/CylinderPrimitiveShape.h"

#include <Eigen/Dense>
#include <vector>
using namespace std;
#include "sehoon/ANNHelper.h"

class Part
{
public:
	Part();
	Part(PrimitiveShape* shape, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals, sehoon::ann::KDTree* tree);
	//void SetData(PrimitiveShape* shape, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals);
	PrimitiveShape* GetShape() const;
	const vector<Eigen::Vector3f>& GetPoints() const;
	const vector<Eigen::Vector3f>& GetNormals() const;
	bool IsPointClose(const Eigen::Vector3f& pt);
private:
	PrimitiveShape* mShape;
	vector<Eigen::Vector3f> mPoints;
	vector<Eigen::Vector3f> mNormals;
	sehoon::ann::KDTree* mKDTreePoints;
};

#endif