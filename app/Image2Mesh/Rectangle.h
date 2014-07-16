#ifndef _RECTANGLE_H
#define _RECTANGLE_H

#include <Eigen/Dense>
#include <vector>
using namespace std;


class PartRectangle
{
public:
	PartRectangle();
	bool IsRectangleInFront(const PartRectangle& rhs);
	bool IsParellel(const PartRectangle& rhs);
	bool IsOrthogonal(const PartRectangle& rhs);
	float DistanceTo(const PartRectangle& rhs);
	void SavePly(const string& filename);
	void GetGeometry(vector<Eigen::Vector3f>& vertices, vector<Eigen::Vector3i>& faces);
	Eigen::Vector3f mNormal;
	float mW;
	Eigen::Vector3f mTangent1;
	Eigen::Vector3f mTangent2;
	Eigen::Vector3f mCenter;
	Eigen::Vector2f mExtent;
	vector<Eigen::Vector3f> mPoints;

	vector<Eigen::Vector3f> mVertices;
	vector<Eigen::Vector3i> mFaces;
private:
	void generateRectangleGeometry();
};



#endif