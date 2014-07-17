#ifndef _RECTANGLE_H
#define _RECTANGLE_H

#include <Eigen/Dense>
#include <vector>
using namespace std;


class PartRectangle
{
public:
	PartRectangle();
	bool IsRectangleInBack(const PartRectangle& rhs) const;
	bool IsParellel(const PartRectangle& rhs) const;
	bool IsOrthogonal(const PartRectangle& rhs) const;
	float DistanceTo(const PartRectangle& rhs) const;
	float Area() const;
	void SavePly(const string& filename);
	void GetGeometry(vector<Eigen::Vector3f>& vertices, vector<Eigen::Vector3i>& faces);
	Eigen::Vector3f mNormal;
	
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