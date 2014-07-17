#include "Rectangle.h"
#include "MeshIO.h"

PartRectangle::PartRectangle()
{

}
bool PartRectangle::IsRectangleInBack(const PartRectangle& rhs) const
{
	int numPointsRhs = static_cast<int>(rhs.mPoints.size());
	const float frontRatioThreshold = 0.99f;
	int vote = 0;
	for (int i = 0; i < numPointsRhs; ++i)
	{
		const Eigen::Vector3f& ptFromRhs = rhs.mPoints[i];
		Eigen::Vector3f offset = ptFromRhs - mCenter;
		float cos = offset.dot(mNormal);
		if (cos < 0)
			vote++;
	}
	float ratio = static_cast<float>(vote) / numPointsRhs;
	if (ratio > frontRatioThreshold)
		return true;
	else
		return false;
}
bool PartRectangle::IsParellel(const PartRectangle& rhs) const
{
	Eigen::Vector3f nRhs = rhs.mNormal;
	const float cosAngleThreshold = 0.9f;
	if (abs(mNormal.dot(nRhs)) > cosAngleThreshold)
		return true;
	else
		return false;
}
bool PartRectangle::IsOrthogonal(const PartRectangle& rhs) const
{
	Eigen::Vector3f nRhs = rhs.mNormal;
	const float cosAngleThreshold = 0.1f;
	if (abs(mNormal.dot(nRhs)) < cosAngleThreshold)
		return true;
	else
		return false;
}
float PartRectangle::DistanceTo(const PartRectangle& rhs) const
{
	float minDist = FLT_MAX;
	int numPoints = static_cast<int>(mPoints.size());
	int numPointsRhs = static_cast<int>(rhs.mPoints.size());
	for (int i = 0; i < numPoints; ++i)
	{
		for (int j = 0; j < numPointsRhs; ++j)
		{
			float dist = (mPoints[i] - rhs.mPoints[j]).norm();
			if (dist < minDist)
				minDist = dist;
		}
	}
	return minDist;
}

float PartRectangle::Area() const
{
	return mExtent[0] * mExtent[1] * 4;
}
void PartRectangle::SavePly(const string& filename)
{
	if (mVertices.empty())
		generateRectangleGeometry();
	SaveMesh(filename, mVertices, mFaces);
}

void PartRectangle::GetGeometry(vector<Eigen::Vector3f>& vertices, vector<Eigen::Vector3i>& faces)
{
	if (mVertices.empty())
		generateRectangleGeometry();
	vertices = mVertices;
	faces = mFaces;
}

void PartRectangle::generateRectangleGeometry()
{
	mVertices.resize(6);
	mVertices[0] = Eigen::Vector3f(-mExtent[0], -mExtent[1], 0);
	mVertices[1] = Eigen::Vector3f(mExtent[0], mExtent[1], 0);
	mVertices[2] = Eigen::Vector3f(-mExtent[0], mExtent[1], 0);


	mVertices[3] = Eigen::Vector3f(-mExtent[0], -mExtent[1], 0);
	mVertices[4] = Eigen::Vector3f(mExtent[0], -mExtent[1], 0);
	mVertices[5] = Eigen::Vector3f(mExtent[0], mExtent[1], 0);


	Eigen::Matrix3f rot;
	rot.col(0) = mTangent1;
	rot.col(1) = mTangent2;
	rot.col(2) = mNormal;
	for (int i = 0; i < 6; ++i)
	{
		mVertices[i] = rot * mVertices[i] + mCenter;
	}
	mFaces.resize(2);
	for (int i = 0; i < 3; ++i)
	{
		mFaces[0][i] = i;
		mFaces[1][i] = 3 + i;
	}
}
