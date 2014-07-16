#include "Rectangle.h"
#include "MeshIO.h"

PartRectangle::PartRectangle()
{

}
bool PartRectangle::IsRectangleInFront(const PartRectangle& rhs)
{

}
bool PartRectangle::IsParellel(const PartRectangle& rhs)
{

}
bool PartRectangle::IsOrthogonal(const PartRectangle& rhs)
{

}
float PartRectangle::DistanceTo(const PartRectangle& rhs)
{

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
	mVertices[1] = Eigen::Vector3f(-mExtent[0], mExtent[1], 0);
	mVertices[2] = Eigen::Vector3f(mExtent[0], mExtent[1], 0);

	mVertices[3] = Eigen::Vector3f(-mExtent[0], -mExtent[1], 0);
	mVertices[4] = Eigen::Vector3f(mExtent[0], mExtent[1], 0);
	mVertices[5] = Eigen::Vector3f(mExtent[0], -mExtent[1], 0);

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
