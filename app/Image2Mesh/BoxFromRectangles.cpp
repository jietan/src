#include "BoxFromRectangles.h"
#include "MeshIO.h"

BoxFromRectangles::BoxFromRectangles() : mIsValid(false)
{

}
bool BoxFromRectangles::Construct(const PartRectangle& rect1, const PartRectangle& rect2)
{
	return mIsValid;
}
void BoxFromRectangles::SavePly(const string& filename)
{
	if (!mIsValid) return;
	if (mVertices.empty())
		generateBoxGeometry();
	SaveMesh(filename, mVertices, mFaces);
}
vector<PartRectangle> BoxFromRectangles::getAllRectangles() const
{

}
void BoxFromRectangles::generateBoxGeometry()
{
	vector<PartRectangle> allRectangle = getAllRectangles();
	for (int i = 0; i < 6; ++i)
	{
		vector<Eigen::Vector3f> vertices;
		vector<Eigen::Vector3i> faces;
		allRectangle[i].GetGeometry(vertices, faces);
		mVertices.insert(mVertices.end(), vertices.begin(), vertices.end());
		mFaces.insert(mFaces.end(), faces.begin(), faces.end());
	}
}