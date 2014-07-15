#include "Part.h"

Part::Part() : mShape(NULL)
{

}
Part::Part(PrimitiveShape* shape, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals, sehoon::ann::KDTree* tree)
{
	mKDTreePoints = tree;
	mShape = shape;
	mPoints = points;
	mNormals = normals;
	int numPoints = static_cast<int>(mPoints.size());
	mKDTreePoints->setDim(3);
	for (int i = 0; i < numPoints; ++i)
	{
		if (i % 10000 == 0)
			LOG(INFO) << "Finish adding " << i << "th points out of " << numPoints;
		mKDTreePoints->add(points[i]);
	}
	mKDTreePoints->initANN();
}

PrimitiveShape* Part::GetShape() const
{
	return mShape;
}
const vector<Eigen::Vector3f>& Part::GetPoints() const
{
	return mPoints;
}
const vector<Eigen::Vector3f>& Part::GetNormals() const
{
	return mNormals;
}
bool Part::IsPointClose(const Eigen::Vector3f& pt)
{
	vector<int> nnId = mKDTreePoints->kSearch(pt, 1);
	Eigen::Vector3f nnPt = mPoints[nnId[0]];
	return (nnPt - pt).norm() < 0.05;
}