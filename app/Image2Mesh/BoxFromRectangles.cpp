#include "BoxFromRectangles.h"
#include "MeshIO.h"
#include "utility/mathlib.h"

BoxFromRectangles::BoxFromRectangles() : mIsValid(false)
{

}
bool BoxFromRectangles::Construct(const PartRectangle& rect1, const PartRectangle& rect2)
{
	if (!rect1.IsRectangleInBack(rect2) || !rect2.IsRectangleInBack(rect1))
	{
		mIsValid = false;
		return mIsValid;
	}
	if (rect1.IsOrthogonal(rect2))
	{
		const float orthogonalDistanceThreshold = 0.1;
		float dist = rect1.DistanceTo(rect2);
		if (dist < orthogonalDistanceThreshold)
		{
			mIsValid = constructBoxOrthogonalRectangles(rect1, rect2);
		}
		else
		{
			mIsValid = false;
		}
	}
	else if (rect1.IsParellel(rect2))
	{
		const float parallelDistanceThreshold = 0.1;
		float dist = rect1.DistanceTo(rect2);
		if (dist < parallelDistanceThreshold)
		{
			mIsValid = constructBoxParallelRectangles(rect1, rect2);
		}
		
	}
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
	vector<PartRectangle> ret;
	ret.resize(6);

	for (int i = 0; i < 3; ++i)
	{
		ret[2 * i].mNormal = -mAxis[(i + 0) % 3];
		ret[2 * i].mTangent1 = mAxis[(i + 1) % 3];
		ret[2 * i].mTangent2 = -mAxis[(i + 2) % 3];
		ret[2 * i].mCenter = mCenter - mExtent[i] * mAxis[i];
		ret[2 * i].mExtent = Eigen::Vector2f(mExtent[(i + 1) % 3], mExtent[(i + 2) % 3]);

		ret[2 * i + 1].mNormal = mAxis[(i + 0) % 3];
		ret[2 * i + 1].mTangent1 = mAxis[(i + 1) % 3];
		ret[2 * i + 1].mTangent2 = mAxis[(i + 2) % 3];
		ret[2 * i + 1].mCenter = mCenter + mExtent[i] * mAxis[i];
		ret[2 * i + 1].mExtent = Eigen::Vector2f(mExtent[(i + 1) % 3], mExtent[(i + 2) % 3]);
	}


	return ret;
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
		int currentFaceNumber = 3 * static_cast<int>(mFaces.size());
		for (int j = 0; j < 2; ++j)
		{
			mFaces.push_back(currentFaceNumber * Eigen::Vector3i::Constant(1) + faces[j]);
		}
		
		
	}
}

bool BoxFromRectangles::constructBoxOrthogonalRectangles(const PartRectangle& rect1, const PartRectangle& rect2)
{
	float a1 = rect1.Area();
	float a2 = rect2.Area();
	const PartRectangle& largeRect = a1 > a2 ? rect1 : rect2;
	const PartRectangle& smallRect = a1 > a2 ? rect2 : rect1;
	mAxis[0] = largeRect.mNormal;
	mAxis[1] = smallRect.mNormal;
	mAxis[2] = mAxis[0].cross(mAxis[1]);
	mAxis[2].normalize();
	mAxis[1] = mAxis[2].cross(mAxis[0]);
	mAxis[1].normalize();
	//Eigen::Vector3f bbMin(FLT_MAX, FLT_MAX, FLT_MAX);
	//Eigen::Vector3f bbMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	//int numPointsLarge = static_cast<int>(largeRect.mPoints.size());
	//int numPointsSmall = static_cast<int>(smallRect.mPoints.size());
	//Eigen::Matrix3f axis;
	//for (int i = 0; i < 3; ++i)
	//	axis.row(i) = mAxis[i].transpose();
	//for (int i = 0; i < numPointsSmall; ++i)
	//{
	//	Eigen::Vector3f proj = axis * smallRect.mPoints[i];
	//	for (int ithDim = 0; ithDim < 3; ++ithDim)
	//	{
	//		if (proj[ithDim] < bbMin[ithDim])
	//			bbMin[ithDim] = proj[ithDim];
	//		if (proj[ithDim] > bbMax[ithDim])
	//			bbMax[ithDim] = proj[ithDim];
	//	}
	//}
	//for (int i = 0; i < numPointsLarge; ++i)
	//{
	//	Eigen::Vector3f proj = axis * largeRect.mPoints[i];
	//	for (int ithDim = 0; ithDim < 3; ++ithDim)
	//	{
	//		if (proj[ithDim] < bbMin[ithDim])
	//			bbMin[ithDim] = proj[ithDim];
	//		if (proj[ithDim] > bbMax[ithDim])
	//			bbMax[ithDim] = proj[ithDim];
	//	}
	//}

	//mExtent = (bbMax - bbMin) / 2.f;
	//mCenter = axis.transpose() * (bbMax + bbMin) / 2.f;

	vector<Eigen::Vector3f> allPoints;
	allPoints.insert(allPoints.end(), largeRect.mPoints.begin(), largeRect.mPoints.end());
	allPoints.insert(allPoints.end(), smallRect.mPoints.begin(), smallRect.mPoints.end());
	computeCenterAndExtent(allPoints, mAxis, &mCenter, &mExtent);

	return true;
}
bool BoxFromRectangles::constructBoxParallelRectangles(const PartRectangle& rect1, const PartRectangle& rect2)
{
	float a1 = rect1.Area();
	float a2 = rect2.Area();
	const PartRectangle& largeRect = a1 > a2 ? rect1 : rect2;
	const PartRectangle& smallRect = a1 > a2 ? rect2 : rect1;
	vector<Eigen::Vector3f> allPoints;
	allPoints.insert(allPoints.end(), largeRect.mPoints.begin(), largeRect.mPoints.end());
	int numPointsSmall = static_cast<int>(smallRect.mPoints.size());
	for (int i = 0; i < numPointsSmall; ++i)
	{
		const Eigen::Vector3f& pt = smallRect.mPoints[i];
		Eigen::Vector3f projPt = pt - (pt - largeRect.mCenter).dot(largeRect.mNormal) * largeRect.mNormal;
		allPoints.push_back(projPt);
	}
	Eigen::Vector3f mean;
	//Eigen::Vector3f axes[3];
	PCAOnPoints(allPoints, mean, mAxis);
	if (mAxis[0].cross(mAxis[1]).dot(mAxis[2]) < 0)
		mAxis[2] = -mAxis[2];
	//mAxis[1] = axes[1];
	//mAxis[2] = axes[2];
	allPoints.clear();
	allPoints.insert(allPoints.end(), largeRect.mPoints.begin(), largeRect.mPoints.end());
	allPoints.insert(allPoints.end(), smallRect.mPoints.begin(), smallRect.mPoints.end());
	computeCenterAndExtent(allPoints, mAxis, &mCenter, &mExtent);
	
	return true;
}

void BoxFromRectangles::computeCenterAndExtent(const vector<Eigen::Vector3f>& points, Eigen::Vector3f axes[3], Eigen::Vector3f* center, Eigen::Vector3f* extent)
{
	Eigen::Matrix3f axis;
	for (int i = 0; i < 3; ++i)
		axis.row(i) = axes[i].transpose();

	Eigen::Vector3f bbMin(FLT_MAX, FLT_MAX, FLT_MAX);
	Eigen::Vector3f bbMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	int numPoints = static_cast<int>(points.size());
	for (int i = 0; i < numPoints; ++i)
	{
		Eigen::Vector3f proj = axis * points[i];
		for (int ithDim = 0; ithDim < 3; ++ithDim)
		{
			if (proj[ithDim] < bbMin[ithDim])
				bbMin[ithDim] = proj[ithDim];
			if (proj[ithDim] > bbMax[ithDim])
				bbMax[ithDim] = proj[ithDim];
		}
	}


	*extent = (bbMax - bbMin) / 2.f;
	*center = axis.transpose() * (bbMax + bbMin) / 2.f;

}
