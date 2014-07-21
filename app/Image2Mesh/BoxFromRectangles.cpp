#include "BoxFromRectangles.h"
#include "MeshIO.h"
#include "utility/mathlib.h"

BoxFromRectangles::BoxFromRectangles() : mIsValid(false), mConfidence(0)
{
	mComponentId.resize(2);
	mAxis.resize(3);
}
bool BoxFromRectangles::Construct(const PartRectangle& rect1, const PartRectangle& rect2, int componentId1, int componentId2, float* score)
{
	mComponentId[0] = componentId1;
	mComponentId[1] = componentId2;
	if (!rect1.IsRectangleInBack(rect2) || !rect2.IsRectangleInBack(rect1))
	{
		mIsValid = false;
		if (score)
			*score = 0;
		return mIsValid;
	}
	if (rect1.IsOrthogonal(rect2))
	{
		const float orthogonalDistanceThreshold = 0.02;
		float dist = rect1.DistanceTo(rect2);
		if (dist < orthogonalDistanceThreshold)
		{
			mIsValid = constructBoxOrthogonalRectangles(rect1, rect2, &mConfidence);
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
			mIsValid = constructBoxParallelRectangles(rect1, rect2, &mConfidence);
		}	
	}
	if (score)
		*score = mConfidence;
	return mIsValid;
}

void BoxFromRectangles::Save(const string& filename)
{
	ofstream out(filename.c_str());
	out << mIsValid << endl;
	out << mCenter[0] << " " << mCenter[1] << " " << mCenter[2] << endl;
	for (int i = 0; i < 3; ++i)
	{
		out << mAxis[i][0] << " " << mAxis[i][1] << " " << mAxis[i][2] << endl;
	}
	
	out << mExtent[0] << " " << mExtent[1] << " " << mExtent[2] << endl;
	out << mComponentId[0] << " " << mComponentId[1]  << endl;
	out << mConfidence;
}
void BoxFromRectangles::Read(const string& filename)
{
	ifstream in(filename.c_str());
	in >> mIsValid;
	in >> mCenter[0] >> mCenter[1] >> mCenter[2];
	for (int i = 0; i < 3; ++i)
	{
		in >> mAxis[i][0] >> mAxis[i][1] >> mAxis[i][2];
	}

	in >> mExtent[0] >> mExtent[1] >> mExtent[2];
	in >> mComponentId[0] >> mComponentId[1];
	in >> mConfidence;
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

void BoxFromRectangles::GetGeometry(vector<Eigen::Vector3f>& vertices, vector<Eigen::Vector3i>& faces)
{
	if (mVertices.empty())
		generateBoxGeometry();
	vertices = mVertices;
	faces = mFaces;
}

bool BoxFromRectangles::constructBoxOrthogonalRectangles(const PartRectangle& rect1, const PartRectangle& rect2, float* score)
{
	float a1 = rect1.Area();
	float a2 = rect2.Area();
	float largeArea = a1 > a2 ? a1 : a2;
	float smallArea = a1 > a2 ? a2 : a1;
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
	if (score)
	{
		//*score = 0.5 * (largeArea / (4 * mExtent[1] * mExtent[2]) + smallArea / (4 * mExtent[0] * mExtent[2]));
		*score = (a1 + a2) / TotalArea();
	}

	return true;
}
bool BoxFromRectangles::constructBoxParallelRectangles(const PartRectangle& rect1, const PartRectangle& rect2, float* score)
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
	if (score)
	{
		//*score = 0.5 * (a1 + a2) / (4 * mExtent[1] * mExtent[2]);
		*score = (a1 + a2) / TotalArea();
	}
	return true;
}

void BoxFromRectangles::computeCenterAndExtent(const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f> axes, Eigen::Vector3f* center, Eigen::Vector3f* extent)
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

float BoxFromRectangles::TotalArea() const
{
	return 8 * (mExtent[0] * mExtent[1] + mExtent[1] * mExtent[2] + mExtent[2] * mExtent[0]);
}

int BoxFromRectangles::ComponentId(int ithId) const
{
	return mComponentId[ithId];
}
float BoxFromRectangles::Confidence() const
{
	return mConfidence;
}

BoxFromRectangles BoxFromRectangles::MirroredBox(const UtilPlane& pl) const
{
	BoxFromRectangles ret = *this;
	ret.mVertices.clear();
	ret.mFaces.clear();
	ret.mCenter = pl.MirrorPoint(mCenter);
	for (int i = 0; i < 3; ++i)
		ret.mAxis[i] = pl.MirrorVector(mAxis[i]);
	return ret;
}

bool BoxFromRectangles::IsBoxSimilar(const BoxFromRectangles& rhs) const
{
	const float centerOffsetPercentageThreshold = 0.75;
	const float extentPercentageThreshold = 0.5;
	const float axisCosineThreshold = 0.9;
	Eigen::Vector3f centerOffset = rhs.mCenter - mCenter;

	Eigen::Matrix3f localCoord;
	for (int i = 0; i < 3; ++i)
		localCoord.col(i) = mAxis[i];
	Eigen::Vector3f centerOffsetLocal = localCoord.transpose() * centerOffset;
	if (abs(centerOffsetLocal[0]) > centerOffsetPercentageThreshold * mExtent[0] 
	 || abs(centerOffsetLocal[1]) > centerOffsetPercentageThreshold * mExtent[1]
	 || abs(centerOffsetLocal[2]) > centerOffsetPercentageThreshold * mExtent[2])
	{
		return false;
	}
	Eigen::Vector3i axisCorrespondence;
	Eigen::Vector3f axisCosine = Eigen::Vector3f::Zero();
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			float cosValue = abs(rhs.mAxis[i].dot(mAxis[j]));
			if (cosValue >= axisCosine[i])
			{
				axisCosine[i] = cosValue;
				axisCorrespondence[i] = j;
			}
		}
	}
	CHECK(axisCorrespondence[0] != axisCorrespondence[1] && axisCorrespondence[0] != axisCorrespondence[2] && axisCorrespondence[2] != axisCorrespondence[1]) << "Two different axes are mapped to the same axis in BoxFromRectangles::IsBoxSimilar().";
	if (axisCosine[0] < axisCosineThreshold || axisCosine[1] < axisCosineThreshold || axisCosine[2] < axisCosineThreshold)
	{
		return false;
	}
	for (int i = 0; i < 3; ++i)
	{
		if (abs(mExtent[i] - rhs.mExtent[axisCorrespondence[i]]) > extentPercentageThreshold * mExtent[i])
			return false;
	}
	return true;
}

const Eigen::Vector3f& BoxFromRectangles::GetCenter() const
{
	return mCenter;
}

const vector<Eigen::Vector3f>& BoxFromRectangles::GetAxes() const
{
	return mAxis;

}