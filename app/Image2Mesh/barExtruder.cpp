#include "barExtruder.h"
#include "utility/mathlib.h"

#define EXTRUDE_NONE 0
#define EXTRUDE_UP 1
#define EXTRUDE_DOWN 2
#define EXTRUDE_BOTH 3

BarExtruder::BarExtruder() : mWall(NULL), mFloor(NULL)
{

}

void BarExtruder::SetWallAndFloor(PrimitiveShape* wall, PrimitiveShape* floor)
{
	mWall = wall;
	mFloor = floor;
}
void BarExtruder::SetReferenceDepthImages(const vector<DepthImage*> refImages)
{
	mRefDepthImages = refImages;
}
bool BarExtruder::Extrude(const BoxFromRectangles& box, const vector<Part>& parts, vector<Eigen::Vector3f>* extrudedPoints, vector<Eigen::Vector3f>* extrudedNormals)
{
	int dir = needExtrude(box);
	if (!dir)
		return false;
	Eigen::Vector3f longAxis;
	float longExtent;
	findBarLongAxisAndExtent(box, &longAxis, &longExtent);

	vector<Eigen::Vector3f> candidatePoints;
	vector<Eigen::Vector3f> candidateNormals;
	if (dir & EXTRUDE_UP)
	{
		float extrudeAmount = findExtrudeAmount(box, EXTRUDE_UP);
		extrude(box, extrudeAmount * longAxis, &candidatePoints, &candidateNormals, false);
	}
	if (dir & EXTRUDE_DOWN)
	{
		float extrudeAmount = findExtrudeAmount(box, EXTRUDE_DOWN);
		extrude(box, -extrudeAmount * longAxis, &candidatePoints, &candidateNormals, false);
	}
	int numCandidatePoints = static_cast<int>(candidatePoints.size());
	for (int i = 0; i < numCandidatePoints; ++i)
	{
		if (!isPointVisible(candidatePoints[i]))
		{
			extrudedPoints->push_back(candidatePoints[i]);
			extrudedNormals->push_back(candidateNormals[i]);
		}
	}
	
	return dir > 0;
}


int BarExtruder::needExtrude(const BoxFromRectangles& box) const
{
	int ret = 0;
	const float unseenRatioThreshold = 0.8;
	if (!isBarShape(box))
		return EXTRUDE_NONE;
	Eigen::Vector3f longAxis;
	float longExtent;
	findBarLongAxisAndExtent(box, &longAxis, &longExtent);
	
	vector<Eigen::Vector3f> extrudedPoints;
	vector<Eigen::Vector3f> extrudedNormals;
	float extrudeAmount = 0.1;
	extrude(box, extrudeAmount * longAxis, &extrudedPoints, &extrudedNormals);
	float unseenRatio = checkUnseenRatio(extrudedPoints);
	if (unseenRatio > unseenRatioThreshold)
		ret |= EXTRUDE_UP;

	extrudedPoints.clear();
	extrudedNormals.clear();
	extrude(box, -extrudeAmount * longAxis, &extrudedPoints, &extrudedNormals);
	unseenRatio = checkUnseenRatio(extrudedPoints);
	if (unseenRatio > unseenRatioThreshold)
		ret |= EXTRUDE_DOWN;
	return ret;
}
bool BarExtruder::isBarShape(const BoxFromRectangles& box) const
{
	const Eigen::Vector3f& ext = box.GetExtents();
	vector<float> extents;
	extents.resize(3);
	Eigen::Map<Eigen::Vector3f>(extents.data()) = ext;
	sort(extents.begin(), extents.end());

	if (extents[2] / extents[1] < 2.f)
	{
		return false;
	}
	else
	{
		return true;
	}
}


float BarExtruder::findExtrudeAmount(const BoxFromRectangles& box, int dir) const
{
	Eigen::Vector3f longAxis;
	float longExtent;
	findBarLongAxisAndExtent(box, &longAxis, &longExtent);

	int sign = 1;
	if (dir == EXTRUDE_DOWN)
		sign = -1;

	float minAmount = 0.1f;
	float maxAmount = longExtent;
	float midAmount = 0;
	while (abs(maxAmount - minAmount) > EPSILON_FLOAT)
	{
		midAmount = (minAmount + maxAmount) / 2.f;
		vector<Eigen::Vector3f> extrudedPoints;
		vector<Eigen::Vector3f> extrudedNormals;
		extrude(box, sign * midAmount * longAxis, &extrudedPoints, &extrudedNormals);
		float unseenRatio = checkUnseenRatio(extrudedPoints);
		if (unseenRatio > 0.9)
		{
			minAmount = midAmount;
		}
		else
		{
			maxAmount = midAmount;
		}
	}
	return midAmount;
}

void BarExtruder::extrude(const BoxFromRectangles& box, const Eigen::Vector3f& extrudeVector, vector<Eigen::Vector3f>* extrudedPoints, vector<Eigen::Vector3f>* extrudedNormals, bool fromOutSideBox) const
{
	vector<Eigen::Vector3f> sourcePoints;
	vector<Eigen::Vector3f> sourceNormals;

	Eigen::Vector3f longAxis;
	float longExtent;
	findBarLongAxisAndExtent(box, &longAxis, &longExtent);
	float sourceExtent = 0.1 * longExtent;
	float extrudeStep = sourceExtent;
	const Eigen::Vector3f& boxCenter = box.GetCenter();
	int numPoints = static_cast<int>(box.mPoints.size());

	for (int i = 0; i < numPoints; ++i)
	{
		float proj = (box.mPoints[i] - boxCenter).dot(longAxis);
		if (abs(proj) < sourceExtent)
		{
			sourcePoints.push_back(box.mPoints[i]);
			sourceNormals.push_back(box.mNormals[i]);
		}
	}
	Eigen::Vector3f extrudedDir = extrudeVector.normalized();
	float extrudeAmount = extrudeVector.norm() + longExtent;

	numPoints = static_cast<int>(sourcePoints.size());
	for (int i = 0; i < numPoints; ++i)
	{
		//if (extrudedDir.dot(sourceNormals[i]) < 0.2f)
		{
			for (float currentExtrusion = 0; currentExtrusion < extrudeAmount; currentExtrusion += extrudeStep)
			{
				Eigen::Vector3f candidatePoint = sourcePoints[i] + currentExtrusion * extrudedDir;
				if (fromOutSideBox && box.IsPointInBox(candidatePoint))
					continue;

				extrudedPoints->push_back(candidatePoint);
				extrudedNormals->push_back(sourceNormals[i]);

			}
		}
	}
}
void BarExtruder::findBarLongAxisAndExtent(const BoxFromRectangles& box, Eigen::Vector3f* axis, float* extent) const
{
	vector<ExtentAxisPair> extentAxes;
	const Eigen::Vector3f extents = box.GetExtents();
	const vector<Eigen::Vector3f>& axes = box.GetAxes();
	for (int i = 0; i < 3; ++i)
	{
		extentAxes.push_back(ExtentAxisPair(extents[i], axes[i]));
	}
	sort(extentAxes.begin(), extentAxes.end());
	*axis = extentAxes[2].mPair.second;
	*extent = extentAxes[2].mPair.first;
}

bool BarExtruder::isPointVisible(const Eigen::Vector3f& point) const
{
	const float threshold = 0.01;
	if (mWall)
	{
		float signedDist = mWall->SignedDistance(Vec3f(point[0], point[1], point[2]));
		if (signedDist < -threshold)
			return true;
	}
	if (mFloor)
	{
		float signedDist = mFloor->SignedDistance(Vec3f(point[0], point[1], point[2]));
		if (signedDist < -threshold)
			return true;
	}

	int numReferenceCameras = static_cast<int>(mRefDepthImages.size());
	for (int ithCamera = 0; ithCamera < numReferenceCameras; ++ithCamera)
	{
		float deltaDepth;
		bool isOccluded = mRefDepthImages[ithCamera]->IsPointBehind(point, deltaDepth);

		if (!isOccluded)
			return true;
	}

	
	return false;
}
float BarExtruder::checkUnseenRatio(const vector<Eigen::Vector3f>& points) const
{
	int numPoints = static_cast<int>(points.size());
	int unseenCount = 0;
	for (int i = 0; i < numPoints; ++i)
	{
		if (!isPointVisible(points[i]))
			unseenCount++;
	}
	float unseenRatio = static_cast<float>(unseenCount) / numPoints;
	return unseenRatio;
}