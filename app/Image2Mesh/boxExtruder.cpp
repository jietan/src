#include "boxExtruder.h"




Extruder::Extruder()
{

}
bool Extruder::Extrude(const BoxFromRectangles& box, const vector<Part>& parts, vector<Eigen::Vector3f>* extrudedPoints, vector<Eigen::Vector3f>* extrudedNormals)
{
	if (!needExtrude(box, parts))
		return false;

	vector<Eigen::Vector3f> sourcePoints;
	vector<Eigen::Vector3f> sourceNormals;
	int sourceComponent;
	findExtrudeSource(box, parts, &sourceComponent, &sourcePoints, &sourceNormals);
	Eigen::Vector3f extrudeVector = findExtrudeVector(box, parts, sourceComponent, sourcePoints);

	extrude(sourcePoints, sourceNormals, extrudeVector, extrudedPoints, extrudedNormals);
	return true;
}

bool Extruder::needExtrude(const BoxFromRectangles& box, const vector<Part>& parts) const
{
	if (!isBoardShape(box))
		return false;
	int componentId = findExtrudeSourceComponent(box, parts);
	return componentId != -1;
}
bool Extruder::isBoardShape(const BoxFromRectangles& box) const
{
	
	const Eigen::Vector3f& ext = box.GetExtents();
	vector<float> extents;
	extents.resize(3);
	Eigen::Map<Eigen::Vector3f>(extents.data()) = ext;
	sort(extents.begin(), extents.end());
	//float middle = (extents[0] + extents[2]) / 2.0;
	//if (middle >= extents[1])
	//{
	//	return false;
	//}
	//else
	//{
	//	return true;
	//}
	if (extents[1] / extents[0] > extents[2] / extents[1])
	{
		return true;
	}
	else
	{
		return false;
	}
}

int Extruder::findExtrudeSourceComponent(const BoxFromRectangles& box, const vector<Part>& parts) const
{
	int sourceComponentId = -1;
	
	Eigen::Vector3f extrudeDir;
	float extent;
	findThinBoardAxisAndExtent(box, &extrudeDir, &extent);

	int numComponents = box.GetNumComponents();
	for (int i = 0; i < numComponents; ++i)
	{
		int componentId = box.ComponentId(i);
		if (abs(parts[componentId].GetNormal().dot(extrudeDir)) > 0.9f)
		{
			if (sourceComponentId == -1)
				sourceComponentId = componentId;
			else
				sourceComponentId = - 1;
		}
	}
	return sourceComponentId;
}

void Extruder::findThinBoardAxisAndExtent(const BoxFromRectangles& box, Eigen::Vector3f* axis, float* extent) const
{
	vector<ExtentAxisPair> extentAxes;
	const Eigen::Vector3f extents = box.GetExtents();
	const vector<Eigen::Vector3f>& axes = box.GetAxes();
	for (int i = 0; i < 3; ++i)
	{
		extentAxes.push_back(ExtentAxisPair(extents[i], axes[i]));
	}
	sort(extentAxes.begin(), extentAxes.end());
	*axis = extentAxes[0].mPair.second;
	*extent = extentAxes[0].mPair.first;
}
Eigen::Vector3f Extruder::findExtrudeVector(const BoxFromRectangles& box, const vector<Part>& parts, int sourceComponentId, const vector<Eigen::Vector3f>& sourcePoints) const
{

	Eigen::Vector3f extrudeDir;
	float extent;
	findThinBoardAxisAndExtent(box, &extrudeDir, &extent);
	if (extrudeDir.dot(parts[sourceComponentId].GetNormal()) > 0)
		extrudeDir = -extrudeDir;


	int numSourcePoints = static_cast<int>(sourcePoints.size());
	float minProj = FLT_MAX, maxProj = -FLT_MAX;
	for (int i = 0; i < numSourcePoints; ++i)
	{
		float proj = sourcePoints[i].dot(extrudeDir);
		if (proj < minProj)
			minProj = proj;
		if (proj > maxProj)
			maxProj = proj;
	}
	float partExtent = (maxProj - minProj) / 2.f;
	float extrudeDepth = (extent - partExtent) * 2.f;
	CHECK(extrudeDepth >= 0) << "No room for extrusion in Extruder::findExtrudeVector().";
	extrudeDir *= extrudeDepth;
	return extrudeDir;
}

void Extruder::findExtrudeSource(const BoxFromRectangles& box, const vector<Part>& parts, int* ithComponent, vector<Eigen::Vector3f>* sourcePoints, vector<Eigen::Vector3f>* sourceNormals) const
{
	*ithComponent = findExtrudeSourceComponent(box, parts);
	CHECK(*ithComponent != -1) << "No component can be extruded in Extruder::findExtrudeSource().";
	*sourcePoints = parts[*ithComponent].GetPoints();
	*sourceNormals = parts[*ithComponent].GetNormals();
}
void Extruder::extrude(const vector<Eigen::Vector3f>& sourcePoints, const vector<Eigen::Vector3f>& sourceNormals, const Eigen::Vector3f& extrudeVector, vector<Eigen::Vector3f>* extrudedPoints, vector<Eigen::Vector3f>* extrudedNormals)
{
	int numPoints = static_cast<int>(sourcePoints.size());
	for (int i = 0; i < numPoints; ++i)
	{
		Eigen::Vector3f newPoints = sourcePoints[i] + extrudeVector;
		Eigen::Vector3f newNormals = -sourceNormals[i];
		extrudedPoints->push_back(newPoints);
		extrudedNormals->push_back(newNormals);
	}
}
