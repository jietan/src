#include "BoxVoter.h"

BoxVoter::BoxVoter()
{

}
BoxVoter::~BoxVoter()
{

}
void BoxVoter::Vote(const vector<BoxFromRectangles>& boxes, const vector<Part>& parts, vector<BoxFromRectangles>* results)
{
	int numBoxes = static_cast<int>(boxes.size());
	int numParts = static_cast<int>(parts.size());
	mConnectedParts.resize(numParts);
	mConnectedBoxes.resize(numParts);
	for (int i = 0; i < numBoxes; ++i)
	{
		int ithComponent = boxes[i].ComponentId(0);
		int jthComponent = boxes[i].ComponentId(1);
		mConnectedParts[ithComponent].insert(ithComponent);
		mConnectedParts[ithComponent].insert(jthComponent);
		mConnectedParts[jthComponent].insert(jthComponent);
		mConnectedParts[jthComponent].insert(ithComponent);
		mConnectedBoxes[ithComponent].insert(i);
		mConnectedBoxes[jthComponent].insert(i);
	}
	bool isUnionFinished = false;
	while (!isUnionFinished)
	{
		vector<set<int> > tmpConnectedParts = mConnectedParts;
		vector<set<int> > tmpConnectedBoxes = mConnectedBoxes;
		for (int i = 0; i < numParts; ++i)
		{
			if (tmpConnectedParts[i].empty()) continue;
			
			for (set<int>::const_iterator it = tmpConnectedParts[i].begin(); it != tmpConnectedParts[i].end(); ++it)
			{
				mConnectedParts[i].insert(tmpConnectedParts[*it].begin(), tmpConnectedParts[*it].end());
				mConnectedBoxes[i].insert(tmpConnectedBoxes[*it].begin(), tmpConnectedBoxes[*it].end());
			}
		}
		isUnionFinished = true;
		for (int i = 0; i < numParts; ++i)
		{
			if (tmpConnectedParts[i].size() != mConnectedParts[i].size())
				isUnionFinished = false;
		}
	}
	mClusters.clear();
	for (int i = 0; i < numParts; ++i)
	{
		if (mConnectedParts[i].empty())
			continue;
		mClusters.insert(pair<PartsInClusterT, BoxesInClusterT>(mConnectedParts[i], mConnectedBoxes[i]));
	}
	
	for (set<pair<PartsInClusterT, BoxesInClusterT> >::const_iterator it = mClusters.begin(); it != mClusters.end(); ++it)
	{
		for (set<int>::const_iterator it1 = it->first.begin(); it1 != it->first.end(); ++it1)
			cout << " " << *it1;
		cout << ":";
		for (set<int>::const_iterator it1 = it->second.begin(); it1 != it->second.end(); ++it1)
		{
			cout << " " << *it1;
		}
		cout << endl;
	}


	printf("hello");
	//for (int i = 0; i < numParts; ++i)
	//{
	//	if (mConnectedParts[i].empty()) continue;
	//	cout << i << ": ";
	//	for (set<int>::const_iterator it = mConnectedParts[i].begin(); it != mConnectedParts[i].end(); ++it)
	//		cout << " " << *it;
	//	cout << endl;
	//}
	//for (int i = 0; i < numBoxes; ++i)
	//{
	//	mComponentsToBox[pair<int, int>(boxes[i].ComponentId(0), boxes[i].ComponentId(1))] = i;
	//}
	//mPartInBox.resize(numParts);
	//clusterPartsToBoxes(boxes, mComponentsToBox, &mPartInBox, &mBoxParts, &mBoxesInCluster);
	
	for (set<pair<PartsInClusterT, BoxesInClusterT> >::const_iterator it = mClusters.begin(); it != mClusters.end(); ++it)
	{
		BoxFromRectangles newBox;
		voteBox(boxes, parts, *it, &newBox);
		results->push_back(newBox);
	}
}

void BoxVoter::voteBox(const vector<BoxFromRectangles>& boxes, const vector<Part>& parts, const pair<PartsInClusterT, BoxesInClusterT>& rectangleCluster, BoxFromRectangles* result)
{
	if (rectangleCluster.first.size() == 2)
	{
		CHECK(rectangleCluster.second.size() == 1) << "rectangleCluster.second.size() should be 1, Something wrong in BoxVoter::voteBox().";
		*result = boxes[*rectangleCluster.second.begin()];
	}
	else
	{
		vector<Part> partsToConsider;
		vector<int> componentIds;
		for (PartsInClusterT::const_iterator it = rectangleCluster.first.begin(); it != rectangleCluster.first.end(); ++it)
		{
			partsToConsider.push_back(parts[*it]);
			componentIds.push_back(*it);
		}
		vector<Eigen::Vector3f> axes;
		Eigen::Vector3f center, extent;
		computeAxis(partsToConsider, &axes);
		computeCenterAndExtent(partsToConsider, axes, 1.0f, &center, &extent);
		result->SetAxes(axes);
		result->SetCenter(center);
		result->SetExtent(extent);
		result->SetComponentIds(componentIds);
		result->SetValid(true);
		int numComponents = static_cast<int>(componentIds.size());
		for (int i = 0; i < numComponents; ++i)
		{
			result->mPoints.insert(result->mPoints.end(), partsToConsider[i].GetPoints().begin(), partsToConsider[i].GetPoints().end());
			result->mNormals.insert(result->mNormals.end(), partsToConsider[i].GetNormals().begin(), partsToConsider[i].GetNormals().end());
		}
		
	}
}

void BoxVoter::groupParallelRectangles(const vector<Part>& parts, vector<ParallelPartGroup>* parallelPartGroup)
{
	parallelPartGroup->clear();
	int numParts = static_cast<int>(parts.size());
	const float parallelCosThreshold = 0.9f;
	for (int i = 0; i < numParts; ++i)
	{
		int numGroups = static_cast<int>(parallelPartGroup->size());
		bool bNotInGroup = true;
		for (int j = 0; j < numGroups; ++j)
		{
			if (abs(parts[i].GetNormal().dot(parallelPartGroup->at(j).mNormal)) > parallelCosThreshold)
			{
				float sign = 1;
				if (parts[i].GetNormal().dot(parallelPartGroup->at(j).mNormal) < 0)
					sign = -1;
				float currentArea = parts[i].GetPoints().size();
				parallelPartGroup->at(j).mPartIdx.push_back(i);
				parallelPartGroup->at(j).mNormal = parallelPartGroup->at(j).mArea * parallelPartGroup->at(j).mNormal + currentArea * sign * parts[i].GetNormal();
				parallelPartGroup->at(j).mNormal.normalize();
				parallelPartGroup->at(j).mArea += currentArea;
				bNotInGroup = false;
			}
		}
		if (bNotInGroup)
		{
			ParallelPartGroup newGroup;
			newGroup.mPartIdx.push_back(i);
			newGroup.mArea = parts[i].GetPoints().size();
			newGroup.mNormal = parts[i].GetNormal();
			parallelPartGroup->push_back(newGroup);
		}
	}
	sort(parallelPartGroup->begin(), parallelPartGroup->end());
	CHECK(parallelPartGroup->size() <= 3) << "More than three parallel rectangles in the same box. Might need to loosen the threhold of parallel";
}

void BoxVoter::refineAxis(const vector<Part>& parts, vector<Eigen::Vector3f>* axes)
{
	const float rotateAngleBound = 5 * CV_PI / 180;
	const float rotateAngleResolution = 1 * CV_PI / 180;
	//int numParts = static_cast<int>(parts.size());
	//vector<Eigen::Vector3f> allPoints;
	//for (int i = 0; i < numParts; ++i)
	//{
	//	allPoints.insert(allPoints.end(), parts[i].GetPoints().begin(), parts[i].GetPoints().end());
	//}
	float minVolume = FLT_MAX;
	vector<Eigen::Vector3f> bestAxis;
	Eigen::Matrix3f currentCoord;
	for (int i = 0; i < 3; ++i)
	{
		currentCoord.col(i) = axes->at(i);
	}
	for (float currentAngle = -rotateAngleBound; currentAngle <= rotateAngleBound; currentAngle += rotateAngleResolution)
	{
		Eigen::Vector3f center;
		Eigen::Vector3f extent;

		Eigen::Vector3f axis1Local(0, cos(currentAngle), sin(currentAngle));
		Eigen::Vector3f axis2Local(0, -sin(currentAngle), cos(currentAngle));
		vector<Eigen::Vector3f> candidateAxis;
		candidateAxis.resize(3);
		candidateAxis[0] = axes->at(0);
		candidateAxis[1] = currentCoord * axis1Local;
		candidateAxis[2] = currentCoord * axis2Local;
		computeCenterAndExtent(parts, candidateAxis, 1.0, &center, &extent);
		float volume = extent[0] * extent[1] * extent[2];
		if (volume < minVolume)
		{
			minVolume = volume;
			bestAxis = candidateAxis;
		}
	}
	*axes = bestAxis;
}

void BoxVoter::computeAxis(const vector<Part>& parts, vector<Eigen::Vector3f>* axes)
{
	//axes->resize(3);
	//vector<Eigen::Vector3f> allPoints;
	//int numParts = static_cast<int>(parts.size());
	//vector<ParallelPartGroup> parallelPartGroup;
	//groupParallelRectangles(parts, &parallelPartGroup);

	//for (int i = 0; i < numParts; ++i)
	//{
	//	allPoints.insert(allPoints.end(), parts[i].GetPoints().begin(), parts[i].GetPoints().end());
	//}
	//Eigen::Vector3f center;
	//PCAOnPoints(allPoints, center, *axes);
	//if (axes->at(0).cross(axes->at(1)).dot(axes->at(2)) < 0)
	//	axes->at(2) = -axes->at(2);
	//return;

	axes->resize(3);
	vector<ParallelPartGroup> parallelPartGroup;
	groupParallelRectangles(parts, &parallelPartGroup);
	//if (parallelPartGroup.size() == 1)
	//{
	//	vector<Eigen::Vector3f> allPoints;
	//	int numPartsInGroup = static_cast<int>(parallelPartGroup[0].mPartIdx.size());
	//	for (int i = 0; i < numPartsInGroup; ++i)
	//	{
	//		allPoints.insert(allPoints.end(), parts[parallelPartGroup[0].mPartIdx[i]].GetPoints().begin(), parts[parallelPartGroup[0].mPartIdx[i]].GetPoints().end());
	//	}
	//	int numPoints = static_cast<int>(allPoints.size());
	//	for (int i = 0; i < numPoints; ++i)
	//	{
	//		const Eigen::Vector3f& pt = allPoints[i];
	//		Eigen::Vector3f projPt = pt - pt.dot(parallelPartGroup[0].mNormal) * parallelPartGroup[0].mNormal;
	//		allPoints[i] = projPt;

	//	}
	//	Eigen::Vector3f mean;
	//	//Eigen::Vector3f axes[3];
	//	PCAOnPoints(allPoints, mean, *axes);
	//	if ((*axes)[0].cross((*axes)[1]).dot((*axes)[2]) < 0)
	//		(*axes)[2] = -(*axes)[2];

	//}
	//else if (parallelPartGroup.size() == 2 || parallelPartGroup.size() == 3)
	//{
	//	(*axes)[0] = parallelPartGroup[0].mNormal;
	//	(*axes)[1] = parallelPartGroup[1].mNormal;
	//	(*axes)[2] = (*axes)[0].cross((*axes)[1]);
	//	(*axes)[2].normalize();
	//	(*axes)[1] = (*axes)[2].cross((*axes)[0]);
	//	(*axes)[1].normalize();
	//}
	if (parallelPartGroup.size() <= 3)
	{
		vector<Eigen::Vector3f> allPoints;
		int numParts = static_cast<int>(parts.size());
		for (int i = 0; i < numParts; ++i)
		{
			allPoints.insert(allPoints.end(), parts[i].GetPoints().begin(), parts[i].GetPoints().end());
		}
		int numPoints = static_cast<int>(allPoints.size());
		for (int i = 0; i < numPoints; ++i)
		{
			const Eigen::Vector3f& pt = allPoints[i];
			Eigen::Vector3f projPt = pt - pt.dot(parallelPartGroup[0].mNormal) * parallelPartGroup[0].mNormal;
			allPoints[i] = projPt;

		}
		Eigen::Vector3f mean;
		//Eigen::Vector3f axes[3];
		PCAOnPoints(allPoints, mean, *axes);
		if ((*axes)[0].cross((*axes)[1]).dot((*axes)[2]) < 0)
			(*axes)[2] = -(*axes)[2];
		//	int numPartsInGroup = static_cast<int>(parallelPartGroup[0].mPartIdx.size());
		//	for (int i = 0; i < numPartsInGroup; ++i)
		//	{
		//		allPoints.insert(allPoints.end(), parts[parallelPartGroup[0].mPartIdx[i]].GetPoints().begin(), parts[parallelPartGroup[0].mPartIdx[i]].GetPoints().end());
		//	}
		//	int numPoints = static_cast<int>(allPoints.size());
		//	for (int i = 0; i < numPoints; ++i)
		//	{
		//		const Eigen::Vector3f& pt = allPoints[i];
		//		Eigen::Vector3f projPt = pt - pt.dot(parallelPartGroup[0].mNormal) * parallelPartGroup[0].mNormal;
		//		allPoints[i] = projPt;

		//	}
		//	Eigen::Vector3f mean;
		//	//Eigen::Vector3f axes[3];
		//	PCAOnPoints(allPoints, mean, *axes);
		//	if ((*axes)[0].cross((*axes)[1]).dot((*axes)[2]) < 0)
		//		(*axes)[2] = -(*axes)[2];
	}
	else
	{
		CHECK(0) << "Impossible case in BoxVoter::computeAxis().";
	}
	refineAxis(parts, axes);

}

void BoxVoter::computeCenterAndExtent(const vector<Part>& parts, const vector<Eigen::Vector3f> axes, float confidenceInteval, Eigen::Vector3f* center, Eigen::Vector3f* extent)
{
	int numParts = static_cast<int>(parts.size());
	vector<Eigen::Vector3f> points;
	for (int i = 0; i < numParts; ++i)
		points.insert(points.end(), parts[i].GetPoints().begin(), parts[i].GetPoints().end());
	Eigen::Matrix3f axis;
	for (int i = 0; i < 3; ++i)
		axis.row(i) = axes[i].transpose();

	Eigen::Vector3f bbMin(FLT_MAX, FLT_MAX, FLT_MAX);
	Eigen::Vector3f bbMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	int numPoints = static_cast<int>(points.size());
	vector<vector<float> > projOnAxis;
	projOnAxis.resize(3);
	for (int i = 0; i < numPoints; ++i)
	{
		Eigen::Vector3f proj = axis * points[i];
		for (int j = 0; j < 3; ++j)
		{
			projOnAxis[j].push_back(proj[j]);
		}

		//for (int ithDim = 0; ithDim < 3; ++ithDim)
		//{
		//	if (proj[ithDim] < bbMin[ithDim])
		//		bbMin[ithDim] = proj[ithDim];
		//	if (proj[ithDim] > bbMax[ithDim])
		//		bbMax[ithDim] = proj[ithDim];
		//}
	}
	int numLeaveOutEachEnd = static_cast<int>((1.f - confidenceInteval) / 2.f * numPoints);
	for (int i = 0; i < 3; ++i)
	{
		sort(projOnAxis[i].begin(), projOnAxis[i].end());
		bbMin[i] = projOnAxis[i][numLeaveOutEachEnd];
		bbMax[i] = projOnAxis[i][numPoints - numLeaveOutEachEnd - 1];
	}
	*extent = (bbMax - bbMin) / 2.f;
	*center = axis.transpose() * (bbMax + bbMin) / 2.f;

}

bool BoxVoter::checkBoxSimilarity(const vector<BoxFromRectangles>& boxes, int boxId1, int boxId2)
{
	bool from1To2 = boxes[boxId1].IsBoxSimilar(boxes[boxId2]);
	bool from2To1 = boxes[boxId2].IsBoxSimilar(boxes[boxId1]);
	return from1To2 || from2To1;
}
//
//void BoxVoter::clusterPartsToBoxes(const vector<BoxFromRectangles>& boxes, const map<pair<int, int>, int>& componentToBox)
//{
//	int numBoxes = static_cast<int>(boxes.size());
//	for (int i = 0; i < numBoxes; ++i)
//	{
//		int ithComponent = boxes[i].ComponentId(0);
//		int jthComponent = boxes[i].ComponentId(1);
//		int clusterI = -1, clusterJ = -1;
//		searchClusters(ithComponent, jthComponent, &clusterI, &clusterJ);
//		if (clusterI == -1, clusterJ == -1)
//		{
//			newCluster(boxes, i);
//		}
//		else if (clusterI != -1 && clusterJ != -1)
//		{
//			bool similarityToClusterI = checkBoxSimilarity(boxes, i, mBoxesInCluster[clusterI]);
//			bool similarityToClusterJ = checkBoxSimilarity(boxes, i, mBoxesInCluster[clusterJ]);
//			if (!similarityToClusterI && !similarityToClusterJ)
//			{
//				newCluster(boxes, i);
//			}
//			else if (similarityToClusterI && similarityToClusterJ)
//			{
//				mergeCluster(boxes, i, clusterI, clusterJ);
//			}
//			else if (similarityToClusterI)
//			{
//				addToCluster(boxes, i, clusterI);
//			}
//			else if (similarityToClusterJ)
//			{
//				addToCluster(boxes, i, clusterJ);
//			}
//			else
//			{
//				CHECK(0);
//			}
//		}
//		else
//		{
//			int clusterId = clusterI == -1 ? clusterJ : clusterI;
//			if (checkBoxSimilarity(boxes, i, mBoxesInCluster[clusterId]))
//			{
//				addToCluster(boxes, i, clusterId);
//			}
//			else
//			{
//				newCluster(boxes, i);
//			}
//		}
//	}
//}

//void BoxVoter::addToCluster(const vector<BoxFromRectangles>& boxes, int boxId, int clusterId)
//{
//	int ithComponent = boxes[boxId].ComponentId(0);
//	int jthComponent = boxes[boxId].ComponentId(1);
//	set<int>& cluster = mBoxParts[clusterId];
//	cluster.insert(ithComponent);
//	cluster.insert(jthComponent);
//	mBoxesInCluster[clusterId].push_back(boxId);
//	mPartInBox[ithComponent].push_back(clusterId);
//	mPartInBox[jthComponent].push_back(clusterId);
//}
//void BoxVoter::newCluster(const vector<BoxFromRectangles>& boxes, int boxId)
//{
//	int ithComponent = boxes[boxId].ComponentId(0);
//	int jthComponent = boxes[boxId].ComponentId(1);
//	set<int> newCluster;
//	newCluster.insert(ithComponent);
//	newCluster.insert(jthComponent);
//	mBoxParts.push_back(newCluster);
//	vector<int> boxIds;
//	boxIds.push_back(boxId);
//	mBoxesInCluster.push_back(boxIds);
//	mPartInBox[ithComponent].push_back(static_cast<int>(mBoxParts.size()) - 1);
//	mPartInBox[jthComponent].push_back(static_cast<int>(mBoxParts.size()) - 1);
//
//}
//void BoxVoter::mergeCluster(const vector<BoxFromRectangles>& boxes, int boxId, int clusterId1, int clusterId2)
//{
//
//}

//
//void BoxVoter::searchClusters(int ithComponent, int jthComponent, int* clusterI, int* clusterJ)
//{
//
//}
