#ifndef _BOX_VOTER_H
#define _BOX_VOTER_H

#include <vector>
#include <set>
#include <map>
using namespace std;
#include "Part.h"
#include "BoxFromRectangles.h"


typedef set<int> PartsInClusterT;
typedef set<int> BoxesInClusterT;

class ParallelPartGroup
{
public:
	vector<int> mPartIdx;
	Eigen::Vector3f mNormal;
	float mArea;

	bool operator< (const ParallelPartGroup& rhs) const
	{
		return mArea > rhs.mArea;
	}
};

class BoxVoter
{
public:
	BoxVoter();
	~BoxVoter();
	void Vote(const vector<BoxFromRectangles>& boxes, const vector<Part>& parts, vector<BoxFromRectangles>* results);
private:
	//void clusterPartsToBoxes(const vector<BoxFromRectangles>& boxes, const map<pair<int, int>, int>& componentToBox);
	void voteBox(const vector<BoxFromRectangles>& boxes, const vector<Part>& parts, const pair<PartsInClusterT, BoxesInClusterT>& rectangleCluster, BoxFromRectangles* result);
	void computeAxis(const vector<Part>& parts, vector<Eigen::Vector3f>* axes);
	void computeCenterAndExtent(const vector<Part>& parts, const vector<Eigen::Vector3f> axes, float confidenceInteval, Eigen::Vector3f* center, Eigen::Vector3f* extent);
	void groupParallelRectangles(const vector<Part>& parts, vector<ParallelPartGroup>* parallelPartGroup);
	//void searchClusters(int ithComponent, int jthComponent, int* clusterI, int* clusterJ);
	//void addToCluster(const vector<BoxFromRectangles>& boxes, int boxId, int clusterId);
	//void newCluster(const vector<BoxFromRectangles>& boxes, int boxId);
	//void mergeCluster(const vector<BoxFromRectangles>& boxes, int boxId, int clusterId1, int clusterId2);
	bool checkBoxSimilarity(const vector<BoxFromRectangles>& boxes, int boxId1, int boxId2);
	//vector<vector<int> > mPartInBox;
	//vector<set<int> >  mBoxParts;
	//vector<vector<int> > mBoxesInCluster;
	vector<set<int> > mConnectedParts;
	vector<set<int> > mConnectedBoxes;
	set<pair<PartsInClusterT, BoxesInClusterT> > mClusters;
	map<pair<int, int>, int> mComponentsToBox;
};

#endif