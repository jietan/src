#ifndef _SYMMETRY_BUILDER_H
#define _SYMMETRY_BUILDER_H
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <vector>
using namespace std;
#include "BoxFromRectangles.h"
#include "utility/mathlib.h"
#include "sehoon/ANNHelper.h"


class MirrorPlaneGenerator
{
public:
	MirrorPlaneGenerator(const Eigen::Vector3f& point, const Eigen::Vector3f& normal, int boxId);
	UtilPlane ToPlane() const;
	Eigen::Vector3f mPoint;
	Eigen::Vector3f mNormal;
	int mBoxId;
};

class SymmetryBuilder
{
public:
	SymmetryBuilder();
	~SymmetryBuilder();
	void SetUpDirection(const Eigen::Vector3f& up);
	bool CheckSymmetry(const vector<BoxFromRectangles>& boxes);
	void RefineSymmetry(int ithSymmetryPlane);
	vector<pair<int, int> > GetSymmetryCorrespondences(int ithSymmetryPlane) const;
	int GetNumSymmetryPlanes() const;
	const UtilPlane& GetSymmetryPlane(int ithSymmetryPlane) const;
	void SaveVisualization(const string& folderName);
	//float GetSymmetryScore() const;
	
private:
	int generateCandidatePlane(const vector<BoxFromRectangles>& boxes, vector<UtilPlane>* planes);
	void generateMirroredBoxes(const vector<BoxFromRectangles>& boxes, const UtilPlane& mirrorPlane, vector<BoxFromRectangles>* mirroredBoxes);
	float evaluateSymmetry(const vector<BoxFromRectangles>& boxes, const vector<BoxFromRectangles>& mirroredBoxes, vector<int>* correspondence);
	float evaluateSymmetryScore(const vector<BoxFromRectangles>& boxes, int ithSymmetryPlane, const UtilPlane& plane);
	void SetPlaneAsZeroXYTheta(const Eigen::Vector3f& point, const Eigen::Vector3f& normal);
	UtilPlane fromXYThetaToPlane(const Eigen::Vector3f& xyTheta);
	void buildKDTreeForPointsInBoxes();
	float evaluateSymmetryScoreForPointsInBoxes(const vector<BoxFromRectangles>& boxes, int originalId, int tobeReflectedId, const UtilPlane& plane);
	bool isCorrespondenceSame(const vector<int>& correspondence1, const vector<int>& correspondence2);
	vector<BoxFromRectangles> mBoxes;
	vector<vector<BoxFromRectangles> > mMirroredBoxes;
	vector<UtilPlane> mCandidateMirrorPlanes;
	vector<MirrorPlaneGenerator> mCandidateMirrorPlaneGenerators;
	vector<vector<int> > mCandidateSymmetryCorrespondences;
	vector<float> mSymmetryScore;
	Eigen::Vector3f mUp;
	float mMaxScore;
	//int mSymmetryPlaneId;
	vector<UtilPlane> mActualMirrorPlanes;
	vector<vector<int> > mActualSymmetryCorrespondences;
	vector<MirrorPlaneGenerator> mActualMirrorPlaneGenerators;
	Eigen::Vector3f mXYThetaOrigin;
	Eigen::Matrix3f mXYThetaAxes;
	vector<sehoon::ann::KDTree*> mKDTreesPointCloud;

};

#endif