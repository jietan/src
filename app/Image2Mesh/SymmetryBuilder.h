#ifndef _SYMMETRY_BUILDER_H
#define _SYMMETRY_BUILDER_H

#include <Eigen/Dense>
#include <vector>
using namespace std;
#include "BoxFromRectangles.h"
#include "utility/mathlib.h"

class SymmetryBuilder
{
public:
	SymmetryBuilder();
	void SetUpDirection(const Eigen::Vector3f& up);
	bool CheckSymmetry(const vector<BoxFromRectangles>& boxes);
	vector<pair<int, int> > GetSymmetryCorrespondences() const;
	const UtilPlane& GetSymmetryPlane() const;
	void SaveVisualization(const string& folderName);
	//float GetSymmetryScore() const;
	
private:
	int generateCandidatePlane(const vector<BoxFromRectangles>& boxes, vector<UtilPlane>* planes);
	void generateMirroredBoxes(const vector<BoxFromRectangles>& boxes, const UtilPlane& mirrorPlane, vector<BoxFromRectangles>* mirroredBoxes);
	float evaluateSymmetry(const vector<BoxFromRectangles>& boxes, const vector<BoxFromRectangles>& mirroredBoxes, vector<int>* correspondence);

	vector<BoxFromRectangles> mBoxes;
	vector<vector<BoxFromRectangles> > mMirroredBoxes;
	vector<UtilPlane> mMirrorPlanes;
	vector<vector<int> > mSymmetryCorrespondences;
	vector<float> mSymmetryScore;
	Eigen::Vector3f mUp;
	float mMaxScore;
	int mSymmetryPlaneId;

};

#endif