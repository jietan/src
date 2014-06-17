#ifndef _DEPTH_INPAINTING_H
#define _DEPTH_INPAINTING_H

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <vector>
using namespace std;

#include "MultilayerDepthImage.h"
#include "extendedDepthPixel.h"
#include "utility/utility.h"
#include "sehoon/ANNHelper.h"

class DepthCamera;

class DepthImageInpainting
{
public:
	DepthImageInpainting();
	~DepthImageInpainting();
	void SetDepthImage(MultilayerDepthImage* depthImage);
	void SetMaskImage(MultilayerMaskImage* maskImage);
	void Inpaint(int patchWidth);
	void SaveResultImage(const string& filename);
	int GetPixelFeatureDim() const;
	int GetPatchFeatureDim() const;
	void SetCamera(DepthCamera* cam);
	const MultilayerDepthImage& GetResultImage() const;
	
private:
	void generatePyramids(int numPyramids);
	void gatherFeatures();
	void gatherHolesForLayer(int ithPyramid, int layer);
	void reconstructHoleDepth(int layer);
	void select();
	void vote();
	void upsample(int ithPyramid, int ithLayer);
	void DepthImageInpainting::visualizeInpaintedFeatures(int ithPyramid, int ithLayer, int ithIteration);
	void DepthImageInpainting::visualizeInpaintedMLDI(int ithIteration);
	Eigen::Vector3i findNearestPatch(const Eigen::Vector3i& coord);
	Eigen::VectorXf computeFeatureForPatch(const Eigen::Vector3i& coord, bool includeUnknownRegion);
	Eigen::VectorXf computeFeatureForPixel(const Eigen::Vector3i& coord, bool correctDistortion, bool includeUnknownRegion);
	void computeFeatureImage(int ithPyramid);
	bool checkPatchValidity(const Eigen::Vector3i & coord, bool includeUnknownRegion);
	bool checkPixelValidity(const Eigen::Vector3i & coord, bool includeUnknownRegion);
	Eigen::SparseMatrix<double> constructPoissonLhs(int layer);
	Eigen::VectorXd constructPoissonRhs(int layer);
	void computeFilledPixelNormals(int ithLayer);
	void recomputeHoleFeatureImage(int layer);
	void expand(int ithRow, int jthCol, int type, Eigen::MatrixXi& holeType);
	void saveInpaintFeatures();
	void readInpaintedFeatures();
	MultilayerDepthImage* mDepthImage;
	MultilayerMaskImage * mMaskImage;

	MultilayerImage<Eigen::VectorXf> mFeatureImage;
	MultilayerImage<Eigen::VectorXf> mFeatureImagePrevPyramid;

	MultilayerDepthImage mCurrentDepthImage;
	MultilayerMaskImage mCurrentMaskImage;

	int mLayers;
	int mPatchWidth;
	vector<Eigen::VectorXf> mFeatures;
	vector<Eigen::Vector3i> mFeatureCoordinates;
	vector<Eigen::Vector3i> mHolePixelCoordinates;
	vector<vector<int> > mHolePixelIdx;
	vector<Eigen::Vector3i> mHolePatchCoordinates;
	vector<Eigen::VectorXf> mHolePatchFeatures;
	vector<float> mHoleFilledDepth;
	sehoon::ann::KDTree mKDTree;
	DepthCamera* mCamera;
	vector<MultilayerDepthImage> mDepthPyramid;
	vector<MultilayerMaskImage> mMaskPyramid;


};

#endif