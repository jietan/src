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

class DepthImageInpainting
{
public:
	DepthImageInpainting();
	~DepthImageInpainting();
	void SetDepthImage(MultilayerDepthImage* depthImage);
	void SetMaskImage(vector<vector<vector<int> > >* maskImage);
	void Inpaint(int patchWidth);
	void SaveResultImage(const string& filename);
	int GetPixelFeatureDim() const;
	int GetPatchFeatureDim() const;
private:
	void gatherFeatures();
	void gatherHolesForLayer(int layer);
	void reconstructHoleDepth(int layer);
	void select();
	void vote();
	Eigen::Vector3i findNearestPatch(const Eigen::Vector3i& coord);
	Eigen::VectorXf computeFeatureForPatch(const Eigen::Vector3i& coord);
	Eigen::VectorXf computeFeatureForPixel(const Eigen::Vector3i& coord);
	void computeFeatureImage();
	bool checkPatchValidity(const Eigen::Vector3i & coord);
	bool checkPixelValidity(const Eigen::Vector3i & coord);
	Eigen::SparseMatrix<float> constructPoissonLhs(int layer);
	Eigen::VectorXf constructPoissonRhs(int layer);
	void computeFilledPixelNormals();
	void recomputeHoleFeatureImage(int layer);
	MultilayerDepthImage* mDepthImage;
	vector<vector<vector<int> > >* mMaskImage;
	vector<vector<vector<Eigen::VectorXf> > > mFeatureImage;
	MultilayerDepthImage mResultImage;
	int mHeight;
	int mWidth;
	int mLayers;
	int mPatchWidth;
	vector<Eigen::VectorXf> mFeatures;
	vector<Eigen::Vector3i> mFeatureCoordinates;
	vector<Eigen::Vector3i> mHolePixelCoordinates;
	Eigen::MatrixXi mHolePixelIdx;
	vector<Eigen::Vector3i> mHolePatchCoordinates;
	vector<Eigen::VectorXf> mHolePatchFeatures;
	vector<float> mHoleFilledDepth;
	sehoon::ann::KDTree mKDTree;


};

#endif