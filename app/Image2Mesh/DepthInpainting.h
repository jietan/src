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
	void SetMaskImage(vector<vector<vector<int> > >* maskImage);
	void Inpaint(int patchWidth);
	void SaveResultImage(const string& filename);
	int GetPixelFeatureDim() const;
	int GetPatchFeatureDim() const;
	void SetVisualizationCamera(DepthCamera* cam);
	const MultilayerDepthImage& GetResultImage() const;
private:
	void gatherFeatures();
	void gatherHolesForLayer(int layer);
	void reconstructHoleDepth(int layer);
	void select();
	void vote();
	void DepthImageInpainting::visualizeInpaintedFeatures(int ithLayer, int ithIteration);
	void DepthImageInpainting::visualizeInpaintedMLDI(int ithIteration);
	Eigen::Vector3i findNearestPatch(const Eigen::Vector3i& coord);
	Eigen::VectorXf computeFeatureForPatch(const Eigen::Vector3i& coord, bool includeUnknownRegion);
	Eigen::VectorXf computeFeatureForPixel(const Eigen::Vector3i& coord, bool includeUnknownRegion);
	void computeFeatureImage();
	bool checkPatchValidity(const Eigen::Vector3i & coord, bool includeUnknownRegion);
	bool checkPixelValidity(const Eigen::Vector3i & coord, bool includeUnknownRegion);
	Eigen::SparseMatrix<double> constructPoissonLhs(int layer);
	Eigen::VectorXd constructPoissonRhs(int layer);
	void computeFilledPixelNormals();
	void recomputeHoleFeatureImage(int layer);
	void expand(int ithRow, int jthCol, int type, Eigen::MatrixXi& holeType);
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
	vector<vector<int> > mHolePixelIdx;
	vector<Eigen::Vector3i> mHolePatchCoordinates;
	vector<Eigen::VectorXf> mHolePatchFeatures;
	vector<float> mHoleFilledDepth;
	sehoon::ann::KDTree mKDTree;
	DepthCamera* mVisualizationCam;


};

#endif