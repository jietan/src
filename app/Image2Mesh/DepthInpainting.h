#ifndef _DEPTH_INPAINTING_H
#define _DEPTH_INPAINTING_H

#include <Eigen/Dense>
using namespace Eigen;

#include <vector>
using namespace std;

#include "MultilayerDepthImage.h"
#include "extendedDepthPixel.h"
#include "utility/utility.h"

class DepthImageInpainting
{
public:
	DepthImageInpainting();
	~DepthImageInpainting();
	void SetDepthImage(MultilayerDepthImage* depthImage);
	void SetMaskImage(vector<vector<vector<int> > >* maskImage);
	void Inpaint(int patchWidth);
	void SaveResultImage(const string& filename);

private:
	void gatherFeatures();
	void gatherHolesForLayer(int layer);
	void reconstructHoleDepth();
	void select();
	void vote();
	Vector3i findNearestPatch(const Vector3i& coord);
	VectorXf computeFeatureForPatch(const Vector3i& coord);
	VectorXf computeFeatureForPixel(const Vector3i& coord);
	void computeFilledPixelNormals();
	MultilayerDepthImage* mDepthImage;
	vector<vector<vector<int> > >* mMaskImage;
	MultilayerDepthImage mResultImage;
	int mHeight;
	int mWidth;
	int mLayers;
	int mPatchWidth;
	vector<VectorXf> mFeatures;
	vector<Vector3i> mFeatureCoordinates;
	vector<Vector3i> mHolePixelCoordinates;
	MatrixXi mHolePixelIdx;
	vector<Vector3i> mHolePatchCoordinates;
	vector<VectorXf> mHolePatchFeatures;
	vector<float> mHoleFilledDepth;
};

#endif