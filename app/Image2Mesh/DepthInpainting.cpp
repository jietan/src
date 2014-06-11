#include "DepthInpainting.h"





DepthImageInpainting::DepthImageInpainting()
{

}
DepthImageInpainting::~DepthImageInpainting()
{

}
void DepthImageInpainting::SetDepthImage(MultilayerDepthImage* depthImage)
{
	mDepthImage = depthImage;
	mHeight = depthImage->Height();
	mWidth = depthImage->Width();
	mLayers = depthImage->NumLayers();
}
void DepthImageInpainting::SetMaskImage(vector<vector<vector<int> > >* maskImage)
{
	mMaskImage = maskImage;
}
void DepthImageInpainting::Inpaint(int patchWidth)
{
	mPatchWidth = patchWidth;
	mResultImage = *mDepthImage;
	const int maxNumIterations = 5;
	gatherFeatures();
	for (int ithLayer = 1; ithLayer < 2; ++ithLayer)
	{
		gatherHolesForLayer(ithLayer);

		for (int ithIteration = 0; ithIteration < maxNumIterations; ++ithIteration)
		{
			select();
			vote();
			reconstructHoleDepth();


		}
		computeFilledPixelNormals();
	}
	
}
void DepthImageInpainting::SaveResultImage(const string& filename)
{
	//cv::Mat resultImage(mNumRows, mNumCols, CV_16U);
	//Matrix<ushort, Dynamic, Dynamic> resultImageEigen = mResultImage.cast<ushort>();
	//cv::eigen2cv(resultImageEigen, resultImage);
	//imwrite(filename, resultImage);
}

void DepthImageInpainting::gatherFeatures()
{
	// build vector<VectorXf> mFeatures and vector<Vector3i> mFeatureCoordinates.
	
	for (int l = 0; l < mLayers; ++l)
	{
		for (int i = 0; i < mHeight; ++i)
		{
			for (int j = 0; j < mWidth; ++j)
			{

			}
		}
	}
}
void DepthImageInpainting::gatherHolesForLayer(int layer)
{
	// build vector<Vector3i> mHolePixelCoordinates and MatrixXi mHolePixelIdx.
	CHECK(mMaskImage) << "No mask specified in DepthImageInpainting::gatherHolesForLayer();";
	mHolePixelIdx = MatrixXi::Constant(mHeight, mWidth, -1);
	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			if ((*mMaskImage)[i][j].size() <= layer)
				continue;
			if ((*mMaskImage)[i][j][layer])
			{
				mHolePixelIdx(i, j) = static_cast<int>(mHolePixelCoordinates.size());
				mHolePixelCoordinates.push_back(Vector3i(i, j, layer));
			}
		}
	}
}
void DepthImageInpainting::reconstructHoleDepth(int layer)
{
	// build vector<float> mHoleFilledDepth from vector<VectorXf> mHolePatchFeatures.

}

void DepthImageInpainting::select()
{
	// build vector<Vector3i> mHolePatchCoordinates.
	int numHolePixels = static_cast<int>(mHolePixelCoordinates.size());
	mHolePatchCoordinates.resize(numHolePixels);
	for (int ithPx = 0; ithPx < numHolePixels; ++ithPx)
	{
		mHolePatchCoordinates[ithPx] = findNearestPatch(mHolePixelCoordinates[ithPx] - Vector3i(mPatchWidth / 2, mPatchWidth, 0));
	}
}
void DepthImageInpainting::vote()
{
	// build vector<VectorXf> mHolePatchFeatures.
	int numHolePixels = static_cast<int>(mHolePixelCoordinates.size());
	mHolePatchFeatures.resize(numHolePixels);
	for (int ithPx = 0; ithPx < numHolePixels; ++ithPx)
	{
		mHolePatchFeatures[ithPx] = computeFeatureForPixel(mHolePatchCoordinates[ithPx]);
	}
}
Vector3i DepthImageInpainting::findNearestPatch(const Vector3i& coord)
{

}


VectorXf DepthImageInpainting::computeFeatureForPatch(const Vector3i& coord)
{
	VectorXf ret = VectorXf::Zero(2 * mPatchWidth * mPatchWidth);
	return ret;

}
VectorXf DepthImageInpainting::computeFeatureForPixel(const Vector3i& coord)
{
	VectorXf ret = VectorXf::Zero(2);
	return ret;
}

void DepthImageInpainting::computeFilledPixelNormals()
{

}