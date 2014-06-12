#include "DepthInpainting.h"
#include "DepthCamera.h"
#include "MeshIO.h"

const int MASK_UNKNOWN = 1;



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

void DepthImageInpainting::visualizeInpaintedFeatures(int ithLayer, int ithIteration)
{
	MultilayerDepthImage inpaintedGx;
	MultilayerDepthImage inpaintedGy;
	inpaintedGx.Create(mHeight, mWidth);
	inpaintedGy.Create(mHeight, mWidth);

	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			if (mFeatureImage[i][j].size() <= ithLayer)
				continue;
			inpaintedGx[i][j].push_back(ExtendedDepthPixel(mFeatureImage[i][j][ithLayer][0]));
			inpaintedGy[i][j].push_back(ExtendedDepthPixel(mFeatureImage[i][j][ithLayer][1]));
		}
	}
	//int numHolePixels = static_cast<int>(mHolePixelCoordinates.size());
	//for (int i = 0; i < numHolePixels; ++i)
	//{
	//	Eigen::Vector3i coord = mHolePixelCoordinates[i];
	//	inpaintedGx[coord[0]][coord[1]][0] = mHolePatchFeatures[i][0];
	//	inpaintedGy[coord[0]][coord[1]][0] = mHolePatchFeatures[i][1];
	//}
	inpaintedGx.Process();
	inpaintedGy.Process();
	char filename[512];
	sprintf(filename, "results/inpaintedLayerGx%d_%d", ithIteration, ithLayer);
	float min = -10;
	float max = 10;
	inpaintedGx.SaveDepthOnionImage(filename, NULL, &min, &max);

	sprintf(filename, "results/inpaintedLayerGy%d_%d", ithIteration, ithLayer);
	inpaintedGy.SaveDepthOnionImage(filename, NULL, &min, &max);
}

void DepthImageInpainting::visualizeInpaintedMLDI(int ithIteration)
{
	mResultImage.Process();
	char filename[512];
	sprintf(filename, "results/inpaintedMLDI%d", ithIteration);
	mResultImage.SaveDepthOnionImage(filename);

	if (mVisualizationCam)
	{
		vector<Eigen::Vector3f> points;
		vector<Eigen::Vector3f> normals;
		vector<Eigen::Vector3i> colors;
		mVisualizationCam->SetData(mResultImage);
		mVisualizationCam->GetPointCloud(points, normals);
		sprintf(filename, "results/inpaintedMLDI%d.ply", ithIteration);
		SavePointCloud(filename, points, colors, normals);
	}
}


void DepthImageInpainting::Inpaint(int patchWidth)
{
	mPatchWidth = patchWidth;
	mResultImage = *mDepthImage;
	const int maxNumIterations = 10;

	//gatherHolesForLayer(1);
	computeFeatureImage();

	//int numHolePixels = static_cast<int>(mHolePixelCoordinates.size());
	//mHolePatchFeatures.resize(numHolePixels);
	//for (int i = 0; i < numHolePixels; ++i)
	//{
	//	Eigen::Vector3i coord = mHolePixelCoordinates[i];
	//	mHolePatchFeatures[i] = mFeatureImage[coord[0]][coord[1]][coord[2]];
	//}
	//reconstructHoleDepth(1);
	//visualizeInpaintedMLDI();
	//return;

	gatherFeatures();
	for (int ithLayer = 1; ithLayer < 2; ++ithLayer)
	{
		gatherHolesForLayer(ithLayer);

		for (int ithIteration = 0; ithIteration < maxNumIterations; ++ithIteration)
		{
			select();
			vote();
			visualizeInpaintedFeatures(ithLayer, ithIteration);
			reconstructHoleDepth(ithLayer);
			visualizeInpaintedMLDI(ithIteration);
			recomputeHoleFeatureImage(ithLayer);
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

int DepthImageInpainting::GetPixelFeatureDim() const
{
	return 2;
}
int DepthImageInpainting::GetPatchFeatureDim() const
{
	return mPatchWidth * mPatchWidth * GetPixelFeatureDim();
}

void DepthImageInpainting::gatherFeatures()
{
	LOG(INFO) << __FUNCTION__;

	// build vector<Eigen::VectorXf> mFeatures and vector<Vector3i> mFeatureCoordinates.
	
	for (int l = 0; l < 1; ++l)
	{
		for (int i = 0; i < mHeight; ++i)
		{
			for (int j = 0; j < mWidth; ++j)
			{
				Eigen::Vector3i coord(i, j, l);
				if (checkPatchValidity(coord, false))
				{
					Eigen::VectorXf patchFeature = computeFeatureForPatch(coord, false);
					mFeatures.push_back(patchFeature);
					mFeatureCoordinates.push_back(coord);
				}
			}
		}
	}

	mKDTree.setDim(GetPatchFeatureDim());
	int numPatches = static_cast<int>(mFeatures.size());
	for (int i = 0; i < numPatches; ++i)
	{
		mKDTree.add(mFeatures[i]);
	}
	mKDTree.initANN();
}
void DepthImageInpainting::gatherHolesForLayer(int layer)
{
	LOG(INFO) << __FUNCTION__;

	// build vector<Vector3i> mHolePixelCoordinates and MatrixXi mHolePixelIdx.
	CHECK(mMaskImage) << "No mask specified in DepthImageInpainting::gatherHolesForLayer();";
	mHolePixelCoordinates.clear();
	mHolePixelIdx.resize(mHeight);
	for (int i = 0; i < mHeight; ++i)
	{
		mHolePixelIdx[i].resize(mWidth, -1);
		for (int j = 0; j < mWidth; ++j)
		{
			if ((*mMaskImage)[i][j].size() <= layer)
				continue;
			if ((*mMaskImage)[i][j][layer] && i < 400)
			{
				mHolePixelIdx[i][j] = static_cast<int>(mHolePixelCoordinates.size());
				mHolePixelCoordinates.push_back(Eigen::Vector3i(i, j, layer));
			}
		}
	}
}

Eigen::SparseMatrix<double> DepthImageInpainting::constructPoissonLhs(int layer)
{
	LOG(INFO) << __FUNCTION__;

	int numHolePixels = static_cast<int>(mHolePixelCoordinates.size());
	vector<Eigen::Triplet<double> > triplet;
	Eigen::SparseMatrix<double> ret(numHolePixels, numHolePixels);
	for (int i = 0; i < numHolePixels; ++i)
	{
		Eigen::Vector3i coord = mHolePixelCoordinates[i];
		CHECK(layer == coord[2]) << "Layers do not agree in DepthImageInpainting::constructPoissonRhs().";
		int numNormalBoundary = 4;
		
		for (int neighborOffsetI = -1; neighborOffsetI <= 1; ++neighborOffsetI)
		{
			for (int neighborOffsetJ = -1; neighborOffsetJ <= 1; ++neighborOffsetJ)
			{
				int neighborI = coord[0] + neighborOffsetI;
				int neighborJ = coord[1] + neighborOffsetJ;
				if (abs(neighborOffsetI) == abs(neighborOffsetJ)) continue;
				if (neighborI < 0 || neighborI >= mHeight || neighborJ < 0 || neighborJ >= mWidth || (*mMaskImage)[neighborI][neighborJ].size() <= layer)
				{
					numNormalBoundary--;
					//neumann boundary condition
				}
				else if (mHolePixelIdx[neighborI][neighborJ] != - 1)//((*mMaskImage)[neighborI][neighborJ][layer] == MASK_UNKNOWN)
				{
					//normal situation
					int j = mHolePixelIdx[neighborI][neighborJ];
					CHECK(j >= 0) << "Hole pixel is not in mHolePixelIdx.";
					triplet.push_back(Eigen::Triplet<double>(i, j, -1));
				}
				else
				{
					//dirichlet boundary condition
					
				}

			}
		}
		if (numNormalBoundary <= 0)
		{
			LOG(WARNING) << "Degenerate linear system.";
			numNormalBoundary = 1;
		}
		
		triplet.push_back(Eigen::Triplet<double>(i, i, numNormalBoundary));
	}
	ret.setFromTriplets(triplet.begin(), triplet.end());
	return ret;
}
Eigen::VectorXd DepthImageInpainting::constructPoissonRhs(int layer)
{

	LOG(INFO) << __FUNCTION__;
	int numHolePixels = static_cast<int>(mHolePixelCoordinates.size());
	Eigen::VectorXd ret = Eigen::VectorXd::Zero(numHolePixels);
	for (int i = 0; i < numHolePixels; ++i)
	{
		Eigen::Vector3i coord = mHolePixelCoordinates[i];
		CHECK(layer == coord[2]) << "Layers do not agree in DepthImageInpainting::constructPoissonRhs().";
		for (int neighborOffsetI = -1; neighborOffsetI <= 1; ++neighborOffsetI)
		{
			for (int neighborOffsetJ = -1; neighborOffsetJ <= 1; ++neighborOffsetJ)
			{
				int neighborI = coord[0] + neighborOffsetI;
				int neighborJ = coord[1] + neighborOffsetJ;
				if (abs(neighborOffsetI) == abs(neighborOffsetJ)) continue;

				int gradientIIdx = coord[0] + (neighborOffsetI - 1) / 2;
				int gradientJIdx = coord[1] + (neighborOffsetJ - 1) / 2;

				if (neighborI <= 0 || neighborI >= mHeight || neighborJ <= 0 || neighborJ >= mWidth || (*mMaskImage)[neighborI][neighborJ].size() <= layer)
				{
					//neumann boundary condition
				}
				else if (mHolePixelIdx[neighborI][neighborJ] != -1)//((*mMaskImage)[neighborI][neighborJ][layer] == MASK_UNKNOWN)
				{
					//normal situation
					ret[i] += -(neighborOffsetI * mFeatureImage[gradientIIdx][gradientJIdx][layer][0] + neighborOffsetJ * mFeatureImage[gradientIIdx][gradientJIdx][layer][1]);
				}
				else
				{
					//dirichlet boundary condition

					ret[i] += (abs(neighborOffsetI) * mResultImage[neighborI][neighborJ][layer].d + abs(neighborOffsetJ) * mResultImage[neighborI][neighborJ][layer].d);
					ret[i] += -(neighborOffsetI * mFeatureImage[gradientIIdx][gradientJIdx][layer][0] + neighborOffsetJ * mFeatureImage[gradientIIdx][gradientJIdx][layer][1]);
				}

			}
		}

	}
	return ret;
}

void DepthImageInpainting::reconstructHoleDepth(int layer)
{
	LOG(INFO) << __FUNCTION__;

	// build vector<float> mHoleFilledDepth from vector<Eigen::VectorXf> mHolePatchFeatures.
	int numHolePixels = static_cast<int>(mHolePixelCoordinates.size());

	Eigen::SparseMatrix<double> Lhs = constructPoissonLhs(layer);
	Eigen::VectorXd rhs = constructPoissonRhs(layer);
	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double> > chol(Lhs);  // performs a Cholesky factorization of A
	Eigen::VectorXd x = chol.solve(rhs);
	Eigen::VectorXd recoveredRhs = Lhs * x;
	
	for (int i = 0; i < numHolePixels; ++i)
	{
		if (x[i] != x[i])
			printf("solver does not work.");
		if (recoveredRhs[i] != recoveredRhs[i])
			printf("solver does not work.");
		if (abs(recoveredRhs[i] - rhs[i]) > 1e-6)
			printf("solver does not work.");
	}
	mHoleFilledDepth.resize(numHolePixels);
	for (int i = 0; i < numHolePixels; ++i)
	{
		mHoleFilledDepth[i] = static_cast<float>(x[i]);
	}
	
	for (int ithHolePixel = 0; ithHolePixel < numHolePixels; ++ithHolePixel)
	{
		Eigen::Vector3i pxCoord = mHolePixelCoordinates[ithHolePixel];
		float originalDepth = mResultImage[pxCoord[0]][pxCoord[1]][pxCoord[2]].d;
		float currentDepth = mHoleFilledDepth[ithHolePixel];
		mResultImage[pxCoord[0]][pxCoord[1]][pxCoord[2]].d = mHoleFilledDepth[ithHolePixel];
	}
}

void DepthImageInpainting::select()
{
	LOG(INFO) << __FUNCTION__;

	// build vector<Vector3i> mHolePatchCoordinates.
	int numHolePixels = static_cast<int>(mHolePixelCoordinates.size());
	mHolePatchCoordinates.resize(numHolePixels);
	for (int ithPx = 0; ithPx < numHolePixels; ++ithPx)
	{
		mHolePatchCoordinates[ithPx] = findNearestPatch(mHolePixelCoordinates[ithPx]);
	}
}
void DepthImageInpainting::vote()
{
	LOG(INFO) << __FUNCTION__;

	// build vector<Eigen::VectorXf> mHolePatchFeatures.
	int numHolePixels = static_cast<int>(mHolePixelCoordinates.size());
	mHolePatchFeatures.resize(numHolePixels);
	for (int ithPx = 0; ithPx < numHolePixels; ++ithPx)
	{
		mHolePatchFeatures[ithPx] = mFeatureImage[mHolePatchCoordinates[ithPx][0]][mHolePatchCoordinates[ithPx][1]][mHolePatchCoordinates[ithPx][2]];
	}
	for (int ithPx = 0; ithPx < numHolePixels; ++ithPx)
	{
		Eigen::Vector3i coord = mHolePixelCoordinates[ithPx];
		mFeatureImage[coord[0]][coord[1]][coord[2]] = mHolePatchFeatures[ithPx];
	}
}
Eigen::Vector3i DepthImageInpainting::findNearestPatch(const Eigen::Vector3i& coord)
{
	const int kNN = 1;
	Eigen::VectorXf currentPatchFeatures = computeFeatureForPatch(coord, true);
	vector<int> idx = mKDTree.kSearch(currentPatchFeatures, kNN);
	Eigen::Vector3i nearestPatchCoord = mFeatureCoordinates[idx[0]];
	return nearestPatchCoord;
}

Eigen::VectorXf DepthImageInpainting::computeFeatureForPixel(const Eigen::Vector3i& coord, bool includeUnknownRegion)
{
	Eigen::VectorXf ret = Eigen::VectorXf::Zero(2);
	int layer = coord[2];
	int currentI = coord[0];
	int currentJ = coord[1];
	
	//int leftNeighborI = coord[0] - 1;
	//int leftNeighborJ = coord[1];
	int rightNeighborI = coord[0] + 1;
	int rightNeighborJ = coord[1];
	//int validCount = 0;
	float currentValue = mResultImage[currentI][currentJ][layer].d;
	//float leftValue = mResultImage[currentI][currentJ][layer].d;
	float rightValue = currentValue;
	
	//if (checkPixelValidity(Eigen::Vector3i(leftNeighborI, leftNeighborJ, layer), includeUnknownRegion))
	//{
	//	leftValue = mResultImage[leftNeighborI][leftNeighborJ][layer].d;
	//	validCount++;
	//}
	if (checkPixelValidity(Eigen::Vector3i(rightNeighborI, rightNeighborJ, layer), includeUnknownRegion))
	{
		rightValue = mResultImage[rightNeighborI][rightNeighborJ][layer].d;
		//validCount++;
	}
	ret[0] = rightValue - currentValue;
	//if (validCount)
	//{
	//	ret[0] = (rightValue - leftValue) / validCount;
	//}
	//else
	//{
	//	ret[0] = 0;
	//}

	//int upperNeighborI = coord[0];
	//int upperNeighborJ = coord[1] - 1;
	int lowerNeighborI = coord[0];
	int lowerNeighborJ = coord[1] + 1;
	//validCount = 0;
	//float upperValue = mResultImage[currentI][currentJ][layer].d;
	float lowerValue = currentValue;
	//if (checkPixelValidity(Eigen::Vector3i(upperNeighborI, upperNeighborJ, layer), includeUnknownRegion))
	//{
	//	upperValue = mResultImage[upperNeighborI][upperNeighborJ][layer].d;
	//	validCount++;
	//}
	if (checkPixelValidity(Eigen::Vector3i(lowerNeighborI, lowerNeighborJ, layer), includeUnknownRegion))
	{
		lowerValue = mResultImage[lowerNeighborI][lowerNeighborJ][layer].d;
		//validCount++;
	}
	ret[1] = lowerValue - currentValue;
	//if (validCount)
	//{
	//	ret[1] = (lowerValue - upperValue) / validCount;
	//}
	//else
	//{
	//	ret[1] = 0;
	//}
	return ret;
}
Eigen::VectorXf DepthImageInpainting::computeFeatureForPatch(const Eigen::Vector3i& coord, bool includeUnknownRegion)
{
	Eigen::VectorXf ret = Eigen::VectorXf::Zero(GetPatchFeatureDim());
	int count = 0;
	int pixelFeatureDim = GetPixelFeatureDim();
	int layer = coord[2];
	for (int i = -mPatchWidth / 2; i <= mPatchWidth / 2; ++i)
	{
		for (int j = -mPatchWidth / 2; j <= mPatchWidth / 2; ++j)
		{
			int neighborCoordI = coord[0] + i;
			int neighborCoordJ = coord[1] + j;
			if (!checkPixelValidity(Eigen::Vector3i(neighborCoordI, neighborCoordJ, layer), includeUnknownRegion))
			{
				for (int ithFeaturePerPixel = 0; ithFeaturePerPixel < pixelFeatureDim; ++ithFeaturePerPixel)
				{
					ret[count + ithFeaturePerPixel] = 0;
				}
			}
			else
			{
				for (int ithFeaturePerPixel = 0; ithFeaturePerPixel < pixelFeatureDim; ++ithFeaturePerPixel)
				{
					ret[count + ithFeaturePerPixel] = mFeatureImage[neighborCoordI][neighborCoordJ][layer][ithFeaturePerPixel];
				}

			}
			count += pixelFeatureDim;
		}
	}
	return ret;
}

void DepthImageInpainting::recomputeHoleFeatureImage(int layer)
{
	int numHolePixels = static_cast<int>(mHolePixelCoordinates.size());
	for (int i = 0; i < numHolePixels; ++i)
	{
		Eigen::Vector3i pxCoord = mHolePixelCoordinates[i];
		mFeatureImage[pxCoord[0]][pxCoord[1]][pxCoord[2]] = computeFeatureForPixel(pxCoord, true);
	}
}
void DepthImageInpainting::computeFeatureImage()
{
	MultilayerDepthImage featureGx;
	MultilayerDepthImage featureGy;
	featureGx.Create(mHeight, mWidth);
	featureGy.Create(mHeight, mWidth);
	mFeatureImage.resize(mHeight);
	for (int i = 0; i < mHeight; ++i)
	{
		mFeatureImage[i].resize(mWidth);
		for (int j = 0; j < mWidth; ++j)
		{
			int numLayers = static_cast<int>(mResultImage[i][j].size());
			mFeatureImage[i][j].resize(numLayers);
			for (int l = 0; l < numLayers; ++l)
			{
				mFeatureImage[i][j][l] = computeFeatureForPixel(Eigen::Vector3i(i, j, l), true);
				featureGx[i][j].push_back(ExtendedDepthPixel(mFeatureImage[i][j][l][0]));
				featureGy[i][j].push_back(ExtendedDepthPixel(mFeatureImage[i][j][l][1]));
			}
		}
	}
	featureGx.Process();
	featureGy.Process();
	float min = -10;
	float max = 10;
	featureGx.SaveDepthOnionImage("results/featureGx", NULL, &min, &max);
	featureGy.SaveDepthOnionImage("results/featureGy", NULL, &min, &max);
}

void DepthImageInpainting::computeFilledPixelNormals()
{

}

bool DepthImageInpainting::checkPixelValidity(const Eigen::Vector3i &coord, bool includeUnknownRegion)
{
	int coordI = coord[0];
	int coordJ = coord[1];
	int layer = coord[2];
	if (coordI < 0 || coordI >= mHeight || coordJ < 0 || coordJ >= mWidth)
		return false;
	if ((*mDepthImage)[coordI][coordJ].size() <= layer)
		return false;
	if (!includeUnknownRegion && (*mMaskImage)[coordI][coordJ][layer] == MASK_UNKNOWN) //(mHolePixelIdx[coordI][coordJ] != -1))//
		return false;
	return true;
}

bool DepthImageInpainting::checkPatchValidity(const Eigen::Vector3i & coord, bool includeUnknownRegion)
{
	int layer = coord[2];
	for (int i = -mPatchWidth / 2; i <= mPatchWidth / 2; ++i)
	{
		for (int j = -mPatchWidth / 2; j <= mPatchWidth / 2; ++j)
		{
			int neighborCoordI = coord[0] + i;
			int neighborCoordJ = coord[1] + j;
			if (!checkPixelValidity(Eigen::Vector3i(neighborCoordI, neighborCoordJ, layer), includeUnknownRegion))
				return false;
		}
	}
	return true;
}

const MultilayerDepthImage& DepthImageInpainting::GetResultImage() const
{
	return mResultImage;
}

void DepthImageInpainting::SetVisualizationCamera(DepthCamera* cam)
{
	mVisualizationCam = cam;
}