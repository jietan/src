#include "DepthInpainting.h"
#include "DepthCamera.h"
#include "MeshIO.h"
#include "utility/mathlib.h"
#include <queue>
using namespace std;

const int MASK_UNKNOWN = 1;
const int MASK_KNOWN = 0;
const int NEUMANN_HOLE = 0;
const int DIRICHELT_HOLE = 1;
const int NOT_HOLE = 2;

DepthImageInpainting::DepthImageInpainting()
{

}
DepthImageInpainting::~DepthImageInpainting()
{

}
void DepthImageInpainting::SetDepthImage(MultilayerDepthImage* depthImage)
{
	mDepthImage = depthImage;
	mLayers = depthImage->NumLayers();
}
void DepthImageInpainting::SetMaskImage(MultilayerMaskImage* maskImage)
{
	mMaskImage = maskImage;
}

void DepthImageInpainting::visualizeInpaintedFeatures(int ithPyramid, int ithLayer, int ithIteration)
{
	MultilayerDepthImage inpaintedGx;
	MultilayerDepthImage inpaintedGy;

	int height = mCurrentDepthImage.Height();
	int width = mCurrentDepthImage.Width();

	inpaintedGx.Create(height, width);
	inpaintedGy.Create(height, width);

	for (int i = 0; i < height; ++i)
	{
		for (int j = 0; j < width; ++j)
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
	sprintf(filename, "results/inpaintedGx-P%d-L%d-I%d", ithPyramid, ithLayer, ithIteration);
	float min = -100.f / powf(2.f, ithPyramid);
	float max = -min;
	inpaintedGx.SaveDepthOnionImage(filename, NULL, &min, &max);

	sprintf(filename, "results/inpaintedGy-P%d-L%d-I%d", ithPyramid, ithLayer, ithIteration);
	inpaintedGy.SaveDepthOnionImage(filename, NULL, &min, &max);
}

void DepthImageInpainting::visualizeInpaintedMLDI(int ithIteration)
{
	mCurrentDepthImage.Process();
	char filename[512];
	sprintf(filename, "results/inpaintedMLDI%d", ithIteration);
	mCurrentDepthImage.SaveDepthOnionImage(filename);

	if (mCamera)
	{
		vector<Eigen::Vector3f> points;
		vector<Eigen::Vector3f> normals;
		vector<Eigen::Vector3i> colors;
		mCamera->SetData(mCurrentDepthImage);
		mCamera->GetPointCloud(points, normals);
		sprintf(filename, "results/inpaintedMLDI%d.ply", ithIteration);
		SavePointCloud(filename, points, colors, normals);
	}
}


void DepthImageInpainting::Inpaint(int patchWidth)
{
	mPatchWidth = patchWidth;
	int maxNumIterations = 10;
	const int maxNumPyramids = 4;
	//for (int i = 0; i < mHeight; ++i)
	//{
	//	for (int j = 0; j < mWidth; ++j)
	//	{
	//		if (i > 280 || j > 300)
	//		{
	//			(*mDepthImage)[i][j].clear();
	//			(*mMaskImage)[i][j].clear();
	//		}
	//	}
	//}
	//(*mMaskImage)[267][337][1] = MASK_UNKNOWN;
	//(*mMaskImage)[281][327][1] = MASK_UNKNOWN;
	//gatherHolesForLayer(1);


	//int numHolePixels = static_cast<int>(mHolePixelCoordinates.size());
	//mHolePatchCoordinates.resize(numHolePixels);
	//mHolePatchFeatures.resize(numHolePixels);
	//for (int i = 0; i < numHolePixels; ++i)
	//{
	//	Eigen::Vector3i coord;
	//	coord[0] = static_cast<int>(RandDouble(160, 240));
	//	coord[1] = static_cast<int>(RandDouble(160, 360));
	//	coord[2] = 0;
	//	mHolePatchCoordinates[i] = coord;
	//	mHolePatchFeatures[i] = mFeatureImage[coord[0]][coord[1]][coord[2]];
	//}
	//for (int ithPx = 0; ithPx < numHolePixels; ++ithPx)
	//{
	//	Eigen::Vector3i coord = mHolePixelCoordinates[ithPx];
	//	mFeatureImage[coord[0]][coord[1]][coord[2]] = mHolePatchFeatures[ithPx];
	//}
	//visualizeInpaintedFeatures(1, 0);
	//reconstructHoleDepth(1);
	//recomputeHoleFeatureImage(1);
	//computeFilledPixelNormals(1);
	//visualizeInpaintedMLDI(0);
	//
	//return;
	generatePyramids(maxNumPyramids);

	for (int ithPyramid = 0; ithPyramid < maxNumPyramids; ++ithPyramid)
	{
		mCurrentDepthImage = mDepthPyramid[ithPyramid];
		mCurrentMaskImage = mMaskPyramid[ithPyramid];

		if (ithPyramid > 0)
			mFeatureImagePrevPyramid = mFeatureImage;
		computeFeatureImage(ithPyramid);
		gatherFeatures();
		for (int ithLayer = 1; ithLayer < 2; ++ithLayer)
		{
			gatherHolesForLayer(ithPyramid, ithLayer);
			//visualizeInpaintedFeatures(ithPyramid, ithLayer, -2);
			upsample(ithPyramid, ithLayer);
			//visualizeInpaintedFeatures(ithPyramid, ithLayer, -1);
			for (int ithIteration = 0; ithIteration < maxNumIterations; ++ithIteration)
			{
				LOG(INFO) << "Inpainting " << ithPyramid << "th pyramid, " << ithLayer << "th layer and " << ithIteration << "th iteration.";
 				select();
				vote();
				visualizeInpaintedFeatures(ithPyramid, ithLayer, ithIteration);
				//reconstructHoleDepth(ithLayer);
				//visualizeInpaintedMLDI(ithIteration);
				//recomputeHoleFeatureImage(ithLayer);
			}
			//computeFilledPixelNormals(ithLayer);
		}
		maxNumIterations -= 3;
	}
	saveInpaintFeatures();
	reconstructHoleDepth(1);
	computeFilledPixelNormals(1);
	visualizeInpaintedMLDI(0);
}

void DepthImageInpainting::saveInpaintFeatures()
{
	ofstream out("results/inpaintedFeatures.txt");
	int numHolePixels = static_cast<int>(mHolePixelCoordinates.size());
	out << numHolePixels << endl;
	for (int i = 0; i < numHolePixels; ++i)
	{
		Eigen::Vector3i coord = mHolePixelCoordinates[i];
		out << "(" << coord[0] << "," << coord[1] << "," << coord[2] << "): " << mFeatureImage[coord[0]][coord[1]][coord[2]][0] << "," << mFeatureImage[coord[0]][coord[1]][coord[2]][1] << endl;
	}
}
void DepthImageInpainting::upsample(int ithPyramid, int ithLayer)
{
	if (ithPyramid == 0) return;
	int numHolePixels = static_cast<int>(mHolePixelCoordinates.size());

	for (int ithPx = 0; ithPx < numHolePixels; ++ithPx)
	{
		Eigen::Vector3i coord = mHolePixelCoordinates[ithPx];
		if (mFeatureImagePrevPyramid[coord[0] / 2][coord[1] / 2].size() > coord[2])
			mFeatureImage[coord[0]][coord[1]][coord[2]] = mFeatureImagePrevPyramid[coord[0] / 2][coord[1] / 2][coord[2]] / 2;
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
	int height = mCurrentDepthImage.Height();
	int width = mCurrentDepthImage.Width();

	for (int l = 0; l < 1; ++l)
	{
		for (int i = 0; i < height; ++i)
		{
			for (int j = 0; j < width; ++j)
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
	mKDTree.clean();
	mKDTree.setDim(GetPatchFeatureDim());
	int numPatches = static_cast<int>(mFeatures.size());
	for (int i = 0; i < numPatches; ++i)
	{
		mKDTree.add(mFeatures[i]);
	}
	mKDTree.initANN();
}

void DepthImageInpainting::expand(int ithRow, int jthCol, int type, Eigen::MatrixXi& holeType)
{
	int height = mCurrentDepthImage.Height();
	int width = mCurrentDepthImage.Width();

	Eigen::MatrixXi inQ = Eigen::MatrixXi::Zero(height, width);
	queue<Eigen::Vector2i> toVisit;
	toVisit.push(Eigen::Vector2i(ithRow, jthCol));
	inQ(ithRow, jthCol) = 1;
	

	while (!toVisit.empty())
	{
		Eigen::Vector2i visiting = toVisit.front();
		//LOG(INFO) << visiting[0] << " " << visiting[1];

		toVisit.pop();
		holeType(visiting[0], visiting[1]) = DIRICHELT_HOLE;
		if (visiting[0] - 1 >= 0 && holeType(visiting[0] - 1, visiting[1]) == NEUMANN_HOLE && !inQ(visiting[0] - 1, visiting[1]))
		{
			inQ(visiting[0] - 1, visiting[1]) = 1;
			toVisit.push(Eigen::Vector2i(visiting[0] - 1, visiting[1]));
		}
		if (visiting[0] + 1 < height && holeType(visiting[0] + 1, visiting[1]) == NEUMANN_HOLE && !inQ(visiting[0] + 1, visiting[1]))
		{
			inQ(visiting[0] + 1, visiting[1]) = 1;
			toVisit.push(Eigen::Vector2i(visiting[0] + 1, visiting[1]));
		}
		if (visiting[1] - 1 >= 0 && holeType(visiting[0], visiting[1] - 1) == NEUMANN_HOLE && !inQ(visiting[0], visiting[1] - 1))
		{
			inQ(visiting[0], visiting[1] - 1) = 1;
			toVisit.push(Eigen::Vector2i(visiting[0], visiting[1] - 1));
		}
		if (visiting[1] + 1 < width && holeType(visiting[0], visiting[1] + 1) == NEUMANN_HOLE && !inQ(visiting[0], visiting[1] + 1))
		{
			inQ(visiting[0], visiting[1] + 1) = 1;
			toVisit.push(Eigen::Vector2i(visiting[0], visiting[1] + 1));
		}
	}
	
}
void DepthImageInpainting::gatherHolesForLayer(int ithPyramid, int layer)
{
	LOG(INFO) << __FUNCTION__;

	// build vector<Vector3i> mHolePixelCoordinates and MatrixXi mHolePixelIdx.
	CHECK(mMaskImage) << "No mask specified in DepthImageInpainting::gatherHolesForLayer();";
	int height = mCurrentDepthImage.Height();
	int width = mCurrentDepthImage.Width();
	int testNum = 0;
	Eigen::MatrixXi holeType = Eigen::MatrixXi::Constant(height, width, NOT_HOLE);
	for (int i = 0; i < height; ++i)
	{
		for (int j = 0; j < width; ++j)
		{
			if (mCurrentMaskImage[i][j].size() > layer && mCurrentMaskImage[i][j][layer] == MASK_UNKNOWN)
			{
				holeType(i, j) = NEUMANN_HOLE;
				testNum++;
			}
		}
	}
	for (int i = 0; i < height; ++i)
	{
		for (int j = 0; j < width; ++j)
		{
			if (mCurrentMaskImage[i][j].size() > layer && mCurrentMaskImage[i][j][layer] == MASK_UNKNOWN)
			{
				if (i > 0 && mCurrentMaskImage[i - 1][j].size() > layer && mCurrentMaskImage[i - 1][j][layer] == MASK_KNOWN
					|| j > 0 && mCurrentMaskImage[i][j - 1].size() > layer && mCurrentMaskImage[i][j - 1][layer] == MASK_KNOWN
					|| i < height - 1 && mCurrentMaskImage[i + 1][j].size() > layer && mCurrentMaskImage[i + 1][j][layer] == MASK_KNOWN
					|| j < width - 1 && mCurrentMaskImage[i][j + 1].size() > layer && mCurrentMaskImage[i][j + 1][layer] == MASK_KNOWN)
				{
					expand(i, j, DIRICHELT_HOLE, holeType);
				}
			}
		}
	}
	mHolePixelCoordinates.clear();
	mHolePixelIdx.clear();
	mHolePixelIdx.resize(height);
	for (int i = 0; i < height; ++i)
	{
		mHolePixelIdx[i].resize(width, -1);
		for (int j = 0; j < width; ++j)
		{
			if (holeType(i, j) == DIRICHELT_HOLE)
			{
				mHolePixelIdx[i][j] = static_cast<int>(mHolePixelCoordinates.size());
				mHolePixelCoordinates.push_back(Eigen::Vector3i(i, j, layer));
			}
		}
	}

	//mHolePixelCoordinates.clear();
	//mHolePixelIdx.resize(mHeight);
	//for (int i = 0; i < mHeight; ++i)
	//{
	//	mHolePixelIdx[i].resize(mWidth, -1);
	//	for (int j = 0; j < mWidth; ++j)
	//	{
	//		if ((*mMaskImage)[i][j].size() <= layer)
	//			continue;
	//		if ((*mMaskImage)[i][j][layer] && i < 400)
	//		{
	//			mHolePixelIdx[i][j] = static_cast<int>(mHolePixelCoordinates.size());
	//			mHolePixelCoordinates.push_back(Eigen::Vector3i(i, j, layer));
	//		}
	//	}
	//}
}

Eigen::SparseMatrix<double> DepthImageInpainting::constructPoissonLhs(int layer)
{
	LOG(INFO) << __FUNCTION__;

	int numHolePixels = static_cast<int>(mHolePixelCoordinates.size());
	vector<Eigen::Triplet<double> > triplet;
	Eigen::SparseMatrix<double> ret(numHolePixels, numHolePixels);
	int height = mCurrentDepthImage.Height();
	int width = mCurrentDepthImage.Width();
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
				if (neighborI < 0 || neighborI >= height || neighborJ < 0 || neighborJ >= width || mCurrentMaskImage[neighborI][neighborJ].size() <= layer)
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
					//LOG(WARNING) << i << ":" << j << ":" << -1;
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
		//LOG(WARNING) << i << ":" << i << ":" << numNormalBoundary;
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
	int height = mCurrentDepthImage.Height();
	int width = mCurrentDepthImage.Width();
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

				if (neighborI <= 0 || neighborI >= height || neighborJ <= 0 || neighborJ >= width || mCurrentMaskImage[neighborI][neighborJ].size() <= layer)
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

					ret[i] += (abs(neighborOffsetI) * mCurrentDepthImage[neighborI][neighborJ][layer].d + abs(neighborOffsetJ) * mCurrentDepthImage[neighborI][neighborJ][layer].d);
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
		float originalDepth = mCurrentDepthImage[pxCoord[0]][pxCoord[1]][pxCoord[2]].d;
		float currentDepth = mHoleFilledDepth[ithHolePixel];
		mCurrentDepthImage[pxCoord[0]][pxCoord[1]][pxCoord[2]].d = mHoleFilledDepth[ithHolePixel];
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

Eigen::VectorXf DepthImageInpainting::computeFeatureForPixel(const Eigen::Vector3i& coord, bool correctDistortion, bool includeUnknownRegion)
{
	Eigen::VectorXf ret = Eigen::VectorXf::Zero(2);


	int layer = coord[2];
	int currentI = coord[0];
	int currentJ = coord[1];
	
	int lowerNeighborI = coord[0] + 1;
	int lowerNeighborJ = coord[1];

	float currentValue = mCurrentDepthImage[currentI][currentJ][layer].d;

	float lowerValue = currentValue;
	
	if (checkPixelValidity(Eigen::Vector3i(lowerNeighborI, lowerNeighborJ, layer), includeUnknownRegion))
	{
		lowerValue = mCurrentDepthImage[lowerNeighborI][lowerNeighborJ][layer].d;
	}
	ret[0] = lowerValue - currentValue; //Gy

	int rightNeighborI = coord[0];
	int rightNeighborJ = coord[1] + 1;
	float rightValue = currentValue;
	if (checkPixelValidity(Eigen::Vector3i(rightNeighborI, rightNeighborJ, layer), includeUnknownRegion))
	{
		rightValue = mCurrentDepthImage[rightNeighborI][rightNeighborJ][layer].d;
	}
	ret[1] = rightValue - currentValue; //Gx

	//if (correctDistortion)
	//{
	//	CHECK(mCamera) << "Camera information is needed for perspective distortion correction.";
	//	float fx = mCamera->GetFocalLength();
	//	float fy = mCamera->GetFocalLength();
	//	float cx = (mWidth - 1) / 2.f;
	//	float cy = (mHeight - 1) / 2.f;

	//	ret[0] *= fy;
	//	ret[0] /= abs((coord[0] + 1 - cy) * lowerValue - (coord[0] - cy) * currentValue);

	//	ret[1] *= fx;
	//	ret[1] /= abs((coord[1] + 1 - cx) * rightValue - (coord[1] - cx) * currentValue);
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
		mFeatureImage[pxCoord[0]][pxCoord[1]][pxCoord[2]] = computeFeatureForPixel(pxCoord, false, true);
	}
}
void DepthImageInpainting::computeFeatureImage(int ithPyramid)
{
	MultilayerDepthImage featureGx;
	MultilayerDepthImage featureGy;
	int width = mCurrentDepthImage.Width();
	int height = mCurrentDepthImage.Height();
	featureGx.Create(height, width);
	featureGy.Create(height, width);
	mFeatureImage.Create(height, width);

	for (int i = 0; i < height; ++i)
	{
		for (int j = 0; j < width; ++j)
		{
			int numLayers = static_cast<int>(mCurrentDepthImage[i][j].size());
			mFeatureImage[i][j].resize(numLayers);
			for (int l = 0; l < numLayers; ++l)
			{
				mFeatureImage[i][j][l] = computeFeatureForPixel(Eigen::Vector3i(i, j, l), false, true);
				featureGx[i][j].push_back(ExtendedDepthPixel(mFeatureImage[i][j][l][0]));
				featureGy[i][j].push_back(ExtendedDepthPixel(mFeatureImage[i][j][l][1]));
			}
		}
	}
	featureGx.Process();
	featureGy.Process();
	float min = -100.f / powf(2.f, ithPyramid);
	float max = -min;
	featureGx.SaveDepthOnionImage("results/featureGx", NULL, &min, &max);
	featureGy.SaveDepthOnionImage("results/featureGy", NULL, &min, &max);
}

void DepthImageInpainting::computeFilledPixelNormals(int ithLayer)
{
	int height = mCurrentDepthImage.Height();
	int width = mCurrentDepthImage.Width();

	for (int v = 0; v < height; ++v)
	{
		for (int u = 0; u < width; ++u)
		{
			if (mCurrentMaskImage[v][u].size() <= ithLayer || mCurrentMaskImage[v][u][ithLayer] == MASK_KNOWN) continue;
			Eigen::Vector3f tangentialAxis1(0, 1, mFeatureImage[v][u][ithLayer][0]);
			Eigen::Vector3f tangentialAxis2(1, 0, mFeatureImage[v][u][ithLayer][1]);
			Eigen::Vector3f normal = tangentialAxis1.cross(tangentialAxis2);
			normal = -normal.normalized();
			CHECK(mCamera) << "Camera is not set in DepthImageInpainting::computeFilledPixelNormals().";

			Eigen::Vector3f gn = mCamera->GetCameraPose().block(0, 0, 3, 3) * normal;
			mCurrentDepthImage[v][u][ithLayer].n = gn;
		}
	}

}

bool DepthImageInpainting::checkPixelValidity(const Eigen::Vector3i &coord, bool includeUnknownRegion)
{
	int coordI = coord[0];
	int coordJ = coord[1];
	int layer = coord[2];
	int height = mCurrentDepthImage.Height();
	int width = mCurrentDepthImage.Width();
	if (coordI < 0 || coordI >= height || coordJ < 0 || coordJ >= width)
		return false;
	if (mCurrentDepthImage[coordI][coordJ].size() <= layer)
		return false;
	if (!includeUnknownRegion && mCurrentMaskImage[coordI][coordJ][layer] == MASK_UNKNOWN) //(mHolePixelIdx[coordI][coordJ] != -1))//
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
	return mCurrentDepthImage;
}

void DepthImageInpainting::SetCamera(DepthCamera* cam)
{
	mCamera = cam;
}

void DepthImageInpainting::generatePyramids(int numPyramids)
{
	mDepthPyramid.clear();
	mDepthPyramid.resize(numPyramids);
	mMaskPyramid.clear();
	mMaskPyramid.resize(numPyramids);
	mDepthPyramid[numPyramids - 1] = *mDepthImage;
	mMaskPyramid[numPyramids - 1] = *mMaskImage;
	for (int i = numPyramids - 2; i >= 0; --i)
	{
		mDepthPyramid[i] = mDepthPyramid[i + 1];
		mDepthPyramid[i].DownSample();
		mMaskPyramid[i] = mMaskPyramid[i + 1];
		mMaskPyramid[i].DownSample();
	}
}