#include "DepthInpainting.h"
#include "DepthCamera.h"
#include "MeshIO.h"
#include "utility/mathlib.h"
#include <queue>
using namespace std;


const int NEUMANN_HOLE = 0; // a hole with all neumann boundaries
const int DIRICHELT_HOLE = 1; // a hole with at least dirichelt boundary
const int NOT_HOLE = 2;

const int NEUMANN_BOUNDARY = 0;
const int DIRICHELT_BOUNDARY = 1;
const int NOT_BOUNDARY = 2;

DepthImageInpainting::DepthImageInpainting() : mMaxNumPyramids(0)
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
		mCamera->GetPointCloud(points, normals, PORTION_ALL);
		sprintf(filename, "results/inpaintedMLDI%d.ply", ithIteration);
		SavePointCloud(filename, points, colors, normals);
	}
}

void DepthImageInpainting::dumpInpaintedPoints()
{
	int numHolePixels = static_cast<int>(mHolePixelCoordinates.size());

	vector<Eigen::Vector3f> points;
	vector<Eigen::Vector3f> normals;
	vector<Eigen::Vector3i> colors;

	points.resize(numHolePixels);
	normals.resize(numHolePixels);

	for (int i = 0; i < numHolePixels; ++i)
	{
		Eigen::Vector3i coord = mHolePixelCoordinates[i];
		float depth = mCurrentDepthImage[coord[0]][coord[1]][coord[2]].d;
		Eigen::Vector3f pt = mCamera->GetPoint(coord[0], coord[1], depth);
		points[i] = pt;
		normals[i] = mCurrentDepthImage[coord[0]][coord[1]][coord[2]].n;
	}
	SavePointCloud("results/inpaintedPoints.ply", points, colors, normals);
}

void DepthImageInpainting::deformMesh()
{
	int numVertices = static_cast<int>(mMeshVertices.size());
	int ithRow, jthCol;
	for (int i = 0; i < numVertices; ++i)
	{
		float depth = mCamera->GetDepth(mMeshVertices[i], &ithRow, &jthCol);
		if (ithRow < 0 || ithRow >= mDepthImage->Height() || jthCol < 0 || jthCol >= mDepthImage->Width())
			continue;
		int numLayers = static_cast<int>((*mDepthImage)[ithRow][jthCol].size());
		for (int kthLayer = 0; kthLayer < numLayers; ++kthLayer)
		{
			if (abs((*mDepthImage)[ithRow][jthCol][kthLayer].d - depth) < 1)
			{
				float newDepth = mCurrentDepthImage[ithRow][jthCol][kthLayer].d;
				Eigen::Vector3f newVertices = mCamera->GetPoint(ithRow, jthCol, newDepth);
				mMeshVertices[i] = newVertices;
			}
		}
	}
	SaveMesh("results/deformedMesh.ply", mMeshVertices, mMeshIndices);
}

void DepthImageInpainting::Inpaint(int patchWidth)
{
	mPatchWidth = patchWidth;
	int maxNumIterations = 10;
	mMaxNumPyramids = 1;
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
	int layerToInpaint = 0;
	mCurrentDepthImage = *mDepthImage;
	mCurrentMaskImage = *mMaskImage;
	computeFeatureImage(0);
	gatherHolesForLayer(0, layerToInpaint);

	int numHolePixels = static_cast<int>(mHolePixelCoordinates.size());
	mHolePatchCoordinates.resize(numHolePixels);
	mHolePatchFeatures.resize(numHolePixels);
	for (int i = 0; i < numHolePixels; ++i)
	{
		Eigen::Vector3i coord;
		coord[0] = 400;// static_cast<int>(RandDouble(320, 400));
		coord[1] = mHolePixelCoordinates[i][1];  //static_cast<int>(RandDouble(200, 450));
		coord[2] = 0;
		mHolePatchCoordinates[i] = coord;
		mHolePatchFeatures[i] = mFeatureImage[coord[0]][coord[1]][coord[2]];
		mHolePatchFeatures[i][0] = 0;
	}
	for (int ithPx = 0; ithPx < numHolePixels; ++ithPx)
	{
		Eigen::Vector3i coord = mHolePixelCoordinates[ithPx];
		mFeatureImage[coord[0]][coord[1]][coord[2]] = mHolePatchFeatures[ithPx];
		if (coord[0] - 1 >= 0 && coord[2] < mFeatureImage[coord[0] - 1][coord[1]].size())
			mFeatureImage[coord[0] - 1][coord[1]][coord[2]] = mHolePatchFeatures[ithPx];
		if (coord[1] - 1 >= 0 && coord[2] < mFeatureImage[coord[0]][coord[1] - 1].size())
			mFeatureImage[coord[0]][coord[1] - 1][coord[2]] = mHolePatchFeatures[ithPx];
	}
	visualizeInpaintedFeatures(0, layerToInpaint, 0);
	reconstructHoleDepth(layerToInpaint);
	recomputeHoleFeatureImage(layerToInpaint);
	computeFilledPixelNormals(layerToInpaint);
	visualizeInpaintedMLDI(0);
	//deformMesh();
	dumpInpaintedPoints();
	return;


	generatePyramids(mMaxNumPyramids);

	for (int ithPyramid = 0; ithPyramid < mMaxNumPyramids; ++ithPyramid)
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

			upsample(ithPyramid, ithLayer);
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
	//readInpaintedFeatures();
	reconstructHoleDepth(1);
	recomputeHoleFeatureImage(1);
	computeFilledPixelNormals(1);
	visualizeInpaintedMLDI(0);
}

void DepthImageInpainting::readInpaintedFeatures()
{
	FILE* fp = fopen("results/inpaintedFeatures.txt", "r");
	int numHolePixels;
	fscanf(fp, "%d\n", &numHolePixels);
	for (int i = 0; i < numHolePixels; ++i)
	{
		Eigen::Vector3i coord;
		float gx, gy;
		fscanf(fp, "(%d,%d,%d): %f,%f\n", &coord[0], &coord[1], &coord[2], &gx, &gy);
		mFeatureImage[coord[0]][coord[1]][coord[2]][0] = gx;
		mFeatureImage[coord[0]][coord[1]][coord[2]][1] = gy;
	}
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

void DepthImageInpainting::expand(int ithRow, int jthCol, int kthLayer, int type, int ithHole, Eigen::MatrixXi& holeType)
{
	if (holeType(ithRow, jthCol) == type) return;
	int height = mCurrentDepthImage.Height();
	int width = mCurrentDepthImage.Width();

	Eigen::MatrixXi inQ = Eigen::MatrixXi::Zero(height, width);
	queue<Eigen::Vector2i> toVisit;
	toVisit.push(Eigen::Vector2i(ithRow, jthCol));
	inQ(ithRow, jthCol) = 1;
	
	mConnectedHolePixelCoordinates.resize(mConnectedHolePixelCoordinates.size() + 1);
	while (!toVisit.empty())
	{
		Eigen::Vector2i visiting = toVisit.front();
		//LOG(INFO) << visiting[0] << " " << visiting[1];

		toVisit.pop();
		holeType(visiting[0], visiting[1]) = DIRICHELT_HOLE;
		mConnectedHolePixelCoordinates[ithHole].push_back(Eigen::Vector3i(visiting[0], visiting[1], kthLayer));
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

	const float diricheletBoundaryDepthDifferenceThreshold = 10;
	// build vector<Vector3i> mHolePixelCoordinates and MatrixXi mHolePixelIdx.
	CHECK(mMaskImage) << "No mask specified in DepthImageInpainting::gatherHolesForLayer();";
	int height = mCurrentDepthImage.Height();
	int width = mCurrentDepthImage.Width();
	int testNum = 0;
	Eigen::MatrixXi holeType = Eigen::MatrixXi::Constant(height, width, NOT_HOLE);
	mBoundaryType.clear();
	mConnectedHolePixelCoordinates.clear();
	int ithHole = 0;
	for (int i = 0; i < height; ++i)
	{
		mBoundaryType.resize(height);
		for (int j = 0; j < width; ++j)
		{
			mBoundaryType[i].resize(width, NOT_BOUNDARY);

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
				for (int offsetI = -1; offsetI <= 1; ++offsetI)
				{
					for (int offsetJ = -1; offsetJ <= 1; ++offsetJ)
					{
						if (abs(offsetI) == abs(offsetJ))
							continue;
						if (i + offsetI < 0 || i + offsetI >= height || j + offsetJ < 0 || j + offsetJ >= width)
							continue;
						if (mCurrentMaskImage[i + offsetI][j + offsetJ].size() <= layer)
						{
							mBoundaryType[i + offsetI][j + offsetJ] = NEUMANN_BOUNDARY;
						}
						else if (mCurrentMaskImage[i + offsetI][j + offsetJ][layer] == MASK_KNOWN)
						{
							mBoundaryType[i + offsetI][j + offsetJ] = DIRICHELT_BOUNDARY;

							//if (i + offsetI > 190 && i + offsetI < 225 && j + offsetJ > 300 && j + offsetJ < 330 || j + offsetJ > 480)
							//	mBoundaryType[i + offsetI][j + offsetJ] = NEUMANN_BOUNDARY;
							
							//if (ithPyramid == mMaxNumPyramids - 1 && abs(mCurrentDepthImage[i][j][layer].d - mCurrentDepthImage[i + offsetI][j + offsetJ][layer].d) >= diricheletBoundaryDepthDifferenceThreshold)
							//{
							//	mBoundaryType[i + offsetI][j + offsetJ] = NEUMANN_BOUNDARY;
							//} 

						}
						else
							mBoundaryType[i + offsetI][j + offsetJ] = NOT_BOUNDARY;
					}
				}

				//if (i > 0 && mCurrentMaskImage[i - 1][j].size() > layer && mCurrentMaskImage[i - 1][j][layer] == MASK_KNOWN
				//	|| j > 0 && mCurrentMaskImage[i][j - 1].size() > layer && mCurrentMaskImage[i][j - 1][layer] == MASK_KNOWN
				//	|| i < height - 1 && mCurrentMaskImage[i + 1][j].size() > layer && mCurrentMaskImage[i + 1][j][layer] == MASK_KNOWN
				//	|| j < width - 1 && mCurrentMaskImage[i][j + 1].size() > layer && mCurrentMaskImage[i][j + 1][layer] == MASK_KNOWN)
				if (i > 0 && mBoundaryType[i - 1][j] == DIRICHELT_BOUNDARY
					|| j > 0 && mBoundaryType[i][j - 1] == DIRICHELT_BOUNDARY
					|| i < height - 1 && mBoundaryType[i - 1][j] == DIRICHELT_BOUNDARY
					|| j < width - 1 && mBoundaryType[i][j - 1] == DIRICHELT_BOUNDARY)
				{
					expand(i, j, layer, DIRICHELT_HOLE, static_cast<int>(mConnectedHolePixelCoordinates.size()), holeType);
				}

			}
		}
	}
	vector<vector<Eigen::Vector3i> > tmpContainer;
	for (vector<vector<Eigen::Vector3i> >::iterator it = mConnectedHolePixelCoordinates.begin(); it != mConnectedHolePixelCoordinates.end(); ++it)
	{
		if (it->size() > 200)
		{
			tmpContainer.push_back(*it);
		}
	}
	mConnectedHolePixelCoordinates = tmpContainer;
	mHolePixelCoordinates.clear();
	mHolePixelIdx.clear();
	mConnectedHolePixelStartId.clear();
	mHolePixelIdx.resize(height);
	for (int i = 0; i < height; ++i)
	{
		mHolePixelIdx[i].resize(width, -1);
	}
	int numHoles = static_cast<int>(mConnectedHolePixelCoordinates.size());
	for (int ithHole = 0; ithHole < numHoles; ++ithHole)
	{
		mConnectedHolePixelStartId.push_back(mHolePixelCoordinates.size());
		int numPxPerHole = static_cast<int>(mConnectedHolePixelCoordinates[ithHole].size());
		for (int ithPxPerHole = 0; ithPxPerHole < numPxPerHole; ++ithPxPerHole)
		{
			int ithRow = mConnectedHolePixelCoordinates[ithHole][ithPxPerHole][0];
			int jthCol = mConnectedHolePixelCoordinates[ithHole][ithPxPerHole][1];
			mHolePixelIdx[ithRow][jthCol] = static_cast<int>(mHolePixelCoordinates.size());
			mHolePixelCoordinates.push_back(Eigen::Vector3i(ithRow, jthCol, layer));
		}
	}
	//for (int i = 0; i < height; ++i)
	//{
	//	mHolePixelIdx[i].resize(width, -1);
	//	}
	//	for (int j = 0; j < width; ++j)
	//	{
	//		if (holeType(i, j) == DIRICHELT_HOLE)
	//		{
	//			mHolePixelIdx[i][j] = static_cast<int>(mHolePixelCoordinates.size());
	//			mHolePixelCoordinates.push_back(Eigen::Vector3i(i, j, layer));
	//		}
	//	}
	//}

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

Eigen::SparseMatrix<double> DepthImageInpainting::constructPoissonLhs(int layer, int ithHole, bool bAllNeumann)
{
	LOG(INFO) << __FUNCTION__;

	int numHolePixels = static_cast<int>(mConnectedHolePixelCoordinates[ithHole].size());
	vector<Eigen::Triplet<double> > triplet;
	Eigen::SparseMatrix<double> ret(numHolePixels, numHolePixels);
	int height = mCurrentDepthImage.Height();
	int width = mCurrentDepthImage.Width();
	bool bFirst = true;
	for (int i = 0; i < numHolePixels; ++i)
	{
		Eigen::Vector3i coord = mConnectedHolePixelCoordinates[ithHole][i];
		CHECK(layer == coord[2]) << "Layers do not agree in DepthImageInpainting::constructPoissonRhs().";
		int numNormalBoundary = 4;
		
		for (int neighborOffsetI = -1; neighborOffsetI <= 1; ++neighborOffsetI)
		{
			for (int neighborOffsetJ = -1; neighborOffsetJ <= 1; ++neighborOffsetJ)
			{
				int neighborI = coord[0] + neighborOffsetI;
				int neighborJ = coord[1] + neighborOffsetJ;
				if (abs(neighborOffsetI) == abs(neighborOffsetJ)) continue;
				//if (neighborI == 35 && neighborJ == 322)
				//	__debugbreak();
				//if (neighborI < 0 || neighborI >= height || neighborJ < 0 || neighborJ >= width || mCurrentMaskImage[neighborI][neighborJ].size() <= layer)
				if (neighborI < 0 || neighborI >= height || neighborJ < 0 || neighborJ >= width || mBoundaryType[neighborI][neighborJ] == NEUMANN_BOUNDARY)
				{
					numNormalBoundary--;
					//neumann boundary condition
				}
				else if (mHolePixelIdx[neighborI][neighborJ] != -1)//((*mMaskImage)[neighborI][neighborJ][layer] == MASK_UNKNOWN)
				{
					//normal situation
					CHECK(mBoundaryType[neighborI][neighborJ] == NOT_BOUNDARY) << "Boundaries do not agree.";
					int j = mHolePixelIdx[neighborI][neighborJ];
					j -= mConnectedHolePixelStartId[ithHole];
					CHECK(j >= 0) << "Hole pixel is not in mHolePixelIdx.";
					triplet.push_back(Eigen::Triplet<double>(i, j, -1));
					//LOG(WARNING) << i << ":" << j << ":" << -1;
				}
				else
				{
					if (!bAllNeumann)
					{
						//dirichlet boundary condition
						if (neighborJ < 480)
							LOG(INFO) << "Dirichlet boundary conditions: " << neighborI << ", " << neighborJ << ":" << mCurrentDepthImage[neighborI][neighborJ][layer].d;
						CHECK(mBoundaryType[neighborI][neighborJ] == DIRICHELT_BOUNDARY) << "Boundaries do not agree.";
					}
					else if (bFirst)
					{
						bFirst = false;
					}
					else
					{
						numNormalBoundary--;
					}
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
Eigen::VectorXd DepthImageInpainting::constructPoissonRhs(int layer, int ithHole, bool bAllNeumann)
{

	LOG(INFO) << __FUNCTION__;
	int numHolePixels = static_cast<int>(mConnectedHolePixelCoordinates[ithHole].size());
	Eigen::VectorXd ret = Eigen::VectorXd::Zero(numHolePixels);
	int height = mCurrentDepthImage.Height();
	int width = mCurrentDepthImage.Width();
	for (int i = 0; i < numHolePixels; ++i)
	{
		Eigen::Vector3i coord = mConnectedHolePixelCoordinates[ithHole][i];
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

				//if (neighborI <= 0 || neighborI >= height || neighborJ <= 0 || neighborJ >= width || mCurrentMaskImage[neighborI][neighborJ].size() <= layer)
				if (neighborI < 0 || neighborI >= height || neighborJ < 0 || neighborJ >= width || mBoundaryType[neighborI][neighborJ] == NEUMANN_BOUNDARY)
				{
					//neumann boundary condition
				}
				else if (mHolePixelIdx[neighborI][neighborJ] != -1)//((*mMaskImage)[neighborI][neighborJ][layer] == MASK_UNKNOWN)
				{
					//normal situation
					CHECK(mBoundaryType[neighborI][neighborJ] == NOT_BOUNDARY) << "Boundaries do not agree.";

					ret[i] += -(neighborOffsetI * mFeatureImage[gradientIIdx][gradientJIdx][layer][0] + neighborOffsetJ * mFeatureImage[gradientIIdx][gradientJIdx][layer][1]);
				}
				else if (!bAllNeumann)
				{
					//dirichlet boundary condition
					CHECK(mBoundaryType[neighborI][neighborJ] == DIRICHELT_BOUNDARY) << "Boundaries do not agree.";

					ret[i] += (abs(neighborOffsetI) * mCurrentDepthImage[neighborI][neighborJ][layer].d + abs(neighborOffsetJ) * mCurrentDepthImage[neighborI][neighborJ][layer].d);
					//ret[i] += (abs(neighborOffsetI) * 2040 + abs(neighborOffsetJ) * 2040);
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
	mHoleFilledDepth.resize(mHolePixelCoordinates.size());

	int numHoles = static_cast<int>(mConnectedHolePixelCoordinates.size());
	for (int ithHole = 0; ithHole < numHoles; ++ithHole)
	{
		int numHolePixels = static_cast<int>(mConnectedHolePixelCoordinates[ithHole].size());

		if (numHolePixels > 100) // find the right boundary conditions
		{
			Eigen::SparseMatrix<double> Lhs = constructPoissonLhs(layer, ithHole, true);
			Eigen::VectorXd rhs = constructPoissonRhs(layer, ithHole, true);
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

			searchOptimalBoundaryCondition(ithHole, x);
		}

		Eigen::SparseMatrix<double> Lhs = constructPoissonLhs(layer, ithHole, false);
		Eigen::VectorXd rhs = constructPoissonRhs(layer, ithHole, false);
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
		for (int i = 0; i < numHolePixels; ++i)
		{
			const Eigen::Vector3i& holePxCoord = mConnectedHolePixelCoordinates[ithHole][i];
			int index1D = mHolePixelIdx[holePxCoord[0]][holePxCoord[1]];
			mHoleFilledDepth[index1D] = static_cast<float>(x[i]);
		}
	}


	int numHolePixels = static_cast<int>(mHolePixelCoordinates.size());
	for (int ithHolePixel = 0; ithHolePixel < numHolePixels; ++ithHolePixel)
	{
		Eigen::Vector3i pxCoord = mHolePixelCoordinates[ithHolePixel];
		float originalDepth = mCurrentDepthImage[pxCoord[0]][pxCoord[1]][pxCoord[2]].d;
		float currentDepth = mHoleFilledDepth[ithHolePixel];
		mCurrentDepthImage[pxCoord[0]][pxCoord[1]][pxCoord[2]].d = mHoleFilledDepth[ithHolePixel];
	}
}

void DepthImageInpainting::identifyBoundaryPixels(const vector<Eigen::Vector3i>& holePixels, vector<int>* boundaryPixels) const
{
	int numHolePixels = static_cast<int>(holePixels.size());
	boundaryPixels->clear();
	int height = mCurrentDepthImage.Height();
	int width = mCurrentDepthImage.Width();

	for (int i = 0; i < numHolePixels; ++i)
	{
		const Eigen::Vector3i& coord = holePixels[i];
		for (int neighborOffsetI = -1; neighborOffsetI <= 1; ++neighborOffsetI)
		{
			for (int neighborOffsetJ = -1; neighborOffsetJ <= 1; ++neighborOffsetJ)
			{
				int neighborI = coord[0] + neighborOffsetI;
				int neighborJ = coord[1] + neighborOffsetJ;
				if (abs(neighborOffsetI) == abs(neighborOffsetJ)) continue;

				if (neighborI < 0 || neighborI >= height || neighborJ < 0 || neighborJ >= width || mBoundaryType[neighborI][neighborJ] == NEUMANN_BOUNDARY)
				{
					continue;
				}
				else if (mBoundaryType[neighborI][neighborJ] == DIRICHELT_BOUNDARY)
				{
					boundaryPixels->push_back(i);
				}
			}
		}
	}

}

void DepthImageInpainting::computeBaseDiscrepancy(const vector<Eigen::Vector3i>& holePixels, const vector<int>& boundaryPixels, const Eigen::VectorXd& baseDepth, vector<float>* boundaryDiscrepancy) const
{
	boundaryDiscrepancy->clear();
	int height = mCurrentDepthImage.Height();
	int width = mCurrentDepthImage.Width();
	int numBoundaryPixels = static_cast<int>(boundaryPixels.size());
	for (int i = 0; i < numBoundaryPixels; ++i)
	{
		int currentPxIdx = boundaryPixels[i];
		const Eigen::Vector3i& coord = holePixels[currentPxIdx];
		for (int neighborOffsetI = -1; neighborOffsetI <= 1; ++neighborOffsetI)
		{
			for (int neighborOffsetJ = -1; neighborOffsetJ <= 1; ++neighborOffsetJ)
			{
				int neighborI = coord[0] + neighborOffsetI;
				int neighborJ = coord[1] + neighborOffsetJ;
				if (abs(neighborOffsetI) == abs(neighborOffsetJ)) continue;

				if (neighborI < 0 || neighborI >= height || neighborJ < 0 || neighborJ >= width || mBoundaryType[neighborI][neighborJ] == NEUMANN_BOUNDARY)
				{
					continue;
				}
				else if (mBoundaryType[neighborI][neighborJ] == DIRICHELT_BOUNDARY)
				{
					int gradientIIdx = coord[0] + (neighborOffsetI - 1) / 2;
					int gradientJIdx = coord[1] + (neighborOffsetJ - 1) / 2;

					float currentGradient = baseDepth[currentPxIdx] - mCurrentDepthImage[neighborI][neighborJ][coord[2]].d;
					float desiredGradient = mFeatureImage[gradientIIdx][gradientJIdx][coord[2]][abs(neighborOffsetJ)];
					float discrepancy = currentGradient + (neighborOffsetI + neighborOffsetJ)* desiredGradient;

					boundaryDiscrepancy->push_back(discrepancy);
				}
			}
		}
	}
}

float DepthImageInpainting::evaluateBoundaryDiscrepancy(const vector<float>& boundaryDiscrepancy, float depth) const
{
	int len = static_cast<int>(boundaryDiscrepancy.size());
	float ret = 0;
	for (int i = 0; i < len; ++i)
	{
		ret += abs(boundaryDiscrepancy[i] + depth);
	}
	return ret;
}

void DepthImageInpainting::updateBoundaryType(const vector<Eigen::Vector3i>& holePixels, const vector<int>& boundaryPixels, const Eigen::VectorXd& baseDepth, float optimalDepthBoundary)
{
	
	int height = mCurrentDepthImage.Height();
	int width = mCurrentDepthImage.Width();
	int numBoundaryPixels = static_cast<int>(boundaryPixels.size());
	for (int i = 0; i < numBoundaryPixels; ++i)
	{
		int currentPxIdx = boundaryPixels[i];
		const Eigen::Vector3i& coord = holePixels[currentPxIdx];
		for (int neighborOffsetI = -1; neighborOffsetI <= 1; ++neighborOffsetI)
		{
			for (int neighborOffsetJ = -1; neighborOffsetJ <= 1; ++neighborOffsetJ)
			{
				int neighborI = coord[0] + neighborOffsetI;
				int neighborJ = coord[1] + neighborOffsetJ;
				if (abs(neighborOffsetI) == abs(neighborOffsetJ)) continue;

				if (neighborI < 0 || neighborI >= height || neighborJ < 0 || neighborJ >= width || mBoundaryType[neighborI][neighborJ] == NEUMANN_BOUNDARY)
				{
					continue;
				}
				else if (mBoundaryType[neighborI][neighborJ] == DIRICHELT_BOUNDARY)
				{
					int gradientIIdx = coord[0] + (neighborOffsetI - 1) / 2;
					int gradientJIdx = coord[1] + (neighborOffsetJ - 1) / 2;

					float currentGradient = baseDepth[currentPxIdx] + optimalDepthBoundary - mCurrentDepthImage[neighborI][neighborJ][coord[2]].d;
					float desiredGradient = mFeatureImage[gradientIIdx][gradientJIdx][coord[2]][abs(neighborOffsetJ)];
					float discrepancy = currentGradient + (neighborOffsetI + neighborOffsetJ)* desiredGradient;

					if (abs(discrepancy) > 20) //threshold value, jie tan hack, 20 for the top view
					{
						mBoundaryType[neighborI][neighborJ] = NEUMANN_BOUNDARY;
					}
				}
			}
		}
	}
}

void DepthImageInpainting::searchOptimalBoundaryCondition(int ithHole, const Eigen::VectorXd& baseDepth)
{
	float startDepth = 0;
	float endDepth = 5000;
	float step = 10;
	float minScore = FLT_MAX;
	float bestDepth = 0;
	vector<int> boundaryPixels;
	vector<float> boundaryDiscrepancy;
	identifyBoundaryPixels(mConnectedHolePixelCoordinates[ithHole], &boundaryPixels);

	int numBoundaryPixels = static_cast<int>(boundaryPixels.size());
	computeBaseDiscrepancy(mConnectedHolePixelCoordinates[ithHole], boundaryPixels, baseDepth, &boundaryDiscrepancy);
	for (float currentDepth = startDepth; currentDepth < endDepth; currentDepth += step)
	{
		float score = evaluateBoundaryDiscrepancy(boundaryDiscrepancy, currentDepth);
		if (score < minScore)
		{
			minScore = score;
			bestDepth = currentDepth;
		}
		LOG(INFO) << currentDepth << ": " << score;
	}
	for (int i = 0; i < numBoundaryPixels; ++i) //update the dirichelet boundary condition
	{
		updateBoundaryType(mConnectedHolePixelCoordinates[ithHole], boundaryPixels, baseDepth, bestDepth);
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
	float min = -10.f / powf(2.f, ithPyramid);
	float max = -min;
	featureGx.SaveDepthOnionImage("results/featureGx", NULL, &min, &max);
	featureGy.SaveDepthOnionImage("results/featureGy", NULL, &min, &max);
}

void DepthImageInpainting::computeFilledPixelNormals(int ithLayer)
{
	int height = mCurrentDepthImage.Height();
	int width = mCurrentDepthImage.Width();
	float stepSize = mCamera->GetOrthoWidth() / width;
	for (int v = 0; v < height; ++v)
	{
		for (int u = 0; u < width; ++u)
		{
			if (mCurrentMaskImage[v][u].size() <= ithLayer || mCurrentMaskImage[v][u][ithLayer] == MASK_KNOWN) continue;

			Eigen::Vector3f gp = mCamera->GetPoint(v, u, mCurrentDepthImage[v][u][ithLayer].d);
			//if ((gp - Eigen::Vector3f(1.05669, 1.34141, 1.33236)).norm() < 1e-5)
			//	printf("hello");

			Eigen::Vector3f tangentialAxis1(0, stepSize, mFeatureImage[v][u][ithLayer][0] / 1000.f);
			Eigen::Vector3f tangentialAxis2(stepSize, 0, mFeatureImage[v][u][ithLayer][1] / 1000.f);
			Eigen::Vector3f normal = tangentialAxis1.cross(tangentialAxis2);
			normal = normal.normalized(); //sometimes I flip the normal mannually.
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

void DepthImageInpainting::SetMesh(const vector<Eigen::Vector3f>& vertices, const vector<Eigen::Vector3i>& indices)
{
	mMeshVertices = vertices;
	mMeshIndices = indices;
}