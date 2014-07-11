#include "DepthCamera.h"
#include <algorithm>

using namespace std;
#include <glog/logging.h>
using namespace google;
#include "utility/mathlib.h"
#include "utility/utility.h"
#include "KDTree.h"
#include <intrin.h>



float dist(const ExtendedDepthPixel& lhs, const ExtendedDepthPixel& rhs)
{
	return (abs(lhs.d - rhs.d));
}

DepthCamera::DepthCamera() : mProjType(PERSP_PROJ)
{

}
DepthCamera::~DepthCamera()
{

}

void DepthCamera::SetSimplifiedPointCloud(const vector<Eigen::Vector3f>& points)
{
	mSimplifiedPointCloud = points;
	int numPoints = static_cast<int>(points.size());
	mKDTreePointCloud.setDim(3);
	for (int i = 0; i < numPoints; ++i)
	{
		if (i % 10000 == 0)
			LOG(INFO) << "Finish adding " << i << "th points out of " << numPoints;
		mKDTreePointCloud.add(points[i]);
	}
	mKDTreePointCloud.initANN();
}

void DepthCamera::SetComparisonROI(int left, int right, int high, int low, float near, float far)
{
	mComparisonROI = ROI(left, right, high, low, near, far);
}

float DepthCamera::GetOrthoWidth() const
{
	return mOrthoWidth;
}
void DepthCamera::SetOrthoWidth(float width)
{
	mOrthoWidth = width;
	mOrthoHeight = mHeight * mOrthoWidth / mWidth;
}
void DepthCamera::SetProjectionType(int projectionType)
{
	mProjType = projectionType;
}
float DepthCamera::GetFocalLength() const
{
	return mFocalLength;
}

const Eigen::Matrix4f& DepthCamera::GetCameraPose() const
{
	return mPose;
}
void DepthCamera::SetIntrinsicParameters(int numPxWidth, int numPxHeight, float focalLenth)
{
	mWidth = numPxWidth;
	mHeight = numPxHeight;
	mFocalLength = focalLenth;
}
void DepthCamera::SetExtrinsicParameters(const Eigen::Matrix4f& pose)
{
	mPose = pose;
	mInvPose = mPose.inverse();
}

void DepthCamera::SaveMultilayerDepthImage(const string& filename)
{
	mDepthMap.Save(filename);
}
void DepthCamera::ReadMultilayerDepthImage(const string& filename)
{
	mDepthMap.Read(filename);

}
void DepthCamera::ProcessMultiLayerDepthImage()
{
	mDepthMap.Process();
}

void DepthCamera::SimplifyMultiLayerDepthImage()
{
	mDepthMap.Simplify();
}

void DepthCamera::Capture(const vector<Eigen::Vector3f>& vertices, const vector<Eigen::Vector3i>& indices)
{
	const int maxNumDepthLayers = 20;
	const float maxDepth = 50;
	//Vector3f rayOrigin = mPose.col(3).head(3);
	//Eigen::Vector3f rayOrigin = Eigen::Vector3f::Zero();

	int numVertices = static_cast<int>(vertices.size());
	int numFaces = static_cast<int>(indices.size());
	vector<Eigen::Vector3f> faceNormals;
	faceNormals.resize(numFaces);
	for (int i = 0; i < numFaces; ++i)
	{
		Eigen::Vector3f v01 = vertices[indices[i][1]] - vertices[indices[i][0]];
		Eigen::Vector3f v02 = vertices[indices[i][2]] - vertices[indices[i][0]];
		Eigen::Vector3f n = v01.cross(v02);
		n.normalize();
		faceNormals[i] = n;
	}

	Eigen::Matrix4f poseInv = mPose.inverse();
	aiMatrix4x4 identity;
	aiMesh mesh;

	mesh.mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
	mesh.mNumVertices = numVertices;
	mesh.mNumFaces = numFaces;
	mesh.mVertices = new aiVector3D[mesh.mNumVertices];
	for (int i = 0; i < numVertices; ++i)
	{
		Eigen::Vector4f verticesInCameraSpace = /*poseInv * */Eigen::Vector4f(vertices[i][0], vertices[i][1], vertices[i][2], 1);
		mesh.mVertices[i] = aiVector3D(verticesInCameraSpace[0], verticesInCameraSpace[1], verticesInCameraSpace[2]);
	}
	mesh.mFaces = new aiFace[mesh.mNumFaces];
	for (int i = 0; i < numFaces; ++i)
	{
		mesh.mFaces[i].mNumIndices = 3;
		mesh.mFaces[i].mIndices = new unsigned int[3];
		mesh.mFaces[i].mIndices[0] = indices[i][0];
		mesh.mFaces[i].mIndices[1] = indices[i][1];
		mesh.mFaces[i].mIndices[2] = indices[i][2];
	}

	mKDTree.addMesh(&mesh, identity, NULL, 1);
	mKDTree.process();
	Eigen::Vector3f lookAtDir = mPose.block(0, 0, 3, 3) * Eigen::Vector3f(0, 0, 1);
	mDepthMap.Create(mHeight, mWidth);
	for (int i = 0; i < mHeight; i += 1)
	{	
		LOG(INFO) << "DepthCamera::Capture() is processing " << i << "th row.";

		for (int j = 0; j < mWidth; ++j)
		{
			//if (!(i == 267 && abs(j - 337) <= 0)) continue;
			//__debugbreak();
			Eigen::Vector3f rayOrigin = constructRayOrigin(i, j);
			Eigen::Vector3f rayDir = constructRayDirection(i, j);

			aiVector3D rayDirection(rayDir[0], rayDir[1], rayDir[2]);
			aiVector3D rayStart(rayOrigin[0], rayOrigin[1], rayOrigin[2]);
			aiVector3D rayEnd = rayStart + maxDepth * rayDirection;
			for (int ithLayer = 0; ithLayer < maxNumDepthLayers; ++ithLayer)
			{
				aiVector3D intersectionPt;
				int triangleId;
				bool bHit = mKDTree.intersect(rayStart, rayEnd, intersectionPt, triangleId);
				if (!bHit) break;
				Eigen::Vector3f intersection(intersectionPt[0], intersectionPt[1], intersectionPt[2]);
				float distance = (intersection - rayOrigin).norm();
				float depth = (intersection - rayOrigin).dot(lookAtDir); //intersectionPt[2];
				if (depth > 0)
					mDepthMap[i][j].push_back(ExtendedDepthPixel(depth * 1000, faceNormals[triangleId]));
				rayStart = rayEnd - (maxDepth - distance - 0.005f) * rayDirection;
			}
		}
	}
	ProcessMultiLayerDepthImage();
	
	char depthImageFileName[512];
	char* projType;
	if (mProjType == ORTHO_PROJ)
	{
		projType = "Ortho";
	}
	else
	{
		projType = "Persp";
	}
	sprintf(depthImageFileName, "results/depthFromMultiviewMesh_%s.png", projType);

	mDepthMap.SaveDepthImage(depthImageFileName);
	sprintf(depthImageFileName, "results/depthFromMultiviewMesh_%s.data", projType);
	SaveMultilayerDepthImage(depthImageFileName);
	
}

void DepthCamera::Capture(const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals)
{
	
	mDepthMap.Create(mHeight, mWidth);
	//Vector3f rayOrigin = mPose.col(3).head(3);
	//for (int i = 0; i < mHeight; ++i)
	//{
	//	mDepthMap[i].resize(mWidth);
	//	
	//	for (int j = 0; j < mWidth; ++j)
	//	{
	//		LOG(INFO) << "DepthCamera::Capture() is processing " << i << "th row, " << j << "th column.";

	//		Vector3f rayDir = constructRayDirection(i, j);
	//		constructDepthMap(rayOrigin, rayDir, points, normals, mDepthMap[i][j]);
	//		if (!mDepthMap[i][j].empty())
	//		{
	//			sort(mDepthMap[i][j].begin(), mDepthMap[i][j].end());
	//			mSensorMeasurement.at<float>(i, j) = *(mDepthMap[i][j].begin());
	//		}
	//		else
	//		{
	//			mSensorMeasurement.at<float>(i, j) = 0;
	//		}
	//		
	//	}
	//}

	int numPoints = static_cast<int>(points.size());
	int numPointsOnePercent = numPoints / 100;
	vector<Eigen::Vector3f> pointsInCameraSpace;
	pointsInCameraSpace.resize(numPoints);
	Eigen::Matrix4f poseInv = mPose.inverse();
	float fx = mFocalLength;
	float fy = mFocalLength;
	float cx = (mWidth - 1) / 2.f;
	float cy = (mHeight - 1) / 2.f;
	if (mProjType == PERSP_PROJ)
	{
		for (int i = 0; i < numPoints; ++i)
		{
			if (i % numPointsOnePercent == 0)
				LOG(INFO) << "DepthCamera::Capture() finished " << i / numPointsOnePercent << " percent.";
			pointsInCameraSpace[i] = (poseInv * Eigen::Vector4f(points[i][0], points[i][1], points[i][2], 1)).head(3);
			double x = pointsInCameraSpace[i][0];
			double y = pointsInCameraSpace[i][1];
			double z = pointsInCameraSpace[i][2];
			double u = x * fx / z + cx;
			double v = y * fy / z + cy;
			int uIdx = static_cast<int>(u + 0.5);
			int vIdx = static_cast<int>(v + 0.5);
			//if (abs(uIdx - static_cast<int>(cx)) < 5 && abs(vIdx - static_cast<int>(cy)) < 5)
			//{
			//	LOG(INFO) << x << "," << y << ": " << i << ": " << uIdx << "," << vIdx;
			//}
			if (uIdx < 0 || uIdx >= mWidth - 1 || vIdx < 0 || vIdx >= mHeight - 1)
				continue;
			mDepthMap[vIdx][uIdx].push_back(ExtendedDepthPixel(static_cast<float>(1000 * z), normals[i]));
		}
	}
	else
	{
		//GetOrthoProjBoundingBox();

		for (int i = 0; i < numPoints; ++i)
		{
			if (i % numPointsOnePercent == 0)
				LOG(INFO) << "DepthCamera::Capture() finished " << i / numPointsOnePercent << " percent.";
			pointsInCameraSpace[i] = (poseInv * Eigen::Vector4f(points[i][0], points[i][1], points[i][2], 1)).head(3);
			double x = pointsInCameraSpace[i][0];
			double y = pointsInCameraSpace[i][1];
			double z = pointsInCameraSpace[i][2];
			double stepSize = mWidth / mOrthoWidth;
			double u = x * stepSize + cx;
			double v = y * stepSize + cy;
			int uIdx = static_cast<int>(u + 0.5);
			int vIdx = static_cast<int>(v + 0.5);
			//if (abs(uIdx - static_cast<int>(cx)) < 5 && abs(vIdx - static_cast<int>(cy)) < 5)
			//{
			//	LOG(INFO) << x << "," << y << ": " << i << ": " << uIdx << "," << vIdx;
			//}
			if (uIdx < 0 || uIdx >= mWidth - 1 || vIdx < 0 || vIdx >= mHeight - 1)
				continue;
			mDepthMap[vIdx][uIdx].push_back(ExtendedDepthPixel(static_cast<float>(1000 * z), normals[i]));
		}
	}

	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			if (!mDepthMap[i][j].empty())
			{
				sort(mDepthMap[i][j].begin(), mDepthMap[i][j].end());
			}
		}
	}
	ProcessMultiLayerDepthImage();
	//fromMultiLayerToSinglelLayerDepthImage();
	char depthImageFileName[512];
	char* projType;
	if (mProjType == ORTHO_PROJ)
	{
		projType = "Ortho";
	}
	else
	{
		projType = "Persp";
	}
	sprintf(depthImageFileName, "results/depthFromMultiview_%s.png", projType);
	mDepthMap.SaveDepthImage(depthImageFileName);


	sprintf(depthImageFileName, "results/depthFromMultiview_%s.data", projType);
	mDepthMap.Save(depthImageFileName);

}

float DepthCamera::GetDepth(const Eigen::Vector3f& pt, int* ithRow, int* jthCol) const
{
	Eigen::Vector3f ptInCameraSpace = (mInvPose * Eigen::Vector4f(pt[0], pt[1], pt[2], 1)).head(3);

	float cx = (mWidth - 1) / 2.f;
	float cy = (mHeight - 1) / 2.f;
	double stepSize = mWidth / mOrthoWidth;
	double u = ptInCameraSpace[0] * stepSize + cx;
	double v = ptInCameraSpace[1] * stepSize + cy;
	if (ithRow)
	{
		*ithRow = static_cast<int>(v + 0.5);
	}
	if (jthCol)
	{
		*jthCol = static_cast<int>(u + 0.5);
	}
	return ptInCameraSpace[2] * 1000;
}

Eigen::Vector3f DepthCamera::GetPoint(int ithRow, int jthCol, float depth) const
{
	float fx = mFocalLength;
	float fy = mFocalLength;
	float cx = (mWidth - 1) / 2.f;
	float cy = (mHeight - 1) / 2.f;
	Eigen::Vector4f w;
	float z = depth / 1000.f;

	if (mProjType == ORTHO_PROJ)
	{
		float stepSize = mWidth / mOrthoWidth;
		float x = (jthCol - cx) / stepSize;
		float y = (ithRow - cy) / stepSize;

		w = mPose * Eigen::Vector4f(x, y, z, 1);
	}
	else
	{

		float x = (jthCol - cx) * z / fx;
		float y = (ithRow - cy) * z / fy;

		w = mPose * Eigen::Vector4f(x, y, z, 1);
	}
	return w.head(3);
}

void DepthCamera::getPointCloud(const MultilayerDepthImage& image, vector<Eigen::Vector3f>& points, vector<Eigen::Vector3f>& normals, int portion)
{
	points.clear();
	normals.clear();
	
	float fx = mFocalLength;
	float fy = mFocalLength;
	float cx = (mWidth - 1) / 2.f;
	float cy = (mHeight - 1) / 2.f;
	if (mMask.Height() != mHeight)
		portion = PORTION_ALL;
	if (mProjType == ORTHO_PROJ)
	{
		for (int v = 0; v < mHeight; ++v)
		{
			for (int u = 0; u < mWidth; ++u)
			{
				int len = static_cast<int>(image[v][u].size());

				for (int k = 0; k < len; ++k)
				{
					if (portion == PORTION_ALL || mMask[v][u][k] == portion)
					{
						float d = image[v][u][k].d;
						float z = d / 1000.f;

						float stepSize = mWidth / mOrthoWidth;
						float x = (u - cx) / stepSize;
						float y = (v - cy) / stepSize;

						Eigen::Vector4f w = mPose * Eigen::Vector4f(x, y, z, 1);

						points.push_back(w.head(3));
						normals.push_back(image[v][u][k].n);
					}
				}

			}
		}
	}
	else
	{
		for (int v = 0; v < mHeight; ++v)
		{
			for (int u = 0; u < mWidth; ++u)
			{
				int len = static_cast<int>(image[v][u].size());

				for (int k = 0; k < len; ++k)
				{
					if (portion == PORTION_ALL || mMask[v][u][k] == portion)
					{
						float d = image[v][u][k].d;
						float z = d / 1000.f;

						float x = (u - cx) * z / fx;
						float y = (v - cy) * z / fy;

						Eigen::Vector4f w = mPose * Eigen::Vector4f(x, y, z, 1);

						points.push_back(w.head(3));
						normals.push_back(image[v][u][k].n);
					}
				}

			}
		}
	}

}
void DepthCamera::GetPointCloud(vector<Eigen::Vector3f>& points, vector<Eigen::Vector3f>& normals, int portion)
{
	getPointCloud(mDepthMap, points, normals, portion);
}

void DepthCamera::SetData(const MultilayerDepthImage& depthMap)
{
	mDepthMap = depthMap;
}

void DepthCamera::SetData(const DepthImage& depthMap)
{

	int height = depthMap.NumRows();
	int width = depthMap.NumCols();
	mDepthMap.Create(height, width);
	for (int i = 0; i < height; ++i)
	{
		for (int j = 0; j < width; ++j)
		{
			if (depthMap.Depth(i, j) < 0.01) continue;
			mDepthMap[i][j].push_back(ExtendedDepthPixel(depthMap.Depth(i, j)));
		}
	}
	float stepSize = GetOrthoWidth() / width;
	for (int i = 0; i < height; ++i)
	{
		for (int j = 0; j < width; ++j)
		{
			if (depthMap.Depth(i, j) < 0.01) continue;

			if (i - 1 >= 0 && j - 1 >= 0)
			{
				Eigen::Vector3f tangentialAxis1(0, stepSize, (mDepthMap[i][j][0].d - mDepthMap[i - 1][j][0].d) / 1000.f);
				Eigen::Vector3f tangentialAxis2(stepSize, 0, (mDepthMap[i][j][0].d - mDepthMap[i][j - 1][0].d) / 1000.f);
				Eigen::Vector3f normal = tangentialAxis1.cross(tangentialAxis2);
				normal.normalize(); //sometimes I flip the normal mannually.

				Eigen::Vector3f gn = GetCameraPose().block(0, 0, 3, 3) * normal;
				mDepthMap[i][j][0].n = gn;
			}
		}
	}
	mDepthMap.Process();

}

void DepthCamera::SetMask(const MultilayerMaskImage& mask)
{
	mMask = mask;
}

void DepthCamera::SetReferenceDepthImages(const vector<DepthImage*> refImages)
{
	mRefDepthImages = refImages;
}
Eigen::Vector3f DepthCamera::constructRayOrigin(int i, int j)
{
	Eigen::Vector4f ret;
	if (mProjType == ORTHO_PROJ)
	{
		float cx = (mWidth - 1) / 2.f;
		float cy = (mHeight - 1) / 2.f;
		float stepSize = mWidth / mOrthoWidth;
		float x = (j - cx) / stepSize;
		float y = (i - cy) / stepSize;
		//LOG(INFO) << "Ray origin : (" << x << ", " << y << ").";
		ret = Eigen::Vector4f(x, y, 0, 1);
	}
	else
	{
		ret = Eigen::Vector4f(0, 0, 0, 1);
	}
	return (mPose * ret).head(3);
	//return ret.head(3);
}

Eigen::Vector3f DepthCamera::constructRayDirection(int i, int j)
{
	Eigen::Vector3f ret;

	float fx = mFocalLength;
	float fy = mFocalLength;
	float cx = (mWidth - 1) / 2.f;
	float cy = (mHeight - 1) / 2.f;

	if (mProjType == PERSP_PROJ)
	{
		ret[0] = (j - cx) / fx;
		ret[1] = (i - cy) / fy;
		ret[2] = 1;
	}
	else
	{
		ret = Eigen::Vector3f(0, 0, 1);
	}
	ret.normalize();

	ret = mPose.block(0, 0, 3, 3) * ret;
	return ret;
}

void DepthCamera::constructDepthMap(const Eigen::Vector3f& rayOrigin, const Eigen::Vector3f& rayDir, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals, vector<float>& depthMap)
{
	int numPoints = static_cast<int>(points.size());
	const double distThreshold = 0.01;
	for (int i = 0; i < numPoints; ++i)
	{
		double depth;
		double distance = PointRayDistance(points[i], rayOrigin, rayDir, depth);
		if (distance < distThreshold)
		{
			depthMap.push_back(static_cast<float>(depth * 1000));
		}
	}
}

const MultilayerDepthImage& DepthCamera::GetDepthImage() const
{
	return mDepthMap;
}
const MultilayerMaskImage& DepthCamera::GetMaskImage() const
{
	return mMask;
}

bool DepthCamera::needLeaveTrace(int ithRow, int jthCol, int kthLayer, int moveDir)
{
	switch (moveDir)
	{
	case MOVE_UP:
		if (jthCol + 1 < mHeight)
		{
			int numAdjacentLayers = static_cast<int>(mMask[ithRow][jthCol + 1].size());
			for (int ithAdjacentLayer = 0; ithAdjacentLayer < numAdjacentLayers; ++ithAdjacentLayer)
			{
				if (mMask[ithRow][jthCol + 1][ithAdjacentLayer] == MASK_KNOWN && abs(mDepthMap[ithRow][jthCol + 1][ithAdjacentLayer].d - mDepthMap[ithRow][jthCol][kthLayer].d) < 5)
					return true;
			}
		}
		break;
	//case MOVE_DOWN:
	//	break;
	//case MOVE_LEFT:
	//	break;
	//case MOVE_RIGHT:
	//	break;
	default:
		CHECK(0) << "Move direction is not recognized in DepthCamera::needLeaveTrace()";
	}
	
	return false;
	//Eigen::Vector3f pt = GetPoint(ithRow, jthCol, mDepthMap[ithRow][jthCol][kthLayer].d);
	//vector<int> nearestPtIdx = mKDTreePointCloud.kSearch(pt, 1);
	//Eigen::Vector3f nearestPt = mSimplifiedPointCloud[nearestPtIdx[0]];
	//int nearestPtIthRow, nearestPtIthCol;
	//GetDepth(nearestPt, &nearestPtIthRow, &nearestPtIthCol);
	//if ((pt - nearestPt).norm() < 0.04 && abs(nearestPtIthCol - jthCol) < 2)
	//{
	//	return true;
	//}
	//return false;
}

void DepthCamera::leaveTrace(int ithRowOld, int jthColOld, float depthOld, int ithRowNew, int jthColNew, float depthNew, int kthLayer, int moveDir, MultilayerDepthImageWithMask* image)
{
	//if (needLeaveTrace(ithRowOld, jthColOld, kthLayer, moveDir))
	{
		int currentDistance = 0;
		Eigen::Vector2f traceDir = Eigen::Vector2f(ithRowNew, jthColNew) - Eigen::Vector2f(ithRowOld, jthColOld);
		float lenOfTrace = traceDir.norm();
		traceDir.normalize();
		while (currentDistance < lenOfTrace)
		{
			float ratio = currentDistance / lenOfTrace;
			float depthInTrace = depthOld;// LinearInterpolate(depthOld, depthNew, ratio);
			Eigen::Vector2f pixelPos = Eigen::Vector2f(ithRowOld, jthColOld) + currentDistance * traceDir;
			(*image)[static_cast<int>(pixelPos[0])][static_cast<int>(pixelPos[1])].push_back(ExtendedDepthPixelWithMask(depthInTrace, Eigen::Vector3f::Zero(), MASK_UNKNOWN));
			currentDistance += 1;
		}
	}
}

bool DepthCamera::isPixelKnown(int ithRow, int jthCol, int kthLayer, const MultilayerMaskImage& mask)
{
	if (ithRow < 0 || ithRow >= mask.Height() || jthCol < 0 || jthCol >= mask.Width() || mask[ithRow][jthCol].size() <= kthLayer)
		return true;
	return mask[ithRow][jthCol][kthLayer] == MASK_KNOWN;
}

void DepthCamera::CrossViewMaskUpdate1(const DepthCamera& otherViewOld, const DepthCamera& otherViewNew, const MultilayerDepthImage& depthImagePointCloudOtherView, int moveDir, MultilayerDepthImage* newDepthImage, MultilayerMaskImage* newMaskImage)
{
	if (moveDir != MOVE_UP)
	{
		CHECK(0) << "Only move up is implemented in DepthCamera::CrossViewMaskUpdate1()";
	}
	vector<Eigen::Vector3i> rootCandidate;
	vector<pair<int, int> > rootNeighbors; //first is the lowest neighbor which is unknown and second is the highest neighbor which is unknown
	for (int i = 240; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			int numLayers = static_cast<int>(mDepthMap[i][j].size());
			for (int k = 0; k < numLayers; ++k)
			{
				if (j == 299 && i == 282)
					printf("hello");
				if (isPixelKnown(i, j, k, mMask))
				{
					for (int l = 0; l < mDepthMap[i - 1][j].size(); ++l)
					{
						if (!isPixelKnown(i - 1, j, l, mMask) && abs(mDepthMap[i][j][k].d - mDepthMap[i - 1][j][l].d) < 100)
						{

							rootCandidate.push_back(Eigen::Vector3i(i, j, k));
							int start = i - 1;
							while (!isPixelKnown(start, j, k, mMask) && start >= 0)
							{
								start--;
							}
							rootNeighbors.push_back(pair<int, int>(i - 1, start + 1));
						}
					}

				}
			}
		}
	}

	newDepthImage->Create(mHeight, mWidth);
	newMaskImage->Create(mHeight, mWidth);

	MultilayerDepthImageWithMask intermediateResult;
	intermediateResult.Create(mHeight, mWidth);

	const MultilayerDepthImage& depthImageOtherViewOld = otherViewOld.GetDepthImage();
	const MultilayerMaskImage& maskImageOtherViewOld = otherViewOld.GetMaskImage();
	const MultilayerDepthImage& depthImageOtherViewNew = otherViewNew.GetDepthImage();
	int ithRowOtherView, ithColOtherView, ithRowNewPt, ithColNewPt;

	MultilayerImage<Eigen::Vector2i> newCoordinates;
	newCoordinates.Create(mHeight, mWidth);

	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			
			//__debugbreak();
			int numLayers = static_cast<int>(mDepthMap[i][j].size());
			newCoordinates[i][j].resize(numLayers, Eigen::Vector2i(-1, -1));
			for (int k = 0; k < numLayers; ++k)
			{
				if (i == 411 && j == 309 && k == 4)
					printf("hello");
				if (mMask[i][j][k] == MASK_KNOWN)
				{
					intermediateResult[i][j].push_back(ExtendedDepthPixelWithMask(mDepthMap[i][j][k], mMask[i][j][k]));
					continue;
				}
				float depthDelta;

				Eigen::Vector3f pt = GetPoint(i, j, mDepthMap[i][j][k].d);
				float otherViewDepth = otherViewOld.GetDepth(pt, &ithRowOtherView, &ithColOtherView);
				if (ithRowOtherView < 0 || ithRowOtherView >= mHeight || ithColOtherView < 0 || ithColOtherView >= mWidth)
				{
					intermediateResult[i][j].push_back(ExtendedDepthPixelWithMask(mDepthMap[i][j][k], MASK_UNKNOWN));
		
					continue;
				}
				const vector<ExtendedDepthPixel>& otherViewPixel = depthImageOtherViewOld[ithRowOtherView][ithColOtherView];
				int lenOtherView = static_cast<int>(otherViewPixel.size());
				bool bFound = false;
				for (int l = 0; l < lenOtherView; ++l)
				{
					if (abs(otherViewPixel[l].d - otherViewDepth) < 2)
					{
						bFound = true;
						Eigen::Vector3f newPt = otherViewNew.GetPoint(ithRowOtherView, ithColOtherView, depthImageOtherViewNew[ithRowOtherView][ithColOtherView][l].d);

						float newDepth = GetDepth(newPt, &ithRowNewPt, &ithColNewPt);
						//if (abs(depthImageOtherViewNew[ithRowOtherView][ithColOtherView][l].n.dot(GetCameraFront())) > 0.1)
						intermediateResult[ithRowNewPt][ithColNewPt].push_back(ExtendedDepthPixelWithMask(newDepth, depthImageOtherViewNew[ithRowOtherView][ithColOtherView][l].n, MASK_UNKNOWN));
						newCoordinates[i][j][k] = (Eigen::Vector2i(ithRowNewPt, ithColNewPt));
						break;
					}
				}
				if (!bFound)
				{
					//if (isDepthValid(i, j, mDepthMap[i][j][k].d, depthDelta))
					//intermediateResult[i][j].push_back(ExtendedDepthPixelWithMask(mDepthMap[i][j][k], MASK_UNKNOWN));
				
				}
			}
		}
	}

	int numRoots = static_cast<int>(rootCandidate.size());
	{
		for (int i = 0; i < numRoots; ++i)
		{
			const Eigen::Vector3i& rootCoord = rootCandidate[i];
			if (rootCoord[1] == 299)
				printf("hello");
			int rootHeight = rootCoord[0];
			int minHeight = rootHeight;
			for (int start = rootNeighbors[i].first; start >= rootNeighbors[i].second; --start)
			{
				int neighborHeight = newCoordinates[start][rootCoord[1]][rootCoord[2]][0];
				if (neighborHeight >= 0 && neighborHeight < minHeight)
					minHeight = neighborHeight;
			}

			if (minHeight < rootHeight)
			{
				leaveTrace(rootNeighbors[i].first, rootCoord[1], mDepthMap[rootCoord[0]][rootCoord[1]][rootCoord[2]].d, minHeight, rootCoord[1], mDepthMap[rootCoord[0]][rootCoord[1]][rootCoord[2]].d, rootCoord[2], MOVE_UP, &intermediateResult);
			}
		}
	}

	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			if (!intermediateResult[i][j].empty())
				sort(intermediateResult[i][j].begin(), intermediateResult[i][j].end());

		}
	}
	intermediateResult.Simplify();
	float stepSize = GetOrthoWidth() / mWidth;
	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			int len = static_cast<int>(intermediateResult[i][j].size());
			for (int k = 0; k < len; ++k)
			{

				if (i - 1 >= 0 && j - 1 >= 0 && k < intermediateResult[i - 1][j].size() && k < intermediateResult[i][j - 1].size())
				{
					Eigen::Vector3f tangentialAxis1(0, stepSize, (intermediateResult[i][j][k].d - intermediateResult[i - 1][j][k].d) / 1000.f);
					Eigen::Vector3f tangentialAxis2(stepSize, 0, (intermediateResult[i][j][k].d - intermediateResult[i][j - 1][k].d) / 1000.f);
					Eigen::Vector3f normal = tangentialAxis1.cross(tangentialAxis2);
					normal.normalize(); //sometimes I flip the normal mannually.

					Eigen::Vector3f gn = GetCameraPose().block(0, 0, 3, 3) * normal;
					intermediateResult[i][j][k].n = gn;
				}
				(*newDepthImage)[i][j].push_back(ExtendedDepthPixel(intermediateResult[i][j][k].d, intermediateResult[i][j][k].n));
				(*newMaskImage)[i][j].push_back(intermediateResult[i][j][k].m);
			}
		}
	}
//	boundariesFromNearestNeighbor(*newDepthImage, *newMaskImage);
}
void DepthCamera::CrossViewMaskUpdate(const DepthCamera& otherViewOld, const DepthCamera& otherViewNew, const MultilayerDepthImage& depthImagePointCloudOtherView, int moveDir, MultilayerDepthImage* newDepthImage, MultilayerMaskImage* newMaskImage)
{
	newDepthImage->Create(mHeight, mWidth);
	newMaskImage->Create(mHeight, mWidth);

	MultilayerDepthImageWithMask intermediateResult;
	intermediateResult.Create(mHeight, mWidth);

	const MultilayerDepthImage& depthImageOtherViewOld = otherViewOld.GetDepthImage();
	const MultilayerMaskImage& maskImageOtherViewOld = otherViewOld.GetMaskImage();
	const MultilayerDepthImage& depthImageOtherViewNew = otherViewNew.GetDepthImage();
	int ithRowOtherView, ithColOtherView, ithRowNewPt, ithColNewPt;

	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			if (i == 222 && j == 343)
				printf("hello");
				//__debugbreak();
			int numLayers = static_cast<int>(mDepthMap[i][j].size());
			for (int k = 0; k < numLayers; ++k)
			{
				if (mMask[i][j][k] == MASK_KNOWN)
				{
					intermediateResult[i][j].push_back(ExtendedDepthPixelWithMask(mDepthMap[i][j][k], mMask[i][j][k]));
					continue;
				}
				float depthDelta;

				Eigen::Vector3f pt = GetPoint(i, j, mDepthMap[i][j][k].d);
				float otherViewDepth = otherViewOld.GetDepth(pt, &ithRowOtherView, &ithColOtherView);
				if (ithRowOtherView < 0 || ithRowOtherView >= mHeight || ithColOtherView < 0 || ithColOtherView >= mWidth)
				{
					intermediateResult[i][j].push_back(ExtendedDepthPixelWithMask(mDepthMap[i][j][k], MASK_UNKNOWN));
					continue;
				}
				const vector<ExtendedDepthPixel>& otherViewPixel = depthImageOtherViewOld[ithRowOtherView][ithColOtherView];
				int lenOtherView = static_cast<int>(otherViewPixel.size());
				bool bFound = false;
				for (int l = 0; l < lenOtherView; ++l)
				{
					if (abs(otherViewPixel[l].d - otherViewDepth) < 2)
					{
						bFound = true;
						Eigen::Vector3f newPt = otherViewNew.GetPoint(ithRowOtherView, ithColOtherView, depthImageOtherViewNew[ithRowOtherView][ithColOtherView][l].d);

						float newDepth = GetDepth(newPt, &ithRowNewPt, &ithColNewPt);
						//if (abs(depthImageOtherViewNew[ithRowOtherView][ithColOtherView][l].n.dot(GetCameraFront())) > 0.1)
							intermediateResult[ithRowNewPt][ithColNewPt].push_back(ExtendedDepthPixelWithMask(newDepth, depthImageOtherViewNew[ithRowOtherView][ithColOtherView][l].n, MASK_UNKNOWN));
						bool bNeedLeaveTrace = false;
						int nextDepthPointId = linearSearchInsertPos<ExtendedDepthPixel>(depthImagePointCloudOtherView[ithRowOtherView][ithColOtherView], otherViewDepth);
						if (nextDepthPointId < depthImagePointCloudOtherView[ithRowOtherView][ithColOtherView].size() && abs(depthImagePointCloudOtherView[ithRowOtherView][ithColOtherView][nextDepthPointId].d - otherViewDepth) < 100)
							bNeedLeaveTrace = true;
						if (bNeedLeaveTrace)
						{
						//	leaveTrace(i, j, mDepthMap[i][j][k].d, ithRowNewPt, ithColNewPt, newDepth, k, moveDir, &intermediateResult);
						}
						break;
					}
				}
				if (!bFound)
				{
					//if (isDepthValid(i, j, mDepthMap[i][j][k].d, depthDelta))
					//intermediateResult[i][j].push_back(ExtendedDepthPixelWithMask(mDepthMap[i][j][k], MASK_UNKNOWN));
				}
			}
		}
	}

	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			if (!intermediateResult[i][j].empty())
				sort(intermediateResult[i][j].begin(), intermediateResult[i][j].end());

		}
	}
	intermediateResult.Simplify();
	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			int len = static_cast<int>(intermediateResult[i][j].size());
			for (int k = 0; k < len; ++k)
			{
				(*newDepthImage)[i][j].push_back(ExtendedDepthPixel(intermediateResult[i][j][k].d, intermediateResult[i][j][k].n));
				(*newMaskImage)[i][j].push_back(intermediateResult[i][j][k].m);
			}
		}
	}
	//boundariesFromNearestNeighbor(*newDepthImage, *newMaskImage);
}
// assuming self is a noisy depth camera view but represent the truth, rhs is from the view of Poisson constructed mesh.
void DepthCamera::Compare(const DepthCamera& rhs, bool isBoundaryFromNearestNeighbor, MultilayerDepthImage& mergedDepthMap, MultilayerMaskImage& mask)
{
	const float pointMeshMergingThreshold = 15;

	mergedDepthMap.Create(mHeight, mWidth);
	mask.Create(mHeight, mWidth);
	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			//check whether the depth samples from mesh are presented as points
			//if (mDepthMap[i][j].empty()) continue;

			int selfSize = static_cast<int>(mDepthMap[i][j].size());
			int rhsSize = static_cast<int>(rhs.mDepthMap[i][j].size());
			vector<ExtendedDepthPixel> processingDepthMapFromMesh;
			vector<ExtendedDepthPixel> processingDepthSampleFromPoints;

			//VectorXi nearestSurfaceId = VectorXi::Constant(selfSize, - 1);
			Eigen::VectorXi numPointsAgreeWithSurface = Eigen::VectorXi::Zero(rhsSize);
			for (int ithDepthSample = 0; ithDepthSample < selfSize; ++ithDepthSample)
			{
				int id = findNearestPoint(rhs.mDepthMap[i][j], mDepthMap[i][j][ithDepthSample], pointMeshMergingThreshold, dist);
				if (id >= 0)
				{
					numPointsAgreeWithSurface[id] += 1;
				}
			}
			for (int ithDepthSample = 0; ithDepthSample < rhsSize; ++ithDepthSample)
			{
				//if (i == 231 && j == 292)
				//	__debugbreak();
				int insertPos = linearSearchInsertPos(mDepthMap[i][j], rhs.mDepthMap[i][j][ithDepthSample]);
				//if (insertPos == 0 && abs(rhs.mDepthMap[i][j][ithDepthSample].d - mDepthMap[i][j][0].d) >= pointMeshMergingThreshold)
				//{
				//	continue;
				//}
				if (/*mergedDepthMap[i][j].empty() || */numPointsAgreeWithSurface[ithDepthSample])
				{
					mask[i][j].push_back(MASK_KNOWN);
					//int id = findNearestPoint(mDepthMap[i][j], rhs.mDepthMap[i][j][ithDepthSample], pointMeshMergingThreshold, dist);
					//CHECK(id >= 0) << "Things go wrong in DepthCamera::Compare().";
					//mergedDepthMap[i][j].push_back(mDepthMap[i][j][id]);
					mergedDepthMap[i][j].push_back(rhs.mDepthMap[i][j][ithDepthSample]);
				}
				//else if (mComparisonROI.IsWithin(i, j, rhs.mDepthMap[i][j][ithDepthSample].d))//if (!mergedDepthMap[i][j].empty())
				//{
				//	mask[i][j].push_back(MASK_UNKNOWN);
				//	mergedDepthMap[i][j].push_back(ExtendedDepthPixel(rhs.mDepthMap[i][j][ithDepthSample].d, Eigen::Vector3f::Zero()));
				//}
				

			}
				//if (rhsSize == 0)
			//{
			//	processingDepthSampleFromPoints = mDepthMap[i][j];
			//}
			//else
			//{
			//	for (int ithDepthSample = 0; ithDepthSample < selfSize; ++ithDepthSample)
			//	{
			//		int id = findNearestPoint(rhs.mDepthMap[i][j], mDepthMap[i][j][ithDepthSample], pointMeshMergingThreshold, dist);
			//		nearestSurfaceId[ithDepthSample] = id;
			//		if (id >= 0)
			//		{
			//			numPointsAgreeWithSurface[id] += 1;
			//		}
			//	}


			//	for (int ithDepthSample = 0; ithDepthSample < rhsSize; ++ithDepthSample)
			//	{
			//		int mergeingStart = 0;
			//		if (numPointsAgreeWithSurface[ithDepthSample] == 0)
			//		{
			//			int insertPos = linearSearchInsertPos(mDepthMap[i][j], rhs.mDepthMap[i][j][ithDepthSample]);
			//			if (insertPos == 0 || insertPos == selfSize)
			//				continue;
			//			processingDepthMapFromMesh.push_back(rhs.mDepthMap[i][j][ithDepthSample]);
			//		}
			//	}
			//	if (selfSize > 2)
			//		printf("hello");
			//	int startId = 0;
			//	int countToMerge = 1;
			//	int prevId = nearestSurfaceId[0];
			//	for (int ithDepthSample = 1; ithDepthSample < selfSize; ++ithDepthSample)
			//	{
			//		int currentNearestId = nearestSurfaceId[ithDepthSample];
			//		if (currentNearestId == -1)
			//		{
			//			processingDepthSampleFromPoints.push_back(mDepthMap[i][j][ithDepthSample]);
			//		}
			//		else if (currentNearestId == prevId)
			//		{
			//			countToMerge++;
			//		}
			//		else
			//		{
			//			if (countToMerge > 0)
			//				processingDepthSampleFromPoints.push_back(mDepthMap[i][j][startId + countToMerge / 2]);
			//			startId = ithDepthSample;
			//			countToMerge = 1;
			//			prevId = currentNearestId;
			//		}
			//	}
			//	if (countToMerge > 0)
			//		processingDepthSampleFromPoints.push_back(mDepthMap[i][j][startId + countToMerge / 2]);
			//}

			//for (int ithDepthSample = 0; ithDepthSample < rhsSize; ++ithDepthSample)
			//{
			//	int insertPos = linearSearchInsertPos(mDepthMap[i][j], rhs.mDepthMap[i][j][ithDepthSample]);
			//	if (insertPos == 0 && abs(rhs.mDepthMap[i][j][ithDepthSample].d - mDepthMap[i][j][0].d) >= pointMeshMergingThreshold)
			//	{
			//		continue;
			//	}
			//	else if (insertPos == 0 && abs(rhs.mDepthMap[i][j][ithDepthSample].d - mDepthMap[i][j][0].d) < pointMeshMergingThreshold)
			//	{
			//		mergedDepthMap[i][j].push_back(rhs.mDepthMap[i][j][ithDepthSample]);
			//		mask[i][j].push_back(MASK_KNOWN);
			//		continue;
			//	}
			//	else if (insertPos == selfSize && abs(rhs.mDepthMap[i][j][ithDepthSample].d - mDepthMap[i][j][insertPos - 1].d) < pointMeshMergingThreshold)
			//	{
			//		mergedDepthMap[i][j].push_back(rhs.mDepthMap[i][j][ithDepthSample]);
			//		mask[i][j].push_back(MASK_KNOWN);
			//		continue;
			//	}
			//	else if (abs(rhs.mDepthMap[i][j][ithDepthSample].d - mDepthMap[i][j][insertPos].d) < pointMeshMergingThreshold || abs(rhs.mDepthMap[i][j][ithDepthSample].d - mDepthMap[i][j][insertPos - 1].d) < pointMeshMergingThreshold)
			//	{
			//		mergedDepthMap[i][j].push_back(rhs.mDepthMap[i][j][ithDepthSample]);
			//		mask[i][j].push_back(MASK_KNOWN);
			//		continue;
			//	}
			//	else
			//	{
			//		mergedDepthMap[i][j].push_back(rhs.mDepthMap[i][j][ithDepthSample]);
			//		mask[i][j].push_back(MASK_UNKNOWN);
			//		//processingDepthMapFromMesh.push_back(rhs.mDepthMap[i][j][ithDepthSample]);
			//	}
			//}




			//processingDepthSampleFromPoints = mDepthMap[i][j];
			////

			////merge source two list of depth samples
			//
			//rhsSize = static_cast<int>(processingDepthMapFromMesh.size());
			//selfSize = static_cast<int>(processingDepthSampleFromPoints.size());
			//for (int ithDepthSample = 0; ithDepthSample < rhsSize; ++ithDepthSample)
			//{
			//	processingDepthMapFromMesh[ithDepthSample].n = Vector3f::Zero(); // make it color so we can see it clearly in mesh lab.
			//}
			//int selfIdx = 0;
			//int rhsIdx = 0;
			//while (selfIdx < selfSize && rhsIdx < rhsSize)
			//{
			//	if (processingDepthSampleFromPoints[selfIdx] <= processingDepthMapFromMesh[rhsIdx])
			//	{
			//		mergedDepthMap[i][j].push_back(processingDepthSampleFromPoints[selfIdx]);
			//		mask[i][j].push_back(MASK_KNOWN);
			//		selfIdx++;
			//	}
			//	else
			//	{
			//		mergedDepthMap[i][j].push_back(processingDepthMapFromMesh[rhsIdx]);
			//		mask[i][j].push_back(MASK_UNKNOWN);
			//		rhsIdx++;
			//	}
			//}
			//if (selfIdx >= selfSize - 1)
			//{
			//	mergedDepthMap[i][j].insert(mergedDepthMap[i][j].end(), processingDepthMapFromMesh.begin() + rhsIdx, processingDepthMapFromMesh.end());
			//	mask[i][j].insert(mask[i][j].end(), rhsSize - rhsIdx, MASK_UNKNOWN);

			//}
			//if (rhsIdx >= rhsSize - 1)
			//{
			//	mergedDepthMap[i][j].insert(mergedDepthMap[i][j].end(), processingDepthSampleFromPoints.begin() + selfIdx, processingDepthSampleFromPoints.end());
			//	mask[i][j].insert(mask[i][j].end(), selfSize - selfIdx, MASK_KNOWN);
			//}
		}
	}
	//mask.Filter();
	if (isBoundaryFromNearestNeighbor)
		boundariesFromNearestNeighbor(mergedDepthMap, mask);
	//boundariesFromMultiview(mergedDepthMap, mask);
	mergedDepthMap.Process();

}

void DepthCamera::boundariesFromNearestNeighbor(MultilayerDepthImage& mergedDepthMap, MultilayerMaskImage& mask)
{
	vector<Eigen::Vector3i> newBoundaryConditionIdx;
	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			int numLayers = static_cast<int>(mask[i][j].size());
			for (int k = 0; k < numLayers; ++k)
			{
				float depth = mergedDepthMap[i][j][k].d;
				Eigen::Vector3f pt = GetPoint(i, j, depth);
				//if ((pt - Eigen::Vector3f(0.82916, 1.52718, 1.10822)).norm() < 1e-4)
				//	printf("hello");
				//if (i == 275 && j == 200 && k == 0)
				//	//printf("hello");
				//	__debugbreak();
				if (mask[i][j][k] == MASK_UNKNOWN && isHoleBoundary(i, j, k, mask))
				{
					vector<int> nearestPtIdx = mKDTreePointCloud.kSearch(pt, 1);
					Eigen::Vector3f nearestPt = mSimplifiedPointCloud[nearestPtIdx[0]];
					if ((pt - nearestPt).norm() < 0.05)
					{
						newBoundaryConditionIdx.push_back(Eigen::Vector3i(i, j, k));
						int v, u;
						float nearestDepth = GetDepth(nearestPt, &v, &u);
						mergedDepthMap[i][j][k] = nearestDepth;
					}
					//else
					//{
					//	mergedDepthMap[i][j].erase(mergedDepthMap[i][j].begin() + k);
					//	mask[i][j].erase(mask[i][j].begin() + k);
					//	numLayers = static_cast<int>(mask[i][j].size());
					//}
				}
			}
		}
	}
	int numNewBoundaryPixels = static_cast<int>(newBoundaryConditionIdx.size());
	LOG(INFO) << "Number of new dirichelet boundary conditions: " << numNewBoundaryPixels;
	for (int i = 0; i < numNewBoundaryPixels; ++i)
	{
		const Eigen::Vector3i& idx = newBoundaryConditionIdx[i];
		LOG(INFO) << "(" << idx[0] << "," << idx[1] << "," << idx[2] << "): " << mergedDepthMap[idx[0]][idx[1]][idx[2]].d;
		mask[idx[0]][idx[1]][idx[2]] = MASK_KNOWN;
	}
}

bool DepthCamera::isDepthValid(int ithRow, int jthCol, float depth, float& depthDelta)
{
	Eigen::Vector3f pt = GetPoint(ithRow, jthCol, depth);
	int numReferences = static_cast<int>(mRefDepthImages.size());
	float minDepthDelta = 10000;
	for (int ithReference = 0; ithReference < numReferences; ++ithReference)
	{
		float delta;
		if (mRefDepthImages[ithReference]->IsPointBehind(pt, delta))
		{
			if (delta < minDepthDelta)
				minDepthDelta = delta;
		}
		else
		{
			return false;
		}
	}
	depthDelta = minDepthDelta;
	return true;
}

bool DepthCamera::moveDepthUntilValid(int ithRow, int jthCol, int kthLayer, MultilayerDepthImage& mergedDepthMap) // return whether the pixel can be considered as a dirichelet boundary condition
{
	float depthCandidate = mergedDepthMap[ithRow][jthCol][kthLayer].d;
	Eigen::Vector3f pt = GetPoint(ithRow, jthCol, depthCandidate);

	const int maxNumMovements = 13;
	float depthDelta;

	if (!isDepthValid(ithRow, jthCol, depthCandidate, depthDelta))
	{
		float lowerBound = 0.f;
		float upperBound = 10000.f;
		bool bMoveNearer = true;

		if (kthLayer > 0)
			lowerBound = mergedDepthMap[ithRow][jthCol][kthLayer - 1].d;
		else
			LOG(FATAL) << "This should never happen.";
		if (kthLayer + 1 < mergedDepthMap[ithRow][jthCol].size())
		{
			upperBound = mergedDepthMap[ithRow][jthCol][kthLayer + 1].d;
		}


		if (isDepthValid(ithRow, jthCol, upperBound - 5, depthDelta))
		{
			lowerBound = depthCandidate;
			bMoveNearer = false;
		}
		if (isDepthValid(ithRow, jthCol, lowerBound + 5, depthDelta))
		{
			upperBound = depthCandidate;
			bMoveNearer = true;
		}
		else
		{
			if (abs(depthCandidate - lowerBound) > abs(depthCandidate - upperBound))
			{
				lowerBound = depthCandidate;
				bMoveNearer = false;
			}
			else
			{
				upperBound = depthCandidate;
				bMoveNearer = true;
			}
			//LOG(WARNING) << "No idea which direction to move in DepthCamera::moveDepthUntilValid().";
			//return false;
		}
		bool hasEverBeenValid = false;
		float minDepthDelta = 10000.f;
		for (int ithSearch = 0; ithSearch < maxNumMovements; ++ithSearch)
		{
			depthCandidate = (lowerBound + upperBound) / 2.f;
			if (isDepthValid(ithRow, jthCol, depthCandidate, depthDelta))
			{
				hasEverBeenValid = true;
				CHECK(depthDelta > 0);
				if (depthDelta < minDepthDelta)
					minDepthDelta = depthDelta;
				if (bMoveNearer)
					lowerBound = depthCandidate;
				else
					upperBound = depthCandidate;
			}
			else
			{
				if (bMoveNearer)
					upperBound = depthCandidate;
				else
					lowerBound = depthCandidate;
			}
		}
		if (hasEverBeenValid && minDepthDelta < 20)
		{
			mergedDepthMap[ithRow][jthCol][kthLayer].d = depthCandidate;
			return true;
		}
		return false;
	}
	return false;
}

bool DepthCamera::isHoleBoundary(int ithRow, int jthCol, int kthLayer, const MultilayerMaskImage& mask) //outer most hole pixel
{
	if (mask[ithRow][jthCol][kthLayer] == MASK_KNOWN) return false;
	if (ithRow > 0 && (mask[ithRow - 1][jthCol].size() <= kthLayer || mask[ithRow - 1][jthCol][kthLayer] == MASK_KNOWN))
		return true;
	if (ithRow + 1 < mHeight && (mask[ithRow + 1][jthCol].size() <= kthLayer || mask[ithRow + 1][jthCol][kthLayer] == MASK_KNOWN))
		return true;
	if (jthCol > 0 && (mask[ithRow][jthCol - 1].size() <= kthLayer || mask[ithRow][jthCol - 1][kthLayer] == MASK_KNOWN))
		return true;
	if (jthCol + 1 < mHeight && (mask[ithRow][jthCol + 1].size() <= kthLayer || mask[ithRow][jthCol + 1][kthLayer] == MASK_KNOWN))
		return true;
	return false;
}

void DepthCamera::boundariesFromMultiview(MultilayerDepthImage& mergedDepthMap, MultilayerMaskImage& mask)
{
	vector<Eigen::Vector3i> newBoundaryConditionIdx;
	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			int numLayers = static_cast<int>(mask[i][j].size());
			for (int k = 0; k < numLayers; ++k)
			{
				
				//if ((pt - Eigen::Vector3f(0.82916, 1.52718, 1.10822)).norm() < 1e-4)
				//	printf("hello");
				//if (i == 36 && j == 317 && k == 1)
				//	printf("hello");
				if (mask[i][j][k] == MASK_UNKNOWN && isHoleBoundary(i, j, k, mask))
				{
					if (moveDepthUntilValid(i, j, k, mergedDepthMap))
					{
						newBoundaryConditionIdx.push_back(Eigen::Vector3i(i, j, k));
					}
					//else
					//{
					//	mergedDepthMap[i][j].erase(mergedDepthMap[i][j].begin() + k);
					//	mask[i][j].erase(mask[i][j].begin() + k);
					//	numLayers = static_cast<int>(mask[i][j].size());
					//}
				}
			}
		}
	}
	int numNewBoundaryPixels = static_cast<int>(newBoundaryConditionIdx.size());
	LOG(INFO) << "Number of new dirichelet boundary conditions: " << numNewBoundaryPixels;
 	for (int i = 0; i < numNewBoundaryPixels; ++i)
	{
		const Eigen::Vector3i& idx = newBoundaryConditionIdx[i];
		LOG(INFO) << "(" << idx[0] << "," << idx[1] << "," << idx[2] << "): " << mergedDepthMap[idx[0]][idx[1]][idx[2]].d;
		mask[idx[0]][idx[1]][idx[2]] = MASK_KNOWN;
	}
}

Eigen::Vector3f DepthCamera::GetCameraFront() const
{
	return mPose.col(2).head(3);
}
Eigen::Vector3f DepthCamera::GetCameraPosition() const
{
	return mPose.col(3).head(3);
}
Eigen::Vector3f DepthCamera::GetCameraUp() const
{
	return mPose.col(1).head(3);
}
