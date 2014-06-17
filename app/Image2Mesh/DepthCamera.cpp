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

DepthCamera::DepthCamera() : mOrthoReference(NULL), mProjType(PERSP_PROJ)
{

}
DepthCamera::~DepthCamera()
{

}

void DepthCamera::SetOrthoReference(MultilayerDepthImage* fromPerspectiveProj)
{
	mOrthoReference = fromPerspectiveProj;
}

void DepthCamera::GetOrthoProjBoundingBox()
{
	CHECK(mOrthoReference) << "Reference multilayer depth image is needed for orthographic projection.";
	vector<Eigen::Vector3f> points;
	vector<Eigen::Vector3f> normals;
	getPointCloud(*mOrthoReference, points, normals);
	int numPoints = static_cast<int>(points.size());
	Eigen::Matrix4f poseInv = mPose.inverse();
	mOrthoWidth = 0;
	mOrthoHeight = 0;

	for (int i = 0; i < numPoints; ++i)
	{
		Eigen::Vector4f pointsInCameraSpace = poseInv * Eigen::Vector4f(points[i][0], points[i][1], points[i][2], 1);
		if (abs(pointsInCameraSpace[0]) > mOrthoWidth)
			mOrthoWidth = abs(pointsInCameraSpace[0]);
		if (abs(pointsInCameraSpace[1]) > mOrthoHeight)
			mOrthoHeight = abs(pointsInCameraSpace[1]);
	}
	if (static_cast<float>(mOrthoWidth) / mWidth > static_cast<float>(mOrthoHeight) / mHeight)
	{
		mOrthoHeight = static_cast<float>(mOrthoWidth) / mWidth * mHeight;
	}
	else
	{
		mOrthoWidth = static_cast<float>(mOrthoHeight) / mHeight * mWidth;
	}
	//mOrthoWidth *= 2;
	//mOrthoHeight *= 2;
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
			//if (i != 150 || j != 300) continue;
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

void DepthCamera::getPointCloud(const MultilayerDepthImage& image, vector<Eigen::Vector3f>& points, vector<Eigen::Vector3f>& normals)
{
	points.clear();
	normals.clear();
	
	float fx = mFocalLength;
	float fy = mFocalLength;
	float cx = (mWidth - 1) / 2.f;
	float cy = (mHeight - 1) / 2.f;

	if (mProjType == ORTHO_PROJ)
	{
		for (int v = 0; v < mHeight; ++v)
		{
			for (int u = 0; u < mWidth; ++u)
			{
				int len = static_cast<int>(image[v][u].size());

				for (int k = 0; k < len; ++k)
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
	else
	{
		for (int v = 0; v < mHeight; ++v)
		{
			for (int u = 0; u < mWidth; ++u)
			{
				int len = static_cast<int>(image[v][u].size());

				for (int k = 0; k < len; ++k)
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
void DepthCamera::GetPointCloud(vector<Eigen::Vector3f>& points, vector<Eigen::Vector3f>& normals)
{
	getPointCloud(mDepthMap, points, normals);
}

void DepthCamera::SetData(const MultilayerDepthImage& depthMap)
{
	mDepthMap = depthMap;
}

void DepthCamera::SetMask(const MultilayerMaskImage& mask)
{
	mMask = mask;
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


// assuming self is a noisy depth camera view but represent the truth, rhs is from the view of Poisson constructed mesh.
void DepthCamera::Compare(const DepthCamera& rhs, MultilayerDepthImage& mergedDepthMap, MultilayerMaskImage& mask)
{
	const int MASK_UNKNOWN = 1;
	const int MASK_KNOWN = 0;
	const float pointMeshMergingThreshold = 50;

	mergedDepthMap.Create(mHeight, mWidth);
	mask.Create(mHeight, mWidth);
	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			//check whether the depth samples from mesh are presented as points
			if (mDepthMap[i][j].empty()) continue;

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
				int insertPos = linearSearchInsertPos(mDepthMap[i][j], rhs.mDepthMap[i][j][ithDepthSample]);
				if (insertPos == 0 && abs(rhs.mDepthMap[i][j][ithDepthSample].d - mDepthMap[i][j][0].d) >= pointMeshMergingThreshold)
				{
					continue;
				}
				if (mergedDepthMap[i][j].empty() || numPointsAgreeWithSurface[ithDepthSample])
					mask[i][j].push_back(MASK_KNOWN);
				else
					mask[i][j].push_back(MASK_UNKNOWN);
				mergedDepthMap[i][j].push_back(rhs.mDepthMap[i][j][ithDepthSample]);

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
	mergedDepthMap.Process();
}