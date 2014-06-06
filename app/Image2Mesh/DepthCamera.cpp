#include "DepthCamera.h"
#include <algorithm>

using namespace std;
#include <glog/logging.h>
using namespace google;
#include "utility/mathlib.h"
#include "utility/utility.h"
#include "KDTree.h"
#include <intrin.h>

DepthCamera::DepthCamera()
{
	mMinDepth = FLT_MAX;
	mMaxDepth = 0;
}
DepthCamera::~DepthCamera()
{

}
void DepthCamera::SetIntrinsicParameters(int numPxWidth, int numPxHeight, float focalLenth)
{
	mWidth = numPxWidth;
	mHeight = numPxHeight;
	mFocalLength = focalLenth;
}
void DepthCamera::SetExtrinsicParameters(const Matrix4f& pose)
{
	mPose = pose;
}

void DepthCamera::Capture(const vector<Vector3f>& vertices, const vector<Vector3i>& indices)
{
	const int maxNumDepthLayers = 20;
	const float maxDepth = 50;
	//Vector3f rayOrigin = mPose.col(3).head(3);
	Vector3f rayOrigin = Vector3f::Zero();

	int numVertices = static_cast<int>(vertices.size());
	int numFaces = static_cast<int>(indices.size());
	vector<Vector3f> faceNormals;
	faceNormals.resize(numFaces);
	for (int i = 0; i < numFaces; ++i)
	{
		Vector3f v01 = vertices[indices[i][1]] - vertices[indices[i][0]];
		Vector3f v02 = vertices[indices[i][2]] - vertices[indices[i][0]];
		Vector3f n = v01.cross(v02);
		n.normalize();
		faceNormals[i] = n;
	}

	Matrix4f poseInv = mPose.inverse();
	aiMatrix4x4 identity;
	aiMesh mesh;

	mesh.mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
	mesh.mNumVertices = numVertices;
	mesh.mNumFaces = numFaces;
	mesh.mVertices = new aiVector3D[mesh.mNumVertices];
	for (int i = 0; i < numVertices; ++i)
	{
		Vector4f verticesInCameraSpace = poseInv * Vector4f(vertices[i][0], vertices[i][1], vertices[i][2], 1);
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

	mDepthMap.resize(mHeight);
	for (int i = 0; i < mHeight; ++i)
	{
		mDepthMap[i].resize(mWidth);
		
		//if (i > 250) continue;
		LOG(INFO) << "DepthCamera::Capture() is processing " << i << "th row.";

		for (int j = 0; j < mWidth; ++j)
		{
			
			Vector3f rayDir = constructRayDirection(i, j);

			aiVector3D rayDirection(rayDir[0], rayDir[1], rayDir[2]);
			aiVector3D rayStart(rayOrigin[0], rayOrigin[1], rayOrigin[2]);
			aiVector3D rayEnd = rayStart + maxDepth * rayDirection;
			for (int ithLayer = 0; ithLayer < maxNumDepthLayers; ++ithLayer)
			{
				aiVector3D intersectionPt;
				int triangleId;
				bool bHit = mKDTree.intersect(rayStart, rayEnd, intersectionPt, triangleId);
				if (!bHit) break;
				Vector3f intersection(intersectionPt[0], intersectionPt[1], intersectionPt[2]);
				float distance = (intersection - rayOrigin).norm();
				float depth = intersectionPt[2];
				mDepthMap[i][j].push_back(ExtendedDepthPixel(depth * 1000, faceNormals[triangleId]));
				rayStart = rayEnd - (maxDepth - distance - 0.005) * rayDirection;
			}
		}
	}
	ProcessMultiLayerDepthImage();
	fromMultiLayerToSinglelLayerDepthImage();

	string depthImageFileName = "results/depthFromMultiviewFromMesh.png";
	SaveDepthImage(depthImageFileName);
	string mutlilayerDepthImageFilename = "results/depthFromMultiviewFromMesh.data";
	SaveMultilayerDepthImage(mutlilayerDepthImageFilename);
	
}

void DepthCamera::Capture(const vector<Vector3f>& points, const vector<Vector3f>& normals)
{
	
	mDepthMap.resize(mHeight);
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
	for (int i = 0; i < mHeight; ++i)
	{
		mDepthMap[i].resize(mWidth);
	}
	int numPoints = static_cast<int>(points.size());
	int numPointsOnePercent = numPoints / 100;
	vector<Vector3f> pointsInCameraSpace;
	pointsInCameraSpace.resize(numPoints);
	Matrix4f poseInv = mPose.inverse();
	float fx = mFocalLength;
	float fy = mFocalLength;
	float cx = (mWidth - 1) / 2.f;
	float cy = (mHeight - 1) / 2.f;
	for (int i = 0; i < numPoints; ++i)
	{
		if (i % numPointsOnePercent == 0)
			LOG(INFO) << "DepthCamera::Capture() finished " << i / numPointsOnePercent << " percent.";
		pointsInCameraSpace[i] = (poseInv * Vector4f(points[i][0], points[i][1], points[i][2], 1)).head(3);
		double x = pointsInCameraSpace[i][0];
		double y = pointsInCameraSpace[i][1];
		double z = pointsInCameraSpace[i][2];
		double u = x * fx / z + cx;
		double v = y * fy / z + cy;
		int uIdx = static_cast<int>(u + 0.5);
		int vIdx = static_cast<int>(v + 0.5);
		if (uIdx < 0 || uIdx >= mWidth - 1 || vIdx < 0 || vIdx >= mHeight - 1)
			continue;
		mDepthMap[vIdx][uIdx].push_back(ExtendedDepthPixel(static_cast<float>(1000 * z), normals[i]));
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
	
	string depthImageFileName = "results/depthFromMultiview.png";
	string mutlilayerDepthImageFilename = "results/depthFromMultiview.data";
	
	SaveMultilayerDepthImage(mutlilayerDepthImageFilename);
	SaveDepthImage(depthImageFileName);
}

void DepthCamera::ProcessMultiLayerDepthImage()
{
	findMinMaxDepth();
	fromMultiLayerToSinglelLayerDepthImage();
}

void DepthCamera::GetPointCloud(vector<Vector3f>& points, vector<Vector3f>& normals)
{
	points.clear();
	normals.clear();
	
	float fx = mFocalLength;
	float fy = mFocalLength;
	float cx = (mWidth - 1) / 2.f;
	float cy = (mHeight - 1) / 2.f;

	for (int v = 0; v < mHeight; ++v)
	{
		for (int u = 0; u < mWidth; ++u)
		{
			int len = static_cast<int>(mDepthMap[v][u].size());

			for (int k = 0; k < len; ++k)
			{
				float d = mDepthMap[v][u][k].d;
				float z = d / 1000.f;

				float x = (u - cx) * z / fx;
				float y = (v - cy) * z / fy;

				Eigen::Vector4f w = mPose * Eigen::Vector4f(x, y, z, 1);

				points.push_back(w.head(3));
				normals.push_back(mDepthMap[v][u][k].n);
			}
			
		}
	}
}

void DepthCamera::SetData(const vector<vector<vector<ExtendedDepthPixel> > >& depthMap)
{
	mDepthMap = depthMap;
}

void DepthCamera::SetMask(const vector<vector<vector<int> > >& mask)
{
	mMask = mask;
}

void DepthCamera::ReadMultilayerDepthImage(const string& filename)
{
	LOG(INFO) << "Start DepthCamera::ReadMultilayerDepthImage()";
	ifstream inFile(filename.c_str(), ios::in | ios::binary);
	inFile.read((char*)&mWidth, sizeof(mWidth));
	inFile.read((char*)&mHeight, sizeof(mHeight));
	mDepthMap.resize(mHeight);
	for (int i = 0; i < mHeight; ++i)
	{
		mDepthMap[i].resize(mWidth);
		for (int j = 0; j < mWidth; ++j)
		{
			int len = 0;
			inFile.read((char*)&len, sizeof(int));
			if (len)
			{
				//mDepthMap[i][j].resize(len);
				for (int k = 0; k < len; ++k)
				{
					float depth, nx, ny, nz;
					
					inFile.read((char*)&depth, sizeof(float));
					inFile.read((char*)&nx, sizeof(float));
					inFile.read((char*)&ny, sizeof(float));
					inFile.read((char*)&nz, sizeof(float));
					if (depth > 0)
						mDepthMap[i][j].push_back(ExtendedDepthPixel(depth, Vector3f(nx, ny, nz)));
				}
			}
		}
	}
	LOG(INFO) << "End DepthCamera::ReadMultilayerDepthImage()";
}
void DepthCamera::SaveMultilayerDepthImage(const string& filename)
{
	LOG(INFO) << "Start DepthCamera::SaveMultilayerDepthImage()";
	ofstream outFile(filename.c_str(), ios::out | ios::binary);
	outFile.write((char*)&mWidth, sizeof(mWidth));
	outFile.write((char*)&mHeight, sizeof(mHeight));
	
	const vector<vector<vector<ExtendedDepthPixel> > >& image = mDepthMap;
	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			int len = static_cast<int>(image[i][j].size());
			outFile.write((char*)&len, sizeof(len));
			for (int k = 0; k < len; ++k)
			{
				outFile.write((char*)&(image[i][j][k].d), sizeof(float));
				outFile.write((char*)&(image[i][j][k].n[0]), sizeof(float));
				outFile.write((char*)&(image[i][j][k].n[1]), sizeof(float));
				outFile.write((char*)&(image[i][j][k].n[2]), sizeof(float));
			}
		}
	}
	LOG(INFO) << "End DepthCamera::SaveMultilayerDepthImage()";
}

void DepthCamera::SaveDepthThresholdingImage(const string& filename, int numThresholds)
{
	float stepSize = (mMaxDepth - mMinDepth) / numThresholds;
	cv::Mat1f thresholdImage;
	cv::Mat1i maskImage;
	thresholdImage.create(mHeight, mWidth);
	maskImage.create(mHeight, mWidth);
	for (int ithImage = 0; ithImage < numThresholds; ++ithImage)
	{
		float threshold = mMinDepth - EPSILON_FLOAT + stepSize * ithImage;
		for (int i = 0; i < mHeight; ++i)
		{
			for (int j = 0; j < mWidth; ++j)
			{
				maskImage.at<int>(i, j) = 0;
				if (mDepthMap[i][j].empty())
				{
					thresholdImage.at<float>(i, j) = 0;
				}
				else
				{
					int idToInsert = linearSearchInsertPos<ExtendedDepthPixel>(mDepthMap[i][j], ExtendedDepthPixel(threshold, Vector3f::Zero()));
					if (idToInsert == mDepthMap[i][j].size())
					{
						thresholdImage.at<float>(i, j) = 0;
					}
					else
					{
						thresholdImage.at<float>(i, j) = mDepthMap[i][j][idToInsert].d;
						if (!mMask.empty())
							maskImage.at<int>(i, j) = mMask[i][j][idToInsert];
					}
				}
			}
		}
		char newFileName[512];
		sprintf(newFileName, "%s%03d.png", filename.c_str(), ithImage);
		saveDepthImageVisualization(newFileName, &thresholdImage, &maskImage);
	}
}

void DepthCamera::SaveDepthOnionImage(const string& filename)
{

	int count = 1;
	//int count = 0;
	cv::Mat1f onionImage;
	onionImage.create(mHeight, mWidth);


	while (true)
	{
		int numValidPx = 0;
		for (int i = 0; i < mHeight; ++i)
		{
			for (int j = 0; j < mWidth; ++j)
			{
				int len = static_cast<int>(mDepthMap[i][j].size());
				if (mDepthMap[i][j].empty())
				{
					onionImage.at<float>(i, j) = 0;
				}
				//else if (len <= count)
				//{
				//	onionImage.at<float>(i, j) = 0;
				//}
				//else
				//{
				//	onionImage.at<float>(i, j) = mSimplifiedDepthMap[i][j][count];
				//	numValidPx++;
				//}
				else if (len <= count)
				{
					onionImage.at<float>(i, j) = mDepthMap[i][j][0].d;
				}
				else
				{
					onionImage.at<float>(i, j) = mDepthMap[i][j][len - count].d;
					numValidPx++;
				}
			}
		}
		if (!numValidPx) break;
		char newFileName[512];
		sprintf(newFileName, "%s%03dOnion.png", filename.c_str(), count);
		saveDepthImageVisualization(newFileName, &onionImage);
		count++;
	}
}


void DepthCamera::SimplifyMultilayerDepthImage()
{
	const float depthMergeThreshold = 20;
	
	vector<vector<vector<ExtendedDepthPixel> > > sDepthMap;
	sDepthMap.resize(mHeight);
	for (int i = 0; i < mHeight; ++i)
	{
		LOG(INFO) << "DepthCamera::simplifyMultilayerDepthImage() is processing " << i << "th row.";
		sDepthMap[i].resize(mWidth);
		for (int j = 0; j < mWidth; ++j)
		{

			if (mDepthMap[i][j].empty()) continue;
			int nDepthValues = static_cast<int>(mDepthMap[i][j].size());
			float depthCenter = mDepthMap[i][j][0].d;
			Vector3f normalCenter = mDepthMap[i][j][0].n;
			int count = 1;
			for (int k = 1; k < nDepthValues; ++k)
			{
				float currentDepth = mDepthMap[i][j][k].d;
				Vector3f currentNormal = mDepthMap[i][j][k].n;
				if (abs(depthCenter - currentDepth) < depthMergeThreshold)
				{
					depthCenter = (depthCenter * count + currentDepth) / (count + 1);
					normalCenter += currentNormal;
					count++;
				}
				else
				{
					normalCenter.normalize();
					sDepthMap[i][j].push_back(ExtendedDepthPixel(depthCenter, normalCenter));
					depthCenter = currentDepth;
					normalCenter = currentNormal;
					count = 1;
				}
			}
			normalCenter.normalize();
			sDepthMap[i][j].push_back(ExtendedDepthPixel(depthCenter, normalCenter));
		}
	}
	mDepthMap = sDepthMap;
	return;

	//const int numDepthSegments = 10;
	//vector<vector<vector<ExtendedDepthPixel> > > simplifiedDepthMap;
	//simplifiedDepthMap.resize(mHeight);
	//for (int i = 0; i < mHeight; ++i)
	//{
	//	simplifiedDepthMap[i].resize(mWidth);
	//	LOG(INFO) << "DepthCamera::simplifyMultilayerDepthImage() is processing " << i << "th row.";
	//	for (int j = 0; j < mWidth; ++j)
	//	{
	//		if (mDepthMap[i][j].empty()) continue;

	//		vector<float> simplifiedDepths;
	//		int depthListLen = static_cast<int>(mDepthMap[i][j].size());
	//		simplifiedDepths.resize(depthListLen);
	//		cv::Mat labels;
	//		
	//		for (int ithDepth = 0; ithDepth < depthListLen; ++ithDepth)
	//		{
	//			simplifiedDepths[ithDepth] = mDepthMap[i][j][ithDepth].d;
	//		}

	//		if (mDepthMap[i][j].size() > numDepthSegments)
	//		{

	//			cv::Mat depthData(simplifiedDepths, true);
	//			cv::Mat centers;
	//			cv::kmeans(depthData, numDepthSegments, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0), 1, cv::KMEANS_PP_CENTERS, centers);
	//			
	//			simplifiedDepths.clear();
	//			for (int ithCenter = 0; ithCenter < numDepthSegments; ++ithCenter)
	//				simplifiedDepths.push_back(centers.at<float>(ithCenter));
	//			sort(simplifiedDepths.begin(), simplifiedDepths.end());
	//		}
	//		else
	//		{
	//			labels.create(static_cast<int>(mDepthMap[i][j].size()), 1, CV_32S);
	//		}
	//		
	//		int nDepthValues = static_cast<int>(simplifiedDepths.size());
	//		vector<set<int> > groups;
	//		set<int> group;

	//		for (int k = 0; k < nDepthValues; ++k)
	//		{
	//			float currentDepth = simplifiedDepths[k];
	//			float nextDepth = k + 1 < nDepthValues ? simplifiedDepths[k + 1] : simplifiedDepths[k];
	//			if (abs(nextDepth - currentDepth) < depthMergeThreshold)
	//			{
	//				group.insert(group.end(), k);
	//				group.insert(group.end(), k + 1);
	//			}
	//			else
	//			{
	//				groups.push_back(group);
	//				group.clear();
	//			}
	//		}
	//		vector<ExtendedDepthPixel> mergedDepthPixels;
	//		mergePointsAndNormals(mDepthMap[i][j], labels, groups, mergedDepthPixels);
	//		simplifiedDepthMap[i][j] = mergedDepthPixels;

	//	}
	//}
	//mDepthMap = simplifiedDepthMap;

	//SaveMultilayerDepthImage("results/simplifiedDepthFromMultiview.data");
}

void DepthCamera::mergePointsAndNormals(const vector<ExtendedDepthPixel>& originalDepthPixel, const cv::Mat& labels, const vector<set<int> >& groups, vector<ExtendedDepthPixel>& mergedDepthPixel)
{
	int numGroups = static_cast<int>(groups.size());
	int numDepths = static_cast<int>(originalDepthPixel.size());
	mergedDepthPixel.resize(numGroups);
	for (int ithGroup = 0; ithGroup < numGroups; ++ithGroup)
	{
		int count = 0;
		float accumDepth = 0;
		Vector3f accumNormal = Vector3f::Zero();
		for (int jthPx = 0; jthPx < numDepths; ++jthPx)
		{
			int currentLabel = labels.at<int>(jthPx);
			if (groups[ithGroup].find(currentLabel) != groups[ithGroup].end())
			{
				count++;
				accumDepth += originalDepthPixel[jthPx].d;
				accumNormal += originalDepthPixel[jthPx].n;
			}
		}
		accumDepth /= count;
		accumNormal.normalize();
		mergedDepthPixel[ithGroup] = ExtendedDepthPixel(accumDepth, accumNormal);
	}
}

void DepthCamera::saveDepthImageVisualization(const string& filename, const cv::Mat1f* image, const cv::Mat1i* mask)
{
	cv::Mat visualizationImage(mHeight, mWidth, CV_8UC3);
	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			float normalizedDepth = 255 * ((image->at<float>(i, j) - mMinDepth) / (1.3f * (mMaxDepth - mMinDepth)) + 0.2f);
			uchar col = static_cast<uchar>(Clamp<float>(normalizedDepth, 0, 255));
			visualizationImage.at<cv::Vec3b>(i, j) = cv::Vec3b(col, col, col);
			if (mask && mask->at<int>(i, j) == 1)
				visualizationImage.at<cv::Vec3b>(i, j) = cv::Vec3b(200, 0, 200);
		}
	}
	imwrite(filename, visualizationImage);
}
void DepthCamera::findMinMaxDepth()
{
	mMinDepth = FLT_MAX;
	mMaxDepth = 0;
	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			if (!mDepthMap[i][j].empty())
			{
				if (mDepthMap[i][j].begin()->d < mMinDepth)
					mMinDepth = mDepthMap[i][j].begin()->d;
				if (mDepthMap[i][j].rbegin()->d > mMaxDepth)
					mMaxDepth = mDepthMap[i][j].rbegin()->d;
			}
		}
	}
}
void DepthCamera::fromMultiLayerToSinglelLayerDepthImage()
{
	mSensorMeasurement.create(mHeight, mWidth);
	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			if (mDepthMap[i][j].empty())
			{
				mSensorMeasurement.at<float>(i, j) = 0;
			}
			else
			{
				mSensorMeasurement.at<float>(i, j) = mDepthMap[i][j].begin()->d;
			}
		}
	}
}

Vector3f DepthCamera::constructRayDirection(int i, int j)
{
	Vector3f ret;

	float fx = mFocalLength;
	float fy = mFocalLength;
	float cx = (mWidth - 1) / 2.f;
	float cy = (mHeight - 1) / 2.f;

	ret[0] = (j - cx) / fx;
	ret[1] = (i - cy) / fy;
	ret[2] = 1;
	ret.normalize();

	//ret = mPose.block(0, 0, 3, 3) * ret;
	return ret;
}

void DepthCamera::constructDepthMap(const Vector3f& rayOrigin, const Vector3f& rayDir, const vector<Vector3f>& points, const vector<Vector3f>& normals, vector<float>& depthMap)
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

void DepthCamera::SaveDepthImage(const string& filename)
{
	int numRows = mHeight;
	int numCols = mWidth;
	cv::Mat1w imageToWrite;
	imageToWrite.create(numRows, numCols);
	for (int i = 0; i < numRows; ++i)
	{
		for (int j = 0; j < numCols; ++j)
		{
			imageToWrite.at<ushort>(i, j) = static_cast<ushort>(mSensorMeasurement.at<float>(i, j));
		}
	}
	cv::imwrite(filename, imageToWrite);
}

// assuming self is a noisy depth camera view but represent the truth, rhs is from the view of Poisson constructed mesh.
void DepthCamera::Compare(const DepthCamera& rhs, vector<vector<vector<ExtendedDepthPixel> > >& mergedDepthMap, vector<vector<vector<int> > >& mask)
{
	const int MASK_UNKNOWN = 1;
	const int MASK_KNOWN = 0;
	const float pointMeshMergingThreshold = 50;


	mergedDepthMap.clear();
	mergedDepthMap.resize(mHeight);

	mask.clear();
	mask.resize(mHeight);
	for (int i = 0; i < mHeight; ++i)
	{
		mergedDepthMap[i].resize(mWidth);
		mask[i].resize(mWidth);
		
		for (int j = 0; j < mWidth; ++j)
		{
			//check whether the depth samples from mesh are presented as points
			if (mDepthMap[i][j].empty()) continue;
			vector<ExtendedDepthPixel> processingDepthMapFromMesh;
			int selfSize = static_cast<int>(mDepthMap[i][j].size());
			int rhsSize = static_cast<int>(rhs.mDepthMap[i][j].size());
			for (int ithDepthSample = 0; ithDepthSample < rhsSize; ++ithDepthSample)
			{
				int insertPos = linearSearchInsertPos(mDepthMap[i][j], rhs.mDepthMap[i][j][ithDepthSample]);
				if (insertPos == 0)
					continue;
				else if (insertPos == selfSize && abs(rhs.mDepthMap[i][j][ithDepthSample].d - mDepthMap[i][j][insertPos - 1].d) < pointMeshMergingThreshold)
					continue;
				else if (abs(rhs.mDepthMap[i][j][ithDepthSample].d - mDepthMap[i][j][insertPos].d) < pointMeshMergingThreshold || abs(rhs.mDepthMap[i][j][ithDepthSample].d - mDepthMap[i][j][insertPos - 1].d) < pointMeshMergingThreshold)
					continue;
				else
					processingDepthMapFromMesh.push_back(rhs.mDepthMap[i][j][ithDepthSample]);
			}
			

			//merge source two list of depth samples
			
			rhsSize = static_cast<int>(processingDepthMapFromMesh.size());
			for (int ithDepthSample = 0; ithDepthSample < rhsSize; ++ithDepthSample)
			{
				processingDepthMapFromMesh[ithDepthSample].n = Vector3f::Zero(); // make it color so we can see it clearly in mesh lab.
			}
			int selfIdx = 0;
			int rhsIdx = 0;
			while (selfIdx < selfSize && rhsIdx < rhsSize)
			{
				if (mDepthMap[i][j][selfIdx] <= processingDepthMapFromMesh[rhsIdx])
				{
					mergedDepthMap[i][j].push_back(mDepthMap[i][j][selfIdx]);
					mask[i][j].push_back(MASK_KNOWN);
					selfIdx++;
				}
				else
				{
					mergedDepthMap[i][j].push_back(processingDepthMapFromMesh[rhsIdx]);
					mask[i][j].push_back(MASK_UNKNOWN);
					rhsIdx++;
				}
			}
			if (selfIdx >= selfSize - 1)
			{
				mergedDepthMap[i][j].insert(mergedDepthMap[i][j].end(), processingDepthMapFromMesh.begin() + rhsIdx, processingDepthMapFromMesh.end());
				mask[i][j].insert(mask[i][j].end(), rhsSize - rhsIdx, MASK_UNKNOWN);

			}
			if (rhsIdx >= rhsSize - 1)
			{
				mergedDepthMap[i][j].insert(mergedDepthMap[i][j].end(), mDepthMap[i][j].begin() + selfIdx, mDepthMap[i][j].end());
				mask[i][j].insert(mask[i][j].end(), selfSize - selfIdx, MASK_KNOWN);
			}
		}
	}
	
}