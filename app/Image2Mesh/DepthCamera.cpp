#include "DepthCamera.h"
#include <algorithm>
using namespace std;
#include <glog/logging.h>
using namespace google;
#include "utility/mathlib.h"
#include "utility/utility.h"


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


void DepthCamera::Capture(const vector<Vector3f>& points, const vector<Vector3f>& normals, const string& depthImageFileName)
{
	mSensorMeasurement.create(mHeight, mWidth);
	Vector3f rayOrigin = mPose.col(3).head(3);
	mDepthMap.resize(mHeight);
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
	string mutlilayerDepthImageFilename = depthImageFileName;
	mutlilayerDepthImageFilename += ".data";
	SaveMultilayerDepthImage(mutlilayerDepthImageFilename);
	SaveDepthImage(depthImageFileName);
}

void DepthCamera::ProcessMultiLayerDepthImage()
{
	findMinMaxDepth();
	fromMultiLayerToSinglelLayerDepthImage();
}

void DepthCamera::GetSimplifiedPointCloud(vector<Vector3f>& points, vector<Vector3f>& normals)
{

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
					//inFile.read((char*)&nx, sizeof(float));
					//inFile.read((char*)&ny, sizeof(float));
					//inFile.read((char*)&nz, sizeof(float));
					if (depth > 0 && depth < 2000)
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
	
	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			int len = static_cast<int>(mDepthMap[i][j].size());
			outFile.write((char*)&len, sizeof(len));
			for (int k = 0; k < len; ++k)
			{
				outFile.write((char*)&(mDepthMap[i][j][k].d), sizeof(float));
				outFile.write((char*)&(mDepthMap[i][j][k].n[0]), sizeof(float));
				outFile.write((char*)&(mDepthMap[i][j][k].n[1]), sizeof(float));
				outFile.write((char*)&(mDepthMap[i][j][k].n[2]), sizeof(float));
			}
		}
	}
	LOG(INFO) << "End DepthCamera::SaveMultilayerDepthImage()";
}

void DepthCamera::SaveDepthThresholdingImage(const string& filename, int numThresholds)
{
	float stepSize = (mMaxDepth - mMinDepth) / numThresholds;
	cv::Mat1f thresholdImage;
	thresholdImage.create(mHeight, mWidth);
	for (int ithImage = 0; ithImage < numThresholds; ++ithImage)
	{
		float threshold = mMinDepth - EPSILON_FLOAT + stepSize * ithImage;
		for (int i = 0; i < mHeight; ++i)
		{
			for (int j = 0; j < mWidth; ++j)
			{
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
					}
				}
			}
		}
		char newFileName[512];
		sprintf(newFileName, "%s%03d.png", filename.c_str(), ithImage);
		saveDepthImageVisualization(newFileName, thresholdImage);
	}
}

void DepthCamera::SaveDepthOnionImage(const string& filename)
{
	if (mSimplifiedDepthMap.empty())
		simplifyMultilayerDepthImage();
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
				int len = static_cast<int>(mSimplifiedDepthMap[i][j].size());
				if (mSimplifiedDepthMap[i][j].empty())
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
					onionImage.at<float>(i, j) = mSimplifiedDepthMap[i][j][0].d;
				}
				else
				{
					onionImage.at<float>(i, j) = mSimplifiedDepthMap[i][j][len - count].d;
					numValidPx++;
				}
			}
		}
		if (!numValidPx) break;
		char newFileName[512];
		sprintf(newFileName, "%s%03dOnion.png", filename.c_str(), count);
		saveDepthImageVisualization(newFileName, onionImage);
		count++;
	}
}


void DepthCamera::simplifyMultilayerDepthImage()
{
	const float depthMergeThreshold = 10;
	
	const int numDepthSegments = 10;
	mSimplifiedDepthMap.resize(mHeight);
	for (int i = 0; i < mHeight; ++i)
	{
		mSimplifiedDepthMap[i].resize(mWidth);
		LOG(INFO) << "DepthCamera::simplifyMultilayerDepthImage() is processing " << i << "th row.";
		for (int j = 0; j < mWidth; ++j)
		{
			if (mDepthMap[i][j].empty()) continue;
			vector<ExtendedDepthPixel> simplifiedDepths;
			if (mDepthMap[i][j].size() < numDepthSegments)
			{
				simplifiedDepths = mDepthMap[i][j];
			}
			else
			{
				vector<float> depthDataVector;
				int depthListLen = static_cast<int>(mDepthMap[i][j].size());
				depthDataVector.resize(depthListLen);
				for (int ithDepth = 0; ithDepth < depthListLen; ++ithDepth)
				{
					depthDataVector[ithDepth] = mDepthMap[i][j][ithDepth].d;
				}

				cv::Mat depthData(depthDataVector, true);
				cv::Mat labels;
				cv::Mat centers;
				cv::kmeans(depthData, numDepthSegments, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0), 1, cv::KMEANS_PP_CENTERS, centers);

				for (int ithCenter = 0; ithCenter < numDepthSegments; ++ithCenter)
					simplifiedDepths.push_back(ExtendedDepthPixel(centers.at<float>(ithCenter), Vector3f::Zero()));
				sort(simplifiedDepths.begin(), simplifiedDepths.end());
			}
			float depthCenter = simplifiedDepths[0].d;
			int count = 1;
			int nDepthValues = static_cast<int>(simplifiedDepths.size());
			for (int k = 1; k < nDepthValues; ++k)
			{
				float currentDepth = simplifiedDepths[k].d;
				if (abs(depthCenter - currentDepth) < depthMergeThreshold)
				{
					depthCenter = (depthCenter * count + currentDepth) / (count + 1);
					count++;
				}
				else
				{
					mSimplifiedDepthMap[i][j].push_back(ExtendedDepthPixel(depthCenter, Vector3f::Zero()));
					depthCenter = currentDepth;
					count = 1;
				}
			}
			mSimplifiedDepthMap[i][j].push_back(ExtendedDepthPixel(depthCenter, Vector3f::Zero()));
			//depthData.create(mDepthMap[i][j].size(), 1);

			//int nDepthValues = static_cast<int>(mDepthMap[i][j].size());
			//float depthCenter = mDepthMap[i][j][0];
			//int count = 1;
			//for (int k = 1; k < nDepthValues; ++k)
			//{
			//	float currentDepth = mDepthMap[i][j][k];
			//	if (abs(depthCenter - currentDepth) < depthMergeThreshold)
			//	{
			//		depthCenter = (depthCenter * count + currentDepth) / (count + 1);
			//		count++;
			//	}
			//	else
			//	{
			//		mSimplifiedDepthMap[i][j].push_back(depthCenter);
			//		depthCenter = currentDepth;
			//		count = 1;
			//	}
			//}

		}
	}
}

void DepthCamera::saveDepthImageVisualization(const string& filename, const cv::Mat1f image)
{
	cv::Mat visualizationImage(mHeight, mWidth, CV_8U);
	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			float normalizedDepth = 255 * ((image.at<float>(i, j) - mMinDepth) / (1.3f * (mMaxDepth - mMinDepth)) + 0.2f);
			visualizationImage.at<uchar>(i, j) = static_cast<uchar>(Clamp<float>(normalizedDepth, 0, 255));
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

	ret[0] = (j - cy) / fy;
	ret[1] = (i - cx) / fx;
	ret[2] = 1;
	ret.normalize();

	ret = mPose.block(0, 0, 3, 3) * ret;
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
