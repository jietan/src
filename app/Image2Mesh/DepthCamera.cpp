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
void DepthCamera::SetIntrinsicParameters(int numPxWidth, int numPxHeight, double focalLenth)
{
	mWidth = numPxWidth;
	mHeight = numPxHeight;
	mFocalLength = focalLenth;
}
void DepthCamera::SetExtrinsicParameters(const Matrix4d& pose)
{
	mPose = pose;
}


void DepthCamera::Capture(const vector<Vector3d>& points, const vector<Vector3d>& normals, const string& depthImageFileName)
{
	mSensorMeasurement.create(mHeight, mWidth);
	Vector3d rayOrigin = mPose.col(3).head(3);
	mDepthMap.resize(mHeight);

	//for (int i = 0; i < mHeight; ++i)
	//{
	//	mDepthMap[i].resize(mWidth);
	//	
	//	for (int j = 0; j < mWidth; ++j)
	//	{
	//		LOG(INFO) << "DepthCamera::Capture() is processing " << i << "th row, " << j << "th column.";

	//		Vector3d rayDir = constructRayDirection(i, j);
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
	vector<Vector3d> pointsInCameraSpace;
	pointsInCameraSpace.resize(numPoints);
	Matrix4d poseInv = mPose.inverse();
	double fx = mFocalLength;
	double fy = mFocalLength;
	double cx = (mWidth - 1) / 2.0;
	double cy = (mHeight - 1) / 2.0;
	for (int i = 0; i < numPoints; ++i)
	{
		if (i % numPointsOnePercent == 0)
			LOG(INFO) << "DepthCamera::Capture() finished " << i / numPointsOnePercent << " percent.";
		pointsInCameraSpace[i] = (poseInv * Vector4d(points[i][0], points[i][1], points[i][2], 1)).head(3);
		double x = pointsInCameraSpace[i][0];
		double y = pointsInCameraSpace[i][1];
		double z = pointsInCameraSpace[i][2];
		double u = x * fx / z + cx;
		double v = y * fy / z + cy;
		int uIdx = static_cast<int>(u + 0.5);
		int vIdx = static_cast<int>(v + 0.5);
		if (uIdx < 0 || uIdx >= mWidth - 1 || vIdx < 0 || vIdx >= mHeight - 1)
			continue;
		mDepthMap[vIdx][uIdx].push_back(static_cast<float>(1000 * z));
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
					float depth;
					inFile.read((char*)&depth, sizeof(float));
					if (depth > 0)
						mDepthMap[i][j].push_back(depth);
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
			if (len)
				outFile.write((char*)&(mDepthMap[i][j][0]), len * sizeof(float));
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
		double threshold = mMinDepth - EPSILON_FLOAT + stepSize * ithImage;
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
					int idToInsert = linearSearchInsertPos<float>(mDepthMap[i][j], threshold);
					if (idToInsert == mDepthMap[i][j].size())
					{
						thresholdImage.at<float>(i, j) = 0;
					}
					else
					{
						thresholdImage.at<float>(i, j) = mDepthMap[i][j][idToInsert];
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
	//int count = 1;
	int count = 0;
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
				else if (len <= count)
				{
					onionImage.at<float>(i, j) = 0;
				}
				else
				{
					onionImage.at<float>(i, j) = mSimplifiedDepthMap[i][j][count];
					numValidPx++;
				}
				//else if (len <= count)
				//{
				//	onionImage.at<float>(i, j) = mSimplifiedDepthMap[i][j][0];
				//}
				//else
				//{
				//	onionImage.at<float>(i, j) = mSimplifiedDepthMap[i][j][len - count];
				//	numValidPx++;
				//}
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
	const float depthMergeThreshold = 50;
	
	const int numDepthSegments = 10;
	mSimplifiedDepthMap.resize(mHeight);
	for (int i = 0; i < mHeight; ++i)
	{
		mSimplifiedDepthMap[i].resize(mWidth);
		LOG(INFO) << "DepthCamera::simplifyMultilayerDepthImage() is processing " << i << "th row.";
		for (int j = 0; j < mWidth; ++j)
		{
			if (mDepthMap[i][j].empty()) continue;
			vector<float> simplifiedDepths;
			if (mDepthMap[i][j].size() < numDepthSegments)
			{
				simplifiedDepths = mDepthMap[i][j];
			}
			else
			{
				cv::Mat depthData(mDepthMap[i][j], true);
				cv::Mat labels;
				cv::Mat centers;
				cv::kmeans(depthData, numDepthSegments, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0), 1, cv::KMEANS_PP_CENTERS, centers);

				for (int ithCenter = 0; ithCenter < numDepthSegments; ++ithCenter)
					simplifiedDepths.push_back(centers.at<float>(ithCenter));
				sort(simplifiedDepths.begin(), simplifiedDepths.end());
			}
			float depthCenter = simplifiedDepths[0];
			int count = 1;
			int nDepthValues = static_cast<int>(simplifiedDepths.size());
			for (int k = 1; k < nDepthValues; ++k)
			{
				float currentDepth = simplifiedDepths[k];
				if (abs(depthCenter - currentDepth) < depthMergeThreshold)
				{
					depthCenter = (depthCenter * count + currentDepth) / (count + 1);
					count++;
				}
				else
				{
					mSimplifiedDepthMap[i][j].push_back(depthCenter);
					depthCenter = currentDepth;
					count = 1;
				}
			}
			mSimplifiedDepthMap[i][j].push_back(depthCenter);
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
			float normalizedDepth = 255 * ((image.at<float>(i, j) - mMinDepth) / (1.3 * (mMaxDepth - mMinDepth)) + 0.2);
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
				if (mDepthMap[i][j][0] < mMinDepth)
					mMinDepth = mDepthMap[i][j][0];
				if (*(mDepthMap[i][j].end() - 1) > mMaxDepth)
					mMaxDepth = *(mDepthMap[i][j].end() - 1);
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
				mSensorMeasurement.at<float>(i, j) = *(mDepthMap[i][j].begin());
			}
		}
	}
}

Vector3d DepthCamera::constructRayDirection(int i, int j)
{
	Vector3d ret;

	double fx = mFocalLength;
	double fy = mFocalLength;
	double cx = (mWidth - 1) / 2.0;
	double cy = (mHeight - 1) / 2.0;

	ret[0] = (j - cy) / fy;
	ret[1] = (i - cx) / fx;
	ret[2] = 1;
	ret.normalize();

	ret = mPose.block(0, 0, 3, 3) * ret;
	return ret;
}

void DepthCamera::constructDepthMap(const Vector3d& rayOrigin, const Vector3d& rayDir, const vector<Vector3d>& points, const vector<Vector3d>& normals, vector<float>& depthMap)
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
