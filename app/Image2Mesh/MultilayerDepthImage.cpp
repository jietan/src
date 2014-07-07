#include "MultilayerDepthImage.h"
#include <glog/logging.h>
#include <fstream>
#include "utility/utility.h"
#include "utility/mathlib.h"


MultilayerDepthImage::MultilayerDepthImage() : mMinDepth(0),
mMaxDepth(0)
{

}
MultilayerDepthImage::~MultilayerDepthImage()
{

}


void MultilayerDepthImage::Process()
{
	findMinMaxDepth();
	findNumLayers();
}


void MultilayerDepthImage::Read(const string& filename)
{
	LOG(INFO) << "Start DepthCamera::ReadMultilayerDepthImage()";
	ifstream inFile(filename.c_str(), ios::in | ios::binary);
	inFile.read((char*)&mWidth, sizeof(mWidth));
	inFile.read((char*)&mHeight, sizeof(mHeight));
	mData.resize(mHeight);
	for (int i = 0; i < mHeight; ++i)
	{
		mData[i].resize(mWidth);
		for (int j = 0; j < mWidth; ++j)
		{
			int len = 0;
			inFile.read((char*)&len, sizeof(int));
			if (len)
			{
				//mData[i][j].resize(len);
				for (int k = 0; k < len; ++k)
				{
					float depth, nx, ny, nz;

					inFile.read((char*)&depth, sizeof(float));
					inFile.read((char*)&nx, sizeof(float));
					inFile.read((char*)&ny, sizeof(float));
					inFile.read((char*)&nz, sizeof(float));
					if (depth > 0)
						mData[i][j].push_back(ExtendedDepthPixel(depth, Eigen::Vector3f(nx, ny, nz)));
				}
			}
		}
	}
	LOG(INFO) << "End DepthCamera::ReadMultilayerDepthImage()";
}
void MultilayerDepthImage::Save(const string& filename)
{
	LOG(INFO) << "Start DepthCamera::SaveMultilayerDepthImage()";
	ofstream outFile(filename.c_str(), ios::out | ios::binary);
	outFile.write((char*)&mWidth, sizeof(mWidth));
	outFile.write((char*)&mHeight, sizeof(mHeight));

	const vector<vector<vector<ExtendedDepthPixel> > >& image = mData;
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


void MultilayerDepthImage::SaveDepthImage(const string& filename)
{
	int numRows = mHeight;
	int numCols = mWidth;
	cv::Mat1w imageToWrite;
	imageToWrite.create(numRows, numCols);
	for (int i = 0; i < numRows; ++i)
	{
		for (int j = 0; j < numCols; ++j)
		{
			if (mData[i][j].empty())
				imageToWrite.at<ushort>(i, j) = 0;
			else
				imageToWrite.at<ushort>(i, j) = static_cast<ushort>(mData[i][j][0].d);
		}
	}
	cv::imwrite(filename, imageToWrite);
}

void MultilayerDepthImage::SaveDepthThresholdingImage(const string& filename, int numThresholds, const MultilayerMaskImage* mask, float* min, float* max)
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
				if (mData[i][j].empty())
				{
					thresholdImage.at<float>(i, j) = 0;
				}
				else
				{
					int idToInsert = linearSearchInsertPos<ExtendedDepthPixel>(mData[i][j], ExtendedDepthPixel(threshold, Eigen::Vector3f::Zero()));
					if (idToInsert == mData[i][j].size())
					{
						thresholdImage.at<float>(i, j) = 0;
					}
					else
					{
						thresholdImage.at<float>(i, j) = mData[i][j][idToInsert].d;
						if (mask)
							maskImage.at<int>(i, j) = (*mask)[i][j][idToInsert];
					}
				}
			}
		}
		char newFileName[512];
		sprintf(newFileName, "%s%03d.png", filename.c_str(), ithImage);
		saveDepthImageVisualization(newFileName, &thresholdImage, mask?&maskImage:NULL, min, max);
	}
}

void MultilayerDepthImage::SaveDepthOnionImage(const string& filename, const MultilayerMaskImage* mask, float* min, float* max)
{

	//int count = 1;
	int count = 0;
	cv::Mat1f onionImage;
	onionImage.create(mHeight, mWidth);


	while (true)
	{
		int numValidPx = 0;
		cv::Mat1i maskImage = cv::Mat::zeros(mHeight, mWidth, CV_32S);
		for (int i = 0; i < mHeight; ++i)
		{
			for (int j = 0; j < mWidth; ++j)
			{
				int len = static_cast<int>(mData[i][j].size());
				if (mData[i][j].empty())
				{
					onionImage.at<float>(i, j) = 0;
				}
				else if (len <= count)
				{
					onionImage.at<float>(i, j) = 0;// mData[i][j][len - 1].d;
				}
				else
				{
					onionImage.at<float>(i, j) = mData[i][j][count].d;
					if (mask)
						maskImage.at<int>(i, j) = (*mask)[i][j][count];
					numValidPx++;
				}
				//else if (len <= count)
				//{
				//	onionImage.at<float>(i, j) = mData[i][j][0].d;
				//	if (!mMask.empty())
				//		maskImage.at<int>(i, j) = mMask[i][j][0];
				//}
				//else
				//{
				//	onionImage.at<float>(i, j) = mData[i][j][len - count].d;
				//	if (!mMask.empty())
				//		maskImage.at<int>(i, j) = mMask[i][j][len - count];
				//	numValidPx++;
				//}
			}
		}
		if (!numValidPx) break;
		char newFileName[512];
		sprintf(newFileName, "%sOnion%03d.png", filename.c_str(), count);
		saveDepthImageVisualization(newFileName, &onionImage, mask?&maskImage:NULL, min, max);
		count++;
	}
}


void MultilayerDepthImage::Simplify()
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

			if (mData[i][j].empty()) continue;
			int nDepthValues = static_cast<int>(mData[i][j].size());
			float depthCenter = mData[i][j][0].d;
			Eigen::Vector3f normalCenter = mData[i][j][0].n;
			int count = 1;
			for (int k = 1; k < nDepthValues; ++k)
			{
				float currentDepth = mData[i][j][k].d;
				Eigen::Vector3f currentNormal = mData[i][j][k].n;
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
	mData = sDepthMap;
}



void MultilayerDepthImage::saveDepthImageVisualization(const string& filename, const cv::Mat1f* image, const cv::Mat1i* mask, float* min, float* max)
{
	cv::Mat visualizationImage(mHeight, mWidth, CV_8UC3);
	float minValue = mMinDepth;
	float maxValue = mMaxDepth;
	if (min)
		minValue = *min;
	if (max)
		maxValue = *max;
	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			float normalizedDepth = 255 * ((image->at<float>(i, j) - minValue) / (1.3f * (maxValue - minValue)) + 0.2f);
			uchar col = static_cast<uchar>(Clamp<float>(normalizedDepth, 0, 255));
			visualizationImage.at<cv::Vec3b>(i, j) = cv::Vec3b(col, col, col);
			if (mask && mask->at<int>(i, j) == 1)
				visualizationImage.at<cv::Vec3b>(i, j) = cv::Vec3b(200, 0, 200);
		}
	}
	imwrite(filename, visualizationImage);
}

void MultilayerDepthImage::findMinMaxDepth()
{
	mMinDepth = FLT_MAX;
	mMaxDepth = 0;
	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			if (!mData[i][j].empty())
			{
				if (mData[i][j].begin()->d < mMinDepth)
					mMinDepth = mData[i][j].begin()->d;
				if (mData[i][j].rbegin()->d > mMaxDepth)
					mMaxDepth = mData[i][j].rbegin()->d;
			}
		}
	}
}


MultilayerDepthImageWithMask::~MultilayerDepthImageWithMask()
{

}
void MultilayerDepthImageWithMask::Simplify()
{
	const float depthMergeThreshold = 20;

	vector<vector<vector<ExtendedDepthPixelWithMask> > > sDepthMap;
	sDepthMap.resize(mHeight);
	for (int i = 0; i < mHeight; ++i)
	{
		LOG(INFO) << "DepthCamera::simplifyMultilayerDepthImage() is processing " << i << "th row.";
		sDepthMap[i].resize(mWidth);
		for (int j = 0; j < mWidth; ++j)
		{

			if (mData[i][j].empty()) continue;
			int nDepthValues = static_cast<int>(mData[i][j].size());
			float depthCenter = mData[i][j][0].d;
			int maskCenter = mData[i][j][0].m;
			Eigen::Vector3f normalCenter = mData[i][j][0].n;
			int count = 1;
			for (int k = 1; k < nDepthValues; ++k)
			{
				float currentDepth = mData[i][j][k].d;
				Eigen::Vector3f currentNormal = mData[i][j][k].n;
				int currentMask = mData[i][j][k].m;
				if (abs(depthCenter - currentDepth) < depthMergeThreshold && maskCenter == currentMask)
				{
					depthCenter = (depthCenter * count + currentDepth) / (count + 1);
					normalCenter += currentNormal;
					count++;
				}
				else
				{
					normalCenter.normalize();
					sDepthMap[i][j].push_back(ExtendedDepthPixelWithMask(depthCenter, normalCenter, maskCenter));
					depthCenter = currentDepth;
					normalCenter = currentNormal;
					maskCenter = currentMask;
					count = 1;
				}
			}
			normalCenter.normalize();
			sDepthMap[i][j].push_back(ExtendedDepthPixelWithMask(depthCenter, normalCenter, maskCenter));
		}
	}
	mData = sDepthMap;
}