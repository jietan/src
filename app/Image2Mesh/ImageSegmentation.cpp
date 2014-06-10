#include "ImageSegmentation.h"
#include "utility/mathlib.h"
#include <queue>
#include <algorithm>

unsigned char SegmentColors[MAX_NUM_SEGMENTS][3] = {
		{ 0, 0, 0 },
		{ 0, 0, 255 },
		{ 0, 255, 0 },
		{ 0, 255, 255 },
		{ 255, 0, 0 },
		{ 255, 0, 255 },
		{ 255, 255, 0 },
		{ 255, 255, 255 },
		{ 128, 128, 128 },
		{ 128, 0, 128 },
		{ 128, 0, 0 },
		{ 0, 0, 128 },
};

void ImageSegmentation::depthImageToNormalsFeaturePreserving(const cv::Mat& depthImg, double curvatureTheshold)
{
	int numRows = depthImg.rows;
	int numCols = depthImg.cols;
	mNormalImg.create(numRows, numCols);
	int numTotalPixels = numRows * numCols;

	Vector3d tangentialAxis1;
	Vector3d tangentialAxis2;

	for (int i = 0; i < numTotalPixels; ++i)
	{
		int current = i;
		float dCurrent = getDepth(depthImg, current);

		int u, v;
		indexTo2d(current, v, u, numRows, numCols);

		int left = leftNeighbor(current, numRows, numCols);
		int right = rightNeighbor(current, numRows, numCols);
		float dLeft = getDepth(depthImg, left);
		float dRight = getDepth(depthImg, right);
		if (abs(2 * dCurrent - dLeft - dRight) > curvatureTheshold)
		{
			double diff = abs(dRight - dCurrent) > abs(dCurrent - dLeft) ? dCurrent - dLeft : dRight - dCurrent;
			tangentialAxis1 = Vector3d(1, 0, diff);
		}
		else
		{
			tangentialAxis1 = Vector3d(2, 0, dRight - dLeft);
		}

		int upper = upperNeighbor(current, numRows, numCols);
		int lower = lowerNeighbor(current, numRows, numCols);
		float dUpper = getDepth(depthImg, upper);
		float dLower = getDepth(depthImg, lower);
		if (abs(2 * dCurrent - dUpper - dLower) > curvatureTheshold)
		{
			double diff = abs(dUpper - dCurrent) > abs(dCurrent - dLower) ? dLower - dCurrent : dCurrent - dUpper;
			tangentialAxis2 = Vector3d(0, 1, diff);
		}
		else
		{
			tangentialAxis2 = Vector3d(0, 2, dLower - dUpper);
		}
		Vector3d normal = tangentialAxis1.cross(tangentialAxis2);
		normal = -normal.normalized();
		mNormalImg.at<cv::Vec3d>(v, u) = cv::Vec3d(normal[0], normal[1], normal[2]);
	}
}

void ImageSegmentation::depthImageToNormals(const cv::Mat& depthImg)
{
	mNormalImg.create(depthImg.rows, depthImg.cols);

	for (int v = 0; v < depthImg.rows; ++v)
	{
		for (int u = 0; u < depthImg.cols; ++u)
		{
			float d = depthImg.at<float>(v, u);
			double z = d / 1000.0;

			Vector3d tangentialAxis1;
			Vector3d tangentialAxis2;

			if (v == 0)
			{
				double dYPlus = depthImg.at<float>(v + 1, u);
				double dYMinus = depthImg.at<float>(v, u);
				tangentialAxis2 = Vector3d(0, 1, dYPlus - dYMinus);
			}
			else if (v == depthImg.rows - 1)
			{
				double dYPlus = depthImg.at<float>(v, u);
				double dYMinus = depthImg.at<float>(v - 1, u);
				tangentialAxis2 = Vector3d(0, 1, dYPlus - dYMinus);
			}
			else
			{
				double dYPlus = depthImg.at<float>(v + 1, u);
				double dYMinus = depthImg.at<float>(v - 1, u);
				tangentialAxis2 = Vector3d(0, 2, dYPlus - dYMinus);
			}
			if (u == 0)
			{
				double dXPlus = depthImg.at<float>(v, u + 1);
				double dXMinus = depthImg.at<float>(v, u);
				tangentialAxis1 = Vector3d(1, 0, dXPlus - dXMinus);
			}
			else if (u == depthImg.cols - 1)
			{
				double dXPlus = depthImg.at<float>(v, u);
				double dXMinus = depthImg.at<float>(v, u - 1);
				tangentialAxis1 = Vector3d(1, 0, dXPlus - dXMinus);
			}
			else
			{
				double dXPlus = depthImg.at<float>(v, u + 1);
				double dXMinus = depthImg.at<float>(v, u - 1);
				tangentialAxis1 = Vector3d(2, 0, dXPlus - dXMinus);
			}


			Vector3d normal = tangentialAxis1.cross(tangentialAxis2);
			normal = -normal.normalized();
			mNormalImg.at<cv::Vec3d>(v, u) = cv::Vec3d(normal[0], normal[1], normal[2]);
		}
	}
}

ImageSegmentation::ImageSegmentation()
{

}
ImageSegmentation::~ImageSegmentation()
{

}
const Segmentation& ImageSegmentation::Segment(const cv::Mat& depthImg, double depthThreshold, double angleThreshold)
{
	int numRows = depthImg.rows;
	int numCols = depthImg.cols;
	int numTotalPixels = numRows * numCols;
	
	Segmentation& ret = mSegmentation;
	ret.mSegmentedImg = MatrixXi::Constant(numRows, numCols, 0);
	depthImageToNormalsFeaturePreserving(depthImg, 1);
	//depthImageToNormals(depthImg);
	computeBoundaryImages(depthImg, depthThreshold, angleThreshold);
	
	set<int> unsegmentedPixelIdx;
	
	for (int i = 0; i < numTotalPixels; ++i)
		unsegmentedPixelIdx.insert(unsegmentedPixelIdx.end(), i);

	int currentSegmentationId = 0;
	while (!unsegmentedPixelIdx.empty())
	{
		int seed = *(unsegmentedPixelIdx.begin());
		if (isBoundary(seed))
		{
			unsegmentedPixelIdx.erase(seed);
			continue;
		}
		expand(depthImg, seed, numRows, numCols, currentSegmentationId, depthThreshold, angleThreshold, unsegmentedPixelIdx, ret);
		currentSegmentationId++;
	}
	return ret;
}
void ImageSegmentation::SaveSegmentedImage(const string& filename)
{
	int numRows = mNormalImg.rows;
	int numCols = mNormalImg.cols;

	cv::Mat3b segmentImage;
	segmentImage.create(numRows, numCols);
	for (int i = 0; i < numRows; ++i)
	{
		for (int j = 0; j < numCols; ++j)
		{
			int segmentId = mSegmentation.mSegmentedImg(i, j);
			if (segmentId)
				segmentId = segmentId % (MAX_NUM_SEGMENTS - 1) + 1;
			segmentImage.at<cv::Vec3b>(i, j) = cv::Vec3b(SegmentColors[segmentId]);
		}
	}
	imwrite(filename, segmentImage);

	cv::Mat3b normalImg;
	normalImg.create(numRows, numCols);
	for (int i = 0; i < numRows; ++i)
	{
		for (int j = 0; j < numCols; ++j)
		{
			normalImg.at<cv::Vec3b>(i, j)(0) = static_cast<uchar>(128 * mNormalImg.at<cv::Vec3d>(i, j)(0) + 128);
			normalImg.at<cv::Vec3b>(i, j)(1) = static_cast<uchar>(128 * mNormalImg.at<cv::Vec3d>(i, j)(1) + 128);
			normalImg.at<cv::Vec3b>(i, j)(2) = static_cast<uchar>(128 * mNormalImg.at<cv::Vec3d>(i, j)(2) + 128);
		}
	}
	string depthFileName = filename;
	depthFileName += "normal.png";
	imwrite(depthFileName, normalImg);

	string depthBoundaryFileName = filename;
	depthBoundaryFileName += "depthBoundary.png";
	imwrite(depthBoundaryFileName, mDepthBoundaryImg);

	string normalBoundaryFileName = filename;
	normalBoundaryFileName += "normalBoundary.png";
	imwrite(normalBoundaryFileName, mNormalBoundaryImg);

	string boundaryFileName = filename;
	boundaryFileName += "Boundary.png";
	imwrite(boundaryFileName, mBoundaryImg);
}
void ImageSegmentation::expand(const cv::Mat& depthImg, int seed, int numRows, int numCols, int currentSegmentationId, double depthThreshold, double angleThreshold, set<int>& unsegmentedPixelIdx, Segmentation& result)
{
	queue<int> neighbors;
	neighbors.push(seed);
	set<int> currentSegmentationPixelIndex;
	vector<int> bVisited;
	bVisited.resize(numRows * numCols, 0);
	int counter = 0;
	Vector3d normalAccumulator = Vector3d::Zero();

	while (!neighbors.empty())
	{
		int current = neighbors.front();
		neighbors.pop();

		if (counter % 1000 == 0)
		{
			LOG(INFO) << unsegmentedPixelIdx.size() << " to process.";
		}

		int u, v;
		indexTo2d(current, v, u, numRows, numCols);

		result.mSegmentedImg(v, u) = currentSegmentationId;
		currentSegmentationPixelIndex.insert(current);
		unsegmentedPixelIdx.erase(current);

		normalAccumulator += getNormal(current);
		Vector3d avgNormal = normalAccumulator.normalized();
		double cosineThreshold = cos(angleThreshold / 180 * M_PI);

		int left = leftNeighbor(current, numRows, numCols);
		int right = rightNeighbor(current, numRows, numCols);
		int upper = upperNeighbor(current, numRows, numCols);
		int lower = lowerNeighbor(current, numRows, numCols);

		//if (!isSegmented(left, result.mSegmentedImg) && !bVisited[left] && isContinuous(depthImg, current, left, depthThreshold, angleThreshold))
		//{
		//	neighbors.push(left);
		//	bVisited[left] = 1;
		//}
		//if (!isSegmented(right, result.mSegmentedImg) && !bVisited[right] && isContinuous(depthImg, current, left, depthThreshold, angleThreshold))
		//{
		//	neighbors.push(right);
		//	bVisited[right] = 1;
		//}
		//if (!isSegmented(upper, result.mSegmentedImg) && !bVisited[upper] && isContinuous(depthImg, current, left, depthThreshold, angleThreshold))
		//{
		//	neighbors.push(upper);
		//	bVisited[upper] = 1;
		//}
		//if (!isSegmented(lower, result.mSegmentedImg) && !bVisited[lower] && isContinuous(depthImg, current, left, depthThreshold, angleThreshold))
		//{
		//	neighbors.push(lower);
		//	bVisited[lower] = 1;
		//}

		if (!isSegmented(left, result.mSegmentedImg) && !bVisited[left] && !isBoundary(left) && avgNormal.dot(getNormal(left)) > cosineThreshold)
		{
			neighbors.push(left);
			bVisited[left] = 1;
		}
		if (!isSegmented(right, result.mSegmentedImg) && !bVisited[right] && !isBoundary(right) && avgNormal.dot(getNormal(right)) > cosineThreshold)
		{
			neighbors.push(right);
			bVisited[right] = 1;
		}
		if (!isSegmented(upper, result.mSegmentedImg) && !bVisited[upper] && !isBoundary(upper) && avgNormal.dot(getNormal(upper)) > cosineThreshold)
		{
			neighbors.push(upper);
			bVisited[upper] = 1;
		}
		if (!isSegmented(lower, result.mSegmentedImg) && !bVisited[lower] && !isBoundary(lower) && avgNormal.dot(getNormal(lower)) > cosineThreshold)
		{
			neighbors.push(lower);
			bVisited[lower] = 1;
		}
		counter++;
	}
	result.mSegmentedPixelIdx.push_back(currentSegmentationPixelIndex);
}
bool ImageSegmentation::isBoundary(int idx)
{
	int u, v;
	indexTo2d(idx, v, u, mBoundaryImg.rows, mBoundaryImg.cols);
	return mBoundaryImg.at<uchar>(v, u) > 128;
}
void ImageSegmentation::computeBoundaryImages(const cv::Mat& depthImg, double depthThreshold, double angleThreshold)
{
	int numRows = depthImg.rows;
	int numCols = depthImg.cols;
	mDepthBoundaryImg.create(numRows, numCols);
	mNormalBoundaryImg.create(numRows, numCols);
	mBoundaryImg.create(numRows, numCols);
	int numTotalPixels = numRows * numCols;
	for (int i = 0; i < numTotalPixels; ++i)
	{
		int current = i;
		int u, v;
		indexTo2d(current, v, u, numRows, numCols);
		int left = leftNeighbor(current, numRows, numCols);
		int right = rightNeighbor(current, numRows, numCols);
		int upper = upperNeighbor(current, numRows, numCols);
		int lower = lowerNeighbor(current, numRows, numCols);
		if (isDepthContinous(depthImg, current, left, depthThreshold) && isDepthContinous(depthImg, current, right, depthThreshold) && isDepthContinous(depthImg, current, upper, depthThreshold) && isDepthContinous(depthImg, current, lower, depthThreshold))
		{
			mDepthBoundaryImg.at<uchar>(v, u) = 0;
		}
		else
		{
			mDepthBoundaryImg.at<uchar>(v, u) = 255;
		}
		if (isNormalContinous(current, left, angleThreshold) && isNormalContinous(current, right, angleThreshold) && isNormalContinous(current, upper, angleThreshold) && isNormalContinous(current, lower, angleThreshold))
		{
			mNormalBoundaryImg.at<uchar>(v, u) = 0;
		}
		else
		{
			mNormalBoundaryImg.at<uchar>(v, u) = 255;
		}
	}
	for (int i = 0; i < numRows; ++i)
	{
		for (int j = 0; j < numCols; ++j)
		{
			mBoundaryImg.at<uchar>(i, j) = mDepthBoundaryImg.at<uchar>(i, j);// std::max(mDepthBoundaryImg.at<uchar>(i, j), mNormalBoundaryImg.at<uchar>(i, j));
		}
	}
}

int ImageSegmentation::leftNeighbor(int idx, int numRows, int numCols)
{
	int u, v;
	indexTo2d(idx, v, u, numRows, numCols);
	u -= 1;
	if (u < 0)
		u = 0;
	return indexTo1d(v, u, numRows, numCols);
}
int ImageSegmentation::rightNeighbor(int idx, int numRows, int numCols)
{
	int u, v;
	indexTo2d(idx, v, u, numRows, numCols);
	u += 1;
	if (u >= numCols)
		u = numCols - 1;
	return indexTo1d(v, u, numRows, numCols);
}
int ImageSegmentation::upperNeighbor(int idx, int numRows, int numCols)
{
	int u, v;
	indexTo2d(idx, v, u, numRows, numCols);
	v -= 1;
	if (v < 0)
		v = 0;
	return indexTo1d(v, u, numRows, numCols);
}
int ImageSegmentation::lowerNeighbor(int idx, int numRows, int numCols)
{
	int u, v;
	indexTo2d(idx, v, u, numRows, numCols);
	v += 1;
	if (v >= numRows)
		v = numRows - 1;
	return indexTo1d(v, u, numRows, numCols);
}
bool ImageSegmentation::isSegmented(int idx, const MatrixXi& segmentedImg)
{
	int u, v;
	indexTo2d(idx, v, u, segmentedImg.rows(), segmentedImg.cols());
	return segmentedImg(v, u) != 0;
}

bool ImageSegmentation::isDepthContinous(const cv::Mat& depthImg, int idx1, int idx2, double depthThreshold)
{
	int u1, v1, u2, v2;
	indexTo2d(idx1, v1, u1, depthImg.rows, depthImg.cols);
	indexTo2d(idx2, v2, u2, depthImg.rows, depthImg.cols);
	float d1 = depthImg.at<float>(v1, u1);
	float d2 = depthImg.at<float>(v2, u2);
	if (abs(d1 - d2) < depthThreshold)
		return true;
	else
		return false;
}
bool ImageSegmentation::isNormalContinous(int idx1, int idx2, double angleThreshold)
{
	int u1, v1, u2, v2;
	indexTo2d(idx1, v1, u1, mNormalImg.rows, mNormalImg.cols);
	indexTo2d(idx2, v2, u2, mNormalImg.rows, mNormalImg.cols);
	cv::Vec3d n1 = mNormalImg.at<cv::Vec3d>(v1, u1);
	cv::Vec3d n2 = mNormalImg.at<cv::Vec3d>(v2, u2);
	double cosineThreshold = cos(angleThreshold / 180 * M_PI);

	if (cv::dot<double>(n1, n2) > cosineThreshold) return true;
	else return false;
}

bool ImageSegmentation::isContinuous(const cv::Mat& depthImg, int idx1, int idx2, double depthThreshold, double angleThreshold)
{
	return isDepthContinous(depthImg, idx1, idx2, depthThreshold) && isNormalContinous(idx1, idx2, angleThreshold);
}

float ImageSegmentation::getDepth(const cv::Mat& depthImg, int idx)
{
	int u, v;
	indexTo2d(idx, v, u, depthImg.rows, depthImg.cols);
	return depthImg.at<float>(v, u);
}

Vector3d ImageSegmentation::getNormal(int idx)
{
	int u, v;
	indexTo2d(idx, v, u, mNormalImg.rows, mNormalImg.cols);
	const cv::Vec3d& n = mNormalImg.at<cv::Vec3d>(v, u);
	return Vector3d(n[0], n[1], n[2]);

}