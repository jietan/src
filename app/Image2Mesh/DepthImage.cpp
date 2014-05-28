#include "DepthImage.h"
#include "utility/mathlib.h"

float DepthImage::msDepthThreshold = 10;
float DepthImage::msCurvatureTheshold = 5000;

DepthImage::DepthImage()
{

}
DepthImage::DepthImage(const string& filename)
{
	ReadFromFile(filename);
}

DepthImage::DepthImage(const DepthImage& rhs)
{
	copyFrom(rhs);
}

DepthImage& DepthImage::operator= (const DepthImage& rhs)
{
	if (this == &rhs) 
		return *this;
	copyFrom(rhs);
	return *this;
}

void DepthImage::ReadFromFile(const string& filename)
{
	mOriginalData = cv::imread(filename.c_str(), CV_LOAD_IMAGE_UNCHANGED);

	int numRows = mOriginalData.rows;
	int numCols = mOriginalData.cols;
	mData.create(numRows, numCols);
	for (int i = 0; i < numRows; ++i)
	{
		for (int j = 0; j < numCols; ++j)
		{
			mData.at<float>(i, j) = mOriginalData.at<ushort>(i, j);
		}
	}
	cv::Mat smoothedData;
	cv::bilateralFilter(mData, smoothedData, 5, 1000, 5);
	mData = smoothedData;
	double minValue;
	double maxValue;
	cv::Point minLoc;
	cv::Point maxLoc;
	cv::minMaxLoc(mData, &minValue, &maxValue, &minLoc, &maxLoc);
	mMinDepth = static_cast<float>(minValue);
	mMaxDepth = static_cast<float>(maxValue);
}

float DepthImage::MaxDepth() const
{
	return mMaxDepth;
}
float DepthImage::MinDepth() const
{
	return mMinDepth;
}
void DepthImage::SaveToFile(const string& filename) const
{
	int numRows = mData.rows;
	int numCols = mData.cols;
	cv::Mat1w imageToWrite;
	imageToWrite.create(numRows, numCols);
	for (int i = 0; i < numRows; ++i)
	{
		for (int j = 0; j < numCols; ++j)
		{
			imageToWrite.at<ushort>(i, j) = static_cast<ushort>(mData.at<float>(i, j));
		}
	}
	cv::imwrite(filename, imageToWrite);
}
int DepthImage::NumRows() const
{
	return mData.rows;
}
int DepthImage::NumCols() const
{
	return mData.cols;
}

const vector<Vector3d>& DepthImage::GetPoints() const
{
	return mPoints;
}
const vector<Vector3d>& DepthImage::GetNormals() const
{
	return mNormals;
}

const cv::Mat1f& DepthImage::Data() const
{
	return mData;
}
float DepthImage::Depth(int idx) const
{
	int u, v;
	indexTo2d(idx, v, u, NumRows(), NumCols());
	return Depth(v, u);
}
float DepthImage::Depth(int ithRow, int jthCol) const
{
	return mData.at<float>(ithRow, jthCol);
}

Vector3d DepthImage::GlobalNormal(int idx) const
{
	return mNormals[idx];
}
Vector3d DepthImage::GlobalNormal(int ithRow, int jthCol) const
{
	 cv::Vec3f n = mNormalImage.at<cv::Vec3f>(ithRow, jthCol);
	 return Vector3d(n[0], n[1], n[2]);
}

Vector3d DepthImage::GlobalPoint(int idx) const
{
	return mPoints[idx];
}
Vector3d DepthImage::GlobalPoint(int ithRow, int jthCol) const
{
	cv::Vec3f p = mPointImage.at<cv::Vec3f>(ithRow, jthCol);
	return Vector3d(p[0], p[1], p[2]);
}

void DepthImage::Process(const Matrix4d& cameraPose)
{
	mCameraPose = cameraPose;
	depthToPoints();
	depthToNormals();
}
void DepthImage::depthToPoints()
{
	mPointImage.create(mData.rows, mData.cols);

	const double fx = 525.0; const double fy = 525.0; // default focal length
	const double cx = 319.5; const double cy = 239.5; // default optical center

	// translation from depth pixel (u,v,d) to a point (x,y,z)

	for (int v = 0; v < mData.rows; ++v)
	{
		for (int u = 0; u < mData.cols; ++u)
		{
			double d = mData.at<float>(v, u);
			double z = d / 1000.0;
			if (z > msDepthThreshold) continue;
			double x = (u - cx) * z / fx;
			double y = (v - cy) * z / fy;

			Eigen::Vector4d w = mCameraPose * Eigen::Vector4d(x, y, z, 1);

			mPointImage.at<cv::Vec3f>(v, u) = cv::Vec3f(w[0], w[1], w[2]);
			mPoints.push_back(w.head(3));
		}
	}
}
void DepthImage::depthToNormals()
{
	int numRows = mData.rows;
	int numCols = mData.cols;
	mNormalImage.create(numRows, numCols);
	int numTotalPixels = numRows * numCols;

	Vector3d tangentialAxis1;
	Vector3d tangentialAxis2;

	for (int i = 0; i < numTotalPixels; ++i)
	{
		int current = i;
		float dCurrent = Depth(current);

		int u, v;
		indexTo2d(current, v, u, numRows, numCols);

		int left = LeftNeighbor(current);
		int right = RightNeighbor(current);
		float dLeft = Depth(left);
		float dRight = Depth(right);
		if (abs(2 * dCurrent - dLeft - dRight) > msCurvatureTheshold)
		{
			double diff = abs(dRight - dCurrent) > abs(dCurrent - dLeft) ? dCurrent - dLeft : dRight - dCurrent;
			tangentialAxis1 = Vector3d(1, 0, diff);
		}
		else
		{
			tangentialAxis1 = Vector3d(2, 0, dRight - dLeft);
		}

		int upper = UpperNeighbor(current);
		int lower = LowerNeighbor(current);
		float dUpper = Depth(upper);
		float dLower = Depth(lower);
		if (abs(2 * dCurrent - dUpper - dLower) > msCurvatureTheshold)
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
		Vector3d gn = mCameraPose.block(0, 0, 3, 3) * normal;

		mNormalImage.at<cv::Vec3f>(v, u) = cv::Vec3f(gn[0], gn[1], gn[2]);
		mNormals.push_back(gn);
	}
}

int DepthImage::LeftNeighbor(int idx) const
{
	int u, v;
	int numRows = NumRows();
	int numCols = NumCols();
	indexTo2d(idx, v, u, numRows, numCols);
	u -= 1;
	if (u < 0)
		u = 0;
	return indexTo1d(v, u, numRows, numCols);
}
int DepthImage::RightNeighbor(int idx) const
{
	int u, v;
	int numRows = NumRows();
	int numCols = NumCols();
	indexTo2d(idx, v, u, numRows, numCols);
	u += 1;
	if (u >= numCols)
		u = numCols - 1;
	return indexTo1d(v, u, numRows, numCols);
}
int DepthImage::UpperNeighbor(int idx) const
{
	int u, v;
	int numRows = NumRows();
	int numCols = NumCols();
	indexTo2d(idx, v, u, numRows, numCols);
	v -= 1;
	if (v < 0)
		v = 0;
	return indexTo1d(v, u, numRows, numCols);
}
int DepthImage::LowerNeighbor(int idx) const
{
	int u, v;
	int numRows = NumRows();
	int numCols = NumCols();
	indexTo2d(idx, v, u, numRows, numCols);
	v += 1;
	if (v >= numRows)
		v = numRows - 1;
	return indexTo1d(v, u, numRows, numCols);
}

void DepthImage::copyFrom(const DepthImage& rhs)
{
	mPoints = rhs.mPoints;
	mNormals = rhs.mNormals;

	mOriginalData = rhs.mOriginalData.clone();
	mData = rhs.mData.clone();

	mNormalImage = rhs.mNormalImage.clone();
	mPointImage = rhs.mPointImage.clone();

	mCameraPose = rhs.mCameraPose;
}