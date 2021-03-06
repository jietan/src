#include "DepthImage.h"
#include "utility/mathlib.h"

float DepthImage::msDepthFarThreshold = 2.5f;
float DepthImage::msDepthNearThreshold = 0.2f;
float DepthImage::msAngleThreshold = 0.15f;

DepthImage::DepthImage() : mIsCameraSet(false), mHeight(0), mWidth(0)
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
	mHeight = numRows;
	mWidth = numCols;
	mData.create(numRows, numCols);
	for (int i = 0; i < numRows; ++i)
	{
		for (int j = 0; j < numCols; ++j)
		{
			mData.at<float>(i, j) = mOriginalData.at<ushort>(i, j);
		}
	}
	cv::Mat smoothedData;
	cv::medianBlur(mData, smoothedData, 5);
	mData = smoothedData.clone();
	//mDataForNormal = mData;
	cv::bilateralFilter(mData, mDataForNormal, 5, 1000, 5);
	
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

bool DepthImage::IsPointBehind(const Eigen::Vector3f& point, float& depthDelta) const
{
	int ithRow, jthCol;
	float depth = pointToDepth(point, ithRow, jthCol);
	if (ithRow < 0 || ithRow >= mHeight || jthCol < 0 || jthCol >= mWidth)
	{
		depthDelta = FLT_MAX;
		return true;
	}
	float cameraDepth = mData.at<float>(ithRow, jthCol);
	//if (cameraDepth < 10)
	//{
	//	depthDelta = 0;
	//	return false;
	//}
	depthDelta = depth - cameraDepth;
	return depth > cameraDepth;
	
}

void DepthImage::SetCameraPose(const Eigen::MatrixXf& pose)
{
	mIsCameraSet = true;
	mCameraPose = pose;
	mInvCameraPose = mCameraPose.inverse();
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

const vector<Eigen::Vector3f>& DepthImage::GetPoints() const
{
	return mPoints;
}
const vector<Eigen::Vector3f>& DepthImage::GetNormals() const
{
	return mNormals;
}

const cv::Mat1f& DepthImage::Data() const
{
	return mData;
}

float DepthImage::DepthForNormal(int idx) const
{
	int u, v;
	indexTo2d(idx, v, u, NumRows(), NumCols());
	return DepthForNormal(v, u);
}
float DepthImage::DepthForNormal(int ithRow, int jthCol) const
{
	return mDataForNormal.at<float>(ithRow, jthCol);
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

Eigen::Vector3f DepthImage::GlobalNormal(int idx) const
{
	return mNormals[idx];
}
Eigen::Vector3f DepthImage::GlobalNormal(int ithRow, int jthCol) const
{
	 cv::Vec3f n = mNormalImage.at<cv::Vec3f>(ithRow, jthCol);
	 return Eigen::Vector3f(n[0], n[1], n[2]);
}

Eigen::Vector3f DepthImage::GlobalPoint(int idx) const
{
	return mPoints[idx];
}
Eigen::Vector3f DepthImage::GlobalPoint(int ithRow, int jthCol) const
{
	cv::Vec3f p = mPointImage.at<cv::Vec3f>(ithRow, jthCol);
	return Eigen::Vector3f(p[0], p[1], p[2]);
}

void DepthImage::SetData(const cv::Mat1f& depthData)
{
	mData = depthData;
}

void DepthImage::Process()
{
	CHECK(mIsCameraSet) << "Camera pose should be set before calling DepthImage::Process().";
	depthToPoints();
	depthToNormals();
	selectAndCopy();
	CHECK(mPoints.size() == mNormals.size()) << "Numbers of points and normals do not agree in DepthImage::Process().";
}


void DepthImage::selectAndCopy()
{
	for (int v = 0; v < mData.rows; ++v)
	{
		for (int u = 0; u < mData.cols; ++u)
		{
			float d = mData.at<float>(v, u);
			float z = d / 1000.f;
			const cv::Vec3f& nCV = mNormalImage.at<cv::Vec3f>(v, u);
			Eigen::Vector3f n(nCV[0], nCV[1], nCV[2]);

			if (z > msDepthFarThreshold || z < msDepthNearThreshold || abs(n.dot(Eigen::Vector3f(0, 0, 1))) < msAngleThreshold) continue;

			const cv::Vec3f& pCV = mPointImage.at <cv::Vec3f>(v, u);
			Eigen::Vector3f gp = (mCameraPose * Eigen::Vector4f(pCV[0], pCV[1], pCV[2], 1)).head(3);
			Eigen::Vector3f gn = mCameraPose.block(0, 0, 3, 3) * n;

			mPoints.push_back(gp);
			mNormals.push_back(gn);
		}
	}
}

float DepthImage::pointToDepth(const Eigen::Vector3f& pt, int& ithRow, int& jthCol) const
{
	const float fx = 525.f; const float fy = 525.f; // default focal length
	const float cx = 319.5f; const float cy = 239.5f; // default optical center

	CHECK(mIsCameraSet) << "Camera pose should be set before calling DepthImage::pointToDepth().";
	Eigen::Vector3f pointsInCameraSpace = (mInvCameraPose * Eigen::Vector4f(pt[0], pt[1], pt[2], 1)).head(3);
	double x = pointsInCameraSpace[0];
	double y = pointsInCameraSpace[1];
	double z = pointsInCameraSpace[2];
	double u = x * fx / z + cx;
	double v = y * fy / z + cy;
	jthCol = static_cast<int>(u + 0.5);
	ithRow = static_cast<int>(v + 0.5);

	return static_cast<float>(1000 * z);
}
void DepthImage::depthToPoints()
{
	mPointImage.create(mData.rows, mData.cols);

	const float fx = 525.f; const float fy = 525.f; // default focal length
	const float cx = 319.5f; const float cy = 239.5f; // default optical center

	// translation from depth pixel (u,v,d) to a point (x,y,z)

	for (int v = 0; v < mData.rows; ++v)
	{
		for (int u = 0; u < mData.cols; ++u)
		{
			float d = mData.at<float>(v, u);
			float z = d / 1000.f;

			float x = (u - cx) * z / fx;
			float y = (v - cy) * z / fy;

			mPointImage.at<cv::Vec3f>(v, u) = cv::Vec3f(x, y, z);
		}
	}
}

void DepthImage::depthToNormals()
{
	int numRows = mData.rows;
	int numCols = mData.cols;
	mNormalImage.create(numRows, numCols);
	int numTotalPixels = numRows * numCols;

	Eigen::Vector3f tangentialAxis1;
	Eigen::Vector3f tangentialAxis2;

	for (int i = 0; i < numTotalPixels; ++i)
	{
		int current = i;

		int u, v;
		indexTo2d(current, v, u, numRows, numCols);

		int left = LeftNeighbor(current);
		int right = RightNeighbor(current);
		float dLeft = DepthForNormal(left);
		float dRight = DepthForNormal(right);

		tangentialAxis1 = Eigen::Vector3f(2, 0, dRight - dLeft);


		int upper = UpperNeighbor(current);
		int lower = LowerNeighbor(current);
		float dUpper = DepthForNormal(upper);
		float dLower = DepthForNormal(lower);

		tangentialAxis2 = Eigen::Vector3f(0, 2, dLower - dUpper);

		Eigen::Vector3f normal = tangentialAxis1.cross(tangentialAxis2);
		normal = -normal.normalized();


		mNormalImage.at<cv::Vec3f>(v, u) = cv::Vec3f(normal[0], normal[1], normal[2]);
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
	mInvCameraPose = rhs.mInvCameraPose;

	mIsCameraSet = rhs.mIsCameraSet;

	mHeight = rhs.mHeight;
	mWidth = rhs.mWidth;
}