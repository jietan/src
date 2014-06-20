#ifndef _DEPTH_IMAGE_H
#define _DEPTH_IMAGE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <string>
using namespace std;

#include <Eigen/Dense>

class DepthImage
{
public:
	DepthImage();
	DepthImage(const string& filename);
	DepthImage(const DepthImage& rhs);

	DepthImage& operator= (const DepthImage& rhs);

	void ReadFromFile(const string& filename);
	void SaveToFile(const string& filename) const;
	void SetCameraPose(const Eigen::MatrixXf& pose);
	void SetData(const cv::Mat1f& depthData);
	bool IsPointBehind(const Eigen::Vector3f& point, float& depthDelta) const;
	void Process();

	int NumRows() const;
	int NumCols() const;

	const vector<Eigen::Vector3f>& GetPoints() const;
	const vector<Eigen::Vector3f>& GetNormals() const; //these two vectors should not be indexed by indexFrom2D because of the theshholding when processing the depth image.

	const cv::Mat1f& Data() const;
	float Depth(int idx) const;
	float Depth(int ithRow, int jthCol) const;
	float DepthForNormal(int idx) const;
	float DepthForNormal(int ithRow, int jthCol) const;

	float MaxDepth() const;
	float MinDepth() const;
	
	Eigen::Vector3f GlobalNormal(int idx) const;
	Eigen::Vector3f GlobalNormal(int ithRow, int jthCol) const;

	Eigen::Vector3f GlobalPoint(int idx) const;
	Eigen::Vector3f GlobalPoint(int ithRow, int jthCol) const;

	int LeftNeighbor(int idx) const;
	int RightNeighbor(int idx) const;
	int UpperNeighbor(int idx) const;
	int LowerNeighbor(int idx) const;

	static float msDepthFarThreshold;
	static float msDepthNearThreshold;
	static float msAngleThreshold;

private:
	void copyFrom(const DepthImage& rhs);
	float pointToDepth(const Eigen::Vector3f& pt, int& ithRow, int& jthCol) const;
	void depthToPoints();
	void depthToNormals();
	void selectAndCopy();

	vector<Eigen::Vector3f> mPoints;
	vector<Eigen::Vector3f> mNormals;
	
	cv::Mat1w mOriginalData;
	cv::Mat1f mData;
	cv::Mat1f mDataForNormal;

	cv::Mat3f mNormalImage;
	cv::Mat3f mPointImage;

	float mMinDepth;
	float mMaxDepth;

	Eigen::Matrix4f mCameraPose;
	Eigen::Matrix4f mInvCameraPose;

	bool mIsCameraSet;

	int mHeight;
	int mWidth;
};

#endif