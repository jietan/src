#ifndef _DEPTH_IMAGE_H
#define _DEPTH_IMAGE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <string>
using namespace std;

#include <Eigen/Dense>
using namespace Eigen;

class DepthImage
{
public:
	DepthImage();
	DepthImage(const string& filename);
	DepthImage(const DepthImage& rhs);

	DepthImage& operator= (const DepthImage& rhs);

	void ReadFromFile(const string& filename);
	void SaveToFile(const string& filename) const;

	void SetData(const cv::Mat1f& mDepthData);

	void Process(const Matrix4f& cameraPose);

	int NumRows() const;
	int NumCols() const;

	const vector<Vector3f>& GetPoints() const;
	const vector<Vector3f>& GetNormals() const;

	const cv::Mat1f& Data() const;
	float Depth(int idx) const;
	float Depth(int ithRow, int jthCol) const;

	float MaxDepth() const;
	float MinDepth() const;
	
	Vector3f GlobalNormal(int idx) const;
	Vector3f GlobalNormal(int ithRow, int jthCol) const;

	Vector3f GlobalPoint(int idx) const;
	Vector3f GlobalPoint(int ithRow, int jthCol) const;

	int LeftNeighbor(int idx) const;
	int RightNeighbor(int idx) const;
	int UpperNeighbor(int idx) const;
	int LowerNeighbor(int idx) const;

	static float msDepthThreshold;
	static float msCurvatureTheshold;

private:
	void copyFrom(const DepthImage& rhs);
	void depthToPoints();
	void depthToNormals();

	vector<Vector3f> mPoints;
	vector<Vector3f> mNormals;
	
	cv::Mat1w mOriginalData;
	cv::Mat1f mData;

	cv::Mat3f mNormalImage;
	cv::Mat3f mPointImage;

	float mMinDepth;
	float mMaxDepth;

	Matrix4f mCameraPose;
};

#endif