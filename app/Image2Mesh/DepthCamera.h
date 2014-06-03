#ifndef _DEPTH_CAMERA_H
#define _DEPTH_CAMERA_H

#include <vector>
using namespace std;
#include <Eigen/Dense>
using namespace Eigen;

#include "DepthImage.h"

class DepthCamera
{
public:
	DepthCamera();
	~DepthCamera();
	void SetIntrinsicParameters(int numPxWidth, int numPxHeight, double focalLenth);
	void SetExtrinsicParameters(const Matrix4d& pose);
	void Capture(const vector<Vector3d>& points, const vector<Vector3d>& normals, const string& depthImageFileName);
	void SaveMultilayerDepthImage(const string& filename);
	void ReadMultilayerDepthImage(const string& filename);
	void ProcessMultiLayerDepthImage();
	void SaveDepthImage(const string& filename);
	void SaveDepthThresholdingImage(const string& filename, int numThresholds);
	void SaveDepthOnionImage(const string& filename);
private:
	Vector3d constructRayDirection(int i, int j);
	void constructDepthMap(const Vector3d& rayOrigin, const Vector3d& rayDir, const vector<Vector3d>& points, const vector<Vector3d>& normals, vector<float>& depthMap);
	void findMinMaxDepth();
	void fromMultiLayerToSinglelLayerDepthImage();
	void saveDepthImageVisualization(const string& filename, const cv::Mat1f image);
	void simplifyMultilayerDepthImage();
	cv::Mat1f mSensorMeasurement;
	Matrix4d mPose;
	int mWidth;
	int mHeight;
	double mFocalLength;
	float mMinDepth;
	float mMaxDepth;
	vector<vector<vector<float> > > mDepthMap;
	vector<vector<vector<float> > > mSimplifiedDepthMap;
};

#endif