#ifndef _DEPTH_CAMERA_H
#define _DEPTH_CAMERA_H
#include <set>
#include <list>
#include <vector>
using namespace std;
#include <Eigen/Dense>
using namespace Eigen;

#include "extendedDepthPixel.h"
#include "DepthImage.h"
#include "KDTree.h"


class DepthCamera
{
public:
	DepthCamera();
	~DepthCamera();
	void SetIntrinsicParameters(int numPxWidth, int numPxHeight, float focalLenth);
	void SetExtrinsicParameters(const Matrix4f& pose);
	void Capture(const vector<Vector3f>& points, const vector<Vector3f>& normals);
	void Capture(const vector<Vector3f>& vertices, const vector<Vector3i>& indices);
	void SetData(const vector<vector<vector<ExtendedDepthPixel> > >& depthMap);
	void SetMask(const vector<vector<vector<int> > >& mask);
	void SaveMultilayerDepthImage(const string& filename);
	void ReadMultilayerDepthImage(const string& filename);
	void ProcessMultiLayerDepthImage();
	void GetPointCloud(vector<Vector3f>& points, vector<Vector3f>& normals);
	void SaveDepthImage(const string& filename);
	void SaveDepthThresholdingImage(const string& filename, int numThresholds);
	void SaveDepthOnionImage(const string& filename);
	void SimplifyMultilayerDepthImage();
	void Compare(const DepthCamera& rhs, vector<vector<vector<ExtendedDepthPixel> > >& mergedDepthMap, vector<vector<vector<int> > >& mask);
private:
	Vector3f constructRayDirection(int i, int j);
	void constructDepthMap(const Vector3f& rayOrigin, const Vector3f& rayDir, const vector<Vector3f>& points, const vector<Vector3f>& normals, vector<float>& depthMap);
	void findMinMaxDepth();
	void fromMultiLayerToSinglelLayerDepthImage();
	void saveDepthImageVisualization(const string& filename, const cv::Mat1f* image, const cv::Mat1i* mask = NULL);
	void mergePointsAndNormals(const vector<ExtendedDepthPixel>& originalDepthPixel, const cv::Mat& labels, const vector<set<int> >& groups, vector<ExtendedDepthPixel>& mergedDepthPixel);

	cv::Mat1f mSensorMeasurement;
	Matrix4f mPose;
	int mWidth;
	int mHeight;
	float mFocalLength;
	float mMinDepth;
	float mMaxDepth;
	vector<vector<vector<ExtendedDepthPixel> > > mDepthMap;
	vector<vector<vector<int> > > mMask;
	KDTree mKDTree;
	//vector<vector<vector<ExtendedDepthPixel> > > mSimplifiedDepthMap;
	
};

#endif