#ifndef _DEPTH_CAMERA_H
#define _DEPTH_CAMERA_H
#include <set>
#include <list>
#include <vector>
using namespace std;
#include <Eigen/Dense>
using namespace Eigen;

#include "DepthImage.h"
#include "KDTree.h"

class ExtendedDepthPixel
{
public:
	ExtendedDepthPixel() : d(0), n(Vector3f::Ones())
	{

	}
	ExtendedDepthPixel(float depth, const Vector3f& normal) : d(depth), n(normal)
	{

	}

	bool operator< (const ExtendedDepthPixel& rhs)
	{
		return this->d < rhs.d;
	}
	bool operator<= (const ExtendedDepthPixel& rhs)
	{
		return this->d <= rhs.d;
	}
	bool operator >(const ExtendedDepthPixel& rhs)
	{
		return this->d > rhs.d;
	}
	float d;
	Vector3f n;
};

class DepthCamera
{
public:
	DepthCamera();
	~DepthCamera();
	void SetIntrinsicParameters(int numPxWidth, int numPxHeight, float focalLenth);
	void SetExtrinsicParameters(const Matrix4f& pose);
	void Capture(const vector<Vector3f>& points, const vector<Vector3f>& normals);
	void Capture(const vector<Vector3f>& vertices, const vector<Vector3i>& indices);
	void SaveMultilayerDepthImage(const string& filename, const vector<vector<vector<ExtendedDepthPixel> > >& image);
	void ReadMultilayerDepthImage(const string& filename);
	void ProcessMultiLayerDepthImage();
	void GetPointCloud(vector<Vector3f>& points, vector<Vector3f>& normals);
	void SaveDepthImage(const string& filename);
	void SaveDepthThresholdingImage(const string& filename, int numThresholds);
	void SaveDepthOnionImage(const string& filename);
	void SimplifyMultilayerDepthImage();
	
private:
	Vector3f constructRayDirection(int i, int j);
	void constructDepthMap(const Vector3f& rayOrigin, const Vector3f& rayDir, const vector<Vector3f>& points, const vector<Vector3f>& normals, vector<float>& depthMap);
	void findMinMaxDepth();
	void fromMultiLayerToSinglelLayerDepthImage();
	void saveDepthImageVisualization(const string& filename, const cv::Mat1f image);
	void mergePointsAndNormals(const vector<ExtendedDepthPixel>& originalDepthPixel, const cv::Mat& labels, const vector<set<int> >& groups, vector<ExtendedDepthPixel>& mergedDepthPixel);

	cv::Mat1f mSensorMeasurement;
	Matrix4f mPose;
	int mWidth;
	int mHeight;
	float mFocalLength;
	float mMinDepth;
	float mMaxDepth;
	vector<vector<vector<ExtendedDepthPixel> > > mDepthMap;
	KDTree mKDTree;
	//vector<vector<vector<ExtendedDepthPixel> > > mSimplifiedDepthMap;
	
};

#endif