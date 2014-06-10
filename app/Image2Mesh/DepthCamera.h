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
#include "MultilayerDepthImage.h"


class DepthCamera
{
public:
	DepthCamera();
	~DepthCamera();
	void SetIntrinsicParameters(int numPxWidth, int numPxHeight, float focalLenth);
	void SetExtrinsicParameters(const Matrix4f& pose);
	void Capture(const vector<Vector3f>& points, const vector<Vector3f>& normals);
	void Capture(const vector<Vector3f>& vertices, const vector<Vector3i>& indices);
	void SetData(const MultilayerDepthImage& depthMap);
	void SetMask(const vector<vector<vector<int> > >& mask);
	void SaveMultilayerDepthImage(const string& filename);
	void ReadMultilayerDepthImage(const string& filename);
	void ProcessMultiLayerDepthImage();
	void SimplifyMultiLayerDepthImage();
	void GetPointCloud(vector<Vector3f>& points, vector<Vector3f>& normals);
	void Compare(const DepthCamera& rhs, MultilayerDepthImage& mergedDepthMap, vector<vector<vector<int> > >& mask);
private:
	Vector3f constructRayDirection(int i, int j);
	void constructDepthMap(const Vector3f& rayOrigin, const Vector3f& rayDir, const vector<Vector3f>& points, const vector<Vector3f>& normals, vector<float>& depthMap);

	Matrix4f mPose;
	int mWidth;
	int mHeight;
	float mFocalLength;
	MultilayerDepthImage mDepthMap;
	vector<vector<vector<int> > > mMask;
	KDTree mKDTree;	
};

#endif