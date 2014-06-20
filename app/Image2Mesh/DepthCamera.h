#ifndef _DEPTH_CAMERA_H
#define _DEPTH_CAMERA_H
#include <set>
#include <list>
#include <vector>
using namespace std;
#include <Eigen/Dense>

#include "extendedDepthPixel.h"
#include "DepthImage.h"
#include "KDTree.h"
#include "MultilayerDepthImage.h"
#include "MultilayerMaskImage.h"
#include "sehoon/ANNHelper.h"

#define ORTHO_PROJ 0
#define PERSP_PROJ 1


#define MASK_UNKNOWN 1
#define MASK_KNOWN 0
#define PORTION_UNKNOWN 1
#define PORTION_KNOWN 0
#define PORTION_ALL 2

class DepthCamera
{
public:
	DepthCamera();
	~DepthCamera();
	
	void SetProjectionType(int projectionType);
	void SetIntrinsicParameters(int numPxWidth, int numPxHeight, float focalLenth);
	void SetExtrinsicParameters(const Eigen::Matrix4f& pose);
	void SetOrthoWidth(float width);
	void SetSimplifiedPointCloud(const vector<Eigen::Vector3f>& points);
	float GetOrthoWidth() const;
	float GetFocalLength() const;
	Eigen::Vector3f GetPoint(int ithRow, int jthCol, float depth) const;
	float GetDepth(const Eigen::Vector3f& pt, int* ithRow, int* jthCol) const;
	const Eigen::Matrix4f& GetCameraPose() const;
	void Capture(const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals);
	void Capture(const vector<Eigen::Vector3f>& vertices, const vector<Eigen::Vector3i>& indices);
	void SetData(const MultilayerDepthImage& depthMap);
	void SetMask(const MultilayerMaskImage& mask);
	void SaveMultilayerDepthImage(const string& filename);
	void ReadMultilayerDepthImage(const string& filename);
	void ProcessMultiLayerDepthImage();
	void SimplifyMultiLayerDepthImage();
	void GetPointCloud(vector<Eigen::Vector3f>& points, vector<Eigen::Vector3f>& normals, int portion);
	void Compare(const DepthCamera& rhs, MultilayerDepthImage& mergedDepthMap, MultilayerMaskImage& mask);
	void SetReferenceDepthImages(const vector<DepthImage*> refImages);
private:
	Eigen::Vector3f constructRayDirection(int i, int j);
	Eigen::Vector3f constructRayOrigin(int i, int j);
	bool isDepthValid(int ithRow, int jthCol, float depth, float& depthDelta);
	bool isHoleBoundary(int ithRow, int jthCol, int kthLayer, const MultilayerMaskImage& mask);
	bool moveDepthUntilValid(int ithRow, int jthCol, int kthLayer, MultilayerDepthImage& mergedDepthMap);
	void constructDepthMap(const Eigen::Vector3f& rayOrigin, const Eigen::Vector3f& rayDir, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals, vector<float>& depthMap);
	void getPointCloud(const MultilayerDepthImage& image, vector<Eigen::Vector3f>& points, vector<Eigen::Vector3f>& normals, int portion);
	void boundariesFromMultiview(MultilayerDepthImage& mergedDepthMap, MultilayerMaskImage& mask);
	void boundariesFromNearestNeighbor(MultilayerDepthImage& mergedDepthMap, MultilayerMaskImage& mask);
	Eigen::Matrix4f mPose;
	Eigen::Matrix4f mInvPose;
	int mWidth;
	int mHeight;
	float mFocalLength;
	vector<DepthImage*> mRefDepthImages;
	MultilayerDepthImage mDepthMap;
	MultilayerMaskImage mMask;
	KDTree mKDTree;	
	sehoon::ann::KDTree mKDTreePointCloud;
	int mProjType;
	float mOrthoWidth;
	float mOrthoHeight;
	vector<Eigen::Vector3f> mSimplifiedPointCloud;
};

#endif