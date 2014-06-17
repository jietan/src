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

#define ORTHO_PROJ 0
#define PERSP_PROJ 1

class DepthCamera
{
public:
	DepthCamera();
	~DepthCamera();
	
	void SetProjectionType(int projectionType);
	void SetIntrinsicParameters(int numPxWidth, int numPxHeight, float focalLenth);
	void SetExtrinsicParameters(const Eigen::Matrix4f& pose);
	void SetOrthoWidth(float width);
	float GetFocalLength() const;
	const Eigen::Matrix4f& GetCameraPose() const;
	void Capture(const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals);
	void Capture(const vector<Eigen::Vector3f>& vertices, const vector<Eigen::Vector3i>& indices);
	void SetData(const MultilayerDepthImage& depthMap);
	void SetMask(const MultilayerMaskImage& mask);
	void SaveMultilayerDepthImage(const string& filename);
	void ReadMultilayerDepthImage(const string& filename);
	void ProcessMultiLayerDepthImage();
	void SimplifyMultiLayerDepthImage();
	void GetPointCloud(vector<Eigen::Vector3f>& points, vector<Eigen::Vector3f>& normals);
	void Compare(const DepthCamera& rhs, MultilayerDepthImage& mergedDepthMap, MultilayerMaskImage& mask);
	void SetOrthoReference(MultilayerDepthImage* fromPerspectiveProj);
	void GetOrthoProjBoundingBox();
private:
	Eigen::Vector3f constructRayDirection(int i, int j);
	Eigen::Vector3f constructRayOrigin(int i, int j);
	void constructDepthMap(const Eigen::Vector3f& rayOrigin, const Eigen::Vector3f& rayDir, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals, vector<float>& depthMap);
	void getPointCloud(const MultilayerDepthImage& image, vector<Eigen::Vector3f>& points, vector<Eigen::Vector3f>& normals);
	Eigen::Matrix4f mPose;
	int mWidth;
	int mHeight;
	float mFocalLength;
	MultilayerDepthImage* mOrthoReference;
	MultilayerDepthImage mDepthMap;
	MultilayerMaskImage mMask;
	KDTree mKDTree;	
	int mProjType;
	float mOrthoWidth;
	float mOrthoHeight;
};

#endif