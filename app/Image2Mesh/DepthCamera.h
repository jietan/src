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
#include "pointCloudToPrimitive/PlanePrimitiveShape.h"
#include "pointCloudToPrimitive/CylinderPrimitiveShape.h"
#include "Part.h"

#define ORTHO_PROJ 0
#define PERSP_PROJ 1

#define MOVE_UP 0
#define MOVE_DOWN 1
#define MOVE_LEFT 2
#define MOVE_RIGHT 3


#define PORTION_UNKNOWN 1
#define PORTION_KNOWN 0
#define PORTION_ALL 2

class ROI
{
public:
	int left;
	int right;
	int high;
	int low;
	float near1;
	float far1;

	ROI() : left(-1), right(-1), high(-1), low(-1), near1(-1.f), far1(-1.f)
	{

	}
	ROI(int l, int r, int hi, int lo, float n, float f) : left(l), right(r), high(hi), low(lo), near1(n), far1(f)
	{

	}
	bool IsWithin(int ithRow, int ithCol, float d)
	{
		if (left == -1) //ROI is not used
			return true;
		if (ithRow >= high && ithRow <= low && ithCol >= left && ithCol <= right && d >= near1 && d <= far1)
			return true;
		else
			return false;
	}
};

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
	Eigen::Vector3f GetCameraFront() const;
	Eigen::Vector3f GetCameraPosition() const;
	Eigen::Vector3f GetCameraUp() const;
	Eigen::Vector3f GetPoint(int ithRow, int jthCol, float depth) const;
	float GetDepth(const Eigen::Vector3f& pt, int* ithRow, int* jthCol) const;
	const MultilayerDepthImage& GetDepthImage() const;
	const MultilayerMaskImage& GetMaskImage() const;
	const Eigen::Matrix4f& GetCameraPose() const;
	void Capture(vector<Part>& parts);
	void Capture(const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals);
	void Capture(const vector<Eigen::Vector3f>& vertices, const vector<Eigen::Vector3i>& indices);
	void SetData(const MultilayerDepthImage& depthMap);
	void SetData(const DepthImage& depthMap);
	void SetMask(const MultilayerMaskImage& mask);
	void SaveMultilayerDepthImage(const string& filename);
	void ReadMultilayerDepthImage(const string& filename);
	void ProcessMultiLayerDepthImage();
	void SimplifyMultiLayerDepthImage();
	void GetPointCloud(vector<Eigen::Vector3f>& points, vector<Eigen::Vector3f>& normals, int portion);
	void Compare(const DepthCamera& rhs, bool isBoundaryFromNearestNeighbor, MultilayerDepthImage& mergedDepthMap, MultilayerMaskImage& mask);
	void SetReferenceDepthImages(const vector<DepthImage*> refImages);
	void SetComparisonROI(int left, int right, int high, int low, float near, float far);
	void CrossViewMaskUpdate(const DepthCamera& otherViewOld, const DepthCamera& otherViewNew, const MultilayerDepthImage& depthImagePointCloudOtherView, int moveDir, MultilayerDepthImage* newDepthImage, MultilayerMaskImage* newMaskImage);
	void CrossViewMaskUpdate1(const DepthCamera& otherViewOld, const DepthCamera& otherViewNew, const MultilayerDepthImage& depthImagePointCloudOtherView, int moveDir, MultilayerDepthImage* newDepthImage, MultilayerMaskImage* newMaskImage);
	void SetWallAndFloor(PrimitiveShape* wall, PrimitiveShape* floor);
private:
	Eigen::Vector3f constructRayDirection(int i, int j);
	Eigen::Vector3f constructRayOrigin(int i, int j);
	bool isPixelKnown(int ithRow, int jthCol, int kthLayer, const MultilayerMaskImage& mask);
	bool isDepthValid(int ithRow, int jthCol, float depth, float& depthDelta);
	bool isHoleBoundary(int ithRow, int jthCol, int kthLayer, const MultilayerMaskImage& mask);
	bool moveDepthUntilValid(int ithRow, int jthCol, int kthLayer, MultilayerDepthImage& mergedDepthMap);
	void constructDepthMap(const Eigen::Vector3f& rayOrigin, const Eigen::Vector3f& rayDir, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals, vector<float>& depthMap);
	void getPointCloud(const MultilayerDepthImage& image, vector<Eigen::Vector3f>& points, vector<Eigen::Vector3f>& normals, int portion);
	void boundariesFromMultiview(MultilayerDepthImage& mergedDepthMap, MultilayerMaskImage& mask);
	void boundariesFromNearestNeighbor(MultilayerDepthImage& mergedDepthMap, MultilayerMaskImage& mask);
	bool needLeaveTrace(int ithRow, int jthCol, int kthLayer, int moveDir);
	void leaveTrace(int ithRowOld, int jthColOld, float depthOld, int ithRowNew, int jthColNew, float depthNew, int kthLayer, int moveDir, MultilayerDepthImageWithMask* image);
	bool wallCulling(const Eigen::Vector3f& pt) const;
	bool connectionCulling(Part& part, const Eigen::Vector3f& pt) const;
	void spreadConnectionInfo(const vector<Eigen::Vector2i>& validBoundarySeed, Image<int>& connectionMap);
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
	ROI mComparisonROI;
	PrimitiveShape* mWall;
	PrimitiveShape* mFloor;
};

#endif