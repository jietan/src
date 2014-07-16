#ifndef _PART_H
#define _PART_H


#include "pointCloudToPrimitive/PlanePrimitiveShape.h"
#include "pointCloudToPrimitive/CylinderPrimitiveShape.h"

#include <Eigen/Dense>
#include <vector>
using namespace std;
#include "sehoon/ANNHelper.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Rectangle.h"

class Part
{
public:
	Part();
	Part(PrimitiveShape* shape, int shapeIdx, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals, sehoon::ann::KDTree* tree);
	//void SetData(PrimitiveShape* shape, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals);
	PrimitiveShape* GetShape() const;
	const vector<Eigen::Vector3f>& GetPoints() const;
	const vector<Eigen::Vector3f>& GetNormals() const;
	bool IsPointClose(const Eigen::Vector3f& pt);
	const cv::Mat& ImagePart();
	const cv::Mat& SkeletonPart();
	void Process();
	void DivideByNormalDirection(vector<Part>& newParts);
	const cv::Mat& DivideByConnectivity(vector<Part>& newParts);
	void DivideBySkeleton(vector<Part>& newParts);
	const Eigen::Vector3f& GetNormal() const;
	void Save(const string& filename);
	void Read(const string& filename, const vector<PrimitiveShape*> primitives);
	PartRectangle GetRectangle() const;
private:
	void voteForNormal();
	void findBlobs(const cv::Mat &binary, std::vector < std::vector<cv::Point2i> > &blobs);

	PrimitiveShape* mShape;
	int mShapeIndex;
	vector<Eigen::Vector3f> mPoints;
	vector<Eigen::Vector2f> mLocalCoords;
	vector<Eigen::Vector3f> mNormals;
	sehoon::ann::KDTree* mKDTreePoints;
	Eigen::Vector3f mCenter;
	Eigen::Vector3f mAxis1;
	Eigen::Vector3f mAxis2;
	Eigen::Vector3f mPartNormal;
	Eigen::Vector2f mBBMin;
	Eigen::Vector2f mBBMax;
	float mImageRes;
	
	cv::Mat mImagePart;
	cv::Mat mSkeletonPart;
	cv::Mat mComponentPart;
	float mAreaRatio;
	
	
};

#endif