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
	void DivideBySkeleton(vector<Part>& newParts, int level = 0);
	const Eigen::Vector3f& GetNormal() const;
	void Save(const string& filename);
	void Read(const string& filename, const vector<PrimitiveShape*> primitives);
	PartRectangle GetRectangle() const;
private:
	void voteForNormal();
	void findBlobs(const cv::Mat &binary, std::vector < std::vector<cv::Point2i> > &blobs);
	void seperatePartsInRectangle(const vector<cv::Vec2f>& lines, Part* splittedPart, Part* remainingPart);
	void findPixelAssociatedWithLines(const vector<cv::Vec2f>& lines, vector<vector<Eigen::Vector2i> >* pxWithLines);
	void constructRectangle(const cv::Vec2f& line, const vector<Eigen::Vector2i>& pxWithLines, Eigen::Vector2f axis[2], Eigen::Vector2f* center, Eigen::Vector2f* extent);
	void classifyPointsAgainstRectangle(Eigen::Vector2f axis[2], const Eigen::Vector2f& center, const Eigen::Vector2f& extent, const vector<Eigen::Vector3f>& points, vector<int>* inRectPointId, vector<int>* outRectPointId);
	void computeCenterAndExtent(const vector<Eigen::Vector2f>& px, const Eigen::Vector2f& axis, float* center, float* extent);
	bool isPointInRectangle(const Eigen::Vector2f& localCoord, Eigen::Vector2f axis[2], const Eigen::Vector2f& center, const Eigen::Vector2f& extent);
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
	float mSkeletonResizeRatio;
	float mImageRes;
	
	cv::Mat mImagePart;
	cv::Mat mSkeletonPart;
	cv::Mat mComponentPart;
	float mAreaRatio;
	
	
};

#endif