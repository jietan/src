#ifndef _IMAGE_SEMENTATION
#define _IMAGE_SEMENTATION

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <set>
using namespace std;

#include "DepthImage.h"
#include "Segment.h"

class ImageSegmentation
{
public:
	ImageSegmentation();
	~ImageSegmentation();
	const Segmentation& Segment(const cv::Mat& depthImg, double depthThreshold, double angleThreshold);
	void SaveSegmentedImage(const string& filename);
private:
	
	void expand(const cv::Mat& depthImg, int seed, int numRows, int numCols, int currentSegmentationId, double depthThreshold, double angleThreshold, set<int>& unsegmentedPixelId, Segmentation& result);
	int leftNeighbor(int idx, int numRows, int numCols);
	int rightNeighbor(int idx, int numRows, int numCols);
	int upperNeighbor(int idx, int numRows, int numCols);
	int lowerNeighbor(int idx, int numRows, int numCols);
	bool isSegmented(int idx, const Eigen::MatrixXi& segmentedImg);
	bool isContinuous(const cv::Mat& depthImg, int idx1, int idx2, double depthThreshold, double angleThreshold);
	bool isDepthContinous(const cv::Mat& depthImg, int idx1, int idx2, double depthThreshold);
	bool isNormalContinous(int idx1, int idx2, double angleThreshold);
	void depthImageToNormals(const cv::Mat& depthImg);
	void depthImageToNormalsFeaturePreserving(const cv::Mat& depthImg, double curvatureThreshold);
	void computeBoundaryImages(const cv::Mat& depthImg, double depthThreshold, double angleThreshold);
	float getDepth(const cv::Mat& depthImg, int idx);
	Eigen::Vector3d getNormal(int idx);
	bool isBoundary(int idx);
	Segmentation mSegmentation;
	cv::Mat3d mNormalImg; 
	cv::Mat1b mNormalBoundaryImg;
	cv::Mat1b mDepthBoundaryImg;
	cv::Mat1b mBoundaryImg;
};

#endif