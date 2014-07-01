#ifndef _MULTILAYER_DEPTH_IMAGE_H
#define _MULTILAYER_DEPTH_IMAGE_H
#include "MultilayerImage.h"
#include "MultilayerMaskImage.h"
#include "extendedDepthPixel.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
using namespace std;


class MultilayerDepthImage : public MultilayerImage<ExtendedDepthPixel>
{
public:
	MultilayerDepthImage();
	virtual ~MultilayerDepthImage();
	virtual void Process();

	void SetCameraPose(const Eigen::Matrix4f camPose);
	void Read(const string& filename);
	void Save(const string& filename);
	void SaveDepthThresholdingImage(const string& filename, int numThresholds, const MultilayerMaskImage* mask = NULL, float* min = NULL, float* max = NULL);
	void SaveDepthOnionImage(const string& filename, const MultilayerMaskImage* mask = NULL, float* min = NULL, float* max = NULL);
	void Simplify();

	void SaveDepthImage(const string& filename);
	/* to fit the original vector interface */
private:
	void saveDepthImageVisualization(const string& filename, const cv::Mat1f* image, const cv::Mat1i* mask, float* min = NULL, float* max = NULL);
	void findMinMaxDepth();


	float mMinDepth;
	float mMaxDepth;
	
};

class MultilayerDepthImageWithMask : public MultilayerImage < ExtendedDepthPixelWithMask >
{
public:
	virtual ~MultilayerDepthImageWithMask();
	void Simplify();
};

#endif