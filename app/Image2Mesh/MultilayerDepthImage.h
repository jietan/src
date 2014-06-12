#ifndef _MULTILAYER_DEPTH_IMAGE_H
#define _MULTILAYER_DEPTH_IMAGE_H
#include "extendedDepthPixel.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
using namespace std;

class MultilayerDepthImage
{
public:
	MultilayerDepthImage();
	~MultilayerDepthImage();
	void Create(int nRows, int nCol);
	void Read(const string& filename);
	void Save(const string& filename);
	void SaveDepthThresholdingImage(const string& filename, int numThresholds, const vector<vector<vector<int> > >* mask = NULL, float* min = NULL, float* max = NULL);
	void SaveDepthOnionImage(const string& filename, const vector<vector<vector<int> > >* mask = NULL, float* min = NULL, float* max = NULL);
	void Simplify();
	void Process();
	void SaveDepthImage(const string& filename);
	int Width() const;
	int Height() const;
	int NumLayers() const;
	/* to fit the original vector interface */
	vector<vector<ExtendedDepthPixel> >& operator[] (int i);
	const vector<vector<ExtendedDepthPixel> >& operator[] (int i) const;
private:
	void saveDepthImageVisualization(const string& filename, const cv::Mat1f* image, const cv::Mat1i* mask, float* min = NULL, float* max = NULL);
	void findMinMaxDepth();
	void findNumLayers();
	vector<vector<vector<ExtendedDepthPixel> > > mData;
	int mWidth;
	int mHeight;
	int mLayers;
	float mMinDepth;
	float mMaxDepth;
};

#endif