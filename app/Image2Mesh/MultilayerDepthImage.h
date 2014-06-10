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
	void SaveDepthThresholdingImage(const string& filename, int numThresholds, const vector<vector<vector<int> > >* mask = NULL);
	void SaveDepthOnionImage(const string& filename, const vector<vector<vector<int> > >* mask = NULL);
	void Simplify();
	void Process();
	void SaveDepthImage(const string& filename);
	/* to fit the original vector interface */
	vector<vector<ExtendedDepthPixel> >& operator[] (int i);
	const vector<vector<ExtendedDepthPixel> >& operator[] (int i) const;
private:
	void saveDepthImageVisualization(const string& filename, const cv::Mat1f* image, const cv::Mat1i* mask);
	void findMinMaxDepth();
	
	vector<vector<vector<ExtendedDepthPixel> > > mData;
	int mWidth;
	int mHeight;
	float mMinDepth;
	float mMaxDepth;
};

#endif