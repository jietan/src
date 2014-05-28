#ifndef _DEPTH_SEGMENTATION_H
#define _DEPTH_SEGMENTATION_H

#include "DepthImage.h"
#include "Segment.h"
#include <Eigen/Dense>
using namespace Eigen;

class DepthSegmentation
{
#define NUM_ICOSAGEDRON_FACES 20

public:
	DepthSegmentation(DepthImage* depthImage);
	~DepthSegmentation();
	const Segmentation& Segment(int numSegments);
	void SaveSegmentedImage(const string& filename);
	int NumSegments() const;
private:
	void mergeSegments();
	void initializeAdjacency();
	void initializeNormalHistogram();
	void initializeSimilarity();
	VectorXi computeNormalHistogram(int ithSegment);
	VectorXd normalizeHistogram(const VectorXi& hist);
	double computeSimilarity(int ithSegment, int jthSegment);
	void mergeSegments(int ithSegment, int jthSegment);
	//void DepthSegmentation::preComputeBoundaryPixels();
	double DepthSegmentation::computeBoundaryDepthDifference(int ithSegment, int jthSegment);

	DepthImage* mDepthImage;
	Segmentation mSegmentation;
	//cv::Mat mLabelImage;
	
	MatrixXi mSegmentAdjacency;
	MatrixXf mSegmentSimilarity;

	vector<VectorXi> mNormalHistogram;
	vector<VectorXd> mNormalHistogramNormalized;
	vector<set<int> > mBoundaryIdx;
	int mNumRows;
	int mNumCols;
};


#endif