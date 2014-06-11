#ifndef _DEPTH_SEGMENTATION_H
#define _DEPTH_SEGMENTATION_H

#include "DepthImage.h"
#include "Segment.h"
#include <Eigen/Dense>

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
	Eigen::VectorXi computeNormalHistogram(int ithSegment);
	Eigen::VectorXd normalizeHistogram(const Eigen::VectorXi& hist);
	double computeSimilarity(int ithSegment, int jthSegment);
	void mergeSegments(int ithSegment, int jthSegment);
	//void DepthSegmentation::preComputeBoundaryPixels();
	double DepthSegmentation::computeBoundaryDepthDifference(int ithSegment, int jthSegment);

	DepthImage* mDepthImage;
	Segmentation mSegmentation;
	//cv::Mat mLabelImage;
	
	Eigen::MatrixXi mSegmentAdjacency;
	Eigen::MatrixXf mSegmentSimilarity;

	vector<Eigen::VectorXi> mNormalHistogram;
	vector<Eigen::VectorXd> mNormalHistogramNormalized;
	vector<set<int> > mBoundaryIdx;
	int mNumRows;
	int mNumCols;
};


#endif