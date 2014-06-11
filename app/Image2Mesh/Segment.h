#ifndef _SEGMENT_H
#define _SEGMENT_H

#include <vector>
#include <set>
using namespace std;

#include <Eigen/Dense>

#define MAX_NUM_SEGMENTS 12



class Segmentation
{
public:
	Eigen::MatrixXi mSegmentedImg;
	vector<set<int> > mSegmentedPixelIdx;
	void Rebuild();
	int NumSegments() const
	{
		return static_cast<int>(mSegmentedPixelIdx.size());
	}
};

#endif