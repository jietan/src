#include "Segment.h"
#include "utility/mathlib.h"

void Segmentation::Rebuild()
{
	int numRows = mSegmentedImg.rows();
	int numCols = mSegmentedImg.cols();

	vector<set<int> > newSegmentPixelIdx;
	int numSegments = NumSegments();
	for (int i = 0; i < numSegments; ++i)
	{
		if (mSegmentedPixelIdx.empty()) continue;
		newSegmentPixelIdx.push_back(mSegmentedPixelIdx[i]);
	}
	mSegmentedPixelIdx = newSegmentPixelIdx;

	numSegments = NumSegments();
	for (int i = 0; i < numSegments; ++i)
	{
		for (set<int>::const_iterator it = mSegmentedPixelIdx[i].begin(); it != mSegmentedPixelIdx[i].end(); ++it)
		{
			int idx = *it;
			int u, v;
			indexTo2d(idx, v, u, numRows, numCols);
			mSegmentedImg(v, u) = i;
		}
	}	
}