#ifndef _DEPTH_INPAINTING_H
#define _DEPTH_INPAINTING_H

#include <Eigen/Dense>
using namespace Eigen;

#include <vector>
using namespace std;

#include "extendedDepthPixel.h"


class DepthImageInpainting
{
public:
	DepthImageInpainting();
	~DepthImageInpainting();
	void SetDepthImage(vector<vector<vector<ExtendedDepthPixel> > >* depthImage);
	void SetMaskImage(vector<vector<vector<int> > >* maskImage);
	void Inpaint();
	void SaveResultImage(const string& filename);

private:
	vector<vector<vector<ExtendedDepthPixel> > >* mDepthImage;
	vector<vector<vector<int> > >* mMaskImage;
	vector<vector<vector<ExtendedDepthPixel> > > mResultImage;
	int mNumRows;
	int mNumCols;
};

#endif