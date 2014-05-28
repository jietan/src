#ifndef _DEPTH_INPAINTING_H
#define _DEPTH_INPAINTING_H

#include "DepthImage.h"
#include <Eigen/Dense>
using namespace Eigen;


class DepthImageInpainting
{
public:
	DepthImageInpainting();
	~DepthImageInpainting();
	void SetDepthImage(DepthImage* depthImage);
	void SetMaskImage(const MatrixXi& maskImage);
	void Inpaint();
	void SaveResultImage(const string& filename);

private:
	DepthImage* mDepthImage;
	MatrixXi mMaskImage;
	MatrixXf mResultImage;
	int mNumRows;
	int mNumCols;
};

#endif