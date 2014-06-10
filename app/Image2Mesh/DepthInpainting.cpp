#include "DepthInpainting.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/photo/photo.hpp>
#include "glog/logging.h"
using namespace google;


DepthImageInpainting::DepthImageInpainting()
{

}
DepthImageInpainting::~DepthImageInpainting()
{

}
void DepthImageInpainting::SetDepthImage(vector<vector<vector<ExtendedDepthPixel> > >* depthImage)
{
	mDepthImage = depthImage;
	mNumRows = static_cast<int>(depthImage->size());
	mNumCols = static_cast<int>((*depthImage)[0].size());
}
void DepthImageInpainting::SetMaskImage(vector<vector<vector<int> > >* maskImage)
{
	mMaskImage = maskImage;
}
void DepthImageInpainting::Inpaint()
{

}
void DepthImageInpainting::SaveResultImage(const string& filename)
{
	//cv::Mat resultImage(mNumRows, mNumCols, CV_16U);
	//Matrix<ushort, Dynamic, Dynamic> resultImageEigen = mResultImage.cast<ushort>();
	//cv::eigen2cv(resultImageEigen, resultImage);
	//imwrite(filename, resultImage);
}

