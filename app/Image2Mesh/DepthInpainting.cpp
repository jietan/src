#include "DepthInpainting.h"
#include "glog/logging.h"
using namespace google;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/photo/photo.hpp>


DepthImageInpainting::DepthImageInpainting()
{

}
DepthImageInpainting::~DepthImageInpainting()
{

}
void DepthImageInpainting::SetDepthImage(DepthImage* depthImage)
{
	mDepthImage = depthImage;
	mNumRows = mDepthImage->NumRows();
	mNumCols = mDepthImage->NumCols();
}
void DepthImageInpainting::SetMaskImage(const MatrixXi& maskImage)
{
	mMaskImage = maskImage;
}
void DepthImageInpainting::Inpaint()
{
	CHECK(mNumRows == mMaskImage.rows() && mNumCols == mMaskImage.cols()) << "The depth and mask images are of different size in DepthImageInpainting::Inpaint().";
	
	cv::Mat maskImage(mNumRows, mNumCols, CV_8U);
	Matrix<uchar, Dynamic, Dynamic> maskImageEigen = mMaskImage.cast<uchar>();
	cv::eigen2cv(maskImageEigen, maskImage);
	cv::Mat resultImage;
	//cv::inpaint(mDepthImage->Data(), maskImage, resultImage, 9, cv::INPAINT_NS);
	
	cv::cv2eigen(resultImage, mResultImage);
}
void DepthImageInpainting::SaveResultImage(const string& filename)
{
	cv::Mat resultImage(mNumRows, mNumCols, CV_16U);
	Matrix<ushort, Dynamic, Dynamic> resultImageEigen = mResultImage.cast<ushort>();
	cv::eigen2cv(resultImageEigen, resultImage);
	imwrite(filename, resultImage);
}

