#ifndef _IMAGE_INPAINTING_H
#define _IMAGE_INPAINTING_H

#include "PatchMatch.h"

template <typename T>
class ImageInpainting
{
public:
	ImageInpainting();
	virtual ~ImageInpainting();
	virtual void Inpaint();
	void SetSrc(Image<T>* image, Image<int>* mask);
	void SetDst(Image<T>* image, Image<int>* mask);
	const Image<T>& GetResult() const;
	const Image<Eigen::Vector2i>& GetNNF() const;
	const Image<float>& ImageInpainting::GetNND() const;

private:
	void inpaint(Image<T>* srcImg, Image<int>* srcMask, Image<T>* dstImg, Image<int>* dstMask, int numIterations, Image<T>* resultImage, Image<Eigen::Vector2i>* resultPatchCoord, Image<float>* resultPatchDistance, Image<Eigen::Vector2i>* initialGuess = NULL);
	void generatePyramids(int numPyramids);
	void upSample(const Image<Eigen::Vector2i>* lowRes, Image<Eigen::Vector2i>* highRes);
	void select(PatchMatch<T>* nnSolver, Image<T>* srcImg, Image<int>* srcMask, Image<T>* dstImg, Image<int>* dstMask, Image<Eigen::Vector2i>* nnf, Image<float>* nnd);
	void vote(Image<T>* srcImg, Image<int>* srcMask, Image<T>* dstImg, Image<int>* dstMask, Image<Eigen::Vector2i>* nnf, Image<float>* nnd, Image<T>* resultImage);
	Image<T>* mSrcImg;
	Image<int>* mSrcMask;
	Image<T>* mDstImg;
	Image<int>* mDstMask;
	
	vector<Image<T> > mSrcImagePyramids;
	vector<Image<int> > mSrcMaskPyramids;
	vector<Image<T> > mDstImagePyramids;
	vector<Image<int> > mDstMaskPyramids;
	vector<Image<T> > mResultImagePyramids;
	vector<Image<Eigen::Vector2i> > mResultPatchCoordPyramids;
	vector<Image<float> > mResultPatchDistancePyramids;
};

#include "ImageInpainting.inl"

#endif