#ifndef _PATCH_MATCH_H
#define _PATCH_MATCH_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Dense>
#include "Image.h"
#include "utility/ConfigManager.h"

#define NON_CAUSAL 0
#define SCANLINE_CAUSAL 1
#define REVERSE_SCANLINE_CAUSAL 2

template <typename T>
class PatchMatch
{
public:
	PatchMatch();
	void SetSrc(Image<T>* srcImage, Image<int>* srcMask);
	void SetDst(Image<T>* dstImage, Image<int>* dstMask);
	void SetInitialGuess(Image<Eigen::Vector2i>* initialGuess);
	void SetDistanceMetric(float(*dist)(const Image<T>* src, const Image<int>* sMask, const Image<T>* dst, const Image<int>* dMask, const Eigen::Vector2i& sIdx, const Eigen::Vector2i& dIdx, int patchSize, int causalType));
	void SetPatchWidth(int width);
	void ComputeNNF();
	const Image<Eigen::Vector2i>& GetNNF() const;
	const Image<float>& GetNND() const;
	const Image<T>& GetReconstructedDst() const;
private:
	void initialize(int causalType);
	void propagate(int ithIteration, int i, int j, int causalType);
	void randomSearch(int ithIteration, int i, int j, int causalType);
	void reconstruct();

	Image<T>* mSrcImg;
	Image<int>* mSrcMask;
	Image<T>* mDstImg;
	Image<int>* mDstMask;
	Image<Eigen::Vector2i>* mInitialGuess;
	int mPatchSize;
	Image<Eigen::Vector2i> mNNF; //nearest neighbor field
	Image<float> mNND; //nearest neighbor distance
	Image<T> mReconstructedDst;
	//vector<Eigen::Vector2i> mHolePixelIdx;
	float(*mDistMetric)(const Image<T>* src, const Image<int>* sMask, const Image<T>* dst, const Image<int>* dMask, const Eigen::Vector2i& sIdx, const Eigen::Vector2i& dIdx, int patchSize, int causalType);
};

#include "PatchMatch.inl"
#endif