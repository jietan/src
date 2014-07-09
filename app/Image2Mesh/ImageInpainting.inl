
template <typename T>
ImageInpainting<T>::ImageInpainting()
{

}

template <typename T>
ImageInpainting<T>::~ImageInpainting()
{

}

template <typename T>
void ImageInpainting<T>::Inpaint()
{
	int numPyramids = 1;
	int numIterations = 1;

	DecoConfig::GetSingleton()->GetInt("Inpainting", "NumPyramids", numPyramids);
	DecoConfig::GetSingleton()->GetInt("Inpainting", "NumIteration", numIterations);
	generatePyramids(numPyramids);
	Image<Eigen::Vector2i> initialGuess;
	for (int ithPyramid = 0; ithPyramid < numPyramids; ++ithPyramid)
	{
		if (ithPyramid != 0)
		{
			upSample(&(mResultPatchCoordPyramids[ithPyramid - 1]), &(mResultPatchCoordPyramids[ithPyramid]));
		}
		inpaint(&(mSrcImagePyramids[ithPyramid]), &(mSrcMaskPyramids[ithPyramid]), &(mDstImagePyramids[ithPyramid]), &(mDstMaskPyramids[ithPyramid]), numIterations, &(mResultImagePyramids[ithPyramid]), &(mResultPatchCoordPyramids[ithPyramid]), &(mResultPatchDistancePyramids[ithPyramid]), (ithPyramid == 0 ? NULL : &(mResultPatchCoordPyramids[ithPyramid])));
	}
}

template < typename T >
const Image<float>& ImageInpainting<T>::GetNND() const
{
	return *mResultPatchDistancePyramids.rbegin();
}

template <typename T>
const Image<Eigen::Vector2i>& ImageInpainting<T>::GetNNF() const
{
	return *mResultPatchCoordPyramids.rbegin();
}

template <typename T>
const Image<T>& ImageInpainting<T>::GetResult() const
{
	return *mResultImagePyramids.rbegin();
}

template <typename T>
void ImageInpainting<T>::SetSrc(Image<T>* image, Image<int>* mask)
{
	mSrcImg = image;
	mSrcMask = mask;
}

template <typename T>
void ImageInpainting<T>::SetDst(Image<T>* image, Image<int>* mask)
{
	mDstImg = image;
	mDstMask = mask;
}

template <typename T>
void ImageInpainting<T>::generatePyramids(int numPyramids)
{

	mSrcImagePyramids.clear();
	mSrcMaskPyramids.clear();
	mDstImagePyramids.clear();
	mDstMaskPyramids.clear();
	mResultImagePyramids.clear();
	mResultPatchCoordPyramids.clear();
	mResultPatchDistancePyramids.clear();

	mSrcImagePyramids.resize(numPyramids);
	mSrcMaskPyramids.resize(numPyramids);
	mDstImagePyramids.resize(numPyramids);
	mDstMaskPyramids.resize(numPyramids);
	mResultImagePyramids.resize(numPyramids);
	mResultPatchCoordPyramids.resize(numPyramids);
	mResultPatchDistancePyramids.resize(numPyramids);

	mSrcImagePyramids[numPyramids - 1] = *mSrcImg;
	mSrcMaskPyramids[numPyramids - 1] = *mSrcMask;
	mDstImagePyramids[numPyramids - 1] = *mDstImg;
	mDstMaskPyramids[numPyramids - 1] = *mDstMask;

	for (int i = numPyramids - 2; i >= 0; --i)
	{
		mSrcImagePyramids[i] = mSrcImagePyramids[i + 1];
		mSrcMaskPyramids[i] = mSrcMaskPyramids[i + 1];
		mDstImagePyramids[i] = mDstImagePyramids[i + 1];
		mDstMaskPyramids[i] = mDstMaskPyramids[i + 1];

		mSrcImagePyramids[i].DownSample();
		mSrcMaskPyramids[i].DownSample();
		mDstImagePyramids[i].DownSample();
		mDstMaskPyramids[i].DownSample();
	}
}

template <typename T>
void ImageInpainting<T>::upSample(const Image<Eigen::Vector2i>* lowRes, Image<Eigen::Vector2i>* highRes)
{
	int lowResWidth = lowRes->Width();
	int lowResHeight = lowRes->Height();

	int highResWidth = lowResWidth * 2;
	int highResHeight = lowResHeight * 2;

	highRes->Create(highResHeight, highResWidth);

	for (int i = 0; i < highResHeight; ++i)
	{
		for (int j = 0; j < highResWidth; ++j)
		{
			(*highRes)[i][j][0] = 2 * (*lowRes)[i / 2][j / 2][0] + i % 2;
			(*highRes)[i][j][1] = 2 * (*lowRes)[i / 2][j / 2][1] + j % 2;
		}
	}
}

template <typename T>
void ImageInpainting<T>::inpaint(Image<T>* srcImg, Image<int>* srcMask, Image<T>* dstImg, Image<int>* dstMask, int numIterations, Image<T>* resultImage, Image<Eigen::Vector2i>* resultPatchCoord, Image<float>* resultPatchDistance, Image<Eigen::Vector2i>* initialGuess)
{
	PatchMatch<T> nnSolver;
	
	*resultImage = *dstImg;
	if (initialGuess)
		nnSolver.SetInitialGuess(initialGuess);
	nnSolver.SetDistanceMetric(dist1);

	for (int ithIteration = 0; ithIteration < numIterations; ++ithIteration)
	{
		select(&nnSolver, srcImg, srcMask, resultImage, dstMask, resultPatchCoord, resultPatchDistance);
		vote(srcImg, srcMask, dstImg, dstMask, resultPatchCoord, resultPatchDistance, resultImage);
		nnSolver.SetInitialGuess(resultPatchCoord);
	}

}

template <typename T>
void ImageInpainting<T>::select(PatchMatch<T>* nnSolver, Image<T>* srcImg, Image<int>* srcMask, Image<T>* dstImg, Image<int>* dstMask, Image<Eigen::Vector2i>* nnf, Image<float>* nnd)
{
	nnSolver->SetSrc(srcImg, srcMask);
	nnSolver->SetDst(dstImg, dstMask);
	nnSolver->ComputeNNF();

	*nnf = nnSolver->GetNNF();
	*nnd = nnSolver->GetNND();
}

template <typename T>
void ImageInpainting<T>::vote(Image<T>* srcImg, Image<int>* srcMask, Image<T>* dstImg, Image<int>* dstMask, Image<Eigen::Vector2i>* nnf, Image<float>* nnd, Image<T>* resultImage)
{
	int dstWidth = dstImg->Width();
	int dstHeight = dstImg->Height();
	resultImage->Create(dstHeight, dstWidth);

	for (int i = 0; i < dstHeight; ++i)
	{
		for (int j = 0; j < dstWidth; ++j)
		{
			if ((*dstMask)[i][j] == MASK_KNOWN)
				(*resultImage)[i][j] = (*dstImg)[i][j];
			else
			{
				int srcIndexI = (*nnf)[i][j][0];
				int srcIndexJ = (*nnf)[i][j][1];
				(*resultImage)[i][j] = (*srcImg)[srcIndexI][srcIndexJ];
			}
		}
	}
}
