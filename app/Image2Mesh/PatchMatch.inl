template <typename T>
PatchMatch<T>::PatchMatch() : mSrcImg(NULL), mDstImg(NULL), mSrcMask(NULL), mDstMask(NULL), mInitialGuess(NULL)
{
	mPatchSize = 7;
}

template <typename T>
void PatchMatch<T>::SetSrc(Image<T>* srcImage, Image<int>* srcMask)
{
	mSrcImg = srcImage;
	mSrcMask = srcMask;
}

template <typename T>
void PatchMatch<T>::SetDst(Image<T>* dstImage, Image<int>* dstMask)
{
	mDstImg = dstImage;
	mDstMask = dstMask;
}
template <typename T>
void PatchMatch<T>::SetInitialGuess(Image<Eigen::Vector2i>* initialGuess)
{
	mInitialGuess = initialGuess;
}
template <typename T>
void PatchMatch<T>::SetPatchWidth(int width)
{
	mPatchSize = width;
}

template <typename T>
void PatchMatch<T>::ComputeNNF()
{
	int numIterations = 1;
	int bUseCausalNeighbors = 0;
	int bUsePropagation = 1;
	int bUseRandomSearch = 1;
	DecoConfig::GetSingleton()->GetInt("PatchMatch", "PatchSize", mPatchSize);
	DecoConfig::GetSingleton()->GetInt("PatchMatch", "NumIteration", numIterations);
	DecoConfig::GetSingleton()->GetInt("PatchMatch", "UseCausalNeighbors", bUseCausalNeighbors);
	DecoConfig::GetSingleton()->GetInt("PatchMatch", "UseRandomSearch", bUseRandomSearch);
	DecoConfig::GetSingleton()->GetInt("PatchMatch", "UsePropagation", bUsePropagation);


	initialize(bUseCausalNeighbors?SCANLINE_CAUSAL:NON_CAUSAL);
	int height = mDstImg->Height();
	int width = mDstImg->Width();
	
	for (int ithIteration = 0; ithIteration < numIterations; ++ithIteration)
	{
		LOG(INFO) << "Start " << ithIteration << "th iteration in PatchMatch::ComputeNNF().";
		if (ithIteration % 2 == 0)
		{
			for (int i = 0; i < height; ++i)
			{
				for (int j = 0; j < width; ++j)
				{
					if (bUsePropagation)
						propagate(ithIteration, i, j, bUseCausalNeighbors?SCANLINE_CAUSAL:NON_CAUSAL);
					if (bUseRandomSearch)
						randomSearch(ithIteration, i, j, NON_CAUSAL);
				}
			}
		}
		else
		{
			for (int i = height - 1; i >= 0; --i)
			{
				for (int j = width - 1; j >= 0; --j)
				{
					if (bUsePropagation)
						propagate(ithIteration, i, j, bUseCausalNeighbors?REVERSE_SCANLINE_CAUSAL:NON_CAUSAL);
					if (bUseRandomSearch)
						randomSearch(ithIteration, i, j, NON_CAUSAL);
				}
			}
		}
		float totalError = 0;
		for (int i = 0; i < height; ++i)
		{
			for (int j = 0; j < width; ++j)
			{
				totalError += mNND[i][j];
			}
		}
		LOG(INFO) << "Errors: " << totalError;
	}
	reconstruct();
	float totalError = 0;
	for (int i = 0; i < height; ++i)
	{
		for (int j = 0; j < width; ++j)
		{
			
			totalError += mDistMetric(&mReconstructedDst, mDstMask, mDstImg, mDstMask, Eigen::Vector2i(i, j), Eigen::Vector2i(i, j), mPatchSize, NON_CAUSAL);
			//totalError += mDistMetric(mSrcImg, mSrcMask, mDstImg, mDstMask, mNNF[i][j], Eigen::Vector2i(i, j), mPatchSize, NON_CAUSAL);
		}
	}
	LOG(INFO) << "Final Errors: " << totalError;
}

template <typename T>
const Image<Eigen::Vector2i>& PatchMatch<T>::GetNNF() const
{
	return mNNF;
}

template <typename T>
const Image<float>& PatchMatch<T>::GetNND() const
{
	return mNND;
}
template <typename T>
const Image<T>& PatchMatch<T>::GetReconstructedDst() const
{
	return mReconstructedDst;
}

template <typename T>
void PatchMatch<T>::SetDistanceMetric(float(*dist)(const Image<T>* src, const Image<int>* sMask, const Image<T>* dst, const Image<int>* dMask, const Eigen::Vector2i& sIdx, const Eigen::Vector2i& dIdx, int patchSize, int causalType))
{
	mDistMetric = dist;
}
template <typename T>
void PatchMatch<T>::initialize(int causalType)
{
	srand(0);
	int dstHeight = mDstImg->Height();
	int dstWidth = mDstImg->Width();

	int srcHeight = mSrcImg->Height();
	int srcWidth = mSrcImg->Width();

	mNNF.Create(dstHeight, dstWidth);
	mNND.Create(dstHeight, dstWidth);
	for (int i = 0; i < dstHeight; ++i)
	{
		for (int j = 0; j < dstWidth; ++j)
		{
			while (true)
			{
				int randI = rand() % srcHeight;
				int randJ = rand() % srcWidth;
				if (!mSrcMask || (*mSrcMask)[randI][randJ] == MASK_KNOWN)
				{
					mNNF[i][j] = Eigen::Vector2i(randI, randJ);
					mNND[i][j] = mDistMetric(mSrcImg, mSrcMask, mDstImg, mDstMask, Eigen::Vector2i(randI, randJ), Eigen::Vector2i(i, j), mPatchSize, causalType);
				}
				break;
			}
		}
	}
}
template <typename T>
void PatchMatch<T>::propagate(int ithIteration, int i, int j, int causalType)
{
	if (ithIteration % 2 == 0) // scan-line order
	{
		int srcHeight = mSrcImg->Height();
		int srcWidth = mSrcImg->Width();

		float currentDist = mNND[i][j];
		if (j - 1 >= 0 && mNNF[i][j - 1][1] + 1 < srcWidth)
		{
			float distFromLeft = mDistMetric(mSrcImg, mSrcMask, mDstImg, mDstMask, mNNF[i][j - 1] + Eigen::Vector2i(0, 1), Eigen::Vector2i(i, j), mPatchSize, causalType);
			if (distFromLeft < currentDist)
			{
				currentDist = distFromLeft;
				mNNF[i][j] = mNNF[i][j - 1] + Eigen::Vector2i(0, 1);
				mNND[i][j] = distFromLeft;
			}
		}
		if (i - 1 >= 0 && mNNF[i - 1][j][0] + 1 < srcHeight)
		{
			float distanceFromAbove = mDistMetric(mSrcImg, mSrcMask, mDstImg, mDstMask, mNNF[i - 1][j] + Eigen::Vector2i(1, 0), Eigen::Vector2i(i, j), mPatchSize, causalType);
			if (distanceFromAbove < currentDist)
			{
				currentDist = distanceFromAbove;
				mNNF[i][j] = mNNF[i - 1][j] + Eigen::Vector2i(1, 0);
				mNND[i][j] = distanceFromAbove;
			}
		}

	}
	else // reverse scan-line order
	{
		int height = mDstImg->Height();
		int width = mDstImg->Width();

		float currentDist = mNND[i][j];
		if (j + 1 < width && mNNF[i][j + 1][1] - 1 >= 0)
		{
			float distanceFromRight = mDistMetric(mSrcImg, mSrcMask, mDstImg, mDstMask, mNNF[i][j + 1] + Eigen::Vector2i(0, -1), Eigen::Vector2i(i, j), mPatchSize, causalType);
			if (distanceFromRight < currentDist)
			{
				currentDist = distanceFromRight;
				mNNF[i][j] = mNNF[i][j + 1] + Eigen::Vector2i(0, -1);
				mNND[i][j] = distanceFromRight;
			}
		}
		if (i + 1 < height && mNNF[i + 1][j][0] - 1 >= 0)
		{
			float distanceFromBelow = mDistMetric(mSrcImg, mSrcMask, mDstImg, mDstMask, mNNF[i + 1][j] + Eigen::Vector2i(-1, 0), Eigen::Vector2i(i, j), mPatchSize, causalType);
			if (distanceFromBelow < currentDist)
			{
				currentDist = distanceFromBelow;
				mNNF[i][j] = mNNF[i + 1][j] + Eigen::Vector2i(-1, 0);
				mNND[i][j] = distanceFromBelow;
			}
		}
	}
}
template <typename T>
void PatchMatch<T>::randomSearch(int ithIteration, int i, int j, int causalType)
{
	int w = mSrcImg->Width();
	float alpha = 0.5;

	int count = 1;
	while (true)
	{
		float ratio = w * powf(alpha, count);
		if (ratio < 1) break;

		int randOffsetI = RandDouble(-1, 1) * ratio;
		int randOffsetJ = RandDouble(-1, 1) * ratio;

		int randI = mNNF[i][j][0] + randOffsetI;
		int randJ = mNNF[i][j][1] + randOffsetJ;
		randI = Clamp<int>(randI, 0, mSrcImg->Height() - 1);
		randJ = Clamp<int>(randJ, 0, mSrcImg->Width() - 1);

		float distanceFromRand = mDistMetric(mSrcImg, mSrcMask, mDstImg, mDstMask, Eigen::Vector2i(randI, randJ), Eigen::Vector2i(i, j), mPatchSize, causalType);
		if (distanceFromRand < mNND[i][j])
		{
			mNNF[i][j] = Eigen::Vector2i(randI, randJ);
			mNND[i][j] = distanceFromRand;
		}
		count++;
	}
}

template <typename T>
void PatchMatch<T>::reconstruct()
{
	int numRows = mDstImg->Height();
	int numCols = mDstImg->Width();
	mReconstructedDst.Create(numRows, numCols);
	for (int i = 0; i < numRows; ++i)
	{
		for (int j = 0; j < numCols; ++j)
		{
			mReconstructedDst[i][j] = (*mSrcImg)[mNNF[i][j][0]][mNNF[i][j][1]];
		}
	}

}