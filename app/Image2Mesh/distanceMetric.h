#ifndef _DISTANCE_METRIC_H
#define _DISTANCE_METRIC_H

#include "Image.h"
#include <Eigen/Dense>

template <typename T>
float dist1(const Image<T>* src, const Image<int>* sMask, const Image<T>* dst, const Image<int>* dMask, const Eigen::Vector2i& sIdx, const Eigen::Vector2i& dIdx, int patchSize, int causalType)
{
	Eigen::VectorXf elementDist = Eigen::VectorXf::Zero(patchSize * patchSize);

	int numPatchPixels = patchSize * patchSize;
	int halfPatchSize = patchSize / 2;
	int idx = 0;
	int validCount = 0;
	for (int iOffset = -halfPatchSize; iOffset <= halfPatchSize; ++iOffset)
	{
		for (int jOffset = -halfPatchSize; jOffset <= halfPatchSize; ++jOffset)
		{
			if (causalType == SCANLINE_CAUSAL)
			{
				if (idx > numPatchPixels / 2)
				{
					idx++;
					continue;
				}
			}
			else if (causalType == REVERSE_SCANLINE_CAUSAL)
			{
				if (idx < numPatchPixels / 2)
				{
					idx++;
					continue;
				}
			}
			int realISrc = sIdx[0] + iOffset;
			int realJSrc = sIdx[1] + jOffset;

			int realIDst = dIdx[0] + iOffset;
			int realJDst = dIdx[1] + jOffset;

			if (realISrc < 0 || realISrc >= src->Height() || realJSrc < 0 || realJSrc >= src->Width()
			||  realIDst < 0 || realIDst >= dst->Height() || realJDst < 0 || realJDst >= dst->Width())
			{
				elementDist[idx] = 0;
			}
			else if (!sMask || (*sMask)[realISrc][realJSrc] == MASK_KNOWN)
			{
				validCount++;
				elementDist[idx] = ((*src)[realISrc][realJSrc] - (*dst)[realIDst][realJDst]).norm();
			}
			else
			{
				elementDist[idx] = 0;
			}
			idx++;
		}
	}
	if (!validCount)
		return FLT_MAX;
	return elementDist.norm() / validCount;
}

#endif