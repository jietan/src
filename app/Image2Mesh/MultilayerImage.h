#ifndef _MULTI_LAYER_IMAGE_H
#define _MULTI_LAYER_IMAGE_H

#include <vector>
using namespace std;

#include "Image.h"

template <typename T>
class MultilayerImage : public Image<vector<T> >
{
public:
	MultilayerImage() : mLayers(0)
	{

	}
	virtual ~MultilayerImage()
	{

	}
	virtual void Process()
	{
		findNumLayers();
	}
	virtual int NumLayers() const
	{
		return mLayers;
	}


protected:

	void findNumLayers()
	{
		mLayers = 0;
		if (mData.empty())
			return;
		for (int i = 0; i < mHeight; ++i)
		{
			for (int j = 0; j < mWidth; ++j)
			{
				if (mLayers < mData[i][j].size())
					mLayers = static_cast<int>(mData[i][j].size());
			}
		}
	}

	int mLayers;
	
};

#endif