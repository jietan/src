#ifndef _MULTI_LAYER_IMAGE_H
#define _MULTI_LAYER_IMAGE_H

#include <vector>
using namespace std;

template <typename T>
class MultilayerImage
{
public:
	MultilayerImage() : mWidth(0),
	mHeight(0),
	mLayers(0)
	{

	}
	virtual ~MultilayerImage()
	{

	}

	vector<vector<T> >& operator[] (int i)
	{
		return mData[i];
	}

	const vector<vector<T> >& operator[] (int i) const
	{
		return mData[i];
	}
	virtual void Create(int nRows, int nCol)
	{
		mHeight = nRows;
		mWidth = nCol;
		mData.resize(mHeight);
		for (int i = 0; i < mHeight; ++i)
		{
			mData[i].resize(mWidth);
		}
	}
	virtual void Process()
	{
		findNumLayers();
	}
	virtual void Read(const string& filename)
	{

	}
	virtual void Save(const string& filename)
	{

	}
	virtual int Width() const
	{
		return mWidth;
	}
	virtual int Height() const
	{
		return mHeight;
	}
	virtual int NumLayers() const
	{
		return mLayers;
	}
	virtual void DownSample()
	{
		int newWidth = mWidth / 2;
		int newHeight = mHeight / 2;
		vector<vector<vector<T> > > newData;
		newData.resize(newHeight);
		for (int i = 0; i < newHeight; ++i)
		{
			newData[i].resize(newWidth);
			for (int j = 0; j < newWidth; ++j)
			{
				newData[i][j] = mData[i * 2][j * 2];
			}
		}
		mHeight = newHeight;
		mWidth = newWidth;
		mData = newData;
		Process();
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

	int mWidth;
	int mHeight;
	int mLayers;
	vector<vector<vector<T> > > mData;
};

#endif