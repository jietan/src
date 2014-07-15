#ifndef _GENERIC_IMAGE_H
#define _GENERIC_IMAGE_H



#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>
using namespace std;

template <typename T>
class Image
{
public:
	Image() : mWidth(0), mHeight(0)
	{

	}
	virtual ~Image()
	{

	}
	vector<T>& operator[] (int i)
	{
		return mData[i];
	}

	const vector<T>& operator[] (int i) const
	{
		return mData[i];
	}
	virtual void Create(int nRows, int nCol)
	{
		mData.clear();
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
	virtual void DownSample()
	{
		int newWidth = mWidth / 2;
		int newHeight = mHeight / 2;
		vector<vector<T> > newData;
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
	int mWidth;
	int mHeight;
	vector<vector<T> > mData;
};

#endif 