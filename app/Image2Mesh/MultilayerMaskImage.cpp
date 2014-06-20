#include "MultilayerMaskImage.h"
#include <glog/logging.h>
using namespace google;
#include <fstream>
using namespace std;

MultilayerMaskImage::~MultilayerMaskImage()
{

}

void MultilayerMaskImage::Save(const string& filename)
{
	ofstream outFile(filename.c_str(), ios::out | ios::binary);
	outFile.write((char*)&mWidth, sizeof(mWidth));
	outFile.write((char*)&mHeight, sizeof(mHeight));

	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			int len = static_cast<int>(mData[i][j].size());
			outFile.write((char*)&len, sizeof(len));
			for (int k = 0; k < len; ++k)
			{
				outFile.write((char*)&(mData[i][j][k]), sizeof(int));
			}
		}
	}
}

void MultilayerMaskImage::Read(const string& filename)
{
	int maskValue;
	ifstream inFile(filename.c_str(), ios::in | ios::binary);
	inFile.read((char*)&mWidth, sizeof(mWidth));
	inFile.read((char*)&mHeight, sizeof(mHeight));
	Create(mHeight, mWidth);
	
	for (int i = 0; i < mHeight; ++i)
	{
		for (int j = 0; j < mWidth; ++j)
		{
			int len = 0;
			inFile.read((char*)&len, sizeof(int));
			if (len)
			{
				for (int k = 0; k < len; ++k)
				{
					inFile.read((char*)&maskValue, sizeof(int));
					mData[i][j].push_back(maskValue);
				}
			}
		}
	}
}

void MultilayerMaskImage::Filter()
{
	Process();
	vector<vector<int> > filteredResult;
	filteredResult.resize(mHeight);
	for (int i = 0; i < mHeight; ++i)
	{
		filteredResult[i].resize(mWidth);
	}
	for (int ithLayer = 0; ithLayer < mLayers; ++ithLayer)
	{
		for (int i = 0; i < mHeight; ++i)
		{
			for (int j = 0; j < mWidth; ++j)
			{
				if (mData[i][j].size() <= ithLayer)
					continue;
				int count = 0;
				int vote = 0;
				for (int offsetI = -1; offsetI <= 1; ++offsetI)
				{
					for (int offsetJ = -1; offsetJ <= 1; ++offsetJ)
					{
						if (i + offsetI < 0 || i + offsetI >= mHeight || j + offsetJ < 0 || j + offsetJ >= mWidth)
							continue;
						if (mData[i + offsetI][j + offsetJ].size() > ithLayer)
						{
							vote += mData[i + offsetI][j + offsetJ][ithLayer];
							count++;
						}
					}
				}
				CHECK(count > 0) << "Count should always > 0.";
				filteredResult[i][j] = static_cast<int>(static_cast<float>(vote) / count + 0.5);
			}
		}

		for (int i = 0; i < mHeight; ++i)
		{
			for (int j = 0; j < mWidth; ++j)
			{
				if (mData[i][j].size() > ithLayer)
					mData[i][j][ithLayer] = filteredResult[i][j];
			}
		}

	}

}