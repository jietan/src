#include "MultilayerMaskImage.h"
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
