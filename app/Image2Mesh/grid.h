#ifndef _GRID_H
#define _GRID_H

template<typename type>
class Grid
{
public:
	Grid()
	{

	}
	Grid(const Vector3f& minPoint, const Vector3f& maxPoint, const Vector3i& resolution)
	{
		SetResolutionInfo(minPoint, maxPoint, resolution);
	}
	void SetResolutionInfo(const Vector3f& minPoint, const Vector3f& maxPoint, const Vector3i& resolution)
	{
		mMinPoint = minPoint;
		mMaxPoint = maxPoint;
		mResolution = resolution;
		for (int i = 0; i < 3; ++i)
			mGridSize[i] = (mMaxPoint[i] - mMinPoint[i]) / mResolution[i];
		mData.resize(resolution[0]);
		for (int i = 0; i < resolution[0]; ++i)
		{
			mData[i].resize(resolution[1]);
			for (int j = 0; j < resolution[1]; ++j)
			{
				mData[i][j].resize(resolution[2]);
			}
		}
	}
	const Vector3i& GetResolution() const
	{
		return mResolution;
	}
	~Grid()
	{

	}
	bool Pos2Idx(const Vector3f& pos, int&i, int& j, int& k)
	{
		Vector3f OffsetFromMin = pos - mMinPoint;
		i = static_cast<int>(OffsetFromMin[0] / mGridSize[0]);
		j = static_cast<int>(OffsetFromMin[1] / mGridSize[1]);
		k = static_cast<int>(OffsetFromMin[2] / mGridSize[2]);
		if (i >= 0 && i < mResolution[0] && j >= 0 && j < mResolution[1] && k >= 0 && k < mResolution[2])
			return true;
		else
			return false;
	}
	type& Data(int i, int j, int k)
	{
		return mData[i][j][k];
	}

private:
	vector<vector<vector<type> > > mData;
	Vector3f mMinPoint;
	Vector3f mMaxPoint;
	Vector3i mResolution;
	Vector3f mGridSize;
};

#endif