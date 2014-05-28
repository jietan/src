#include "stdafx.h"
#include "SpaceHash.h"

#define MAX_BUCKET_SIZE 103

HashData::HashData()
{

}
HashData::HashData(const VectorXd& data, int count) : mData(data), mCount(count)
{

}
HashData::~HashData()
{

}


void HashData::Combine(const VectorXd& newData)
{
	//mData = (mData * mCount + newData) / (mCount + 1);
	mCount++;
}
SpaceHash::SpaceHash()
{
	mBuckets.resize(MAX_BUCKET_SIZE);
}
SpaceHash::~SpaceHash()
{

}
void SpaceHash::AddData(const VectorXd& data)
{
	int bucketId = hashFunc1(data);
	mBuckets[bucketId].push_back(HashData(data));
}
void SpaceHash::AddData(const vector<VectorXd>& data)
{
	int numDataEntries = static_cast<int>(data.size());
	for (int i = 0; i < numDataEntries; ++i)
	{
		AddData(data);
	}
}
void SpaceHash::AddDataAndCombine(const VectorXd& data, double epsilon)
{
	int bucketId = hashFunc1(data);
	int prevBucketId = (bucketId + mBuckets.size() - 1) % mBuckets.size();
	int nextBucketId = (bucketId + 1) % mBuckets.size();
	int bucketToCheck[3];
	bucketToCheck[0] = bucketId;
	bucketToCheck[1] = prevBucketId;
	bucketToCheck[2] = nextBucketId;
	for (int ithBucketToCheck = 0; ithBucketToCheck < 3; ++ithBucketToCheck)
	{
		int checkingBucketId = bucketToCheck[ithBucketToCheck];
		int numSamplesInBucket = static_cast<int>(mBuckets[checkingBucketId].size());
		for (int i = 0; i < numSamplesInBucket; ++i)
		{
			if ((data - mBuckets[checkingBucketId][i].mData).norm() < epsilon)
			{
				mBuckets[checkingBucketId][i].Combine(data);
				return;
			}
		}
	}

	mBuckets[bucketId].push_back(data);
}
void SpaceHash::AddDataAndCombine(const vector<VectorXd>& data, double epsilon)
{
	int numDataEntries = static_cast<int>(data.size());
	for (int i = 0; i < numDataEntries; ++i)
	{
		AddDataAndCombine(data[i], epsilon);
	}
}
vector<VectorXd> SpaceHash::GetData() const
{
	vector<VectorXd> ret;
	int numBuckets = static_cast<int>(mBuckets.size());

	for (int i = 0; i < numBuckets; ++i)
	{
		int numEntries = mBuckets[i].size();
		for (int j = 0; j < numEntries; ++j)
		{
			ret.push_back(mBuckets[i][j].mData);
		}
	}
	return ret;
}
int SpaceHash::hashFunc1(const VectorXd& data) const
{
	return static_cast<int>(data.norm()) % mBuckets.size();
}