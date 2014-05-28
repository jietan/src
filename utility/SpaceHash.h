#ifndef _SPACE_HASH_H
#define _SPACE_HASH_H

#include "stdafx.h"

class HashData
{
public:
	HashData();
	HashData(const VectorXd& data, int count = 1);
	~HashData();

	void Combine(const VectorXd& newData);

	VectorXd mData;
	int mCount;

};

class SpaceHash
{
public:
	SpaceHash();
	~SpaceHash();
	void AddData(const VectorXd& data);
	void AddData(const vector<VectorXd>& data);
	void AddDataAndCombine(const VectorXd& data, double epsilon = 1e-6);
	void AddDataAndCombine(const vector<VectorXd>& data, double epsilon = 1e-6);
	vector<VectorXd> GetData() const;
private:
	vector<vector<HashData> > mBuckets;

	int hashFunc1(const VectorXd& data) const;
};
#endif