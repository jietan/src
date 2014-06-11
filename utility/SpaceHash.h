#ifndef _SPACE_HASH_H
#define _SPACE_HASH_H

#include "stdafx.h"

class HashData
{
public:
	HashData();
	HashData(const Eigen::VectorXd& data, int count = 1);
	~HashData();

	void Combine(const Eigen::VectorXd& newData);

	Eigen::VectorXd mData;
	int mCount;

};

class SpaceHash
{
public:
	SpaceHash();
	~SpaceHash();
	void AddData(const Eigen::VectorXd& data);
	void AddData(const vector<Eigen::VectorXd>& data);
	void AddDataAndCombine(const Eigen::VectorXd& data, double epsilon = 1e-6);
	void AddDataAndCombine(const vector<Eigen::VectorXd>& data, double epsilon = 1e-6);
	vector<Eigen::VectorXd> GetData() const;
private:
	vector<vector<HashData> > mBuckets;

	int hashFunc1(const Eigen::VectorXd& data) const;
};
#endif