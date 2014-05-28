#ifndef _PCA_H
#define _PCA_H

#include "stdafx.h"

class PCA
{
public:
	PCA();
	~PCA();

	void Analyze(const vector<VectorXd>& data, vector<double>& eigenValues, vector<VectorXd>& basis);

private:

	void centerData();
	void scaleData();
	void performSVD();
	void checkCenterAndScale();

	int mDim;
	int mNumPoints;
	MatrixXd mData;
	vector<double> mEigenValues;
	vector<VectorXd> mBasis;
};

#endif