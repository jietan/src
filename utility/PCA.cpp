#include "stdafx.h"
#include "PCA.h"
#include "utility/EigenSerializer.h"

PCA::PCA() : mDim(0), mNumPoints(0)
{

}

PCA::~PCA()
{

}

void PCA::Analyze(const vector<VectorXd>& data, vector<double>& eigenValues, vector<VectorXd>& basis)
{
	mNumPoints = static_cast<int>(data.size());
	if (!mNumPoints) return;

	mDim = data[0].size();
	mData = MatrixXd::Zero(mDim, mNumPoints);
	for (int i = 0; i < mNumPoints; ++i)
	{
		mData.col(i) = data[i];
	}
	centerData();
	scaleData();
	checkCenterAndScale();
	EigenSerializer::SaveMatrixToFileDouble("..\\bicycle\\pcaData", mData, false);
	performSVD();

	eigenValues = mEigenValues;
	basis = mBasis;
}

void PCA::centerData()
{
	VectorXd sumVector = VectorXd::Zero(mDim);
	for (int i = 0; i < mNumPoints; ++i)
	{
		sumVector += mData.col(i);
	}
	sumVector /= mNumPoints;
	for (int i = 0; i < mNumPoints; ++i)
	{
		mData.col(i) -= sumVector;
	}
}
void PCA::scaleData()
{
	for (int i = 0; i < mDim; ++i)
	{
		double sqrSum = 0;
		for (int j = 0; j < mNumPoints; ++j)
		{
			sqrSum += mData(i, j) * mData(i, j);
		}
		sqrSum /= mNumPoints;
		if (abs(sqrSum) < 1e-20)
		{
			LOG(WARNING) << i << "th dim has variance " << sqrSum;
			continue;
		}
		mData.row(i) /= sqrt(sqrSum);
	}
}

void PCA::checkCenterAndScale()
{
	VectorXd sumVector = VectorXd::Zero(mDim);
	for (int i = 0; i < mNumPoints; ++i)
	{
		sumVector += mData.col(i);
	}
	
	for (int i = 0; i < mDim; ++i)
	{
		double sqrSum = 0;
		for (int j = 0; j < mNumPoints; ++j)
		{
			sqrSum += mData(i, j) * mData(i, j);
		}
		if (abs(sqrSum) > 1e-20)
			CHECK_NEAR(sqrSum, mNumPoints, 1e-6);
		CHECK_NEAR(sumVector(i), 0, 1e-6);
	}
}
void PCA::performSVD()
{
	JacobiSVD<MatrixXd> svd(mData, ComputeThinU | ComputeThinV);
	MatrixXd U = svd.matrixU();
	VectorXd singularValues = svd.singularValues();
	mEigenValues.resize(mDim);
	mBasis.resize(mDim);

	for (int i = 0; i < mDim; ++i)
	{
		mEigenValues[i] = singularValues[i] * singularValues[i] / mNumPoints;
		mBasis[i] = U.col(i);
	}
}
