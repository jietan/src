#include "stdafx.h"
#include "BlockSparseMatrix.h"
#include "PCGSolver.h"
#include "CSRMatrix.h"

#ifdef _MKL_IMPLEMENT
#include <mkl_cblas.h>
#include <mkl_spblas.h>
#endif

BlockSparseMatrix::BlockSparseMatrix() :
mBlockWidth(0),
mBlockHeight(0),
mNumBlocksWidth(0),
mNumBlocksHeight(0),
mbCSRDirty(true),
mbLLTDirty(true)
{

}

BlockSparseMatrix::BlockSparseMatrix(int blockWidth, int blockHeight, int numBlocksWidth, int numBlocksHeight) :
mBlockWidth(blockWidth),
mBlockHeight(blockHeight),
mNumBlocksWidth(numBlocksWidth),
mNumBlocksHeight(numBlocksHeight),
mbCSRDirty(true),
mbLLTDirty(true)
{
	mMatrix = Eigen::MatrixXd::Zero(blockHeight * numBlocksHeight, blockWidth * numBlocksWidth);

//	int totalBlocks = mNumBlocksWidth * mNumBlocksHeight;
	//mBlockMatrices.resize(totalBlocks);
	//for (int i = 0; i < totalBlocks; ++i)
	//{
	//	mBlockMatrices[i] = Eigen::MatrixXd(blockHeight, blockWidth);
	//	mBlockMatrices[i] = Eigen::MatrixXd::Zero(blockHeight, blockWidth);
	//}
}

void BlockSparseMatrix::PrintDiagonolElement() const
{
	
	for (int i = 0; i < mMatrix.rows(); ++i)
	{
		LOG(INFO) << mMatrix(i, i);
	}
}

void BlockSparseMatrix::PrintMatlabMatrix() const
{
	ofstream toPrint("matlabMatrix.m");
	toPrint << "A = [";
	for (int i = 0; i < mMatrix.rows(); ++i)
	{
		for (int j = 0; j < mMatrix.cols(); ++j)
		{
			toPrint << mMatrix(j, i) << " ";
		}
		toPrint << ";\n";
	}
	toPrint << "];";
	toPrint.flush();
}

void BlockSparseMatrix::SetBlockInfo(int blockWidth, int blockHeight, int numBlocksWidth, int numBlocksHeight)
{
	mBlockWidth = blockWidth;
	mBlockHeight = blockHeight;
	mNumBlocksWidth = numBlocksWidth;
	mNumBlocksHeight = numBlocksHeight;
	mbCSRDirty = true;
	mbLLTDirty = true;
//	mBlockMatrices.clear();
	mNonzeroIndices.clear();

	int totalBlocks = mNumBlocksWidth * mNumBlocksHeight;
//	mBlockMatrices.resize(totalBlocks);
	mNonzeroIndices.resize(totalBlocks);
	for (int i = 0; i < totalBlocks; ++i)
	{
		mNonzeroIndices[i] = 0;
		//mBlockMatrices[i] = Eigen::MatrixXd(blockHeight, blockWidth);
		//mBlockMatrices[i] = Eigen::MatrixXd::Zero(blockHeight, blockWidth);
	}
	mMatrix = Eigen::MatrixXd::Zero(blockHeight * numBlocksHeight, blockWidth * numBlocksWidth);

}

int BlockSparseMatrix::GetNumRows() const
{
	return mNumBlocksHeight * mBlockHeight;
}
int BlockSparseMatrix::GetNumCols() const
{
	return mNumBlocksWidth * mBlockWidth;
}

void BlockSparseMatrix::SetToZero()
{
	for (int ithBlock = 0; ithBlock < mNumBlocksHeight; ++ithBlock)
	{
		for (int jthBlock = 0; jthBlock < mNumBlocksWidth; ++jthBlock)
		{
			int index = indexTo1D(ithBlock, jthBlock);
			if (mNonzeroIndices[index])
			{
				mNonzeroIndices[index] = 0;
//				mBlockMatrices[index] = Eigen::MatrixXd::Zero(mBlockHeight, mBlockWidth);
			}
		}
		
	}
	//for (int i = 0; i < mNonzeroIndices.size(); ++i)
	//{
	//	mNonzeroIndices[i] = 0;
	//}
	mMatrix = Eigen::MatrixXd::Zero(mBlockHeight * mNumBlocksHeight, mBlockWidth * mNumBlocksWidth);
}

bool BlockSparseMatrix::IsNonZeroBlock(int ithBlock, int jthBlock) const
{
	return (mNonzeroIndices[indexTo1D(ithBlock, jthBlock)] == 1);
}

Eigen::MatrixXd BlockSparseMatrix::GetBlockMatrix(int ithBlock, int jthBlock) const
{
	return mMatrix.block(ithBlock * mBlockHeight, jthBlock * mBlockWidth, mBlockHeight, mBlockWidth);
	//int index = indexTo1D(ithBlock, jthBlock);
	//return mBlockMatrices[index];
}
void BlockSparseMatrix::SetBlockMatrix(int ithBlock, int jthBlock, const Eigen::MatrixXd& mat)
{
	int index = indexTo1D(ithBlock, jthBlock);
	//mBlockMatrices[index] = mat;
	mMatrix.block(ithBlock * mBlockHeight, jthBlock * mBlockWidth, mBlockHeight, mBlockWidth) = mat;
	mNonzeroIndices[index] = 1;
	mbCSRDirty = true;
	mbLLTDirty = true;
}

double BlockSparseMatrix::operator() (int i, int j) const
{
	return mMatrix(i, j);
}

BlockSparseMatrix BlockSparseMatrix::operator+ (const BlockSparseMatrix& rhs)
{
    BlockSparseMatrix ret = *this;
    ret += rhs;
    return ret;
}

void BlockSparseMatrix::AddBlockMatrixTo(int ithBlock, int jthBlock, const Eigen::MatrixXd& mat)
{
	int index = indexTo1D(ithBlock, jthBlock);
	//mBlockMatrices[index] += mat;
	mMatrix.block(ithBlock * mBlockHeight, jthBlock * mBlockWidth, mBlockHeight, mBlockWidth) += mat;
	mNonzeroIndices[indexTo1D(ithBlock, jthBlock)] = 1;
	mbCSRDirty = true;
	mbLLTDirty = true;
}

BlockSparseMatrix& BlockSparseMatrix::operator+= (const BlockSparseMatrix& rhs)
{
	for (int ithBlock = 0; ithBlock < mNumBlocksHeight; ++ithBlock)
		for (int jthBlock = 0; jthBlock < mNumBlocksWidth; ++jthBlock)
		{
			int index = indexTo1D(ithBlock, jthBlock);
			if (mNonzeroIndices[index] || rhs.mNonzeroIndices[index])
			{
//				mBlockMatrices[index] += rhs.mBlockMatrices[index];
				mNonzeroIndices[index] = 1;
			}
		}
	mMatrix += rhs.mMatrix;
	mbCSRDirty = true;
	mbLLTDirty = true;
	return *this;
}

BlockSparseMatrix& BlockSparseMatrix::operator*= (double alpha)
{
	//for (int ithBlock = 0; ithBlock < mNumBlocksHeight; ++ithBlock)
	//	for (int jthBlock = 0; jthBlock < mNumBlocksWidth; ++jthBlock)
	//	{	
	//		int index = indexTo1D(ithBlock, jthBlock);
	//		if (mNonzeroIndices[index])
	//			mBlockMatrices[index] *= alpha;
	//	}
	mMatrix *= alpha;
	mbCSRDirty = true;
	mbLLTDirty = true;
	return *this;
}



void BlockSparseMatrix::UpdateCSR()
{
	if (mbCSRDirty)
	{
		mCSREquivalent.ConvertFromBlockSparseMatrix(*this);
		
		mbCSRDirty = false;
	}
}

bool BlockSparseMatrix::CheckSolution(const Eigen::VectorXd& rhs, const Eigen::VectorXd& sol)
{
	Eigen::VectorXd solvedRhs = (*this) * sol;
	Eigen::VectorXd error = solvedRhs - rhs;
	for (int i = 0; i < error.size(); ++i)
	{
		if (abs(error[i]) > 1e-6)
			return false;
	}
	return true;
}

Eigen::VectorXd BlockSparseMatrix::operator* (const Eigen::VectorXd& rhs) const
{
	CHECK(!mbCSRDirty);
	int vectorLength = static_cast<int>(rhs.size());
	double* rhsData = new double[vectorLength];
	double* result = new double[vectorLength];
	memcpy(rhsData, rhs.data(), vectorLength * sizeof(double));
	char trans = 'n';
	int numRows = mBlockHeight * mNumBlocksHeight;

#ifdef _MKL_IMPLEMENT
	mkl_dcsrgemv(&trans, &numRows, const_cast<double*>(mCSREquivalent.GetValueData()), const_cast<int*>(mCSREquivalent.GetRowId()), const_cast<int*>(mCSREquivalent.GetColumnId()), rhsData, result);
#else 
	CHECK(0) << "_MKL_IMPLEMENT not defined!";
#endif
	
	Eigen::VectorXd ret(vectorLength);
	for (int i = 0; i < vectorLength; ++i)
		ret[i] = result[i];
	delete[] rhsData;
	delete[] result;
	return ret;
}

Eigen::MatrixXd BlockSparseMatrix::operator * (const Eigen::MatrixXd& rhs) const
{
//	Eigen::MatrixXd eigenMat = ConvertToEigenMatrix();
	return mMatrix * rhs;
}

void BlockSparseMatrix::UpdateLLT()
{
	//if (mbLLTDirty)
	//{
	//	mLLTDecomp = mMatrix.llt();
	//	//mMatrixInv = mMatrix.inverse();
	//	mbLLTDirty = false;
	//}
}

bool BlockSparseMatrix::LltSolve(const Eigen::VectorXd& rhs, Eigen::VectorXd& sol)
{

	sol = mMatrix.llt().solve(rhs);
	//if (mbLLTDirty)
	//{
	//	mLLTDecomp = mMatrix.llt();
	//	//mMatrixInv = mMatrix.inverse();
	//	mbLLTDirty = false;
	//}
	//
	//sol = mLLTDecomp.solve(rhs);
	//sol = mMatrixInv * rhs;
	return true;
}

bool BlockSparseMatrix::LltSolve(const Eigen::MatrixXd& rhs, Eigen::MatrixXd& sol)
{

	sol = mMatrix.llt().solve(rhs);
	//if (mbLLTDirty)
	//{
	//	//		mLLTDecomp = mMatrix.llt();
	//	mMatrixInv = mMatrix.inverse();
	//	mbLLTDirty = false;
	//}

	////sol = mLLTDecomp.solve(rhs);
	//sol = mMatrixInv * rhs;
	return true;
}


bool BlockSparseMatrix::ConjugateGradientSolve(const Eigen::VectorXd& rhs, Eigen::VectorXd& sol)
{
	//sol = mMatrix.llt().solve(rhs);
	return LltSolve(rhs, sol);

	MatrixSolver* solver = new PCGSolver();
	int vectorLength = static_cast<int>(rhs.size());
	double* result = new double[vectorLength];
	double* rhsData = new double[vectorLength];
	memcpy(rhsData, rhs.data(), vectorLength * sizeof(double));

	if (mbCSRDirty)
	{
		mCSREquivalent.ConvertFromBlockSparseMatrix(*this);
		mbCSRDirty = false;
	}

	solver->SetMatrix(&mCSREquivalent);
	solver->SetRHS(rhsData);
	solver->Solve(result, true);
	sol.resize(vectorLength);
	for (int i = 0; i < vectorLength; ++i)
		sol[i] = result[i];

	delete[] rhsData;
	delete[] result;
	delete solver;
	return true;
}

bool BlockSparseMatrix::ConjugateGradientSolve(const Eigen::MatrixXd& rhs, Eigen::MatrixXd& sol)
{
	return LltSolve(rhs, sol);

	MatrixSolver* solver = new PCGSolver();
	int numRows = rhs.rows();
	int numCols = rhs.cols();
	int num = numRows * numCols;
	double* result = new double[num];
	double* rhsData = new double[num];
	int ithElem = 0;
	for (int i = 0; i < numCols; ++i)
	{
		for (int j = 0; j < numRows; ++j)
		{
			rhsData[ithElem++] = rhs(j, i);
		}
	}

	if (mbCSRDirty)
	{
		mCSREquivalent.ConvertFromBlockSparseMatrix(*this);
		mbCSRDirty = false;
	}

	solver->SetMatrix(&mCSREquivalent);
	solver->SetRHS(rhsData, numCols);
	solver->Solve(result, true);
	sol = Eigen::MatrixXd::Zero(numRows, numCols);
	ithElem = 0;
	for (int i = 0; i < numCols; ++i)
	{
		for (int j = 0; j < numRows; ++j)
		{
			sol(j, i) = result[ithElem++];
		}
	}

	delete[] rhsData;
	delete[] result;
	delete solver;
	return true;
}

//void BlockSparseMatrix::ShrinkRowBlock(int ithRowBlock)
//{
//	int newNumRows = mMatrix.rows() - mBlockHeight;
//	int newNumCols = mMatrix.cols();
//	int numRowsToMove = newNumRows - (ithRowBlock) * mBlockHeight;
//	if (numRowsToMove)
//		mMatrix.block(ithRowBlock * mBlockHeight, 0, numRowsToMove, newNumCols) = mMatrix.block((ithRowBlock + 1) * mBlockHeight, 0, numRowsToMove, newNumCols);
//	mMatrix.resize(newNumRows, newNumCols);
//	Eigen::MatrixXi nonZeroIndices(mNumBlocksHeight, mNumBlocksWidth);
//	for (int i = 0; i < mNumBlocksHeight; ++i)
//	{
//		for (int j = 0; j < mNumBlocksWidth; ++j)
//		{
//			nonZeroIndices(i, j) = mNonzeroIndices[indexTo1D(i, j)];
//		}
//	}
//	if (numRowsToMove)
//		nonZeroIndices.block(ithRowBlock, 0, numRowsToMove / mBlockHeight, newNumCols / mBlockWidth) = nonZeroIndices.block((ithRowBlock + 1), 0, numRowsToMove / mBlockHeight, newNumCols / mBlockWidth);
//	mNonzeroIndices.clear();
//	mNumBlocksHeight--;
//
//	mNonzeroIndices.resize(mNumBlocksHeight * mNumBlocksWidth);
//	for (int i = 0; i < mNumBlocksHeight; ++i)
//	{
//		for (int j = 0; j < mNumBlocksWidth; ++j)
//		{
//			mNonzeroIndices[indexTo1D(i, j)] = nonZeroIndices(i, j);
//		}
//	}
//	mbCSRDirty = true;
//	mbLLTDirty = true;
//
//}
//void BlockSparseMatrix::ShrinkColBlock(int ithColBlock)
//{
//	int newNumRows = mMatrix.rows();
//	int newNumCols = mMatrix.cols() - mBlockWidth;
//	int numColsToMove = newNumCols - (ithColBlock) * mBlockWidth;
//	if (numColsToMove)
//		mMatrix.block(0, ithColBlock * mBlockWidth, newNumRows, numColsToMove) = mMatrix.block(0, (ithColBlock + 1) * mBlockWidth, newNumRows, numColsToMove);
//	mMatrix.resize(newNumRows, newNumCols);
//	Eigen::MatrixXi nonZeroIndices(mNumBlocksHeight, mNumBlocksWidth);
//	for (int i = 0; i < mNumBlocksHeight; ++i)
//	{
//		for (int j = 0; j < mNumBlocksWidth; ++j)
//		{
//			nonZeroIndices(i, j) = mNonzeroIndices[indexTo1D(i, j)];
//		}
//	}
//	if (numColsToMove)
//		nonZeroIndices.block(0, ithColBlock, newNumRows / mNumBlocksHeight, numColsToMove / mNumBlocksWidth) = nonZeroIndices.block(0, (ithColBlock + 1), newNumRows / mNumBlocksHeight, numColsToMove / mNumBlocksWidth);
//	mNonzeroIndices.clear();
//	mNumBlocksWidth--;
//	mNonzeroIndices.resize(mNumBlocksHeight * mNumBlocksWidth);
//
//	for (int i = 0; i < mNumBlocksHeight; ++i)
//	{
//		for (int j = 0; j < mNumBlocksWidth; ++j)
//		{
//			mNonzeroIndices[indexTo1D(i, j)] = nonZeroIndices(i, j);
//		}
//	}
//
//	mbCSRDirty = true;
//	mbLLTDirty = true;
//}

void BlockSparseMatrix::ConvertFromEigenMatrix(const MatrixXd& mat)
{
    mMatrix = mat;
    CHECK(mat.rows() == mNumBlocksHeight * mBlockHeight && mat.cols() == mNumBlocksWidth * mBlockWidth);
    for (int ithRow = 0; ithRow < mat.rows(); ++ithRow)
    {
        for (int jthCol = 0; jthCol < mat.cols(); ++jthCol)
        {
            if (mat(ithRow, jthCol) != 0.0)
            {
                int ithBlock = ithRow / mBlockHeight;
                int jthBlock = jthCol / mBlockWidth;
                int index = indexTo1D(ithBlock, jthBlock);
                mNonzeroIndices[index] = 1;
            }
        }
    }
}

void BlockSparseMatrix::CopyCSR(const BlockSparseMatrix& rhs)
{
	mbCSRDirty = rhs.mbCSRDirty;
	mCSREquivalent = rhs.mCSREquivalent;
}

const Eigen::MatrixXd& BlockSparseMatrix::ConvertToEigenMatrix() const
{

	return mMatrix;
}

bool BlockSparseMatrix::operator== (const BlockSparseMatrix& rhs)
{
	return (rhs.mMatrix == mMatrix && mNonzeroIndices == rhs.mNonzeroIndices);
}

std::ostream& operator<< (std::ostream& out, const BlockSparseMatrix& mat)
{
	out << mat.ConvertToEigenMatrix() << std::endl;
	return out;
}

BlockSparseMatrix operator* (double scalar, const BlockSparseMatrix& rhs)
{
    BlockSparseMatrix ret = rhs;
    ret *= scalar;
    return ret;
}
