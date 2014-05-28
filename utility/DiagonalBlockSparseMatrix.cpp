#include "stdafx.h"
#include "DiagonalBlockSparseMatrix.h"

DiagonalBlockSparseMatrix::DiagonalBlockSparseMatrix()
{

}
DiagonalBlockSparseMatrix::~DiagonalBlockSparseMatrix()
{

}
void DiagonalBlockSparseMatrix::SetToZero()
{
	mDiagElements.clear();
}
int DiagonalBlockSparseMatrix::GetNumRows() const
{
	int numRows = 0;
	int numElements = static_cast<int>(mDiagElements.size());
	for (int i = 0; i < numElements; ++i)
	{
		numRows += mDiagElements[i].GetNumRows();
	}
	return numRows;
}
int DiagonalBlockSparseMatrix::GetNumCols() const
{
	int numCols = 0;
	int numElements = static_cast<int>(mDiagElements.size());
	for (int i = 0; i < numElements; ++i)
	{
		numCols += mDiagElements[i].GetNumCols();
	}
	return numCols;

}
void DiagonalBlockSparseMatrix::Solve(const VectorXd& rhs, VectorXd& sol)
{
	int numRows = GetNumRows();
	int numCols = GetNumCols();
	LOG_IF(FATAL, numRows != numCols) << "DiagonalBlockSparseMatrix is not a square matrix and noninvertible.";
	int vecLength = rhs.size();
	LOG_IF(FATAL, numCols != vecLength) << "DiagonalBlockSparseMatrix and right hand side vector are of different dimensions.";

	sol.resize(numCols);
	int numUntilLast = 0;
	int numDiagElements = static_cast<int>(mDiagElements.size());
	for (int i = 0; i < numDiagElements; ++i)
	{
		int numColsElement = mDiagElements[i].GetNumCols();	
		VectorXd partSol(numColsElement);
		VectorXd partRhs = rhs.segment(numUntilLast, numColsElement);
#if LLT_SOLVE
		mDiagElements[i].LltSolve(partRhs, partSol);
#else
		mDiagElements[i].ConjugateGradientSolve(partRhs, partSol);
#endif		
		sol.segment(numUntilLast, numColsElement) = partSol;
		numUntilLast += numColsElement;
	}
}
void DiagonalBlockSparseMatrix::Solve(const MatrixXd& rhs, MatrixXd& sol)
{
	int numRows = GetNumRows();
	int numCols = GetNumCols();
	LOG_IF(FATAL, numRows != numCols) << "DiagonalBlockSparseMatrix is not a square matrix and noninvertible.";
	int rhsCols = rhs.cols();
	int rhsRows = rhs.rows();
	LOG_IF(FATAL, numCols != rhsRows) << "DiagonalBlockSparseMatrix and right hand side matrix are of different dimensions.";

	sol.resize(rhsRows, rhsCols);
	int numUntilLast = 0;
	int numDiagElements = static_cast<int>(mDiagElements.size());
	for (int i = 0; i < numDiagElements; ++i)
	{
		int numColsElement = mDiagElements[i].GetNumCols();	
		MatrixXd partSol(numColsElement, rhsCols);
		MatrixXd partRhs = rhs.block(numUntilLast, 0, numColsElement, rhsCols);
		
#if LLT_SOLVE
		mDiagElements[i].LltSolve(partRhs, partSol);
#else
		mDiagElements[i].ConjugateGradientSolve(partRhs, partSol);
#endif
		sol.block(numUntilLast, 0, numColsElement, rhsCols) = partSol;
		numUntilLast += numColsElement;
	}
}

void DiagonalBlockSparseMatrix::AddDiagElement(BlockSparseMatrix element)
{
	mDiagElements.push_back(element);
}