#include "stdafx.h"
#include "MatrixSolver.h"

void MatrixSolver::SetMatrix(CSRMatrix* m)
{
	mMatrix = m;
}

void MatrixSolver::SetRHS(double* rhs, int nCols)
{
	mRHS = rhs;
	mNumRhs = nCols;
}