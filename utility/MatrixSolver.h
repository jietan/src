#ifndef _MatrixSolver_H
#define _MatrixSolver_H

#include "CSRMatrix.h"

class MatrixSolver
{
public:
	MatrixSolver() : mErrorThreshold(1e-20)
	{
	};
	virtual~MatrixSolver()
	{

	}
	virtual void SetMatrix(CSRMatrix* m);
	virtual void SetRHS(double* rhs, int nCols = 1);
	virtual void Solve(double* result, bool usePreconditioner = true) = 0;
protected:
	CSRMatrix* mMatrix;
	double* mRHS;
	int mNumRhs;
	double mErrorThreshold;

};

#endif