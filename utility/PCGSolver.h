#ifndef _PCGSolver_H
#define _PCGSolver_H

#include "stdafx.h"
#include "MatrixSolver.h"


class PCGSolver : public MatrixSolver
{
	friend class ParallelCGMultipleRHS;
public:
	PCGSolver();
	~PCGSolver();

	void Solve(double* result, bool usePreconditioner = true);
private:
	void calculatePreconditioner();
	void solveSingRhs(double* result, bool usePreconditioner, double* rhs = NULL);
	void solveMultipleRhs(double* result, bool usePreconditioner);
	double* mPrecon;
    
};

#endif
