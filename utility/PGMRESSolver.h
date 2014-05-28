#ifndef _PGMRESSolver_H
#define _PGMRESSolver_H

#include "MatrixSolver.h"

class PGMRESSolver : public MatrixSolver
{
public:
	PGMRESSolver();
	~PGMRESSolver();

	void Solve(double* result, bool usePreconditioner = true);
private:
	void calculatePreconditioner();

	double* mPrecon;
};

#endif