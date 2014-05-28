#ifndef _JIE_BLAS_H
#define _JIE_BLAS_H

#include "stdafx.h"

void myblas_dcopy(int dim, double* src, double* dest);
void myblas_daxpy(int dim, double alpha, double* x, double* y);  //y = alpha * x + y;
void myblas_dscal(int dim, double alpha, double* x); //x = alpha * x;

void myblas_dcoogemv(int nRows, double* value, int* rowInd, int* colInd, int nnz, double* x, double* y); // y = A * x
void myblas_solve(int nRows, int nCols, double* value, int* rowInd, int* colInd, int nnz, double* b, double* x); // x = b\A
void myblas_solveUpTriangle(int nRows, int nCols, double* mat, double* b, double* x, int* sortedInd = NULL);
void myblas_jacobi(int nRows, int nCols, double* value, int* rowInd, int* colInd, int nnz, double* b, double* x, int iterTimes = 100);

#endif