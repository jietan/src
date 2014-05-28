#include "stdafx.h"
#include "JieBlas.h"
#include "mathlib.h"

void myblas_dcopy(int dim, double* src, double* dest)
{
	memcpy(dest, src, dim * sizeof(double));
}

void myblas_daxpy(int dim, double alpha, double* x, double* y)  //y = alpha * x + y;
{
	for(int i = 0; i < dim; i++)
	{
		y[i] += alpha * x[i];
	}
}

void myblas_dscal(int dim, double alpha, double* x) //x = alpha * x;
{
	for(int i = 0; i < dim; i++)
	{
		x[i] = alpha * x[i];
	}
}

void myblas_dcoogemv(int nRows, double* value, int* rowInd, int* colInd, int nnz, double* x, double* y) // y = A * x
{
	memset(y, 0, nRows * sizeof(double));
	for(int i = 0; i < nnz; i++)
	{
		y[rowInd[i]] += value[i] * x[colInd[i]]; 
	}
}

void myblas_solve(int nRows, int nCols, double* value, int* rowInd, int* colInd, int nnz, double* b, double* x) // x = b\A
{
	double *tempB = new double[nCols];
	memcpy(tempB, b, nCols * sizeof(double));
	int *sortedInd = new int[nRows];
	double *mat = new double[nRows * nCols];
	memset(mat, 0, nRows * nCols * sizeof(double));
	for(int i = 0; i < nnz; i++)
	{
		mat[nCols * rowInd[i] + colInd[i]] = value[i];
	}
	for(int i = 0; i < nRows; i++)
	{
		sortedInd[i] = i*nCols;
	}
	for(int i = 0; i < nRows; i++)
	{
		int maxInd = i;
		for(int j = i + 1; j < nRows; j++)
		{
			if (abs(mat[sortedInd[j] + i]) > abs(mat[sortedInd[maxInd] + i]))
			{
				maxInd = j;
			}
		}
		int temp = sortedInd[i];
		sortedInd[i] = sortedInd[maxInd];
		sortedInd[maxInd] = temp;
		int indFirst = sortedInd[i];
		for (int j = i + 1; j < nRows; j++)
		{
			int indEnd = sortedInd[j];
			double coeff = mat[indEnd + i] / mat[indFirst + i]; 
			for(int k = i; k < nCols; k++)
			{
				mat[indEnd + k] -= coeff * mat[indFirst + k];
			}
			tempB[indEnd / nCols] -= coeff * tempB[indFirst / nCols]; 
		}
	}

	/*if(abs(mat[0]) > EPSILON)
	{
		coeff = mat[0];
		for(int i = 0; i < nCols; i++)
			mat[i] /= coeff;
		b[0] /= coeff;
	}
	for(int i = 1; i < nRows; i++)
	{
		for(int j = 0; j < nCols; j++)
		{
			double coeff = mat[i * nCols + j];
			if(j < i && abs(coeff) > EPSILON)
			{
				for(int k = 0; k < nCols; k++)
					mat[i * nCols + k] = mat[i * nCols + k] / coeff - mat[j * nCols + k];
				b[i] = b[i] / coeff - b[j];
			}
			else if(j == i && abs(coeff) > EPSILON)
			{
				for(int k = 0; k < nCols; k++)
					mat[i * nCols + k] /= coeff;
				b[i] /= coeff;
			}
		}
	}*/
	
	myblas_solveUpTriangle(nRows, nCols, mat, tempB, x, sortedInd);
	delete[] sortedInd;
	delete[] mat;
	delete[] tempB;
}

void myblas_solveUpTriangle(int nRows, int nCols, double* mat, double* b, double* x, int* sortedInd)
{
	if(sortedInd == NULL)
	{
		for(int i = nRows - 1; i >= 0; i--)
		{
			x[i] = b[i];
			for(int j = nCols - 1; j > i; j--)
			{
				x[i] -= x[j] * mat[i * nCols + j];
			}
			x[i] /= mat[i * nCols + i];
		}
	}
	else
	{
		for(int i = nRows - 1; i >= 0; i--)
		{
			int index = sortedInd[i] / nCols;
			x[i] = b[index];
			for(int j = nCols - 1; j > i; j--)
			{
				int index2 = sortedInd[j] / nCols;
				x[i] -= x[j] * mat[index * nCols + j];
			}
			x[i] /= mat[index * nCols + i];
		}
	}
}

void myblas_jacobi(int nRows, int nCols, double* value, int* rowInd, int* colInd, int nnz, double* b, double* x, int iterTimes)
{
	double *y = new double[nRows * 2];
	double *D = &y[nRows];
	double *cur = x;
	double *next = y;
	double *temp;
	double epsilon;
	memset(x, 0, nRows * sizeof(double));
	memset(y, 0, nRows * sizeof(double));
	for(int i = 0; i < nnz; i++)
	{
		if(rowInd[i] == colInd[i])
		{
			D[rowInd[i]] = value[i];
		}
	}
	while(iterTimes > 0)
	{
		epsilon = 0;
		for(int i = 0; i < nnz; i++)
		{
			if(rowInd[i] != colInd[i])
			{
				next[rowInd[i]] += value[i] * cur[colInd[i]];
			}
		}
		for(int i = 0; i < nRows; i++)
		{
			next[i] = (b[i] - next[i]) / D[i];
			epsilon += abs(next[i] - cur[i]);
		}
		if(epsilon < EPSILON_FLOAT)
			break;
		temp = cur;
		cur = next;
		next = temp;
		memset(next, 0, nRows * sizeof(double));
		iterTimes --;
	}
	if(next != x)
		memcpy(x, next, nRows * sizeof(double));
	
	delete[] y;
}