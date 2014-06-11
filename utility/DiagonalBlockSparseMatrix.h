#ifndef _DIAGONAL_BLOCK_SPARSE_MATRIX_H
#define _DIAGONAL_BLOCK_SPARSE_MATRIX_H

#include "Eigen/Dense"

#include <vector>
using namespace std;

#include "BlockSparseMatrix.h"

typedef vector<BlockSparseMatrix> BlockSparseMatrixList;

class DiagonalBlockSparseMatrix
{
public:
	DiagonalBlockSparseMatrix();
	~DiagonalBlockSparseMatrix();
	void SetToZero();
	int GetNumRows() const;
	int GetNumCols() const;
	void Solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& sol);
	void Solve(const Eigen::MatrixXd& rhs, Eigen::MatrixXd& sol);
	void AddDiagElement(BlockSparseMatrix element);

private:
	BlockSparseMatrixList mDiagElements;
};

#endif