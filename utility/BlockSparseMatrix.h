#ifndef _BLOCK_SPARSE_MATRIX
#define _BLOCK_SPARSE_MATRIX

#include "stdafx.h"
#include <set>
#include <utility>
#include "CSRMatrix.h"


typedef pair<int, int> index2DT;
typedef std::vector<int> indexSetT;

class BlockSparseMatrix
{
	friend class CSRMatrix;
	friend std::ostream& operator<< (std::ostream& out, const BlockSparseMatrix& mat);
    friend BlockSparseMatrix operator* (double scalar, const BlockSparseMatrix& rhs);
    
public:
	BlockSparseMatrix();
	BlockSparseMatrix(int blockWidth, int blockHeight, int numBlocksWidth, int numBlocksHeight);
	void SetBlockInfo(int blockWidth, int blockHeight, int numBlocksWidth, int numBlocksHeight);
	void PrintDiagonolElement() const;
	void PrintMatlabMatrix() const;
	int GetNumRows() const;
	int GetNumCols() const;
	void SetToZero();
	bool IsNonZeroBlock(int ithBlock, int jthBlock) const;
	Eigen::MatrixXd GetBlockMatrix(int ithBlock, int jthBlock) const;
	void SetBlockMatrix(int ithBlock, int jthBlock, const Eigen::MatrixXd& mat);
	void AddBlockMatrixTo(int ithBlock, int jthBlock, const Eigen::MatrixXd& mat);
    BlockSparseMatrix operator+ (const BlockSparseMatrix& rhs);
	BlockSparseMatrix& operator+= (const BlockSparseMatrix& rhs);
	BlockSparseMatrix& operator*= (double alpha);
	Eigen::VectorXd operator* (const Eigen::VectorXd& rhs) const;
	Eigen::MatrixXd operator * (const Eigen::MatrixXd& rhs) const;
	double operator() (int i, int j) const;
	const Eigen::MatrixXd& ConvertToEigenMatrix() const;
	bool ConjugateGradientSolve(const Eigen::VectorXd& rhs, Eigen::VectorXd& sol);
	bool ConjugateGradientSolve(const Eigen::MatrixXd& rhs, Eigen::MatrixXd& sol);
	bool CheckSolution(const Eigen::VectorXd& rhs, const Eigen::VectorXd& sol);
    void ConvertFromEigenMatrix(const Eigen::MatrixXd& mat);
	//void ShrinkRowBlock(int ithRowBlock);
	//void ShrinkColBlock(int ithColBlock);
	bool LltSolve(const Eigen::VectorXd& rhs, Eigen::VectorXd& sol);
	bool LltSolve(const Eigen::MatrixXd& rhs, Eigen::MatrixXd& sol);
	void UpdateLLT();
	void UpdateCSR();
	void CopyCSR(const BlockSparseMatrix& rhs);
	bool operator== (const BlockSparseMatrix& rhs);
		
private:
	Eigen::MatrixXd mMatrix;
	indexSetT mNonzeroIndices;
	int mBlockWidth;
	int mBlockHeight;
	int mNumBlocksWidth;
	int mNumBlocksHeight;
	//std::vector<Eigen::MatrixXd> mBlockMatrices;

	Eigen::LLT<Eigen::MatrixXd> mLLTDecomp;
	Eigen::MatrixXd mMatrixInv;

	CSRMatrix mCSREquivalent;
	bool mbCSRDirty;
	bool mbLLTDirty;
	int indexTo1D(int ithBlock, int jthBlock) const
	{
		return ithBlock * mNumBlocksWidth + jthBlock;
	}
};

#endif