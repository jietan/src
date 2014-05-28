#ifndef _CSRMATRIX_H
#define _CSRMATRIX_H

#include <vector>

class BlockSparseMatrix;

class CSRMatrix
{
public:
	std::vector<double> values;
	std::vector<int> columns;
	std::vector<int> rowIndex;
	int n;

	CSRMatrix();

	~CSRMatrix();
	void ConvertFromBlockSparseMatrix(const BlockSparseMatrix& m, bool bSymmetric = false);
	void GetRow(int i, int& startIdx, int& numEntries);
	bool GetDiagonalElement(int i, int& index, double& value);
	bool GetElement(int i, int j, int& index, double& value);
	int GetNumNonZeroEntries()
	{
		return static_cast<int>(values.size());
	}
	bool IsSymmetric()
	{
		return isSymmetric;
	}
	const double* GetValueData() const
	{
		return &(values[0]);
	}
	const int* GetColumnId() const
	{
		return &(columns[0]);
	}
	const int* GetRowId() const
	{
		return &(rowIndex[0]);
	}
	void Transpose();
private:

	bool isSymmetric;
};
#endif