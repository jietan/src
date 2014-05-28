#include "stdafx.h"
#include "CSRMatrix.h"
#include "DecoLogger.h"
#include "mathlib.h"
#include "utility.h"
#include "BlockSparseMatrix.h"


CSRMatrix::CSRMatrix() : n(0), isSymmetric(false)
{
	values.clear();
	columns.clear();
	rowIndex.clear();
}


CSRMatrix::~CSRMatrix()
{
	n = 0;
	values.clear();
	columns.clear();
	rowIndex.clear();
}

bool CSRMatrix::GetDiagonalElement(int i, int& index, double& value)
{
	return GetElement(i, i, index, value);
}

bool CSRMatrix::GetElement(int i, int j, int& index, double& value) 
{
	if (isSymmetric && i > j)
	{
		int tmp = i;
		i = j;
		j = tmp;
	}
	int rowStartIdx = rowIndex[i - 1]; 
	int rowEndIdx = rowIndex[i];

	for (int colIdx = rowStartIdx - 1; colIdx < rowEndIdx - 1; ++colIdx)
	{
		if (columns[colIdx] == j)
		{
			index = colIdx;
			value = values[colIdx];
			return true;
		}
	}
	return false;
}

void CSRMatrix::GetRow(int i, int& startIdx, int& numEntries) 
{
	startIdx = rowIndex[i - 1]; 
	int rowEndIdx = rowIndex[i];
	numEntries = rowEndIdx - startIdx;
}

void CSRMatrix::Transpose() 
{
	vector<double> newValues;
	vector<int> newColumns;
	vector<int> newRowIndex;

	bool sym = isSymmetric;

	isSymmetric = false;
	int idx;
	double value;
	newRowIndex.push_back(1);
	int colCounter = 1;

	for (int i = 1; i <= n; ++i)
	{
		for (int j = 1; j <= n; ++j)
		{
			if (GetElement(j, i, idx, value))
			{
				newValues.push_back(value);
				newColumns.push_back(j);
				colCounter++;
			}
		}
		newRowIndex.push_back(colCounter);
	}
	assert(newValues.size() == values.size());
	assert(newColumns.size() == columns.size());
	assert(newRowIndex.size() == rowIndex.size());

	values = newValues;
	columns = newColumns;
	rowIndex = newRowIndex;
	isSymmetric = sym;
}

void CSRMatrix::ConvertFromBlockSparseMatrix(const BlockSparseMatrix& m, bool bSymmetric)
{
//	assert(m.mNumBlocksWidth == m.mNumBlocksHeight && m.mNumBlocksWidth == m.mNumBlocksHeight);
	n = m.mBlockWidth * m.mNumBlocksWidth;
	values.clear();
	columns.clear();
	rowIndex.clear();
	std::vector<index2DT> indices;
	for (int ithBlock = 0; ithBlock < m.mNumBlocksHeight; ++ithBlock)
	{
		for (int jthBlock = 0; jthBlock < m.mNumBlocksWidth; ++jthBlock)
		{
			if (m.mNonzeroIndices[m.indexTo1D(ithBlock, jthBlock)])
			{
				indices.push_back(index2DT(ithBlock, jthBlock));
			}
		}
	}
	//sort(indices.begin(), indices.end());
	if (indices.empty())
		return;
	rowIndex.push_back(1);
	int ithElem = 0;
	int numIndices = static_cast<int>(indices.size());
	while (ithElem != numIndices)
	{
		int startElem = ithElem;
		for (int ithRowInBlock = 0; ithRowInBlock < m.mBlockHeight; ++ithRowInBlock)
		{
			ithElem = startElem;
			while (true)
			{
				if (ithElem == numIndices || indices[ithElem].first != indices[startElem].first)
				{
					break;
				}
				Eigen::MatrixXd block = m.GetBlockMatrix(indices[ithElem].first, indices[ithElem].second);
				for (int jthColumnInBlock = 0; jthColumnInBlock < m.mBlockWidth; ++jthColumnInBlock)
				{
					values.push_back(block(ithRowInBlock, jthColumnInBlock));
					columns.push_back(jthColumnInBlock + indices[ithElem].second * m.mBlockWidth + 1);
				}
				++ithElem;
			}
			rowIndex.push_back(values.size() + 1);

		}
	}
}



