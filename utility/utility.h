#ifndef  _FLUID_UTILITY
#define _FLUID_UTILITY
#include "stdafx.h"

#include <vector>
#include <string>
#include <Eigen/Dense>
using namespace std;
//#include "MarchingHeap.h"

class LevelSetWithIndex;
class Index3;

template<typename T>
ofstream& operator<< (ofstream& out, const vector<T>& array)
{
	out << "[";
	int arraySize = static_cast<int>(array.size());
	for (int i = 0; i < arraySize - 1; ++i)
	{
		out << array[i] << ", ";
	}
	out << array[arraySize - 1];
	out << "];" << std::endl;
	return out;
}

template <typename T>
void OutputToMFormat(ofstream& out, const string& varName, const vector<T>& array)
{
	out << varName << " = [";
	int n = static_cast<int>(array.size());
	for (int i = 0; i < n; ++i)
	{
		out << array[i] << " ";
	}
	out << "];" << endl;
}

template <typename T>
T*** Malloc3DArray(UINT numFirstElements, UINT numSecondElements, UINT numThirdElements, BOOL beInitToZero = FALSE)
{
  //	assert(numFirstElements > 0 && numSecondElements > 0 && numThirdElements > 0);
	T*** result = new T**[numFirstElements];
	for (UINT i = 0; i < numFirstElements; i++)
	{
		result[i] = new T*[numSecondElements];
		for (UINT j = 0; j < numSecondElements; j++)
		{
			result[i][j] = new T[numThirdElements];
			if (beInitToZero)
				memset(result[i][j], 0, numThirdElements * sizeof(T));
		}
	}
	return result;
}

template <typename T>
T** Malloc2DArray(UINT numFirstElements, UINT numSecondElements, BOOL beInitToZero = FALSE)
{
	// assert(numFirstElements > 0 && numSecondElements > 0);
	T** result = new T*[numFirstElements];
	for (UINT i = 0; i < numFirstElements; i++)
	{
		result[i] = new T[numSecondElements];
		if (beInitToZero)
		{
			memset(result[i], 0, numSecondElements * sizeof(T));
		}
	}
	return result;
}

template <typename T>
T* Malloc1DArray(INT numElements)
{
	// assert(numElements > 0);
	T* result = new T[numElements];
	return result;
}

template <typename T>
void Free3DArray(T ***& Array, UINT numFirstElements, UINT numSecondElements, UINT numThirdElements)
{
  //	assert(Array);
	for (UINT i = 0; i < numFirstElements; i++)
	{
	  //		assert(Array[i]);
		for (UINT j = 0; j < numSecondElements; j++)
		{
		  //			assert (Array[i][j]);
			delete[] Array[i][j];
			Array[i][j] = NULL;
		}		
		delete[] Array[i];
		Array[i] = NULL;
	}
	delete[] Array;
	Array = NULL;
}

template <typename T>
void Free2DArray(T**& array, UINT numFirstElements, UINT numSecondElements)
{
  //	assert(array);
	for (UINT i = 0; i < numFirstElements; i++)
	{
	  //		assert(array[i]);
		delete [] array[i];
		array[i] = NULL;
	}
	delete [] array;
	array = NULL;
}
template <typename T>
void Free1DArray(T *&array, INT size)
{
  //	assert(array);
	delete [] array;
	array = NULL;
}
template <typename T>
void CopyTo3DArray(T*** des, T*** src, UINT numFirstElements, UINT numSecondElements, UINT numThirdElements)
{
  //	assert (des && src);
	// assert(numFirstElements > 0 && numSecondElements > 0 && numThirdElements > 0);
	for (UINT i = 0; i < numFirstElements; i++)
	{
		for (UINT j = 0; j < numSecondElements; j++)
		{
			memcpy(des[i][j], src[i][j], numThirdElements * sizeof(T));
		}
	}
}

template <typename T>
void CopyTo2DArray(T** des, T** src, UINT numFirstElements, UINT numSecondElements)
{
  //	assert (des && src);
	// assert(numFirstElements > 0 && numSecondElements > 0);
	for (UINT i = 0; i < numFirstElements; i++)
	{
		memcpy(des[i], src[i], numSecondElements * sizeof(T));
	}
}

template <typename T>
void Zero3DArray(T*** Array, UINT numFirstElements, UINT numSecondElements, UINT numThirdElements)
{
	// assert(numFirstElements > 0 && numSecondElements > 0 && numThirdElements > 0);
	for (UINT i = 0; i < numFirstElements; i++)
	{
		for (UINT j = 0; j < numSecondElements; j++)
		{
			memset(Array[i][j], 0, numThirdElements * sizeof(T));
		}
	}
}

template <typename T>
void Zero2DArray(T** Array, UINT numFirstElements, UINT numSecondElements)
{
	// assert(numFirstElements > 0 && numSecondElements > 0);
	for (UINT i = 0; i < numFirstElements; i++)
	{
		memset(Array[i], 0, numSecondElements * sizeof(T));
	}
}

template <typename T>
void MemSet3DArray(T*** Array, T num, UINT numFirstElements, UINT numSecondElements, UINT numThirdElements)
{
	// assert(numFirstElements > 0 && numSecondElements > 0 && numThirdElements > 0);
	for (UINT i = 0; i < numFirstElements; i++)
	{
		for (UINT j = 0; j < numSecondElements; j++)
		{
			for (UINT k = 0; k < numThirdElements; k++)
			{
				Array[i][j][k] = num;
			}
		}
	}
}

template <typename T>
void Swap(T& a, T& b)
{
	T tmp;
	tmp = a;
	a = b;
	b = tmp;
}

template <typename T>
void Swap(T* a, T* b)
{
	T* tmp;
	tmp = a;
	a = b;
	b = tmp;
}

template <class T>
class FixLengthQueue
{
public:
	FixLengthQueue() : mQueueHead(0), mQueueTail(0), mMaxLen(0), mCurrentLen(0), mData(NULL)
	{
		mData = new T[mMaxLen];
	}
	FixLengthQueue(int len) : mQueueHead(0), mQueueTail(0), mMaxLen(0), mCurrentLen(0), mData(NULL)
	{
		SetLength(len);
	}
	FixLengthQueue(const FixLengthQueue& rhs) : mQueueHead(rhs.mQueueHead), mQueueTail(rhs.mQueueTail), mMaxLen(rhs.mMaxLen), mCurrentLen(rhs.mCurrentLen)
	{
		mData = new T[mMaxLen];
		memcpy(mData, rhs.mData, mMaxLen * sizeof(T));
	}
	~FixLengthQueue()
	{
		if (mData)
			delete[] mData;
	}
	const FixLengthQueue& operator= (const FixLengthQueue& rhs)
	{
		if (this == &rhs)
			return *this;
		mQueueHead = rhs.mQueueHead;
		mQueueTail = rhs.mQueueTail;
		mMaxLen = rhs.mMaxLen;
		mCurrentLen = rhs.mCurrentLen;
		if (mData)
			delete[] mData;
		mData = new T[mMaxLen];
		memcpy(mData, rhs.mData, mMaxLen * sizeof(T));
		return *this;
	};
	void PushBack(const T& elem)
	{
		if (!mData)
			SetLength(10);

		mData[mQueueTail] = elem;
		mCurrentLen++;

		if (mCurrentLen >= mMaxLen)
		{
			if (mQueueTail == mQueueHead)
			{
				mQueueHead = (mQueueHead + 1) % mMaxLen;
			}
			mQueueTail = (mQueueTail + 1) % mMaxLen;
		}
		else
		{
			mQueueTail++;

		}
		if (mCurrentLen > mMaxLen)
			mCurrentLen = mMaxLen;

	}
	void GetAllElements(std::vector<T>& elements) const
	{
		elements.clear();
		for (int i = 0; i < mCurrentLen; ++i)
		{
			elements.push_back(mData[(mQueueHead + i) % mMaxLen]);
		}
	}
	const T& GetIthElement(int i) const
	{
		// assert(i >= 0 && i < mCurrentLen);
		return mData[(mQueueHead + i) % mMaxLen];
	}
	int GetNumElements() const
	{
		return mCurrentLen;
	}
	void SetLength(int len)
	{
		mMaxLen = len;
		if (mData)
			delete[] mData;
		mData = new T[mMaxLen];
		mQueueHead = mQueueTail = 0;
		mCurrentLen = 0;
	}
private:
	int mQueueHead;
	int mQueueTail;
	int mMaxLen;
	int mCurrentLen;
	T* mData;
};

void DumpToObjFile(const string& filename, const vector<double>& vertexBuff);
bool IsValueInBetween(double value, double a, double b); //the order of a and b is not assumed;
void Tokenize(const std::string& str,
			  std::vector<std::string>& tokens,
			  const std::string& delimiters = " ");
bool IsFileExist(const string& filename);
string CleanPath(const string& path);
void PrintExactDouble(double value);
bool operator< (const LevelSetWithIndex& lfs, const LevelSetWithIndex& rhs);
bool operator> (const LevelSetWithIndex& lfs, const LevelSetWithIndex& rhs);
bool operator< (const Index3& lfs, const Index3& rhs);
bool operator> (const Index3& lfs, const Index3& rhs);
bool operator!= (const Index3& lfs, const Index3& rhs);
void DebugOutput(char* fmt, ...);
#endif
