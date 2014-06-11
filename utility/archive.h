#ifndef _DECO_ARCHIVE
#define _DECO_ARCHIVE

#include "stdafx.h"
#include "RenderType.h"
#include <fstream>

enum ArchiveType
{
	AT_Read,
	AT_Write

};
class DecoArchive
{
private:
	UINT m_version;
	string m_filePath;
	fstream m_fileStream;
	UINT m_type;
	UINT m_fileSize;
public:
	DecoArchive();
	~DecoArchive();
	DecoArchive(const string& filename, UINT type);
	bool Create(const string& filename, UINT type);
	void Close();
	void Flush();
	UINT Tell();
	UINT FileSize() const;
	bool IsEOF();
	string GetFilePath() const
	{
		return m_filePath;
	}
	friend DecoArchive& operator<< (DecoArchive& Ar, DOUBLE value);
	friend DecoArchive& operator<< (DecoArchive& Ar, INT value);
	friend DecoArchive& operator<< (DecoArchive& Ar, FLOAT value);
	friend DecoArchive& operator<< (DecoArchive& Ar, UINT value);
	friend DecoArchive& operator<< (DecoArchive& Ar, const string& str);
	friend DecoArchive& operator<< (DecoArchive& Ar, char chr);
	friend DecoArchive& operator<< (DecoArchive& Ar, BYTE vlaue);
	friend DecoArchive& operator<< (DecoArchive& Ar, SHORT value);
	friend DecoArchive& operator<< (DecoArchive& Ar, const Eigen::Vector3d& vec);
	template <typename T>
	friend DecoArchive& operator<< (DecoArchive& Ar, const vector<T>& array)
	{
		int numElements = static_cast<int>(array.size());
		Ar << numElements;
		for (int i = 0; i < numElements; ++i)
		{
			Ar << array[i];
		}
		return Ar;
	}

	friend DecoArchive& operator>> (DecoArchive& Ar, SHORT& value);
	friend DecoArchive& operator>> (DecoArchive& Ar, DOUBLE &value);
	friend DecoArchive& operator>> (DecoArchive& Ar, INT &value);
	friend DecoArchive& operator>> (DecoArchive& Ar, FLOAT &value);
	friend DecoArchive& operator>> (DecoArchive& Ar, UINT &value);
	friend DecoArchive& operator>> (DecoArchive& Ar, string &str);
	friend DecoArchive& operator>> (DecoArchive& Ar, char &chr);
	friend DecoArchive& operator>> (DecoArchive& Ar, BYTE& vlaue);
	friend DecoArchive& operator>> (DecoArchive& Ar, Eigen::Vector3d& vec);
	template <typename T>
	friend DecoArchive& operator>> (DecoArchive& Ar, vector<T>& array)
	{
		int numElements;
		Ar >> numElements;
		array.resize(numElements);
		for (int i = 0; i < numElements; ++i)
		{
			Ar >> array[i];
		}
		return Ar;
	}
};

#endif