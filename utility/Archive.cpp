#include "stdafx.h"
#include "archive.h"


DecoArchive::DecoArchive() : m_version(0), m_type(0), m_filePath("")
{

}
DecoArchive::~DecoArchive()
{
	Close();
}


DecoArchive::DecoArchive(const string& filename, UINT type) : m_version(0), m_filePath(filename), m_type(type), m_fileSize(0)
{
	Create(filename, type);
}
bool DecoArchive::Create(const string& filename, UINT type)
{
	m_filePath = filename;
	m_type = type;
	m_version = 0;
	if (type == AT_Write)
	{
		m_fileStream.open(filename.c_str(), ios::out | ios::binary);
		if (m_fileStream.fail())
			return false;
	}
	else if (type == AT_Read)
	{
		m_fileStream.open(filename.c_str(), ios::in | ios::binary);
		if (m_fileStream.fail())
			return false;
		filebuf* pbuf = m_fileStream.rdbuf();
		long size = pbuf->pubseekoff(0, ios::end, ios::in);
		pbuf->pubseekpos(0, ios::in);
		m_fileSize = static_cast<UINT>(size);
	}
	return true;
}

void DecoArchive::Close()
{
	if (m_fileStream.is_open())
		m_fileStream.close();
}

bool DecoArchive::IsEOF()
{
	int tellValue = Tell();
	return (m_fileSize == tellValue || tellValue == -1);
//	return m_fileStream.eof();
}

void DecoArchive::Flush()
{
	m_fileStream.flush();
}

UINT DecoArchive::Tell()
{
	if (m_type == AT_Read)
		return m_fileStream.tellg();
	else if (m_type == AT_Write)
		return m_fileStream.tellp();
	else
		return 0;
}

UINT DecoArchive::FileSize() const
{
	return m_fileSize;
}

DecoArchive& operator<< (DecoArchive& Ar, DOUBLE value)
{
	Ar.m_fileStream.write((char*)&value, sizeof(DOUBLE));
	//Ar.m_fileStream << value;
	return Ar;
}

DecoArchive& operator<< (DecoArchive& Ar, SHORT value)
{
	Ar.m_fileStream.write((char*)&value, sizeof(SHORT));
	return Ar;
}

DecoArchive& operator<< (DecoArchive& Ar, INT value)
{
	Ar.m_fileStream.write((char*)&value, sizeof(INT));
	//Ar.m_fileStream << value;
	return Ar;
}
DecoArchive& operator<< (DecoArchive& Ar, FLOAT value)
{
	Ar.m_fileStream.write((char*)&value, sizeof(FLOAT));

	//Ar.m_fileStream << value;
	return Ar;
}
DecoArchive& operator<< (DecoArchive& Ar, UINT value)
{
	Ar.m_fileStream.write((char*)&value, sizeof(UINT));

	//Ar.m_fileStream << value;
	return Ar;
}

DecoArchive& operator<< (DecoArchive& Ar, const string& str)
{
	size_t len = str.length();
	Ar.m_fileStream.write((char*)&len, static_cast<std::streamsize>(sizeof(size_t)));
	Ar.m_fileStream.write(str.c_str(), static_cast<std::streamsize>(len * sizeof(char)));
	//Ar.m_fileStream << str;
	return Ar;
}
DecoArchive& operator<< (DecoArchive& Ar, char chr)
{
	Ar.m_fileStream.write((char*)&chr, sizeof(chr));

	//Ar.m_fileStream << chr;
	return Ar;
}

DecoArchive& operator<< (DecoArchive& Ar, BYTE value)
{
	Ar.m_fileStream.write((char*)&value, sizeof(BYTE));

	//Ar.m_fileStream << value;
	return Ar;

}

DecoArchive& operator<< (DecoArchive& Ar, const Vector3d& vec)
{
	for (int i = 0; i < 3; ++i)
		Ar << vec[i];
	return Ar;
}

DecoArchive& operator>> (DecoArchive& Ar, BYTE& value)
{
	Ar.m_fileStream.read((char*)&value, sizeof(BYTE));

	//Ar.m_fileStream << value;
	return Ar;

}

DecoArchive& operator>> (DecoArchive& Ar, SHORT& value)
{
	Ar.m_fileStream.read((char*)&value, sizeof(SHORT));
	return Ar;
}

DecoArchive& operator>> (DecoArchive& Ar, DOUBLE &value)
{
	Ar.m_fileStream.read((char*)&value, sizeof(DOUBLE));

	//Ar.m_fileStream >> value;
	return Ar;
}
DecoArchive& operator>> (DecoArchive& Ar, INT &value)
{
	//Ar.m_fileStream >> value;
	Ar.m_fileStream.read((char*)&value, sizeof(INT));

	return Ar;
}
DecoArchive& operator>> (DecoArchive& Ar, FLOAT &value)
{
	//Ar.m_fileStream >> value;
	Ar.m_fileStream.read((char*)&value, sizeof(FLOAT));

	return Ar;
}
DecoArchive& operator>> (DecoArchive& Ar, UINT &value)
{
	//Ar.m_fileStream >> value;
	Ar.m_fileStream.read((char*)&value, sizeof(UINT));

	return Ar;
}

DecoArchive& operator>> (DecoArchive& Ar, string &str)
{
	//Ar.m_fileStream >> str;
	size_t len = 0;
	Ar.m_fileStream.read((char*)&len, sizeof(size_t));
	char* readStr = new char[len + 1];
	memset(readStr, 0, (len + 1) * sizeof(char));
	Ar.m_fileStream.read((char*)readStr, static_cast<std::streamsize>(len * sizeof(char)));
	str = readStr;
	delete[] readStr;
	return Ar;
}
DecoArchive& operator>> (DecoArchive& Ar, char &chr)
{
	//Ar.m_fileStream >> chr;
	Ar.m_fileStream.read((char*)&chr, sizeof(char));

	return Ar;
}

DecoArchive& operator>> (DecoArchive& Ar, Vector3d& vec)
{
	for (int i = 0; i < 3; ++i)
		Ar >> vec[i];
	return Ar;
}
