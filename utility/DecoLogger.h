#ifndef _DECOLOGGER_H
#define _DECOLOGGER_H
//#include "stdafx.h"
#include <fstream>
#include <string>

class vector2;
class vector3;
class vector4;
class matrix44;

class DecoLogger
{
public:
	DecoLogger(void);
	~DecoLogger(void);
public:
	static DecoLogger* GetSingleton();
	static void DestroySingleton();
	void WriteLine(const std::string& str);
	friend DecoLogger& operator << (DecoLogger& logger, const std::string& str);
	friend DecoLogger& operator << (DecoLogger& logger, const INT num);
	friend DecoLogger& operator << (DecoLogger& logger, const DOUBLE num);
	friend DecoLogger& operator << (DecoLogger& logger, const vector2& vec);
	friend DecoLogger& operator << (DecoLogger& logger, const vector3& vec);
	friend DecoLogger& operator << (DecoLogger& logger, const vector4& vec);
	friend DecoLogger& operator << (DecoLogger& logger, const matrix44& mat);
	friend DecoLogger& operator << (DecoLogger& logger, const UINT num);
private:
	std::ofstream _logStrm;
	static DecoLogger* msLogger;
};

#endif
