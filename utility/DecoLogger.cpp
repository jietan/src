#include "stdafx.h"
#include "DecoLogger.h"
#include <iomanip>
#include "mathlib.h"
using namespace std;

DecoLogger* DecoLogger::msLogger = NULL;
const char* logDir = "logs\\";//"D:\\CS\\fluid2009\\";


DecoLogger::DecoLogger(void)
{
	char filename[200];
	sprintf(filename, "%slog.txt", logDir);
	_logStrm.open(filename/*, ios::app*/);
	_logStrm<<"DecoLogger::DecoLogger()"<<endl;
}

DecoLogger::~DecoLogger(void)
{
	_logStrm << flush;
	_logStrm.close();
}

DecoLogger* DecoLogger::GetSingleton()
{
	if (!msLogger)
	  msLogger = new DecoLogger();
	return msLogger;

}

void DecoLogger::DestroySingleton()
{
	if (msLogger)
	  delete msLogger;
}

#ifdef _OUTPUT_LOG
void DecoLogger::WriteLine(const std::string& str)
{
	_logStrm<<str<<endl;
}



DecoLogger& operator<< (DecoLogger& logger, const std::string& str)
{
	logger._logStrm<<str;
	logger._logStrm << flush;
	return logger;
}

DecoLogger& operator << (DecoLogger& logger, const INT num)
{
	logger._logStrm<<num;
	logger._logStrm << flush;
	return logger;
}
DecoLogger& operator << (DecoLogger& logger, const DOUBLE num)
{
	logger._logStrm << setiosflags(ios::fixed) << setprecision(20) << num;
	logger._logStrm << flush;
	return logger;
}

DecoLogger& operator << (DecoLogger& logger, const vector2& vec)
{
	logger._logStrm << setiosflags(ios::fixed) << setprecision(8) << "(" << vec.x << "," << vec.y << ")";
	logger._logStrm << flush;
	return logger;
}
DecoLogger& operator << (DecoLogger& logger, const vector3& vec)
{
	logger._logStrm << setiosflags(ios::fixed) << setprecision(8) << "(" << vec.x << "," << vec.y << "," << vec.z << ")";
	logger._logStrm << flush;
	return logger;
}

DecoLogger& operator << (DecoLogger& logger, const vector4& vec)
{
	logger._logStrm << setiosflags(ios::fixed) << setprecision(8) << "(" << vec.x << "," << vec.y << "," << vec.z << "," << vec.w << ")";
	logger._logStrm << flush;
	return logger;
}

DecoLogger& operator << (DecoLogger& logger, const matrix44& mat)
{
	logger._logStrm << "[";
	for (int i = 0; i < 4; ++i)
	{
		logger._logStrm << "[";
		for (int j = 0; j < 4; ++j)
		{
			logger._logStrm << setiosflags(ios::fixed) << setprecision(8) << mat[i][j] << " ";
		}
		logger._logStrm << "]\n";
	}
	logger._logStrm << "]";

	logger._logStrm << flush;
	return logger;
}

DecoLogger& operator << (DecoLogger& logger, const UINT num)
{
	logger._logStrm<<num;
	logger._logStrm << flush;
	return logger;
}
#else
void DecoLogger::WriteLine(const std::string& str)
{

}



DecoLogger& operator<< (DecoLogger& logger, const std::string& str)
{
	return logger;
}

DecoLogger& operator << (DecoLogger& logger, const INT num)
{
	return logger;
}
DecoLogger& operator << (DecoLogger& logger, const DOUBLE num)
{
	return logger;
}

DecoLogger& operator << (DecoLogger& logger, const vector2& vec)
{
	return logger;
}
DecoLogger& operator << (DecoLogger& logger, const vector3& vec)
{
	return logger;
}

DecoLogger& operator << (DecoLogger& logger, const vector4& vec)
{
	return logger;
}

DecoLogger& operator << (DecoLogger& logger, const matrix44& mat)
{
	return logger;
}

DecoLogger& operator << (DecoLogger& logger, const UINT num)
{
	return logger;
}
#endif
