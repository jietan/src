#include "stdafx.h"
#include "utility.h"
#include <iomanip>
#include "ConfigManager.h"
#include <stdarg.h>

void Tokenize(const std::string& str,
			  std::vector<std::string>& tokens,
			  const std::string& delimiters)
{
	// Skip delimiters at beginning.
	std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
	// Find first "non-delimiter".
	std::string::size_type pos     = str.find_first_of(delimiters, lastPos);

	while (std::string::npos != pos || std::string::npos != lastPos)
	{
		// Found a token, add it to the std::vector.
		tokens.push_back(str.substr(lastPos, pos - lastPos));
		// Skip delimiters.  Note the "not_of"
		lastPos = str.find_first_not_of(delimiters, pos);
		// Find next "non-delimiter"
		pos = str.find_first_of(delimiters, lastPos);
	}
}

void PrintExactDouble(double value)
{
	int numBytes = sizeof(double);
#define DOUBLE_SIZE 8
	CHECK(numBytes == DOUBLE_SIZE);
	unsigned int result[DOUBLE_SIZE];
	double toPrint = value;
	unsigned char* pToPrint = (unsigned char*)&toPrint;
	for (int i = 0; i < DOUBLE_SIZE; ++i)
	{
		result[i] = static_cast<unsigned int>(*pToPrint);
		pToPrint++;
	}
#ifdef _WIN32
	LOG(INFO) << hex << 
		setw(2) << setfill('0') << result[0] << 
		setw(2) << setfill('0') << result[1] << 
		setw(2) << setfill('0') << result[2] << 
		setw(2) << setfill('0') << result[3] << 
		setw(2) << setfill('0') << result[4] << 
		setw(2) << setfill('0') << result[5] << 
		setw(2) << setfill('0') << result[6] << 
		setw(2) << setfill('0') << result[7];
#else
	cout << hex << 
	  setw(2) << setfill('0') << result[0] << 
	  setw(2) << setfill('0') << result[1] << 
	  setw(2) << setfill('0') << result[2] << 
		setw(2) << setfill('0') << result[3] << 
		setw(2) << setfill('0') << result[4] << 
		setw(2) << setfill('0') << result[5] << 
		setw(2) << setfill('0') << result[6] << 
		setw(2) << setfill('0') << result[7] << endl;

#endif

}

string CleanPath(const string& path)
{
    size_t found;
    found = path.find_last_of("/\\");
    string filename = path.substr(found + 1);
    found = filename.find_last_of(".");
    string filenameWithoutSuffix = filename.substr(0, found);
    return filenameWithoutSuffix;
}

void DebugOutput(char* fmt, ...)
{
    int bDebugOutput = 0;
    DecoConfig::GetSingleton()->GetInt("MPI", "IsDebugOutput", bDebugOutput);
    if (bDebugOutput)
    {
        va_list args;
        va_start(args, fmt);
        vfprintf(stderr, fmt, args);
        va_end(args);
    }

}

bool IsFileExist(const string& filename)
{
    ifstream infile(filename.c_str());
    return infile.good();
}

bool IsValueInBetween(double value, double a, double b)
{
    double minValue = a <= b ? a : b;
    double maxValue = a > b ? a : b;
    if (value >= minValue && value <= maxValue)
        return true;
    else
        return false;
}

void DumpToObjFile(const string& filename, const vector<double>& vertexBuff)
{
	ofstream out(filename.c_str());

	int numVertices = static_cast<int>(vertexBuff.size()) / 3;
	int numFaces = numVertices / 3;
	for (int i = 0; i < numVertices; i++)
		out<<"v "<< vertexBuff[3 * i + 0] <<" "<< vertexBuff[3 * i + 1] <<" "<< vertexBuff[3 * i + 2] << std::endl;
	for (int i = 0; i < numFaces; i++)
		out<<"f "<< i * 3 + 1 <<" "<< i * 3 + 2<<" "<< i * 3 + 3 << std::endl;

}