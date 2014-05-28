#include "stdafx.h"
#include "ConfigManager.h"
#include "DecoLogger.h"
#include <fstream>

#define STRING_LEN  80
const string DecoConfig::m_configFile = "../config.ini";
DecoConfig* DecoConfig::msConfigManager = NULL;

const string* DecoConfigSection::Find(const char* Key) const
{
	INT vecSize = static_cast<INT>(m_pairs.size());
	for (INT i = 0; i < vecSize; i++)
	{
		string key = m_pairs[i].Key;
		if (!strcmp(Key, key.c_str()))
		{
			return &(m_pairs[i].Value);
		}
	}
	return NULL;
}
void DecoConfigSection::AddKeyPair(const DecoConfigKeyPair& pair)
{
	m_pairs.push_back(pair);
}

void DecoConfig::parseIntoSections (const string& tmpTextBuffer, vector<DecoConfigSection*>& sections)
{
	string line = tmpTextBuffer;	
	const char* tmpBuff = strchr(line.c_str(), '[');

	if (tmpBuff)
	{
		const char* tmpEnd = strchr(tmpBuff, ']');
		UINT startPos = static_cast<UINT>(tmpBuff - line.c_str()) + 1;
		UINT numChar = static_cast<UINT>(tmpEnd - line.c_str()) - 1;
		string secName = line.substr(startPos, numChar);
		DecoConfigSection* newSec = new DecoConfigSection(secName);
		sections.push_back(newSec);
	}
	else
	{

		const char* tmpPairBuff = strchr(line.c_str(), '=');
		if (tmpPairBuff)
		{	
			DecoConfigKeyPair pair;
			UINT numChar = static_cast<UINT>(tmpPairBuff - line.c_str());
			pair.Key = line.substr(0, numChar);

			const char* valueField = tmpPairBuff + 1;
			UINT startPos = static_cast<UINT>(valueField - line.c_str());
			numChar = static_cast<UINT>(line.length() - startPos);

			pair.Value = line.substr(startPos, numChar);
			if (sections.size())
			{
				DecoConfigSection* currentSection = sections[sections.size() - 1];
				currentSection->AddKeyPair(pair);
			}

		}
	}
}


DecoConfig::~DecoConfig()
{
	INT sectionSize = static_cast<INT>(m_sections.size());
	for (INT i = 0; i < sectionSize; i++)
	{
		delete m_sections[i];
	}
	m_sections.clear();

}
void DecoConfig::Init(const char* configPath)
{
  //	tbb::mutex::scoped_lock lock(msMutex);
	const char* realPath = configPath ? configPath : m_configFile.c_str();
	ifstream cin(realPath);
	const INT buffSize = 4096;
	string tmpTextBuffer;
	while (!cin.eof())
	{
		getline(cin, tmpTextBuffer, '\n');
		parseIntoSections(tmpTextBuffer, m_sections);
	}

}

DecoConfig* DecoConfig::GetSingleton()
{

	if (!msConfigManager)
	{
		msConfigManager = new DecoConfig();
		msConfigManager->Init();
	}
	return msConfigManager;
}

void DecoConfig::DestroySingleton()
{

	if (msConfigManager)
		delete msConfigManager;
}



BOOL DecoConfig::GetString( const char* Section, const char* Key, char* Value, INT Size) const
{
  //tbb::mutex::scoped_lock lock(msMutex);
	*Value = 0;

	const DecoConfigSection* Sec = FindSection( Section );
	if( !Sec )
		return 0;
	const string* PairString = Sec->Find( Key );
	if( !PairString )
		return 0;
	strncpy( Value, PairString->c_str(), Size );
	return 1;

}

const DecoConfigSection* DecoConfig::FindSection(const char* Section) const
{
	// tbb::mutex::scoped_lock lock(msMutex);
	INT vecSize = static_cast<INT>(m_sections.size());
	for (INT i = 0; i < vecSize; i++)
	{
		const string& name = m_sections[i]->GetName();
		if (!strcmp(Section, name.c_str()))
		{
			return (m_sections[i]);
		}
	}
	return NULL;
}

// Derived functions.
BOOL DecoConfig::GetString
(
 const char* Section,
 const char* Key,
 string&     Str
 ) const
{
	char Temp[4096]="";
	BOOL Result = GetString( Section, Key, Temp, 4096);

	Str = Temp;
	int strLen = static_cast<int>(Str.size());
	if (strLen)
	{
		if (Str[strLen - 1] == '\r')
			Str = Str.substr(0, strLen - 1);
	}
	return Result;
}


BOOL DecoConfig::GetInt
(
 const char*	Section,
 const char*	Key,
 INT&			Value
 ) const
{
	char Text[STRING_LEN]; 
	if( GetString( Section, Key, Text, STRING_LEN) )
	{
		Value = atoi(Text);
		return 1;
	}
	return 0;

}
BOOL DecoConfig::GetFloat
(
 const char*	Section,
 const char*	Key,
 FLOAT&			Value
 ) const
{
	char Text[STRING_LEN]; 
	if( GetString( Section, Key, Text, STRING_LEN ) )
	{
		Value = static_cast<FLOAT>(atof(Text));
		return 1;
	}
	return 0;
}

BOOL DecoConfig::GetDouble
(
 const char* Section,
 const char* Key,
 DOUBLE& Value
 ) const
{
	char Text[STRING_LEN]; 
	if( GetString( Section, Key, Text, STRING_LEN ) )
	{
		Value = atof(Text);
		return 1;
	}
	return 0;
}
BOOL DecoConfig::GetBool
(
 const char*	Section,
 const char*	Key,
 BOOL&			Value
 ) const
{
	char Text[STRING_LEN]; 
	if( GetString( Section, Key, Text, STRING_LEN) )
	{
		if( strcmp(Text,"True")==0 )
		{
			Value = 1;
		}
		else
		{
			Value = atoi(Text)==1;
		}
		return 1;
	}
	return 0;
}

BOOL DecoConfig::GetVector3(const char* Section, const char* Key, vector3& Value) const
{
	char Text[STRING_LEN]; 
	DOUBLE vectorValue[3];
	if( GetString( Section, Key, Text, STRING_LEN))
	{
		string potentialVector(Text);
		const char* start = strchr(Text, '(');
		if (!start)
			return FALSE;
		start++;
		for (INT i = 0; i < 2; i++)
		{
			const char* comma = strchr(start, ',');
			if (!comma)
				return FALSE;
			UINT numChar = static_cast<UINT>(comma - start);
			UINT startPos = static_cast<UINT>(start - Text);
			string value = potentialVector.substr(startPos, numChar);
			vectorValue[i] = atof(value.c_str());
			start = comma + 1;
		}
		const char* end = strchr(Text, ')');
		if (!end)
			return FALSE;
		UINT numChar = static_cast<UINT>(end - start);
		UINT startPos = static_cast<UINT>(start - Text);
		string value = potentialVector.substr(startPos, numChar);
		vectorValue[2] = atof(value.c_str());
		Value.x = vectorValue[0];
		Value.y = vectorValue[1];
		Value.z = vectorValue[2];
		return TRUE;
	}
	return FALSE;

}
BOOL DecoConfig::GetIndex3(const char* Section, const char* Key, Index3& Value) const
{
	vector3 val;
	BOOL ret = GetVector3(Section, Key, val);
	if (ret)
	{
		Value.m_i = static_cast<SHORT>(val.x + 0.5);
		Value.m_j = static_cast<SHORT>(val.y + 0.5);
		Value.m_k = static_cast<SHORT>(val.z + 0.5);
	}
	return ret;
}

BOOL DecoConfig::GetDoubleVector(const char* Section, const char* Key, vector<double>& ValueArray) const
{
	const int MAX_STR_LEN = 512;
	char Text[MAX_STR_LEN]; 
	ValueArray.clear();
	if( GetString( Section, Key, Text, MAX_STR_LEN) )
	{
		if (!strcmp(Text, "")) return 1;
		stringstream sstream(Text);
		double value = 0;
		while(sstream.good() && strcmp(Text, ""))
		{
			sstream >> skipws >> value;
			ValueArray.push_back(value);
		}
#ifndef _WIN32
		ValueArray.erase(ValueArray.end() - 1);
#endif
		return 1;
	}
	return 0;
}

BOOL DecoConfig::GetIntVector(const char* Section, const char* Key, vector<int>& ValueArray) const
{
	const int MAX_STR_LEN = 512;
	char Text[MAX_STR_LEN]; 
	ValueArray.clear();
	if( GetString( Section, Key, Text, MAX_STR_LEN) )
	{
		stringstream sstream(Text);
		int value = 0;
		while(sstream.good() && strcmp(Text, ""))
		{
			sstream >> skipws >> value;
			ValueArray.push_back(value);
		}
#ifndef _WIN32
		ValueArray.erase(ValueArray.end() - 1);
#endif
		return 1;
	}
	return 0;
}
