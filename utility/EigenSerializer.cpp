#include "stdafx.h"
#include "EigenSerializer.h"
#include "archive.h"



void EigenSerializer::ReadVectorFromFileDouble(const string& filename, VectorXd& vec, bool bBinary)
{
	if (bBinary)
	{
		DecoArchive Ar(filename, AT_Read);
		int length;
		Ar >> length;
	    
		vec = VectorXd::Zero(length);
		for (int i = 0; i < length; ++i)
		{
	        
			Ar >> vec(i);
		}
	}
	else
	{

	}
}
void EigenSerializer::ReadMatrixFromFileDouble(const string& filename, MatrixXd& mat, bool bBinary)
{
	if (bBinary)
	{
		DecoArchive Ar(filename, AT_Read);
		int numRows, numCols;
		Ar >> numRows;
		Ar >> numCols;
		mat = MatrixXd::Zero(numRows, numCols);
		for (int i = 0; i < numRows; ++i)
		{
			for (int j = 0; j < numCols; ++j)
			{
				Ar >> mat(i, j);
			}
		}
	}
	else
	{
		ifstream in(filename.c_str());
		const int lineSize = 12800;
		char line[lineSize];
		double value[200];
		int ithRow = 0;
		while (true)
		{
			if (in.eof())
				return;

			int n = 0;

			memset(line, 0, lineSize * sizeof(char));
			in.getline(line, lineSize);
			if (string(line) == "")
				return; //end of file reached
			char* realLineStart = line;
			if (line[0] == '[')
				realLineStart++;
			istringstream lineStr(realLineStart);

			while(true)
			{
				lineStr >> skipws >> value[n];
				if (lineStr.fail())
					break;
				n++;
			}
			if (ithRow == 0)
			{
				mat = MatrixXd::Zero(n, n);
			}
			for (int i = 0; i < n; ++i)
				mat(ithRow, i) = value[i]; 
			ithRow++;
		}
	}
}

void EigenSerializer::ReadVectorFromFileInt(const string& filename, VectorXi& vec, bool bBinary)
{
	if (bBinary)
	{
		DecoArchive Ar(filename, AT_Read);
		int length;
		Ar >> length;

		vec = VectorXi::Zero(length);
		for (int i = 0; i < length; ++i)
		{

			Ar >> vec(i);
		}
	}
}


void EigenSerializer::SaveMatrixToFileDouble(const string& filename, const MatrixXd& mat, bool bBinary)
{
    if (bBinary)
    {
        DecoArchive Ar(filename, AT_Write);
	int nRows = mat.rows();
	int nCols = mat.cols();
        Ar << nRows;
        Ar << nCols;
        for (int i = 0; i < mat.rows(); ++i)
        {
            for (int j = 0; j < mat.cols(); ++j)
            {
                Ar << mat(i, j);
            }
        }
    }
    else
    {
        SaveToFileAsciiDouble(filename, mat);
    }
}

void EigenSerializer::SaveVectorToFileDouble(const string& filename, const VectorXd& vec, bool bBinary)
{
    if (bBinary)
    {
        DecoArchive Ar(filename, AT_Write);
	int length = vec.size();
        Ar << length;

        for (int i = 0; i < vec.size(); ++i)
        {
            Ar << vec(i);
        }
    }
    else
    {
        SaveToFileAsciiDouble(filename, vec);
    }
}

void EigenSerializer::SaveVectorToFileInt(const string& filename, const VectorXi& vec, bool bBinary)
{
    if (bBinary)
    {
        DecoArchive Ar(filename, AT_Write);
	int length = vec.size();
        Ar << length;

        for (int i = 0; i < vec.size(); ++i)
        {
            Ar << vec(i);
        }
    }
    else
    {
        SaveToFileAsciiInt(filename, vec);
    }
}

void EigenSerializer::SaveToFileAsciiDouble(const string& filename, const MatrixXd& mat)
{
    ofstream out(filename.c_str());
	SaveToFileAsciiDouble(out, mat);
}

void EigenSerializer::SaveToFileAsciiInt(const string& filename, const MatrixXi& mat)
{
    ofstream out(filename.c_str());
	SaveToFileAsciiInt(out, mat);
}

void EigenSerializer::SaveToFileAsciiDouble(ofstream& out, const MatrixXd& mat)
{
	out << "[";
	int numRows = mat.rows();
	int numCols = mat.cols();
	for (int i = 0; i < numRows; ++i)
	{
		for (int j = 0; j < numCols; ++j)
		{
			out << mat(i, j) << " ";
		}
		if (i == numRows - 1)
			out << "];";
		else
			out << ";" << endl;
	}
	out << endl;
}

void EigenSerializer::SaveToFileAsciiInt(ofstream& out, const MatrixXi& mat)
{
	out << "[";
	int numRows = mat.rows();
	int numCols = mat.cols();
	for (int i = 0; i < numRows; ++i)
	{
		for (int j = 0; j < numCols; ++j)
		{
			out << mat(i, j) << " ";
		}
		if (i == numRows - 1)
			out << "];";
		else
			out << ";" << endl;
	}
	
	out << endl;
}

void EigenSerializer::OutputToMFormat(ofstream& out, const string& varName, const MatrixXd& mat)
{
	int numRows = mat.rows();
	int numCols = mat.cols();
	out << varName << " = [" << endl;
	for (int ithRow = 0; ithRow < numRows; ++ithRow)
	{
		for (int ithCol = 0; ithCol < numCols; ++ithCol)
		{
			out << mat(ithRow, ithCol) << " ";
		}
		out << ";" << endl;
	}
	out << "];" << endl;

}

void EigenSerializer::OutputToMFormat(ofstream& out, const string& varName, const VectorXd& vec)
{
	out << varName << " = [ ";
	for (int i = 0; i < vec.size(); ++i)
	{
		out << vec[i] << " ";
	}
	out << "];" << endl;

}

void EigenSerializer::OutputToMFormat(ofstream& out, const string& varName, const MatrixXi& mat)
{
	int numRows = mat.rows();
	int numCols = mat.cols();
	out << varName << " = [" << endl;
	for (int ithRow = 0; ithRow < numRows; ++ithRow)
	{
		for (int ithCol = 0; ithCol < numCols; ++ithCol)
		{
			out << mat(ithRow, ithCol) << " ";
		}
		out << ";" << endl;
	}
	out << "];" << endl;

}

void EigenSerializer::OutputToMFormat(ofstream& out, const string& varName, const VectorXi& vec)
{
	out << varName << " = [ ";
	for (int i = 0; i < vec.size(); ++i)
	{
		out << vec[i] << " ";
	}
	out << "];" << endl;

}