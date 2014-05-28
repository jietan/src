#ifndef _EIGEN_SERIALIZER_H
#define _EIGEN_SERIALIZER_H


#include <iostream>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

class EigenSerializer
{
public:
	static void SaveMatrixToFileDouble(const string& filename, const MatrixXd& mat, bool bBinary);
    static void SaveVectorToFileDouble(const string& filename, const VectorXd& vec, bool bBinary);
    static void SaveVectorToFileInt(const string& filename, const VectorXi& vec, bool bBinary);

    static void ReadMatrixFromFileDouble(const string& filename, MatrixXd& mat, bool bBinary = true);
    static void ReadVectorFromFileDouble(const string& filename, VectorXd& vec, bool bBinary = true);
    static void ReadVectorFromFileInt(const string& filename, VectorXi& vec, bool bBinary = true);

    static void SaveToFileAsciiDouble(const string& filename, const MatrixXd& mat);
    static void SaveToFileAsciiInt(const string& filename, const MatrixXi& mat);

	static void SaveToFileAsciiDouble(ofstream& out, const MatrixXd& mat);
	static void SaveToFileAsciiInt(ofstream& out, const MatrixXi& mat);

	static void OutputToMFormat(ofstream& out, const string& varName, const MatrixXd& mat);
	static void OutputToMFormat(ofstream& out, const string& varName, const VectorXd& vec);
	static void OutputToMFormat(ofstream& out, const string& varName, const MatrixXi& mat);
	static void OutputToMFormat(ofstream& out, const string& varName, const VectorXi& vec);
};

#endif