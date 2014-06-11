#ifndef _EIGEN_SERIALIZER_H
#define _EIGEN_SERIALIZER_H


#include <iostream>
#include <Eigen/Dense>
using namespace std;

class EigenSerializer
{
public:
	static void SaveMatrixToFileDouble(const string& filename, const Eigen::MatrixXd& mat, bool bBinary);
    static void SaveVectorToFileDouble(const string& filename, const Eigen::VectorXd& vec, bool bBinary);
    static void SaveVectorToFileInt(const string& filename, const Eigen::VectorXi& vec, bool bBinary);

    static void ReadMatrixFromFileDouble(const string& filename, Eigen::MatrixXd& mat, bool bBinary = true);
    static void ReadVectorFromFileDouble(const string& filename, Eigen::VectorXd& vec, bool bBinary = true);
    static void ReadVectorFromFileInt(const string& filename, Eigen::VectorXi& vec, bool bBinary = true);

    static void SaveToFileAsciiDouble(const string& filename, const Eigen::MatrixXd& mat);
    static void SaveToFileAsciiInt(const string& filename, const Eigen::MatrixXi& mat);

	static void SaveToFileAsciiDouble(ofstream& out, const Eigen::MatrixXd& mat);
	static void SaveToFileAsciiInt(ofstream& out, const Eigen::MatrixXi& mat);

	static void OutputToMFormat(ofstream& out, const string& varName, const Eigen::MatrixXd& mat);
	static void OutputToMFormat(ofstream& out, const string& varName, const Eigen::VectorXd& vec);
	static void OutputToMFormat(ofstream& out, const string& varName, const Eigen::MatrixXi& mat);
	static void OutputToMFormat(ofstream& out, const string& varName, const Eigen::VectorXi& vec);
};

#endif