#include "stdafx.h"
#include "myFileReader.h"

void ReadDofs(const string& filename, vector<VectorXd>& data)
{
	ifstream inFile(filename.c_str());
	
	while(inFile.good())
	{
		int dim;
		inFile >> dim;
		VectorXd dof = VectorXd::Zero(dim);
		for (int i = 0; i < dim; ++i)
		{
			inFile >> skipws >> dof[i];
		}
		data.push_back(dof);
	}
	data.erase(data.end() - 1);
}