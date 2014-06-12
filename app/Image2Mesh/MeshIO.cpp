#include "MeshIO.h"
#include "PoissonSurface/Ply.h"
#include "PoissonSurface/PointStream.h"

typedef float Real;

void SaveMask(const string& filename, const vector<vector<vector<int> > >& mask)
{
	int width = 0, height = 0;
	height = static_cast<int>(mask.size());
	if (height)
		width = static_cast<int>(mask[0].size());
	ofstream outFile(filename.c_str(), ios::out | ios::binary);
	outFile.write((char*)&width, sizeof(width));
	outFile.write((char*)&height, sizeof(height));

	for (int i = 0; i < height; ++i)
	{
		for (int j = 0; j < width; ++j)
		{
			int len = static_cast<int>(mask[i][j].size());
			outFile.write((char*)&len, sizeof(len));
			for (int k = 0; k < len; ++k)
			{
				outFile.write((char*)&(mask[i][j][k]), sizeof(int));

			}
		}
	}
}

void ReadMask(const string& filename, vector<vector<vector<int> > >& mask)
{
	int width, height, maskValue;
	ifstream inFile(filename.c_str(), ios::in | ios::binary);
	inFile.read((char*)&width, sizeof(width));
	inFile.read((char*)&height, sizeof(height));
	mask.resize(height);
	for (int i = 0; i < height; ++i)
	{
		mask[i].resize(width);
		for (int j = 0; j < width; ++j)
		{
			int len = 0;
			inFile.read((char*)&len, sizeof(int));
			if (len)
			{
				for (int k = 0; k < len; ++k)
				{
					inFile.read((char*)&maskValue, sizeof(int));
					mask[i][j].push_back(maskValue);
				}
			}
		}
	}
}

void ReadPointCloud(const string& filename, vector<Eigen::Vector3f>& points, vector<Eigen::Vector3f>& normals)
{
	points.clear();
	normals.clear();

	PointStream< Real >* pointStream = new PLYPointStream< Real >(filename.c_str());

	Point3D< Real > p, n;
	while (pointStream->nextPoint(p, n))
	{
		Eigen::Vector3f pt(p[0], p[1], p[2]);
		Eigen::Vector3f normal(n[0], n[1], n[2]);
		points.push_back(pt);
		normals.push_back(normal);
	}

}


void SavePointCloud(const string& filename, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3i>& colors, const vector<Eigen::Vector3f>& normals)
{

	std::vector<PlyOrientedVertex<Real> > vertices;
	int numPoints = static_cast<int>(points.size());
	vertices.resize(numPoints);
	for (int i = 0; i < numPoints; ++i)
	{
		vertices[i] = PlyOrientedVertex<Real>(Point3D<Real>(points[i][0], points[i][1], points[i][2]), Point3D<Real>(normals[i][0], normals[i][1], normals[i][2]));
	}
	PlyWritePoints(const_cast<char*>(filename.c_str()), vertices, PlyOrientedVertex<Real>::Properties, PlyOrientedVertex<Real>::Components, PLY_BINARY_NATIVE);
}
