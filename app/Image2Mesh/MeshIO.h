#ifndef _MESH_IO_H
#define _MESH_IO_H


#include <iostream>
#include <vector>
#include <string>
#include <fstream>
using namespace std;

#include <Eigen/Dense>
#include "PoissonSurface/Geometry.h"

template< class Vertex >
void SaveMesh(const string& filename, CoredMeshData< Vertex >* mesh)
{
	PlyWritePolygons(const_cast<char*>(filename.c_str()), mesh, PLY_ASCII);
}

void ReadPointCloud(const string& filename, vector<Eigen::Vector3f>& points, vector<Eigen::Vector3f>& normals);
void SavePointCloud(const string& filename, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3i>& colors, const vector<Eigen::Vector3f>& normals);

#endif

