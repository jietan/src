#ifndef _MESH_IO_H
#define _MESH_IO_H


#include <iostream>
#include <vector>
#include <string>
#include <fstream>
using namespace std;

#include <Eigen/Dense>
#include "PoissonSurface/Geometry.h"
#include "PoissonSurface/Ply.h"

void SaveMesh(const string& filename, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3i>& faces);
void SaveMesh(const string& filename, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3i>& colors, const vector<Eigen::Vector3i>& faces);
template< class Vertex >
void SaveMesh(const string& filename, CoredMeshData< Vertex >* mesh)
{
	PlyWritePolygons(const_cast<char*>(filename.c_str()), mesh, PLY_ASCII);
}

void ReadMesh(const string& filename, vector<Eigen::Vector3f>* vertices, vector<Eigen::Vector3i>* faces);
void ReadPointCloud(const string& filename, vector<Eigen::Vector3f>& points, vector<Eigen::Vector3f>& normals, bool bAppend = false);
void SavePointCloud(const string& filename, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3i>& colors, const vector<Eigen::Vector3f>& normals);
void SavePointCloud(const string& filename, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals);

#endif

