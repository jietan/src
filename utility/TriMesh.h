#ifndef _TRI_MESH
#define _TRI_MESH

#ifndef dDOUBLE
#define dDOUBLE
#endif

#include "ode/ode.h"
#include "stdafx.h"

#include <Eigen/Dense>

class TriMesh
{
public:
	TriMesh();
	TriMesh(const string& filename);
	~TriMesh();
	void ReadFromFile(const string& filename);
	const vector<Eigen::Vector3d>& GetVertexBuffer() const;
	const vector<Eigen::Vector3i>& GetIndexBuffer() const;
	const double* GetGLVertexArray() const;
	const double* GetGLNormalArray() const;
	dReal* GetVerticesODEFormat(int& numVertices) const;
	dTriIndex* GetIndicesODEFormat(int& numIndices) const;
	int GetNumTriangles() const;
	const string& GetFilePath() const { return mFileName; }
private:
	
	void buildODEBuffers();
	void convertToGlBuffers();

	vector<Eigen::Vector3d> mVertexBuffer;
	vector<Eigen::Vector3i> mIndexBuffer;

	vector<double> mGLVertexBuffer;
	vector<double> mGLNormalBuffer;


	dReal* mODEVertices;
	dTriIndex* mODEIndices;
	string mFileName;
};
#endif
