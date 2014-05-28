#include "stdafx.h"
#include "TriMesh.h"

TriMesh::TriMesh() : mODEIndices(NULL), mODEVertices(NULL)
{

}
TriMesh::TriMesh(const string& filename) : mODEIndices(NULL), mODEVertices(NULL)
{
	ReadFromFile(filename);
}
TriMesh::~TriMesh()
{
	if (mODEVertices)
		delete[] mODEVertices;
	if (mODEIndices)
		delete[] mODEIndices;
}
void TriMesh::ReadFromFile(const string& filename)
{
	mVertexBuffer.clear();
	mIndexBuffer.clear();
	mFileName = filename;

	ifstream inFile(filename.c_str());
	if (!inFile.good())
	{
		LOG(WARNING) << filename << " is not found.";
		return;
	}
	while (inFile.good())
	{
		char line[1024];
		inFile.getline(line, 1024);
		char suffix;
		float x, y, z;
		int f1, f2, f3;
		if (line[0] == 'v')
		{
			sscanf(line, "%c %f %f %f\n", &suffix, &x, &y, &z);
			mVertexBuffer.push_back(Eigen::Vector3d(x, y, z));
		}
		else if (line[0] == 'f')
		{
			sscanf(line, "%c %d %d %d\n", &suffix, &f1, &f2, &f3);
			mIndexBuffer.push_back(Eigen::Vector3i(f1 - 1, f2 - 1, f3 - 1));
		}
	}
	buildODEBuffers();
	convertToGlBuffers();
}
const vector<Eigen::Vector3d>& TriMesh::GetVertexBuffer() const
{
	return mVertexBuffer;
}
const vector<Eigen::Vector3i>& TriMesh::GetIndexBuffer() const
{
	return mIndexBuffer;
}

dReal* TriMesh::GetVerticesODEFormat(int& numVertices) const
{
	numVertices = static_cast<int>(mVertexBuffer.size());
	return mODEVertices;
}
dTriIndex* TriMesh::GetIndicesODEFormat(int& numIndices) const
{
	numIndices = static_cast<int>(mIndexBuffer.size()) * 3;
	return mODEIndices;
}

void TriMesh::buildODEBuffers()
{
	if (mODEVertices)
		delete[] mODEVertices;
	if (mODEIndices)
		delete[] mODEIndices;

	int numVertices = static_cast<int>(mVertexBuffer.size());
	int numFaces = static_cast<int>(mIndexBuffer.size());

	mODEVertices = new dReal[4 * numVertices];
	mODEIndices = new dTriIndex[3 * numFaces];

	for (int i = 0; i < numVertices; ++i)
	{
		mODEVertices[4 * i + 0] = mVertexBuffer[i][0];
		mODEVertices[4 * i + 1] = mVertexBuffer[i][1];
		mODEVertices[4 * i + 2] = mVertexBuffer[i][2];
	}
	for (int i = 0; i < numFaces; ++i)
	{
		mODEIndices[3 * i + 0] = mIndexBuffer[i][0];
		mODEIndices[3 * i + 1] = mIndexBuffer[i][1];
		mODEIndices[3 * i + 2] = mIndexBuffer[i][2];
	}
}

void TriMesh::convertToGlBuffers()
{
	const vector<Eigen::Vector3i>& indexBuffer = GetIndexBuffer();
	const vector<Eigen::Vector3d>& vertexBuffer = GetVertexBuffer();

	int numFaces = static_cast<int>(indexBuffer.size());
	mGLVertexBuffer.resize(numFaces * 9);
	mGLNormalBuffer.resize(numFaces * 9);

	for (int i = 0; i < numFaces; ++i)
	{
		Eigen::Vector3d v[3];
		
		for (int j = 0; j < 3; ++j)
		{
			v[j] = vertexBuffer[indexBuffer[i][j]];
		}
		Eigen::Vector3d e01 = v[1] - v[0];
		Eigen::Vector3d e02 = v[2] - v[0];
		Eigen::Vector3d n = e01.cross(e02);
		n = n.normalized();
		for (int j = 0; j < 3; ++j)
		{
			mGLVertexBuffer[i * 9 + 3 * j + 0] = v[j][0];
			mGLVertexBuffer[i * 9 + 3 * j + 1] = v[j][1];
			mGLVertexBuffer[i * 9 + 3 * j + 2] = v[j][2];

			mGLNormalBuffer[i * 9 + 3 * j + 0] = n[0];
			mGLNormalBuffer[i * 9 + 3 * j + 1] = n[1];
			mGLNormalBuffer[i * 9 + 3 * j + 2] = n[2];
		}
	}
}

const double* TriMesh::GetGLVertexArray() const
{
	return &(mGLVertexBuffer[0]);
}
const double* TriMesh::GetGLNormalArray() const
{
	return &(mGLNormalBuffer[0]);
}
int TriMesh::GetNumTriangles() const
{
	return static_cast<int>(mIndexBuffer.size());
}