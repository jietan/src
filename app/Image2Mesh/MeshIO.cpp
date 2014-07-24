#include "MeshIO.h"
#include "PoissonSurface/PointStream.h"
#include <glog/logging.h>
using namespace google;

typedef float Real;

void ReadMesh(const string& filename, vector<Eigen::Vector3f>* points, vector<Eigen::Vector3i>* faces)
{
	vector< PlyVertex< float > > vertices;
	vector< std::vector< int > > polygons;
	int ft;
	PlyReadPolygons(const_cast<char*>(filename.c_str()), vertices, polygons, PlyVertex< float >::Properties, PlyVertex< float >::Components, ft);

	int numVertices = static_cast<int>(vertices.size());
	points->resize(numVertices);
	for (int i = 0; i < numVertices; ++i)
	{
		points->at(i) = Eigen::Vector3f(vertices[i].point[0], vertices[i].point[1], vertices[i].point[2]);
	}
	int numFaces = static_cast<int>(polygons.size());
	faces->resize(numFaces);
	for (int i = 0; i < numFaces; ++i)
	{
		CHECK(polygons[i].size() == 3) << "Non-triangular polygon detected in main().";
		faces->at(i) = Eigen::Vector3i(polygons[i][0], polygons[i][1], polygons[i][2]);
	}
}

void SaveMesh(const string& filename, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3i>& faces)
{
	CoredVectorMeshData<PlyVertex< Real > > mesh;
	int numPoints = static_cast<int>(points.size());
	for (int i = 0; i < numPoints; ++i)
	{
		PlyVertex<Real> p;
		p.point = Point3D<Real>(points[i][0], points[i][1], points[i][2]);
		mesh.addOutOfCorePoint(p);
	}

	int numFaces = static_cast<int>(faces.size());
	for (int i = 0; i < numFaces; ++i)
	{
		vector<CoredVertexIndex> vertexIndices;
		vertexIndices.resize(3);
		for (int j = 0; j < 3; ++j)
		{
			vertexIndices[j].idx = faces[i][j];
		}
		mesh.addPolygon(vertexIndices);
	}

	SaveMesh(filename, &mesh);
}

void SaveMesh(const string& filename, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3i>& colors, const vector<Eigen::Vector3i>& faces)
{
	CoredVectorMeshData<PlyColorVertex < Real > > mesh;
	int numPoints = static_cast<int>(points.size());
	for (int i = 0; i < numPoints; ++i)
	{
		PlyColorVertex<Real> p;
		p.point = Point3D<Real>(points[i][0], points[i][1], points[i][2]);
		p.color[0] = static_cast<unsigned char>(colors[i][0]);
		p.color[1] = static_cast<unsigned char>(colors[i][1]);
		p.color[2] = static_cast<unsigned char>(colors[i][2]);
		mesh.addOutOfCorePoint(p);
	}

	int numFaces = static_cast<int>(faces.size());
	for (int i = 0; i < numFaces; ++i)
	{
		vector<CoredVertexIndex> vertexIndices;
		vertexIndices.resize(3);
		for (int j = 0; j < 3; ++j)
		{
			vertexIndices[j].idx = faces[i][j];
		}
		mesh.addPolygon(vertexIndices);
	}

	SaveMesh(filename, &mesh);
}

void ReadPointCloud(const string& filename, vector<Eigen::Vector3f>& points, vector<Eigen::Vector3f>& normals, bool bAppend)
{
	if (!bAppend)
	{
		points.clear();
		normals.clear();
	}

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

void SavePointCloud(const string& filename, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals)
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
void SavePointCloud(const string& filename, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3i>& colors, const vector<Eigen::Vector3f>& normals)
{
	SavePointCloud(filename, points, normals);
}
