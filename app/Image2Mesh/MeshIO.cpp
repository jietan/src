#include "MeshIO.h"
#include "PoissonSurface/Ply.h"
#include "PoissonSurface/PointStream.h"

typedef float Real;


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
