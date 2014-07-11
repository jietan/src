#include "pointCloudToPrimitive/RansacShapeDetector.h"
#include "pointCloudToPrimitive/PlanePrimitiveShapeConstructor.h"
#include "pointCloudToPrimitive/CylinderPrimitiveShapeConstructor.h"
#include "pointCloudToPrimitive/SpherePrimitiveShapeConstructor.h"
#include "pointCloudToPrimitive/ConePrimitiveShapeConstructor.h"
#include "pointCloudToPrimitive/TorusPrimitiveShapeConstructor.h"

#include <string>
#include <vector>
using namespace std;
#include "PoissonSurface/Geometry.h"
#include "PoissonSurface/Ply.h"
#include "PoissonSurface/PointStream.h" 
#include <Eigen/Dense>

typedef float Real;

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


void ComputeBoundingBox(const vector<Eigen::Vector3f>& points, Eigen::Vector3f& minPt, Eigen::Vector3f& maxPt)
{
	int numPoints = static_cast<int>(points.size());
	//CHECK(numPoints > 0) << "Empty point cloud to ComputeBoundingBox.";

	minPt = maxPt = points[0];
	for (int i = 1; i < numPoints; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (points[i][j] < minPt[j])
				minPt[j] = points[i][j];
			if (points[i][j] > maxPt[j])
				maxPt[j] = points[i][j];
		}
	}
}

int main()
{


	// fill or load point cloud from file
	vector<Eigen::Vector3f> points;
	vector<Eigen::Vector3f> normals;
	//ReadPointCloud("../../../Tables/399318621.826096/points.plysimplified.ply", points, normals, false);
	ReadPointCloud("../../../Tables/399318621.826096/combinedPointMeshDepthImage_Ortho_Front.ply", points, normals, false);
	ReadPointCloud("../../../Tables/399318621.826096/combinedPointMeshDepthImage_Ortho_Top.ply", points, normals, true);
	//ReadPointCloud("C:\\Users\\JieTan\\Documents\\MyProjects\\AdobeIntern\\src\\app\\Image2Mesh\\results\\simplifiedDepthFromMultiview_Ortho.ply", points, normals, false);
	Eigen::Vector3f minPt, maxPt;
	ComputeBoundingBox(points, minPt, maxPt);
	int numPoints = static_cast<int>(points.size());
	vector<Point> pts;
	pts.resize(numPoints);
	for (int i = 0; i < numPoints; ++i)
	{
		pts[i].pos = Vec3f(points[i][0], points[i][1], points[i][2]);
		pts[i].normal = Vec3f(normals[i][0], normals[i][1], normals[i][2]);
	}
	PointCloud pc(&pts[0], pts.size());
	pc.setBBox(Vec3f(minPt.data()), Vec3f(maxPt.data()));
	// don't forget to set the bbox in pc

	RansacShapeDetector::Options ransacOptions;
	ransacOptions.m_epsilon = 0.01f * pc.getScale(); // set distance threshold to .01f of bounding box width
	// NOTE: Internally the distance threshold is taken as 3 * ransacOptions.m_epsilon!!!
	ransacOptions.m_bitmapEpsilon = .02f * pc.getScale(); // set bitmap resolution to .02f of bounding box width
	// NOTE: This threshold is NOT multiplied internally!
	ransacOptions.m_normalThresh = .9f; // this is the cos of the maximal normal deviation
	ransacOptions.m_minSupport = 300; // this is the minimal number of points required for a primitive
	ransacOptions.m_probability = .001f; // this is the "probability" with which a primitive is overlooked

	RansacShapeDetector detector(ransacOptions); // the detector object

	// set which primitives are to be detected by adding the respective constructors
	detector.Add(new PlanePrimitiveShapeConstructor());
	detector.Add(new SpherePrimitiveShapeConstructor());
	detector.Add(new CylinderPrimitiveShapeConstructor());
	//detector.Add(new ConePrimitiveShapeConstructor());
	//detector.Add(new TorusPrimitiveShapeConstructor());

	MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes; // stores the detected shapes
	size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes); // run detection
	cout << "Number of shapes: " << shapes.size() << endl;
	cout << "Remainings: " << remaining << endl;
	int sum = 0;
	for (int i = 0; i < shapes.size(); ++i)
	{
		vector<Eigen::Vector3f> clusteredP;
		vector<Eigen::Vector3f> clusteredN;
		cout << "Shape " << i << ": " << shapes[i].first->Identifier() << ": " << shapes[i].second << endl;

		for (int j = pc.size() - sum - shapes[i].second; j < pc.size() - sum; ++j)
		{
			float x, y, z;
			pc[j].pos.getValue(x, y, z);
			clusteredP.push_back(Eigen::Vector3f(x, y, z));
			pc[j].normal.getValue(x, y, z);
			clusteredN.push_back(Eigen::Vector3f(x, y, z));
		}
		char outFileName[512];
		sprintf(outFileName, "results\\part%03d.ply", i);
		SavePointCloud(outFileName, clusteredP, clusteredN);
		sum += shapes[i].second;
	}
	// returns number of unassigned points
	// the array shapes is filled with pointers to the detected shapes
	// the second element per shapes gives the number of points assigned to that primitive (the support)
	// the points belonging to the first shape (shapes[0]) have been sorted to the end of pc,
	// i.e. into the range [ pc.size() - shapes[0].second, pc.size() )
	// the points of shape i are found in the range
	// [ pc.size() - \sum_{j=0..i} shapes[j].second, pc.size() - \sum_{j=0..i-1} shapes[j].second )
}