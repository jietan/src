#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <float.h>
#ifdef _WIN32
#include <Windows.h>
#include <Psapi.h>
#endif // _WIN32
#include "PoissonSurface/Time.h"
#include "PoissonSurface/MarchingCubes.h"
#include "PoissonSurface/Octree.h"
#include "PoissonSurface/SparseMatrix.h"
#include "PoissonSurface/PPolynomial.h"
#include "PoissonSurface/Ply.h"
#include "PoissonSurface/CmdLineParser.h"
//#include "PoissonSurface/PlyFile.h"
#include "PoissonSurface/MemoryUsage.h"
#include "omp.h"
#include <stdarg.h>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <math.h>
using namespace std;
using namespace std;

#include <Eigen/Dense>

#include "DepthImage.h"
#include "grid.h"
#include "ImageSegmentation.h"
#include "DepthSegmentation.h"
#include "DepthInpainting.h"
#include "DepthCamera.h"
#include "utility/ConfigManager.h"
#include "MultilayerDepthImage.h"
#include "MeshIO.h"
#include "PatchMatch.h"
#include "distanceMetric.h"
#include "CVImageHelper.h"
#include "ImageInpainting.h"
#include "Part.h"
#include "BoxFromRectangles.h"
#include "SymmetryBuilder.h"
#include "BoxVoter.h"
#include "boxExtruder.h"
#include "barExtruder.h"

#include "pointCloudToPrimitive/RansacShapeDetector.h"
#include "pointCloudToPrimitive/PlanePrimitiveShape.h"
#include "pointCloudToPrimitive/CylinderPrimitiveShape.h"


//#include "gflags/gflags.h"
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include "glog/logging.h"
using namespace google;

#define MAX_FILENAME_LEN 512

//string gDataFolder = "../../../Tables/";
////string gDataName = "399318621.826096";
//string gDataName = "399400812.46835";
char* outputFile = NULL;
double gDepthThreshold = 2.5;
const int gDepthImageWidth = 640;
const int gDepthImageHeight = 480;
const float gFocalLenth = 525.0f;
int gCameraProjectionType = 0;
float gCameraWidth = 2.0;
string gTableFolder;
string gTableId;

int echoStdout=0;
typedef float Real;

cv::Mat ImagingPart(const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals, PrimitiveShape* prim);
vector<PrimitiveShape*> ReadPrimitives();
vector<Part> ReadParts(const vector<PrimitiveShape*>& allPrimitives);
vector<BoxFromRectangles> ReadBoxes(bool bAfterVote);

void DumpOutput( const char* format , ... )
{
	if( outputFile )
	{
		FILE* fp = fopen( outputFile , "a" );
		va_list args;
		va_start( args , format );
		vfprintf( fp , format , args );
		fclose( fp );
		va_end( args );
	}
	if( echoStdout )
	{
		va_list args;
		va_start( args , format );
		vprintf( format , args );
		va_end( args );
	}
}
void DumpOutput2( char* str , const char* format , ... )
{
	if( outputFile )
	{
		FILE* fp = fopen( outputFile , "a" );
		va_list args;
		va_start( args , format );
		vfprintf( fp , format , args );
		fclose( fp );
		va_end( args );
	}
	if( echoStdout )
	{
		va_list args;
		va_start( args , format );
		vprintf( format , args );
		va_end( args );
	}
	va_list args;
	va_start( args , format );
	vsprintf( str , format , args );
	va_end( args );
	if( str[strlen(str)-1]=='\n' ) str[strlen(str)-1] = 0;
}

#include "PoissonSurface/MultiGridOctreeData.h"
#ifdef MAX_MEMORY_GB
#undef MAX_MEMORY_GB
#endif // MAX_MEMORY_GB
//#define MAX_MEMORY_GB 8

cmdLineString
In( "in" ) ,
Out( "out" ) ,
VoxelGrid( "voxel" ) ,
XForm( "xForm" );

cmdLineReadable
#ifdef _WIN32
Performance( "performance" ) ,
#endif // _WIN32
ShowResidual( "showResidual" ) ,
NoComments( "noComments" ) ,
PolygonMesh( "polygonMesh" ) ,
Confidence( "confidence" ) ,
NormalWeights( "normalWeight" ) ,
NonManifold( "nonManifold" ) ,
ASCII( "ascii" ) ,
Density( "density" ) ,
Verbose( "verbose" );

cmdLineInt
Depth( "depth" , 8 ) ,
SolverDivide( "solverDivide" , 8 ) ,
IsoDivide( "isoDivide" , 8 ) ,
KernelDepth( "kernelDepth" ) ,
AdaptiveExponent( "adaptiveExp" , 1 ) ,
MinIters( "minIters" , 24 ) ,
FixedIters( "iters" , -1 ) ,
VoxelDepth( "voxelDepth" , -1 ) ,
#if 1
#ifdef _WIN32
#pragma message( "[WARNING] Setting default min-depth to 5" )
#endif // _WIN32
MinDepth( "minDepth" , 5 ) ,
#else
MinDepth( "minDepth" , 0 ) ,
#endif
MaxSolveDepth( "maxSolveDepth" ) ,
BoundaryType( "boundary" , 1 ) ,
Threads( "threads" , omp_get_num_procs() );

cmdLineFloat
SamplesPerNode( "samplesPerNode" , 1.f ) ,
Scale( "scale" , 1.1f ) ,
SolverAccuracy( "accuracy" , float(1e-3) ) ,
PointWeight( "pointWeight" , 4.f );


cmdLineReadable* params[] =
{
	&In , &Depth , &Out , &XForm ,
	&SolverDivide , &IsoDivide , &Scale , &Verbose , &SolverAccuracy , &NoComments ,
	&KernelDepth , &SamplesPerNode , &Confidence , &NormalWeights , &NonManifold , &PolygonMesh , &ASCII , &ShowResidual , &MinIters , &FixedIters , &VoxelDepth ,
	&PointWeight , &VoxelGrid , &Threads , &MinDepth , &MaxSolveDepth ,
	&AdaptiveExponent , &BoundaryType ,
	&Density ,
#ifdef _WIN32
	&Performance ,
#endif // _WIN32
};


int ReadCorrespondence(vector<pair<string, string> >& correspondence)
{
	char filename[MAX_FILENAME_LEN];
	memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
	sprintf(filename, "%s/%s/input.match", gTableFolder.c_str(), gTableId.c_str());
	int numCorrespondence = 0;
	ifstream inFile(filename);
	string depthFilename, colorFilename;
	while (inFile.good())
	{
		inFile >> depthFilename >> colorFilename;
		correspondence.push_back(pair<string, string>(depthFilename, colorFilename));
		numCorrespondence++;
	}
	correspondence.erase(correspondence.end() - 1);
	return numCorrespondence - 1;
}

int ReadCameraPoses(vector<Eigen::MatrixXf>& cameraPose)
{
	char filename[MAX_FILENAME_LEN];
	memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
	sprintf(filename, "%s/%s/poses.txt", gTableFolder.c_str(), gTableId.c_str());
	int numViews = 0;

	ifstream inFile(filename);
	
	Eigen::MatrixXf pose = Eigen::MatrixXf::Zero(4, 4);
	while (inFile.good())
	{
		int ithFrame;
		inFile >> ithFrame >> ithFrame >> ithFrame;
		inFile >> pose(0, 0) >> pose(0, 1) >> pose(0, 2) >> pose(0, 3);
		inFile >> pose(1, 0) >> pose(1, 1) >> pose(1, 2) >> pose(1, 3);
		inFile >> pose(2, 0) >> pose(2, 1) >> pose(2, 2) >> pose(2, 3);
		inFile >> pose(3, 0) >> pose(3, 1) >> pose(3, 2) >> pose(3, 3);

		cameraPose.push_back(pose);
		numViews++;
	}
	cameraPose.erase(cameraPose.end() - 1);
	return numViews - 1;
}

cv::Mat ReadDepthImage(const string& filename)
{
	cv::Mat ret;
	ret = cv::imread(filename.c_str(), CV_LOAD_IMAGE_UNCHANGED);
	return ret;
}

cv::Mat ReadColorImage(const string& filename)
{
	cv::Mat ret;
	ret = cv::imread(filename.c_str(), CV_LOAD_IMAGE_COLOR);
	return ret;
}

void ColorImageToColors(const cv::Mat& colorImg, vector<Eigen::Vector3i>& colors)
{
	for (int v =  0; v < colorImg.rows; ++v)
	{
		for (int u = 0; u < colorImg.cols; ++u)
		{
			cv::Vec3b cl = colorImg.at<cv::Vec3b>(v, u);
			Eigen::Vector3i color(cl[2], cl[1], cl[0]);
			colors.push_back(color);
		}
	}
}


template< class Vertex >
int Points2Mesh(const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals, CoredMeshData< Vertex >& mesh)
{
	int i;
	int paramNum = sizeof(params)/sizeof(cmdLineReadable*);
	int commentNum=0;
	char **comments;

	comments = new char*[paramNum+7];
	for( i=0 ; i<paramNum+7 ; i++ ) comments[i] = new char[1024];

	echoStdout=1;

	XForm4x4< Real > xForm , iXForm;
	xForm = XForm4x4< Real >::Identity();
	iXForm = xForm.inverse();

	DumpOutput2( comments[commentNum++] , "Running Screened Poisson Reconstruction (Version 5.71)\n" );

	double t;
	double tt=Time();
	Real isoValue = 0;

	Octree< 2 , false > tree;
	tree.threads = Threads.value;
	//In.set = true;
	//In.writeValue(filename);
	if( !MaxSolveDepth.set ) MaxSolveDepth.value = Depth.value;
	if( SolverDivide.value<MinDepth.value )
	{
		fprintf( stderr , "[WARNING] %s must be at least as large as %s: %d>=%d\n" , SolverDivide.name , MinDepth.name , SolverDivide.value , MinDepth.value );
		SolverDivide.value = MinDepth.value;
	}
	if( IsoDivide.value<MinDepth.value )
	{
		fprintf( stderr , "[WARNING] %s must be at least as large as %s: %d>=%d\n" , IsoDivide.name , MinDepth.name , IsoDivide.value , IsoDivide.value );
		IsoDivide.value = MinDepth.value;
	}

	OctNode< TreeNodeData< false > , Real >::SetAllocator( MEMORY_ALLOCATOR_BLOCK_SIZE );

	t=Time();
	int kernelDepth = KernelDepth.set ?  KernelDepth.value : Depth.value-2;

	tree.setBSplineData( Depth.value , BoundaryType.value );
	if( kernelDepth>Depth.value )
	{
		fprintf( stderr,"[ERROR] %s can't be greater than %s: %d <= %d\n" , KernelDepth.name , Depth.name , KernelDepth.value , Depth.value );
		return EXIT_FAILURE;
	}

	t=Time() , tree.maxMemoryUsage=0;

	char filename[MAX_FILENAME_LEN];
	memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
	sprintf(filename, "%s/%s/points.ply", gTableFolder.c_str(), gTableId.c_str());

	//int pointCount = tree.setTree( filename, Depth.value , MinDepth.value , kernelDepth , Real(SamplesPerNode.value) , Scale.value , Confidence.set , NormalWeights.set , PointWeight.value , AdaptiveExponent.value , xForm );
	int pointCount = tree.setTree( points, normals, Depth.value , MinDepth.value , kernelDepth , Real(SamplesPerNode.value) , Scale.value , Confidence.set , NormalWeights.set , PointWeight.value , AdaptiveExponent.value , xForm );
	tree.ClipTree();
	tree.finalize( IsoDivide.value );

	double maxMemoryUsage = 0;
	DumpOutput2( comments[commentNum++] , "#             Tree set in: %9.1f (s), %9.1f (MB)\n" , Time()-t , tree.maxMemoryUsage );
	DumpOutput( "Input Points: %d\n" , pointCount );
	DumpOutput( "Leaves/Nodes: %lld/%lld\n" , tree.tree.leaves() , tree.tree.nodes() );
	DumpOutput( "Memory Usage: %.3f MB\n" , float( MemoryInfo::Usage() )/(1<<20) );

	maxMemoryUsage = tree.maxMemoryUsage;
	t=Time() , tree.maxMemoryUsage=0;
	tree.SetLaplacianConstraints();
	DumpOutput2( comments[commentNum++] , "#      Constraints set in: %9.1f (s), %9.1f (MB)\n" , Time()-t , tree.maxMemoryUsage );
	DumpOutput( "Memory Usage: %.3f MB\n" , float( MemoryInfo::Usage())/(1<<20) );
	maxMemoryUsage = std::max< double >( maxMemoryUsage , tree.maxMemoryUsage );

	t=Time() , tree.maxMemoryUsage=0;
	tree.LaplacianMatrixIteration( SolverDivide.value, ShowResidual.set , MinIters.value , SolverAccuracy.value , MaxSolveDepth.value , FixedIters.value );
	DumpOutput2( comments[commentNum++] , "# Linear system solved in: %9.1f (s), %9.1f (MB)\n" , Time()-t , tree.maxMemoryUsage );
	DumpOutput( "Memory Usage: %.3f MB\n" , float( MemoryInfo::Usage() )/(1<<20) );
	maxMemoryUsage = std::max< double >( maxMemoryUsage , tree.maxMemoryUsage );

	if( Verbose.set ) tree.maxMemoryUsage=0;
	t=Time();
	isoValue = tree.GetIsoValue();
	DumpOutput( "Got average in: %f\n" , Time()-t );
	DumpOutput( "Iso-Value: %e\n" , isoValue );

	tree.GetMCIsoTriangles( isoValue , IsoDivide.value , &mesh , 0 , 1 , !NonManifold.set , PolygonMesh.set );
	return EXIT_SUCCESS;
}


void ComputeBoundingBox(const vector<Eigen::Vector3f>& points, Eigen::Vector3f& minPt, Eigen::Vector3f& maxPt)
{
	int numPoints = static_cast<int>(points.size());
	CHECK(numPoints > 0) << "Empty point cloud to ComputeBoundingBox.";
	
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

void MergePoints(const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals, const vector<int>& indices, vector<Eigen::Vector3f>& newPoints, vector<Eigen::Vector3f>& newNormals)
{
	if (indices.empty()) return;
	int numPoints = static_cast<int>(indices.size());
	Eigen::Vector3f avgPoint = Eigen::Vector3f::Zero();
	Eigen::Vector3f avgNormal = Eigen::Vector3f::Zero();

	for (int i = 0; i < numPoints; ++i)
	{
		avgPoint += points[indices[i]];
		avgNormal += normals[indices[i]];
	}
	avgPoint /= numPoints;
	avgNormal = avgNormal.normalized();
	newPoints.push_back(avgPoint);
	newNormals.push_back(avgNormal);
}

void SubsamplePointCloud(const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals, float mergeDistance, vector<Eigen::Vector3f>& newPoints, vector<Eigen::Vector3f>& newNormals)
{
	Eigen::Vector3f minPoint, maxPoint;
	ComputeBoundingBox(points, minPoint, maxPoint);
	//for (int i = 0; i < 3; ++i)
	//{
	//	minPoint[i] = max<float>(-1, minPoint[i]);
	//	//maxPoint[i] = min<double>(2.5, maxPoint[i]);
	//}
	//minPoint[0] = max<float>(-1.f, minPoint[0]);
	//minPoint[1] = max<float>(-1.f, minPoint[1]);
	//minPoint[2] = max<float>(0.f, minPoint[2]);
	//maxPoint[0] = min<float>(2.8f, maxPoint[0]);
	//maxPoint[1] = min<float>(2.9f, maxPoint[1]);
	//maxPoint[2] = min<float>(3.8f, maxPoint[2]);
	

	
	cout << minPoint << endl;
	cout << maxPoint << endl;
	

	Eigen::Vector3f resolutionFloat = (maxPoint - minPoint) / mergeDistance;
	Eigen::Vector3i resolution(int(resolutionFloat[0]) + 1, int(resolutionFloat[1]) + 1, int(resolutionFloat[2]) + 1);
	resolutionFloat = Eigen::Vector3f(static_cast<float>(resolution[0]), static_cast<float>(resolution[1]), static_cast<float>(resolution[2]));
	if (resolution[0] * resolution[1] * resolution[2] > 1000 * 1000 * 1000)
	{
		CHECK(0) << "Too much memory required to subsample point cloud!!!";
	}
	cout << resolution << endl;
	maxPoint = minPoint + mergeDistance * resolutionFloat;

	Grid<vector<int> > grid(minPoint, maxPoint, resolution);
	int numPoints = static_cast<int>(points.size());
	int xIdx, yIdx, zIdx;
	for (int i = 0; i < numPoints; ++i)
	{
		if (i % 10000 == 0)
			LOG(INFO) << "Processing " << i << "th point out of " << numPoints << endl;
		if (grid.Pos2Idx(points[i], xIdx, yIdx, zIdx))
			grid.Data(xIdx, yIdx, zIdx).push_back(i);
	}
	newPoints.clear();
	newNormals.clear();
	for (int i = 0; i < resolution[0]; ++i)
	{
		LOG(INFO) << "Merging " << i << "th row." << endl;
		for (int j = 0; j < resolution[1]; ++j)
		{
			for (int k = 0; k < resolution[2]; ++k)
			{
				const vector<int>& indices = grid.Data(i, j, k);
				MergePoints(points, normals, indices, newPoints, newNormals);
			}
		}
	}
}

void AlignBoundingBox(vector<Eigen::Vector3f>& points, vector<Eigen::Vector3f>& normals)
{
	int numPoints = static_cast<int>(points.size());
	Eigen::Vector3f avgPoint = Eigen::Vector3f::Zero();
	for (int i = 0; i < numPoints; ++i)
	{
		avgPoint += points[i];
	}
	avgPoint /= numPoints;
	for (int i = 0; i < numPoints; ++i)
	{
		points[i] -= avgPoint;
	}
	Eigen::MatrixXf pointMatrix = Eigen::MatrixXf(numPoints, 3);
	for (int i = 0; i < numPoints; ++i)
	{
		pointMatrix.row(i) = points[i];
	}
	Eigen::MatrixXf pointMatrixT = pointMatrix.transpose();
	Eigen::Matrix3f cov = pointMatrixT * pointMatrix;
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(cov);
	Eigen::Matrix3f rot = eigensolver.eigenvectors();
	Eigen::Matrix3f rotT = rot.transpose();
	for (int i = 0; i < numPoints; ++i)
	{
		points[i] = rotT * points[i];
		normals[i] = rotT * normals[i];
	}
}

cv::Mat1f PreprocessDepthImage(const cv::Mat& depthImg)
{
	int numRows = depthImg.rows;
	int numCols = depthImg.cols;

	cv::Mat1f ret;
	cv::Mat1f src;
	src.create(numRows, numCols);
	for (int i = 0; i < numRows; ++i)
	{
		for (int j = 0; j < numCols; ++j)
		{
			src.at<float>(i, j) = static_cast<float>(depthImg.at<ushort>(i, j));
		}
	}
	return src;
	//cv::bilateralFilter(src, ret, 3, 1000, 2);

	//cv::Mat1w imgVisualization;
	//imgVisualization.create(numRows, numCols);
	//for (int i = 0; i < numRows; ++i)
	//{
	//	for (int j = 0; j < numCols; ++j)
	//	{
	//		imgVisualization.at<ushort>(i, j) = static_cast<ushort>(ret.at<float>(i, j));
	//	}
	//}
	//imwrite("depthAfterPreprocessing.png", imgVisualization);
	//return ret;
}

void ConstructPointCloudFromDepthImages(int numFrames, const vector<pair<string, string> >& correspondences, const vector<Eigen::MatrixXf>& cameraPoses, vector<Eigen::Vector3f>& points, vector<Eigen::Vector3f>& normals)
{
	int depthImageStepSize = 1;
	DecoConfig::GetSingleton()->GetInt("Image2Mesh", "DepthImageStepSize", depthImageStepSize);
	for (int i = 0; i < numFrames; i += depthImageStepSize)
	{
		char filename[MAX_FILENAME_LEN];
		memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
		sprintf(filename, "%s/%s/%s", gTableFolder.c_str(), gTableId.c_str(), correspondences[i].first.c_str());
		DepthImage depthImg(filename);
		depthImg.SetCameraPose(cameraPoses[i]);
		depthImg.Process();
		points.insert(points.end(), depthImg.GetPoints().begin(), depthImg.GetPoints().end());
		normals.insert(normals.end(), depthImg.GetNormals().begin(), depthImg.GetNormals().end());

		//memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
		//sprintf(filename, "%s%s/%s", gDataFolder.c_str(), gDataName.c_str(), correspondences[i].second.c_str());
		//cv::Mat colorImg = ReadColorImage(filename);
		//ColorImageToColors(colorImg, colors);
		LOG(INFO) << "Processing " << i << " out of " << numFrames << " frames." << endl;

	}

}


void BuildMultiLayerDepthImage(const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals, const Eigen::Matrix4f& cameraPose)
{
	DepthCamera dCamera;

	dCamera.SetIntrinsicParameters(gDepthImageWidth, gDepthImageHeight, gFocalLenth);
	dCamera.SetExtrinsicParameters(cameraPose);
	
	dCamera.SetProjectionType(gCameraProjectionType);
	dCamera.SetOrthoWidth(gCameraWidth);
	dCamera.Capture(points, normals);
}

void ComputeVertexNormals(const vector<Eigen::Vector3f>& vertices, const vector<Eigen::Vector3i>& faces, vector<Eigen::Vector3f>* meshVertexNormals)
{
	CHECK(meshVertexNormals) << "MeshVertexNormals must be allocated in ComputeVertexNormals().";
	meshVertexNormals->clear();
	int numVertices = static_cast<int>(vertices.size());
	int numFaces = static_cast<int>(faces.size());
	meshVertexNormals->resize(numVertices, Eigen::Vector3f::Zero());
	for (int i = 0; i < numFaces; ++i)
	{
		Eigen::Vector3f n = TriangleNormal(vertices[faces[i][0]], vertices[faces[i][1]], vertices[faces[i][2]]);
		(*meshVertexNormals)[faces[i][0]] += n;
		(*meshVertexNormals)[faces[i][1]] += n;
		(*meshVertexNormals)[faces[i][2]] += n;
	}
	for (int i = 0; i < numVertices; ++i)
	{
		(*meshVertexNormals)[i].normalize();
	}
}
void MeshToPointCloud(int samplingDensity, const vector<Eigen::Vector3f>& vertices, const vector<Eigen::Vector3i>& faces, vector<Eigen::Vector3f>& meshVertexNormals, vector<Eigen::Vector3f>* points, vector<Eigen::Vector3f>* normals)
{
	CHECK(points && normals) << "points and normals should be allocated in MeshToPointCloud().";
	points->clear();
	normals->clear();
	int numFaces = static_cast<int>(faces.size());
	for (int i = 0; i < numFaces; ++i)
	{
		int ia = faces[i][0];
		int ib = faces[i][1];
		int ic = faces[i][2];

		float area = TriangleArea(vertices[ia], vertices[ib], vertices[ic]);
		float numSamples = samplingDensity * area;
		
		float numSamplesChancePart = numSamples - static_cast<int>(numSamples);
		double chance = RandDouble(0, 1);
		if (chance <= numSamplesChancePart)
			numSamples += 1;
		for (int j = 0; j < numSamples; ++j)
		{
			Eigen::Vector3f p, n;
			RandomPointInTriangle(vertices[ia], vertices[ib], vertices[ic], meshVertexNormals[ia], meshVertexNormals[ib], meshVertexNormals[ic], &p, &n);
			if (n[0] != n[0]) continue;
			points->push_back(p);
			normals->push_back(n);
		}
	}
}

void RemoveRedundantPoints(const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals, const vector<Eigen::Vector3f>& refPoints, vector<Eigen::Vector3f>* truePoints, vector<Eigen::Vector3f>* trueNormals)
{
	CHECK(truePoints && trueNormals) << "True points and normals should be allocated in RemoveRedundantPoints().";
	truePoints->clear();
	trueNormals->clear();
	sehoon::ann::KDTree tree;
	int numPoints = static_cast<int>(refPoints.size());
	tree.setDim(3);
	for (int i = 0; i < numPoints; ++i)
	{
		if (i % 10000 == 0)
			LOG(INFO) << "Finish adding " << i << "th points out of " << numPoints;
		tree.add(refPoints[i]);
	}
	tree.initANN();

	numPoints = static_cast<int>(points.size());

	for (int i = 0; i < numPoints; ++i)
	{
		vector<int> idx = tree.kSearch(points[i], 1);
		const Eigen::Vector3f& nnp = refPoints[idx[0]];
		if ((nnp - points[i]).norm() < 0.005)
		{
			truePoints->push_back(points[i]);
			trueNormals->push_back(normals[i]);
		}
	}

}

int main(int argc, char** argv)
{
	//ParseCommandLineFlags(&argc, &argv, true);
	InitGoogleLogging(argv[0]);
	
#ifdef NDEBUG
	string LogFolder = "../logs";
	FLAGS_log_dir = LogFolder;
#endif
	FLAGS_alsologtostderr = true;
	FLAGS_v = 2;

	LOG(INFO) << "start";
	DecoConfig::GetSingleton()->Init("../config.ini");
//	DecoConfig::GetSingleton()->GetString("Image2Mesh", "DataName", gDataName);
	DecoConfig::GetSingleton()->GetString("Image2Mesh", "TableFolder", gTableFolder);
	DecoConfig::GetSingleton()->GetString("Image2Mesh", "TableId", gTableId);

	vector<pair<string, string> > correspondences;
	int numImages = ReadCorrespondence(correspondences);

	vector<Eigen::MatrixXf> cameraPoses;
	int numViews = ReadCameraPoses(cameraPoses);
	
	int numFrames = numImages < numViews ? numImages : numViews;


	int ithDepthToProcess = 0;
	DecoConfig::GetSingleton()->GetInt("Image2Mesh", "ImageToProcess", ithDepthToProcess);

	//char filename[MAX_FILENAME_LEN];
	//memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
	//sprintf(filename, "%s%s/%s", gDataFolder.c_str(), gDataName.c_str(), correspondences[ithDepthToProcess].first.c_str());

	//DepthImage depthImage(filename);
	//depthImage.Process(cameraPoses[ithDepthToProcess]);

	//cv::Mat segmentationImage = cv::imread("DepthSegmentation.pngopaque.png");
	//MatrixXi maskImage = MatrixXi::Zero(segmentationImage.rows, segmentationImage.cols);
	//for (int i = 0; i < segmentationImage.rows; ++i)
	//{
	//	for (int j = 0; j < segmentationImage.cols; ++j)
	//	{
	//		if (segmentationImage.at<uchar>(i, j) != 199)
	//			maskImage(i, j) = 0;
	//		else
	//			maskImage(i, j) = 1;
	//	}
	//}
	//DepthImageInpainting inpainter;
	//inpainter.SetDepthImage(&depthImage);
	//inpainter.SetMaskImage(maskImage);
	//inpainter.Inpaint();
	//inpainter.SaveResultImage("DepthInpaint.png");

	//const int numSegments = 200;
	//DepthSegmentation segmenter(&depthImage);
	//const Segmentation& segment = segmenter.Segment(numSegments);
	//segmenter.SaveSegmentedImage("DepthSegmentation.png");



 //	cv::Mat depthImg = ReadDepthImage(filename);
	//cv::Mat depthImgAfterPreprocessing = PreprocessDepthImage(depthImg);

	//ImageSegmentation segmenter;
	//segmenter.Segment(depthImgAfterPreprocessing, 30, 85);
	//segmenter.SaveSegmentedImage("TestSegmentation.png");
	DecoConfig::GetSingleton()->GetInt("Image2Mesh", "CameraProjection", gCameraProjectionType);
	DecoConfig::GetSingleton()->GetFloat("Image2Mesh", "CameraWidth", gCameraWidth);
	vector<Eigen::Vector3f> points;
	vector<Eigen::Vector3f> normals;
	vector<Eigen::Vector3i> colors;
	Eigen::MatrixXf cameraPose = cameraPoses[ithDepthToProcess];

	Eigen::Vector3f cameraFront = Eigen::Vector3f(-0.0654295f, 0.911619f, 0.405796f);
	Eigen::Vector3f cameraRight = Eigen::Vector3f(0.939416f, -0.118723f, 0.321563f);
	Eigen::Vector3f cameraUp = cameraRight.cross(cameraFront);
	cameraUp.normalize();
	cameraFront = cameraUp.cross(cameraRight);
	cameraFront.normalize();
	Eigen::Vector3f cameraPos = Eigen::Vector3f(1.06286, 1.28835, 1.30427) - cameraFront + 0.2 * cameraRight - 0.2 * cameraUp;

	//Eigen::Vector3f cameraFront = Eigen::Vector3f(0.939416f, -0.118723f, 0.321563f); 
	//cameraFront.normalize();
	//Eigen::Vector3f cameraUp = Eigen::Vector3f(-0.0654295f, 0.911619f, 0.405796f);
	//cameraUp.normalize();
	//
	//Eigen::Vector3f cameraRight = cameraFront.cross(cameraUp);
	//cameraRight.normalize();
	//cameraFront = cameraUp.cross(cameraRight);
	//cameraFront.normalize();

	//Eigen::Vector3f cameraPos = Eigen::Vector3f(0.585555f, 2.11679f, 1.20261f) - cameraFront + 0.6f * Eigen::Vector3f(0.160215f, -0.926381f, -0.340808f);

	cameraPose.col(0) = Eigen::Vector4f(cameraRight[0], cameraRight[1], cameraRight[2], 0);
	cameraPose.col(1) = Eigen::Vector4f(cameraUp[0], cameraUp[1], cameraUp[2], 0);
	cameraPose.col(2) = Eigen::Vector4f(cameraFront[0], cameraFront[1], cameraFront[2], 0);
	cameraPose.col(3) = Eigen::Vector4f(cameraPos[0], cameraPos[1], cameraPos[2], 1);


	int task = 0;
	DecoConfig::GetSingleton()->GetInt("Image2Mesh", "Task", task);
	if (task == 0)
	{
		/* merge depthImages to point cloud (both high res and low res */
		ConstructPointCloudFromDepthImages(numFrames, correspondences, cameraPoses, points, normals);
		char filename[MAX_FILENAME_LEN];
		memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
		sprintf(filename, "%s/%s/points.ply", gTableFolder.c_str(), gTableId.c_str());

		int bSaveAllPoints = 0;
		DecoConfig::GetSingleton()->GetInt("Image2Mesh", "SaveAllPoints", bSaveAllPoints);
		if (bSaveAllPoints)
			SavePointCloud(filename, points, colors, normals);
		float mergeDistance = 0.005f;
		vector<Eigen::Vector3f> newPoints, newNormals;
		SubsamplePointCloud(points, normals, mergeDistance, newPoints, newNormals);
		string simplifiedPointCloudFilename = filename;
		simplifiedPointCloudFilename += "simplified.ply";
		SavePointCloud(simplifiedPointCloudFilename, newPoints, colors, newNormals);
	}
	else if (task == 1)
	{
		/* construct multilayer depth image from high res point cloud */
		char filename[MAX_FILENAME_LEN];
		memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
		sprintf(filename, "%s/%s/points.ply", gTableFolder.c_str(), gTableId.c_str());
//		sprintf(filename, "%s%s/points.plysimplified.ply", gDataFolder.c_str(), gDataName.c_str()); // for speedy test purpose
		ReadPointCloud(filename, points, normals);
		BuildMultiLayerDepthImage(points, normals, cameraPose);
	}
	else if (task == 2)
	{
		/* simplify the multilayer depth image and save the resultant simplified point cloud */
		DepthCamera dCamera;
		dCamera.SetIntrinsicParameters(gDepthImageWidth, gDepthImageHeight, gFocalLenth);
		dCamera.SetExtrinsicParameters(cameraPose);
		dCamera.SetProjectionType(gCameraProjectionType);
		dCamera.SetOrthoWidth(gCameraWidth);
		string filename = "results/DepthFromMultiview_Ortho.data";
		dCamera.ReadMultilayerDepthImage(filename);
		dCamera.ProcessMultiLayerDepthImage();
		dCamera.SimplifyMultiLayerDepthImage();
		dCamera.SaveMultilayerDepthImage("results/simplifiedDepthFromMultiview_Ortho.data");
		dCamera.GetPointCloud(points, normals, PORTION_ALL);
		SavePointCloud("results/simplifiedDepthFromMultiview_Ortho.ply", points, colors, normals);
	}
	else if (task == 3)
	{
		/* read the point cloud and perform Poisson surface reconstruction */
		char filenamePrefix[MAX_FILENAME_LEN];
		memset(filenamePrefix, 0, MAX_FILENAME_LEN * sizeof(char));
		sprintf(filenamePrefix, "%s/%s/points.plysimplified", gTableFolder.c_str(), gTableId.c_str());
		string fileToRead = filenamePrefix;
		fileToRead += ".ply";
		ReadPointCloud(fileToRead, points, normals);
		CoredFileMeshData<PlyVertex< Real > > mesh;
		Points2Mesh(points, normals, mesh);
		string fileToWrite = filenamePrefix;
		fileToWrite += "Mesh.ply";
		SaveMesh(fileToWrite, &mesh);
	}
	else if (task == 4)
	{
		/* create a multi-layer depth image for a triangular mesh */
		DepthCamera dCamera;
		dCamera.SetIntrinsicParameters(gDepthImageWidth, gDepthImageHeight, gFocalLenth);
		dCamera.SetExtrinsicParameters(cameraPose);
		dCamera.SetProjectionType(gCameraProjectionType);
		dCamera.SetOrthoWidth(gCameraWidth);
		string filename = "results/originalDataMesh.ply";
		vector<Eigen::Vector3f> verticesEigen;
		vector<Eigen::Vector3i> indicesEigen;

		ReadMesh(filename, &verticesEigen, &indicesEigen);
		dCamera.Capture(verticesEigen, indicesEigen);
	}
	else if (task == 5)
	{
		/* compare multi-layer depth image of the point cloud against that of the poisson reconstructed mesh */
		DepthCamera dCameraPoints;
		dCameraPoints.SetIntrinsicParameters(gDepthImageWidth, gDepthImageHeight, gFocalLenth);
		dCameraPoints.SetExtrinsicParameters(cameraPose);
		dCameraPoints.SetProjectionType(gCameraProjectionType);
		dCameraPoints.SetOrthoWidth(gCameraWidth);
		string filename = "results/simplifiedDepthFromMultiview_Ortho_Top.data";
		dCameraPoints.ReadMultilayerDepthImage(filename);
		dCameraPoints.ProcessMultiLayerDepthImage();

		DepthCamera dCameraMesh;
		dCameraMesh.SetIntrinsicParameters(gDepthImageWidth, gDepthImageHeight, gFocalLenth);
		dCameraMesh.SetExtrinsicParameters(cameraPose);
		dCameraMesh.SetProjectionType(gCameraProjectionType);
		dCameraMesh.SetOrthoWidth(gCameraWidth);
		//filename = "results/depthFromMultiviewFromMesh.data";
		filename = "results/depthFromMultiviewMesh_Ortho_Top.data";
		dCameraMesh.ReadMultilayerDepthImage(filename);
		dCameraMesh.ProcessMultiLayerDepthImage();
		dCameraMesh.GetPointCloud(points, normals, PORTION_ALL);
		SavePointCloud("results/depthFromMultiviewMesh_Ortho_Top.ply", points, colors, normals);

		//vector<DepthImage*> refDepthImages;
		//for (int i = 0; i < numFrames; i += 50)
		//{
		//	char filename[MAX_FILENAME_LEN];
		//	memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
		//	sprintf(filename, "%s%s/%s", gDataFolder.c_str(), gDataName.c_str(), correspondences[i].first.c_str());
		//	DepthImage* depthImg = new DepthImage(filename);
		//	depthImg->SetCameraPose(cameraPoses[i]);
		//	refDepthImages.push_back(depthImg);
		//}

		MultilayerDepthImage mergedDepthMap;
		MultilayerMaskImage depthMask;
		//dCameraPoints.SetReferenceDepthImages(refDepthImages);
		ReadPointCloud("results/originalData.ply", points, normals);
		dCameraPoints.SetComparisonROI(120, 490, 150, 425, 1000, 2200);
		dCameraPoints.SetSimplifiedPointCloud(points);
		dCameraPoints.Compare(dCameraMesh, false, mergedDepthMap, depthMask);

		DepthCamera dCameraCombined;
		dCameraCombined.SetIntrinsicParameters(gDepthImageWidth, gDepthImageHeight, gFocalLenth);
		dCameraCombined.SetExtrinsicParameters(cameraPose);
		dCameraCombined.SetProjectionType(gCameraProjectionType);
		dCameraCombined.SetOrthoWidth(gCameraWidth);
		dCameraCombined.SetData(mergedDepthMap);
		dCameraCombined.SetMask(depthMask);
		
		dCameraCombined.ProcessMultiLayerDepthImage();
		dCameraCombined.GetPointCloud(points, normals, PORTION_ALL);
		SavePointCloud("results/combinedPointMeshDepthImage_Ortho_Top.ply", points, colors, normals);
		dCameraCombined.GetPointCloud(points, normals, PORTION_UNKNOWN);
		SavePointCloud("results/combinedPointMeshDepthImageHole_Ortho_Top.ply", points, colors, normals);

		mergedDepthMap.Save("results/combinedPointMeshDepthImage_Ortho_Top.data");
		depthMask.Save("results/combinedPointMeshDepthImageMask_Ortho_Top.mask");
		//mergedDepthMap.SaveDepthThresholdingImage("results/combinedPointMeshDepthImage_Ortho.png", 20, &depthMask);
		mergedDepthMap.SaveDepthOnionImage("results/combinedPointMeshDepthImage_Ortho.png", &depthMask);
		//int numDepthImageReferences = static_cast<int>(refDepthImages.size());
		//for (int i = 0; i < numDepthImageReferences; ++i)
		//{
		//	delete refDepthImages[i];
		//}
		//refDepthImages.clear();
	}
	else if (task == 6)
	{
		/* two different ways to visualize the multi-layer depth image */
		MultilayerDepthImage MLDI;

		string filename = "results/simplifiedDepthFromMultiview.data";
		MLDI.Read(filename);
		MLDI.Process();

		string outFileName = "results/simplifiedDepthFromMultiview.png";
		MLDI.Save(outFileName);
		MLDI.SaveDepthThresholdingImage(outFileName, 20);

		outFileName = "results/simplifiedDepthFromMultiview.png";
		MLDI.SaveDepthOnionImage(outFileName);

	}
	else if (task == 7)
	{
		MultilayerDepthImage mergedDepthMap;
		//mergedDepthMap.Read("results/combinedPointMeshDepthImage_Ortho.data");
		mergedDepthMap.Read("results/crossViewDepthImage_Ortho_Top_Front.data");
		mergedDepthMap.Process();

		MultilayerMaskImage depthMask;
		//depthMask.Read("results/combinedPointMeshDepthImageMask_Ortho.mask");
		depthMask.Read("results/crossViewDepthImage_Ortho_Top_Front.mask");

		//mergedDepthMap.SaveDepthOnionImage("results/combinedPointMeshDepthImage.png", &depthMask);
		DepthCamera dCameraInpainted;
		dCameraInpainted.SetIntrinsicParameters(gDepthImageWidth, gDepthImageHeight, gFocalLenth);
		dCameraInpainted.SetExtrinsicParameters(cameraPose);
		dCameraInpainted.SetProjectionType(gCameraProjectionType);
		dCameraInpainted.SetOrthoWidth(gCameraWidth);

		string filename = "results/originalDataMesh.ply";
		vector<Eigen::Vector3f> vertices;
		vector<Eigen::Vector3i> indices;
		ReadMesh(filename, &vertices, &indices);

		DepthImageInpainting inpainter;
		inpainter.SetDepthImage(&mergedDepthMap);
		inpainter.SetMaskImage(&depthMask);
		inpainter.SetCamera(&dCameraInpainted);
		inpainter.SetMesh(vertices, indices);
		inpainter.Inpaint(5);
		//const MultilayerDepthImage& inpaintResult = inpainter.GetResultImage();
		//dCameraInpainted.SetData(inpaintResult);
		//dCameraInpainted.ProcessMultiLayerDepthImage();
		//dCameraInpainted.GetPointCloud(points, normals);
		//SavePointCloud("results/inpaintedDepthImage.ply", points, colors, normals);
	}
	else if (task == 8)
	{
		char filenamePrefix[MAX_FILENAME_LEN];
		memset(filenamePrefix, 0, MAX_FILENAME_LEN * sizeof(char));
		sprintf(filenamePrefix, "%s/%s/points.plysimplified", gTableFolder.c_str(), gTableId.c_str());
		string fileToRead = filenamePrefix;
		fileToRead += ".ply";
		ReadPointCloud(fileToRead, points, normals);

		vector<Eigen::Vector3f> inpaintedPoints;
		vector<Eigen::Vector3f> inpaintedNormals;
		ReadPointCloud("results/inpaintedPoints_Top.ply", inpaintedPoints, inpaintedNormals);
		ReadPointCloud("results/inpaintedPoints_FrontLayer0.ply", inpaintedPoints, inpaintedNormals, true);
		ReadPointCloud("results/inpaintedPoints_FrontLayer1.ply", inpaintedPoints, inpaintedNormals, true);

		points.insert(points.end(), inpaintedPoints.begin(), inpaintedPoints.end());
		normals.insert(normals.end(), inpaintedNormals.begin(), inpaintedNormals.end());
		SavePointCloud("results/simplifiedAndInpaintedPoints.ply", points, colors, normals);

		CoredFileMeshData<PlyVertex< Real > > mesh;
		Points2Mesh(points, normals, mesh);
		string fileToWrite = filenamePrefix;
		fileToWrite += "InpaintedMesh.ply";
		SaveMesh("results/simplifiedAndInpaintedMesh.ply", &mesh);
	}
	else if (task == 9)
	{
		MultilayerDepthImage topViewDepthMap;
		topViewDepthMap.Read("results/combinedPointMeshDepthImage_Ortho_Top.data");
		topViewDepthMap.Process();
		MultilayerMaskImage topViewDepthMask;
		topViewDepthMask.Read("results/combinedPointMeshDepthImageMask_Ortho_Top.mask");
		MultilayerDepthImage topViewDepthMapInpainted;
		topViewDepthMapInpainted.Read("results/InpaintedDepthImage_Ortho_Top.data");

		Eigen::Vector3f cameraFront = Eigen::Vector3f(-0.0654295f, 0.911619f, 0.405796f);
		Eigen::Vector3f cameraRight = Eigen::Vector3f(0.939416f, -0.118723f, 0.321563f);
		Eigen::Vector3f cameraUp = cameraRight.cross(cameraFront);
		cameraUp.normalize();
		cameraFront = cameraUp.cross(cameraRight);
		cameraFront.normalize();
		Eigen::Vector3f cameraPos = Eigen::Vector3f(1.06286f, 1.28835f, 1.30427f) - cameraFront + 0.2f * cameraRight - 0.2f * cameraUp;
		Eigen::Matrix4f cameraPoseTop;
		cameraPoseTop.col(0) = Eigen::Vector4f(cameraRight[0], cameraRight[1], cameraRight[2], 0);
		cameraPoseTop.col(1) = Eigen::Vector4f(cameraUp[0], cameraUp[1], cameraUp[2], 0);
		cameraPoseTop.col(2) = Eigen::Vector4f(cameraFront[0], cameraFront[1], cameraFront[2], 0);
		cameraPoseTop.col(3) = Eigen::Vector4f(cameraPos[0], cameraPos[1], cameraPos[2], 1);

		DepthCamera dCameraTopView;
		dCameraTopView.SetIntrinsicParameters(gDepthImageWidth, gDepthImageHeight, gFocalLenth);
		dCameraTopView.SetExtrinsicParameters(cameraPoseTop);
		dCameraTopView.SetProjectionType(gCameraProjectionType);
		dCameraTopView.SetOrthoWidth(gCameraWidth);
		dCameraTopView.SetData(topViewDepthMap);
		dCameraTopView.SetMask(topViewDepthMask);

		DepthCamera dCameraTopViewInpainted;
		dCameraTopViewInpainted.SetIntrinsicParameters(gDepthImageWidth, gDepthImageHeight, gFocalLenth);
		dCameraTopViewInpainted.SetExtrinsicParameters(cameraPoseTop);
		dCameraTopViewInpainted.SetProjectionType(gCameraProjectionType);
		dCameraTopViewInpainted.SetOrthoWidth(gCameraWidth);
		dCameraTopViewInpainted.SetData(topViewDepthMapInpainted);
		dCameraTopViewInpainted.SetMask(topViewDepthMask);

		MultilayerDepthImage frontViewDepthMap;
		frontViewDepthMap.Read("results/combinedPointMeshDepthImage_Ortho.data");
		frontViewDepthMap.Process();
		MultilayerMaskImage frontViewDepthMask;
		frontViewDepthMask.Read("results/combinedPointMeshDepthImageMask_Ortho.mask");

		DepthCamera dCameraFrontView;
		dCameraFrontView.SetIntrinsicParameters(gDepthImageWidth, gDepthImageHeight, gFocalLenth);
		dCameraFrontView.SetExtrinsicParameters(cameraPose);
		dCameraFrontView.SetProjectionType(gCameraProjectionType);
		dCameraFrontView.SetOrthoWidth(gCameraWidth);
		dCameraFrontView.SetData(frontViewDepthMap);
		dCameraFrontView.SetMask(frontViewDepthMask);
		//vector<DepthImage*> refDepthImages;
		//for (int i = 0; i < numFrames; i += 50)
		//{
		//	char filename[MAX_FILENAME_LEN];
		//	memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
		//	sprintf(filename, "%s%s/%s", gDataFolder.c_str(), gDataName.c_str(), correspondences[i].first.c_str());
		//	DepthImage* depthImg = new DepthImage(filename);
		//	depthImg->SetCameraPose(cameraPoses[i]);
		//	refDepthImages.push_back(depthImg);
		//}
		//dCameraFrontView.SetReferenceDepthImages(refDepthImages);
		//ReadPointCloud("results/originalData.ply", points, normals);
		//dCameraFrontView.SetSimplifiedPointCloud(points);
		MultilayerDepthImage newDepthImage;
		MultilayerMaskImage newMaskImage;

		MultilayerDepthImage topViewDepthMapPointCloud;
		topViewDepthMapPointCloud.Read("results/simplifiedDepthFromMultiview_Ortho_Top.data");
		topViewDepthMapPointCloud.Process();

		dCameraFrontView.CrossViewMaskUpdate1(dCameraTopView, dCameraTopViewInpainted, topViewDepthMapPointCloud, MOVE_UP, &newDepthImage, &newMaskImage);
		newDepthImage.Process();
		//newDepthImage.Save("results/crossViewDepthImage_Ortho_Top_Front.data");
		newDepthImage.SaveDepthOnionImage("results/crossViewDepthImage_Ortho_Top_Front.png", &newMaskImage);
		newDepthImage.SaveDepthImage("results/crossViewDepthImage_Ortho_Top_Front.png");
		//newDepthImage.SaveDepthOnionImage("results/crossViewDepthImage_Ortho_Top_Front.png", NULL);
		newMaskImage.Save("results/crossViewDepthImage_Ortho_Top_Front.mask");
		dCameraFrontView.SetData(newDepthImage);
		dCameraFrontView.GetPointCloud(points, normals, PORTION_ALL);
		SavePointCloud("results/crossViewDepthImage_Ortho_Top_Front.ply", points, colors, normals);
	}
	else if (task == 10)
	{

		cv::Mat srcImageFromFile;
		cv::Mat dstImageFromFile;
		srcImageFromFile = ReadColorImage("../../patchmatch-2.1/a.png");
		dstImageFromFile = ReadColorImage("../../patchmatch-2.1/b.png");

		//cv::namedWindow("src window", cv::WINDOW_AUTOSIZE);
		//cv::imshow("src window", srcImageFromFile);

		//cv::namedWindow("dst window", cv::WINDOW_AUTOSIZE);
		//cv::imshow("dst window", dstImageFromFile);

		Image<Eigen::Vector3f> src;
		Image<Eigen::Vector3f> dst;
		FromCVToImage(srcImageFromFile, src);
		FromCVToImage(dstImageFromFile, dst);
		src.Create(srcImageFromFile.rows, srcImageFromFile.cols);
		dst.Create(dstImageFromFile.rows, dstImageFromFile.cols);

		PatchMatch<Eigen::Vector3f> pmTest;
		pmTest.SetSrc(&src, NULL);
		pmTest.SetDst(&dst, NULL);
		pmTest.SetDistanceMetric(dist1);
		pmTest.ComputeNNF();
		const Image<Eigen::Vector2i>& nnf = pmTest.GetNNF();
		const Image<float>& nnd = pmTest.GetNND();
		const Image<Eigen::Vector3f>& reconstructedImage = pmTest.GetReconstructedDst();

		cv::Mat nnfToFile, reconToFile, nnfVisualizeToFile;
		FromImageToCV(nnf, nnfToFile);
		FromImageToCV(reconstructedImage, reconToFile);
		//FromPosImageToColorCV(nnf, nnfVisualizeToFile);

		cv::imwrite("../../patchmatch-2.1/testNNF.png", nnfToFile);
		cv::imwrite("../../patchmatch-2.1/testRecon.png", reconToFile);
		//cv::imwrite("../../patchmatch-2.1/testNNFVisualize.png", nnfVisualizeToFile);
		//cv::imwrite("../../patchmatch-2.1/testNND.png", nndToFile);
		//cv::waitKey(0);
	}
	else if (task == 11)
	{
		cv::Mat1w imageFromFile;
		cv::Mat maskFromFile;
		imageFromFile = ReadDepthImage("../../patchmatch-2.1/testInpaintingImage.png");
		maskFromFile = ReadColorImage("../../patchmatch-2.1/testInpaintingMask.png");

		Image<Eigen::Vector3f> img;
		FromCVToDepthImage(imageFromFile, img);

		Image<int> mask;
		mask.Create(maskFromFile.rows, maskFromFile.cols);
		for (int i = 0; i < maskFromFile.rows; ++i)
		{
			for (int j = 0; j < maskFromFile.cols; ++j)
			{
				cv::Vec3b& col = maskFromFile.at<cv::Vec3b>(i, j);
				if (col[0] != col[1])
				{
					mask[i][j] = MASK_UNKNOWN;
				}
				else
				{
					mask[i][j] = MASK_KNOWN;
				}

			}
		}
		ImageInpainting<Eigen::Vector3f> inpainter;
		inpainter.SetSrc(&img, &mask);
		inpainter.SetDst(&img, &mask);
		inpainter.Inpaint();
		const Image<Eigen::Vector3f>& result = inpainter.GetResult();
		const Image<Eigen::Vector2i>& nnf = inpainter.GetNNF();
		const Image<float>& nnd = inpainter.GetNND();

		cv::Mat nnfToFile, nnfVisualizeToFile, nndToFile;
		cv::Mat1w inpaintResult;
		FromDepthImageToCV(result, inpaintResult);
		FromImageToCV(nnf, nnfToFile);
		FromImageToCV(nnd, nndToFile);
		FromPosImageToColorCV(nnf, nnfVisualizeToFile);

		cv::namedWindow("inpaint window", cv::WINDOW_AUTOSIZE);
		cv::imshow("inpaint window", inpaintResult);
		cv::imwrite("../../patchmatch-2.1/testInpaintingResult.png", inpaintResult);
		cv::imwrite("../../patchmatch-2.1/testInpaintingNNF.png", nnfToFile);
		cv::imwrite("../../patchmatch-2.1/testInpaintingNND.png", nndToFile);
		cv::imwrite("../../patchmatch-2.1/testInpaintingNNFVis.png", nnfVisualizeToFile);

		DepthCamera dCameraFrontView;
		dCameraFrontView.SetIntrinsicParameters(gDepthImageWidth, gDepthImageHeight, gFocalLenth);
		dCameraFrontView.SetExtrinsicParameters(cameraPose);
		dCameraFrontView.SetProjectionType(gCameraProjectionType);
		dCameraFrontView.SetOrthoWidth(gCameraWidth);
		DepthImage depthImg("../../patchmatch-2.1/testInpaintingResult.png");
		dCameraFrontView.SetData(depthImg);
		dCameraFrontView.GetPointCloud(points, normals, PORTION_ALL);
		SavePointCloud("../../patchmatch-2.1/testInpaintingResult.ply", points, colors, normals);

		return 1;
		//cv::waitKey(0);
	}
	else if (task == 12)
	{
		srand(0);
		string tableFolder, tableId, inputFileName, outputFileName, referenceFileName;
		DecoConfig::GetSingleton()->GetString("Image2Mesh", "TableFolder", tableFolder);
		DecoConfig::GetSingleton()->GetString("Image2Mesh", "TableId", tableId);
		DecoConfig::GetSingleton()->GetString("Image2Mesh", "InputFileName", inputFileName);
		DecoConfig::GetSingleton()->GetString("Image2Mesh", "OutputFileName", outputFileName);
		DecoConfig::GetSingleton()->GetString("Image2Mesh", "ReferenceFileName", referenceFileName);

		string fullInputFileName = tableFolder + "/" + tableId + "/" + inputFileName;
		string fullOutputFileName = tableFolder + "/" + tableId +  "/"+ outputFileName;
		string fullReferenceFileName = tableFolder + "/" + tableId + "/" + referenceFileName;
		
		vector<Eigen::Vector3f> refPoints, refNormals;
		ReadPointCloud(fullReferenceFileName, refPoints, refNormals);


		int samplingDensity;
		DecoConfig::GetSingleton()->GetInt("Image2Mesh", "SampleDensity", samplingDensity);
		vector<Eigen::Vector3f> vertices; 
		vector<Eigen::Vector3f> meshVertexNormals;
		vector<Eigen::Vector3i> faces;
		ReadMesh(fullInputFileName, &vertices, &faces);
		ComputeVertexNormals(vertices, faces, &meshVertexNormals);
		MeshToPointCloud(samplingDensity, vertices, faces, meshVertexNormals, &points, &normals);
		vector<Eigen::Vector3f> truePoints, trueNormals;
		RemoveRedundantPoints(points, normals, refPoints, &truePoints, &trueNormals);
		SavePointCloud(fullOutputFileName, truePoints, colors, trueNormals);
	}
	else if (task == 13)
	{
		
		string inputFileName;

		inputFileName = "allParts.txt";
		string fullInputFileName = gTableFolder + "/" + gTableId + "/" + inputFileName;
		ifstream inParts(fullInputFileName.c_str());
		int numParts;
		inParts >> numParts;

		vector<vector<Eigen::Vector3f> > allPoints;
		vector<vector<Eigen::Vector3f> > allNormals;
		vector<PrimitiveShape*> allPrimitives;
		vector<Part> allParts;
		vector<sehoon::ann::KDTree*> kdtrees;

		allPoints.resize(numParts);
		allNormals.resize(numParts);
		allPrimitives.resize(numParts);

		for (int i = 0; i < numParts; ++i)
		{
			char plyFileName[512];
			sprintf(plyFileName, "%s/%s/parts/part%03d.ply", gTableFolder.c_str(), gTableId.c_str(), i);
			ReadPointCloud(plyFileName, points, normals);

			allPoints[i] = points;
			allNormals[i] = normals;

			int identifier;
			inParts >> identifier;
			PrimitiveShape* prim = NULL;
			if (identifier == 0)
			{
				prim = new PlanePrimitiveShape(&inParts);
			}
			else if (identifier == 2)
			{
				prim = new CylinderPrimitiveShape(&inParts);
			}
			allPrimitives[i] = prim;
			if (points.size() > 500)
			{
				sehoon::ann::KDTree* tree = new sehoon::ann::KDTree();
				kdtrees.push_back(tree);
				allParts.push_back(Part(prim, i, points, normals, tree));
			}
		}
		
		vector<DepthImage*> refDepthImages;
		int stepVerifyCamera = 10;
		int stepProposeCamera = 50;
		for (int i = ithDepthToProcess; i < numFrames; i += stepVerifyCamera)
		{
			char filename[MAX_FILENAME_LEN];
			memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
			sprintf(filename, "%s/%s/%s", gTableFolder.c_str(), gTableId.c_str(), correspondences[i].first.c_str());
			DepthImage* depthImg = new DepthImage(filename);
			depthImg->SetCameraPose(cameraPoses[i]);
			refDepthImages.push_back(depthImg);
		}
		vector<Eigen::Vector3f> unseenPoints, unseenNormals;
		int floorId, wallId;
		DecoConfig::GetSingleton()->GetInt("Image2Mesh", "WallId", wallId);
		DecoConfig::GetSingleton()->GetInt("Image2Mesh", "FloorId", floorId);
		DepthCamera dCamera;
		dCamera.SetIntrinsicParameters(gDepthImageWidth, gDepthImageHeight, gFocalLenth);
		dCamera.SetProjectionType(gCameraProjectionType);
		dCamera.SetOrthoWidth(gCameraWidth);
		dCamera.SetReferenceDepthImages(refDepthImages);
		PrimitiveShape* floor = NULL;
		PrimitiveShape* wall = NULL;
		if (floorId >= 0)
		{
			floor = allPrimitives[floorId];
		}
		if (wallId >= 0)
		{
			wall = allPrimitives[wallId];
		}
		dCamera.SetWallAndFloor(wall, floor);
		//for (int i = ithDepthToProcess; i < numFrames; i += 50)
		for (int i = ithDepthToProcess; i < numFrames; i += stepProposeCamera)
		{
			LOG(INFO) << "Processing " << i << "th camera view out of " << numFrames;
			dCamera.SetExtrinsicParameters(cameraPoses[i]);
			dCamera.Capture(allParts);
			dCamera.GetDepthImage().SaveDepthImage(gTableFolder + "/" + gTableId + "/" + "testDepthImage.png");
			dCamera.GetPointCloud(points, normals, PORTION_ALL);
			//char fullOutputFileName[512];
			//sprintf(fullOutputFileName, "%s\\%s\\UnseenPrimitive%03d.ply", tableFolder.c_str(), tableId.c_str(), i);
			//SavePointCloud(fullOutputFileName, points, colors, normals);


			unseenPoints.insert(unseenPoints.end(), points.begin(), points.end());
			unseenNormals.insert(unseenNormals.end(), normals.begin(), normals.end());
		}

		
		string fullOutputFileName = gTableFolder + "/" + gTableId + "/" + "UnseenPrimitive.ply";
		SavePointCloud(fullOutputFileName, unseenPoints, colors, unseenNormals);
	}
	else if (task == 14)
	{
		string inputFileName;

		inputFileName = "allParts.txt";
		string fullInputFileName = gTableFolder + "/" + gTableId + "/" + inputFileName;
		ifstream inParts(fullInputFileName.c_str());
		int numParts;
		inParts >> numParts;

		vector<vector<Eigen::Vector3f> > allPoints;
		vector<vector<Eigen::Vector3f> > allNormals;
		vector<PrimitiveShape*> allPrimitives;
		vector<Part> allParts;
		

		allPoints.resize(numParts);
		allNormals.resize(numParts);
		allPrimitives.resize(numParts);

		vector<Part> splittedParts;
		for (int i = 0; i < numParts; ++i)
		{
			char plyFileName[512];
			sprintf(plyFileName, "%s/%s/parts/part%03d.ply", gTableFolder.c_str(), gTableId.c_str(), i);
			ReadPointCloud(plyFileName, points, normals);

			LOG(INFO) << "Part " << i;
			allPoints[i] = points;
			allNormals[i] = normals;

			int identifier;
			inParts >> identifier;
			PrimitiveShape* prim = NULL;
			if (identifier == 0)
			{
				prim = new PlanePrimitiveShape(&inParts);
			}
			else
			{
				CHECK(0) << "Only planes are supported.";
			}
			allPrimitives[i] = prim;
			if (points.size() < 500) continue;
			Part part(prim, i, points, normals, NULL);
			

			allParts.push_back(part);
		}
		numParts = static_cast<int>(allParts.size());
		for (int i = 0; i < numParts; ++i)
		{
			allParts[i].Process();
			allParts[i].DivideByNormalDirection(splittedParts);
		}
		allParts = splittedParts;
		numParts = static_cast<int>(allParts.size());
		splittedParts.clear();

		for (int i = 0; i < numParts; ++i)
		{
			allParts[i].Process();
			allParts[i].DivideByConnectivity(splittedParts);
		}
		allParts = splittedParts;
		splittedParts.clear();
		numParts = static_cast<int>(allParts.size());

		for (int i = 0; i < numParts; ++i)
		{
			allParts[i].Process();
			allParts[i].DivideBySkeleton(splittedParts);
		}
		allParts = splittedParts;
		splittedParts.clear();
		numParts = static_cast<int>(allParts.size());

		//int testPartId = 7;
		//DecoConfig::GetSingleton()->GetInt("Image2Mesh", "TestPartId", testPartId);
		//splittedParts.clear();
		//allParts[testPartId].Process();
		//allParts[testPartId].DivideBySkeleton(splittedParts);
		//allParts = splittedParts;
		//numParts = static_cast<int>(allParts.size());
		char filename[512];
		for (int i = 0; i < numParts; ++i)
		{
			//sprintf(plyFileName, "%s/%s/partsSplittedByNormal/part%03d.ply", gTableFolder.c_str(), gTableId.c_str(), i);
			//SavePointCloud(plyFileName, allParts[i].GetPoints(), colors, allParts[i].GetNormals());

			sprintf(filename, "%s/%s/partsSplitted/part%03d", gTableFolder.c_str(), gTableId.c_str(), i);
			allParts[i].Save(filename);

			sprintf(filename, "%s/%s/partsSplitted/rectangle%03d.ply", gTableFolder.c_str(), gTableId.c_str(), i);
			allParts[i].Process();
			allParts[i].GetRectangle().SavePly(filename);
		}
		string numPartsFileName = gTableFolder + "/" + gTableId + "/partsSplitted/numParts.txt";
		ofstream outNumParts(numPartsFileName.c_str());
		outNumParts << numParts;




	}
	else if (task == 15)
	{
		vector<PrimitiveShape*> allPrimitives = ReadPrimitives();
		vector<Part> allParts = ReadParts(allPrimitives);


		int numParts = static_cast<int>(allParts.size());
		char filename[512];
		for (int i = 0; i < numParts; ++i)
		{
			sprintf(filename, "%s/%s/partsSplitted/rectangle%03d.ply", gTableFolder.c_str(), gTableId.c_str(), i);
			allParts[i].GetRectangle().SavePly(filename);
		}
		vector<BoxFromRectangles> boxes;
		for (int i = 1; i < numParts; ++i)
		{
			for (int j = 1; j < numParts; ++j)
			{
				
				if (i >= j) continue;

				LOG(INFO) << "Considering " << i << ", " << j << "pair.";
				BoxFromRectangles box;
				float score;
				if (box.Construct(allParts[i].GetRectangle(), allParts[j].GetRectangle(), i, j, &score))
				{
					boxes.push_back(box);
				}

			}
		}
		sort(boxes.begin(), boxes.end());
		int numBoxes = static_cast<int>(boxes.size());
		string boxInfoFileName = gTableFolder + "/" + gTableId + "/boxes/boxInfo.txt";
		ofstream boxInfoOut(boxInfoFileName.c_str());
		for (int i = 0; i < numBoxes; ++i)
		{
			BoxFromRectangles& box = boxes[i];

			LOG(INFO) << box.ComponentId(0) << " " << box.ComponentId(1);
			sprintf(filename, "%s/%s/boxes/rectangle%02d_%02d_%02d.ply", gTableFolder.c_str(), gTableId.c_str(), i, box.ComponentId(0), box.ComponentId(1));
			box.SavePly(filename);
			LOG(INFO) << filename << ": " << box.Confidence();
			sprintf(filename, "%s/%s/boxes/rectangle%02d_%02d_%02d.box", gTableFolder.c_str(), gTableId.c_str(), i, box.ComponentId(0), box.ComponentId(1));
			box.Save(filename);
			sprintf(filename, "rectangle%02d_%02d_%02d", i, box.ComponentId(0), box.ComponentId(1));
			boxInfoOut << filename << endl;
		}
	}
	else if (task == 16)
	{
		vector<PrimitiveShape*> allPrimitives = ReadPrimitives();
		vector<BoxFromRectangles> boxes = ReadBoxes(true);

		int floorId;
		DecoConfig::GetSingleton()->GetInt("Image2Mesh", "FloorId", floorId);
		PlanePrimitiveShape* floorPrimitive = static_cast<PlanePrimitiveShape*>(allPrimitives[floorId]);
		Vec3f n = floorPrimitive->Internal().getNormal();

		SymmetryBuilder symmBuilder;
		symmBuilder.SetUpDirection(Eigen::Vector3f(n.getValue()));
		bool isSymmetry = symmBuilder.CheckSymmetry(boxes);
		vector<Eigen::Vector3f> allPoints;
		vector<Eigen::Vector3f> allNormals;
		char newPointCloudName[512];

		if (isSymmetry)
		{
			symmBuilder.SaveVisualization(gTableFolder + "/" + gTableId + "/" + "/symmetry");

			string pointCloudName = gTableFolder + "/" + gTableId + "/pointsFromSimplifiedMesh1.ply";
			ReadPointCloud(pointCloudName, points, normals);
			int numPoints = static_cast<int>(points.size());
			allPoints.insert(allPoints.end(), points.begin(), points.end());
			allNormals.insert(allNormals.end(), normals.begin(), normals.end());

			
			int numSymmtryPlanes = symmBuilder.GetNumSymmetryPlanes();
			for (int ithPlane = 0; ithPlane < numSymmtryPlanes; ++ithPlane)
			{
				vector<pair<int, int> > correspondence = symmBuilder.GetSymmetryCorrespondences(ithPlane);
				int numSymmetryGroup = static_cast<int>(correspondence.size());
				for (int i = 0; i < numSymmetryGroup; ++i)
				{
					LOG(INFO) << correspondence[i].first << ": " << correspondence[i].second;
				}
				symmBuilder.RefineSymmetry(ithPlane);
				const UtilPlane& mirrorPlane = symmBuilder.GetSymmetryPlane(ithPlane);

				vector<Eigen::Vector3f> mirroredPoints;
				vector<Eigen::Vector3f> mirroredNormals;
				mirroredPoints.resize(numPoints);
				mirroredNormals.resize(numPoints);

				for (int i = 0; i < numPoints; ++i)
				{
					mirroredPoints[i] = mirrorPlane.MirrorPoint(points[i]);
					mirroredNormals[i] = mirrorPlane.MirrorVector(normals[i]);
				}
				allPoints.insert(allPoints.end(), mirroredPoints.begin(), mirroredPoints.end());
				allNormals.insert(allNormals.end(), mirroredNormals.begin(), mirroredNormals.end());
				sprintf(newPointCloudName, "%s/%s/pointsFromSimplifiedMesh1Mirrored_%d.ply", gTableFolder.c_str(), gTableId.c_str(), ithPlane);
				SavePointCloud(newPointCloudName, mirroredPoints, colors, mirroredNormals);
			}
			sprintf(newPointCloudName, "%s/%s/pointsFromSimplifiedMesh1MirroredAll.ply", gTableFolder.c_str(), gTableId.c_str());
			SavePointCloud(newPointCloudName, allPoints, colors, allNormals);
		}
		
	}
	else if (task == 17)
	{
		vector<PrimitiveShape*> allPrimitives = ReadPrimitives();
		vector<Part> parts = ReadParts(allPrimitives);
		vector<BoxFromRectangles> boxes = ReadBoxes(false);
		
		BoxVoter boxVoter;
		vector<BoxFromRectangles> votedBoxes;
		boxVoter.Vote(boxes, parts, &votedBoxes);
		sort(votedBoxes.begin(), votedBoxes.end());
		int numBoxes = static_cast<int>(votedBoxes.size());
		string boxInfoFileName = gTableFolder + "/" + gTableId + "/boxesVote/boxInfo.txt";
		ofstream boxInfoOut(boxInfoFileName.c_str());
		boxInfoOut << numBoxes << endl;
		char filename[512];

		for (int i = 0; i < numBoxes; ++i)
		{
			sprintf(filename, "%s/%s/boxesVote/rectangle%02d%s.ply", gTableFolder.c_str(), gTableId.c_str(), i, votedBoxes[i].GetComponentString().c_str());
			votedBoxes[i].SavePly(filename);
			LOG(INFO) << filename << ": " << votedBoxes[i].Confidence();
			sprintf(filename, "%s/%s/boxesVote/rectangle%02d%s.box", gTableFolder.c_str(), gTableId.c_str(), i, votedBoxes[i].GetComponentString().c_str());
			votedBoxes[i].Save(filename);
			sprintf(filename, "rectangle%02d%s", i, votedBoxes[i].GetComponentString().c_str());
			boxInfoOut << filename << endl;
		}
	}
	else if (task == 18)
	{
		//points.clear();
		//normals.clear();
		//string pointCloudName = gTableFolder + "/" + gTableId + "/pointsFromSimplifiedMesh1.ply";
		//ReadPointCloud(pointCloudName, points, normals);
		//CoredFileMeshData<PlyVertex< Real > > mesh;
		//Points2Mesh(points, normals, mesh);
		//string fileToWrite = pointCloudName;
		//fileToWrite += "PoissonMesh.ply";
		//SaveMesh(fileToWrite, &mesh);


		//points.clear();
		//normals.clear();
		//pointCloudName = gTableFolder + "/" + gTableId + "/pointsFromSimplifiedMesh1MirroredAll.ply";
		//ReadPointCloud(pointCloudName, points, normals);
		//CoredFileMeshData<PlyVertex< Real > > mesh1;

		vector<PrimitiveShape*> allPrimitives = ReadPrimitives();
		vector<Part> parts = ReadParts(allPrimitives);
		vector<BoxFromRectangles> boxes = ReadBoxes(true);
		int numBoxes = static_cast<int>(boxes.size());		
		//Extruder extruder;
		//for (int i = 0; i < numBoxes; ++i)
		//{
		//	vector<Eigen::Vector3f> extrudedPoints;
		//	vector<Eigen::Vector3f> extrudedNormals;
		//	string extrudedFileName = gTableFolder + "/" + gTableId + "/extrude/box" + boxes[i].GetComponentString() + "_extrude.ply";


		//	if (extruder.Extrude(boxes[i], parts, &extrudedPoints, &extrudedNormals))
		//	{
		//		points.insert(points.end(), extrudedPoints.begin(), extrudedPoints.end());
		//		normals.insert(normals.end(), extrudedNormals.begin(), extrudedNormals.end());
		//		SavePointCloud(extrudedFileName, extrudedPoints, extrudedNormals);
		//	}
		//}
		//Points2Mesh(points, normals, mesh1);
		//fileToWrite = pointCloudName;
		//fileToWrite += "PoissonMeshWithSymmetryExtrusion.ply";
		//SaveMesh(fileToWrite, &mesh1);

		string pointCloudName = gTableFolder + "/" + gTableId + "/pointsFromSimplifiedMesh1MirroredAll.ply";
		ReadPointCloud(pointCloudName, points, normals);
		for (int i = 0; i < numBoxes; ++i)
		{
			boxes[i].ReassignPoints(points, normals);
		}

		BarExtruder barExtruder;
		vector<DepthImage*> refDepthImages;
		int stepVerifyCamera = 10;
		
		for (int i = ithDepthToProcess; i < numFrames; i += stepVerifyCamera)
		{
			char filename[MAX_FILENAME_LEN];
			memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
			sprintf(filename, "%s/%s/%s", gTableFolder.c_str(), gTableId.c_str(), correspondences[i].first.c_str());
			DepthImage* depthImg = new DepthImage(filename);
			depthImg->SetCameraPose(cameraPoses[i]);
			refDepthImages.push_back(depthImg);
			//LOG(INFO) << i << "th reference camera: " << filename;
		}
		int floorId, wallId;
		DecoConfig::GetSingleton()->GetInt("Image2Mesh", "WallId", wallId);
		DecoConfig::GetSingleton()->GetInt("Image2Mesh", "FloorId", floorId);
		PrimitiveShape* floor = NULL;
		PrimitiveShape* wall = NULL;
		if (floorId >= 0)
		{
			floor = allPrimitives[floorId];
		}
		if (wallId >= 0)
		{
			wall = allPrimitives[wallId];
		}
		barExtruder.SetWallAndFloor(wall, floor);
		barExtruder.SetReferenceDepthImages(refDepthImages);
		for (int i = 0; i < numBoxes; ++i)
		{
			vector<Eigen::Vector3f> extrudedPoints;
			vector<Eigen::Vector3f> extrudedNormals;
			string extrudedFileName = gTableFolder + "/" + gTableId + "/extrude/box" + boxes[i].GetComponentString() + "_barExtrude.ply";


			if (barExtruder.Extrude(boxes[i], parts, &extrudedPoints, &extrudedNormals))
			{
				
				points.insert(points.end(), extrudedPoints.begin(), extrudedPoints.end());
				normals.insert(normals.end(), extrudedNormals.begin(), extrudedNormals.end());
				SavePointCloud(extrudedFileName, extrudedPoints, extrudedNormals);
				LOG(INFO) << "Extruding " << extrudedFileName;
			}
		}

	}
	else if (task == 19)
	{

		ReadPointCloud(gTableFolder + "/" + gTableId + "/extrude/box_34_38_41_barExtrude.ply", points, normals);
		ReadPointCloud(gTableFolder + "/" + gTableId + "/extrude/box_26_44_45_46_extrude.ply", points, normals, true);
		ReadPointCloud(gTableFolder + "/" + gTableId + "/pointsFromSimplifiedMesh1MirroredAll.ply", points, normals, true);


		SavePointCloud(gTableFolder + "/" + gTableId + "/pointsFromSimplifiedMesh1SymmetryWithExtrusion.ply", points, normals);
	}
}

vector<PrimitiveShape*> ReadPrimitives()
{
	string inputFileName;

	inputFileName = "allParts.txt";
	string fullInputFileName = gTableFolder + "/" + gTableId + "/" + inputFileName;
	ifstream inParts(fullInputFileName.c_str());
	int numPrimitives;
	inParts >> numPrimitives;
	vector<PrimitiveShape*> allPrimitives;
	allPrimitives.resize(numPrimitives);
	for (int i = 0; i < numPrimitives; ++i)
	{
		int identifier;
		inParts >> identifier;
		PrimitiveShape* prim = NULL;
		if (identifier == 0)
		{
			prim = new PlanePrimitiveShape(&inParts);
		}
		else
		{
			CHECK(0) << "Only planes are supported.";
		}
		allPrimitives[i] = prim;
	}
	return allPrimitives;
}

vector<Part> ReadParts(const vector<PrimitiveShape*>& allPrimitives)
{
	vector<Part> allParts;
	int numParts;
	string numPartsFileName = gTableFolder + "/" + gTableId + "/partsSplitted/numParts.txt";
	ifstream inNumParts(numPartsFileName.c_str());
	inNumParts >> numParts;
	allParts.resize(numParts);
	char filename[512];
	for (int i = 0; i < numParts; ++i)
	{

		sprintf(filename, "%s/%s/partsSplitted/part%03d", gTableFolder.c_str(), gTableId.c_str(), i);
		allParts[i].Read(filename, allPrimitives);
		sprintf(filename, "%s/%s/partsSplitted/rectangle%03d.ply", gTableFolder.c_str(), gTableId.c_str(), i);
		allParts[i].GetRectangle().SavePly(filename);
	}
	return allParts;
}

vector<BoxFromRectangles> ReadBoxes(bool bAfterVote)
{
	string inputFileName = "boxInfo.txt";
	string folderName = gTableFolder + "/" + gTableId + "/" + "/boxes/";
	if (bAfterVote)
	{
		folderName = gTableFolder + "/" + gTableId + "/" + "/boxesVote/";
	}
	string fullInputFileName = folderName + inputFileName;

	
	ifstream inBoxes(fullInputFileName.c_str());
	int numBoxes;
	inBoxes >> numBoxes;
	vector<BoxFromRectangles> boxes;
	boxes.resize(numBoxes);
	for (int i = 0; i < numBoxes; ++i)
	{
		string boxFilename;
		inBoxes >> boxFilename;
		boxFilename += ".box";
		string fullBoxFileName = folderName + boxFilename;
		boxes[i].Read(fullBoxFileName);
	}
	return boxes;
}
