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
using namespace Eigen;

#include "DepthImage.h"
#include "grid.h"
#include "ImageSegmentation.h"
#include "DepthSegmentation.h"
#include "DepthInpainting.h"


//#include "gflags/gflags.h"
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include "glog/logging.h"
using namespace google;

#define MAX_FILENAME_LEN 512

string gDataFolder = "../../../Tables/";
string gDataName = "399318621.826096";
char* outputFile = NULL;
double gDepthThreshold = 10.5;

int echoStdout=0;
typedef float Real;


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
Depth( "depth" , 10 ) ,
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
	sprintf(filename, "%s%s/input.match", gDataFolder.c_str(), gDataName.c_str());
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

int ReadCameraPoses(vector<MatrixXd>& cameraPose)
{
	char filename[MAX_FILENAME_LEN];
	memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
	sprintf(filename, "%s%s/poses.txt", gDataFolder.c_str(), gDataName.c_str());
	int numViews = 0;

	ifstream inFile(filename);
	
	MatrixXd pose = MatrixXd::Zero(4, 4);
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

void ColorImageToColors(const cv::Mat& colorImg, vector<Vector3i>& colors)
{
	for (int v =  0; v < colorImg.rows; ++v)
	{
		for (int u = 0; u < colorImg.cols; ++u)
		{
			cv::Vec3b cl = colorImg.at<cv::Vec3b>(v, u);
			Vector3i color(cl[2], cl[1], cl[0]);
			colors.push_back(color);
		}
	}
}


void SavePointCloud(const string& filename, const vector<Vector3d>& points, const vector<Vector3i>& colors, const vector<Vector3d>& normals)
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

template< class Vertex >
int Points2Mesh(const vector<Vector3d>& points, const vector<Vector3d>& normals, CoredMeshData< Vertex >& mesh)
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
	sprintf(filename, "%s%s/points.ply", gDataFolder.c_str(), gDataName.c_str());

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

template< class Vertex >
void SaveMesh(const string& filename, CoredMeshData< Vertex >* mesh)
{
	PlyWritePolygons(const_cast<char*>(filename.c_str()), mesh , PLY_ASCII);
}

void ComputeBoundingBox(const vector<Vector3d>& points, Vector3d& minPt, Vector3d& maxPt)
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

void MergePoints(const vector<Vector3d>& points, const vector<Vector3d>& normals, const vector<int>& indices, vector<Vector3d>& newPoints, vector<Vector3d>& newNormals)
{
	if (indices.empty()) return;
	int numPoints = static_cast<int>(indices.size());
	Vector3d avgPoint = Vector3d::Zero();
	Vector3d avgNormal = Vector3d::Zero();

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

void SubsamplePointCloud(const vector<Vector3d>& points, const vector<Vector3d>& normals, double mergeDistance, vector<Vector3d>& newPoints, vector<Vector3d>& newNormals)
{
	Vector3d minPoint, maxPoint;
	ComputeBoundingBox(points, minPoint, maxPoint);
	for (int i = 0; i < 3; ++i)
	{
		minPoint[i] = max<double>(-1, minPoint[i]);
		//maxPoint[i] = min<double>(2.5, maxPoint[i]);
	}
	minPoint[0] = max<double>(-1, minPoint[0]);
	minPoint[1] = max<double>(-1, minPoint[1]);
	minPoint[2] = max<double>(0, minPoint[2]);
	maxPoint[0] = min<double>(2.8, maxPoint[0]);
	maxPoint[1] = min<double>(2.9, maxPoint[1]);
	maxPoint[2] = min<double>(3.8, maxPoint[2]);
	

	
	cout << minPoint << endl;
	cout << maxPoint << endl;
	

	Vector3d resolutionFloat = (maxPoint - minPoint) / mergeDistance;
	Vector3i resolution(int(resolutionFloat[0]) + 1, int(resolutionFloat[1]) + 1, int(resolutionFloat[2]) + 1);
	resolutionFloat = Vector3d(resolution[0], resolution[1], resolution[2]);

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

void AlignBoundingBox(vector<Vector3d>& points, vector<Vector3d>& normals)
{
	int numPoints = static_cast<int>(points.size());
	Vector3d avgPoint = Vector3d::Zero();
	for (int i = 0; i < numPoints; ++i)
	{
		avgPoint += points[i];
	}
	avgPoint /= numPoints;
	for (int i = 0; i < numPoints; ++i)
	{
		points[i] -= avgPoint;
	}
	MatrixXd pointMatrix = MatrixXd(numPoints, 3);
	for (int i = 0; i < numPoints; ++i)
	{
		pointMatrix.row(i) = points[i];
	}
	MatrixXd pointMatrixT = pointMatrix.transpose();
	Matrix3d cov = pointMatrixT * pointMatrix;
	SelfAdjointEigenSolver<Matrix3d> eigensolver(cov);
	Matrix3d rot = eigensolver.eigenvectors();
	Matrix3d rotT = rot.transpose();
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

	vector<pair<string, string> > correspondences;
	int numImages = ReadCorrespondence(correspondences);

	vector<MatrixXd> cameraPoses;
	int numViews = ReadCameraPoses(cameraPoses);
	
	int numFrames = numImages < numViews ? numImages : numViews;


	int ithDepthToProcess = 0;
	char filename[MAX_FILENAME_LEN];
	memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
	sprintf(filename, "%s%s/%s", gDataFolder.c_str(), gDataName.c_str(), correspondences[ithDepthToProcess].first.c_str());

	DepthImage depthImage(filename);
	depthImage.Process(cameraPoses[ithDepthToProcess]);

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

	const int numSegments = 200;
	DepthSegmentation segmenter(&depthImage);
	const Segmentation& segment = segmenter.Segment(numSegments);
	segmenter.SaveSegmentedImage("DepthSegmentation.png");



 //	cv::Mat depthImg = ReadDepthImage(filename);
	//cv::Mat depthImgAfterPreprocessing = PreprocessDepthImage(depthImg);

	//ImageSegmentation segmenter;
	//segmenter.Segment(depthImgAfterPreprocessing, 30, 85);
	//segmenter.SaveSegmentedImage("TestSegmentation.png");

	//vector<Vector3d> points;
	//vector<Vector3d> normals;
	//vector<Vector3i> colors;

	//for (int i = 0; i < numFrames; i += 1000)
	//{
	//	char filename[MAX_FILENAME_LEN];
	//	memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
	//	sprintf(filename, "%s%s/%s", gDataFolder.c_str(), gDataName.c_str(), correspondences[i].first.c_str());
	//	DepthImage depthImg(filename);

	//	depthImg.Process(cameraPoses[i]);
	//	points.insert(points.end(), depthImg.GetPoints().begin(), depthImg.GetPoints().end());
	//	normals.insert(normals.end(), depthImg.GetNormals().begin(), depthImg.GetNormals().end());

	//	memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
	//	sprintf(filename, "%s%s/%s", gDataFolder.c_str(), gDataName.c_str(), correspondences[i].second.c_str());
	//	cv::Mat colorImg = ReadColorImage(filename);
	//	ColorImageToColors(colorImg, colors);
	//	LOG(INFO) << "Processing " << i << " out of " << numFrames << " frames." << endl;

	//}
	//double mergeDistance = 0.005;
	//vector<Vector3d> newPoints, newNormals;
	//SubsamplePointCloud(points, normals, mergeDistance, newPoints, newNormals);

	//char filename[MAX_FILENAME_LEN];
	//memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
	//sprintf(filename, "%s%s/points.ply", gDataFolder.c_str(), gDataName.c_str());
	//SavePointCloud(filename, points, colors, normals);

	//CoredFileMeshData<PlyVertex< Real > > mesh;
	//Points2Mesh(points, normals, mesh);

	//memset(filename, 0, MAX_FILENAME_LEN * sizeof(char));
	//sprintf(filename, "%s%s/mesh.ply", gDataFolder.c_str(), gDataName.c_str());
	//SaveMesh(filename, &mesh);

}