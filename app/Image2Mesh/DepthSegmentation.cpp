#include "DepthSegmentation.h"
#include "utility/mathlib.h"
#include <glog/logging.h>
using namespace google;
#include <opencv2/core/eigen.hpp>

#include <algorithm>

DepthSegmentation::DepthSegmentation(DepthImage* depthImage)
{
	mDepthImage = depthImage;

	mNumRows = mDepthImage->NumRows();
	mNumCols = mDepthImage->NumCols();
}
DepthSegmentation::~DepthSegmentation()
{

}
const Segmentation& DepthSegmentation::Segment(int numSegments)
{
	const vector<Vector3d>& points = mDepthImage->GetPoints();
	const vector<Vector3d>& normals = mDepthImage->GetNormals();

	int numPoints = static_cast<int>(points.size());
	cv::Mat1f pointsData;
	cv::Mat labels;
	pointsData.create(numPoints, 3);
	for (int i = 0; i < numPoints; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			pointsData.at<float>(i, j) = points[i][j];
		}
	}

	//pointsData.create(numPoints, 6);
	//for (int i = 0; i < numPoints; ++i)
	//{
	//	for (int j = 0; j < 3; ++j)
	//	{
	//		pointsData.at<float>(i, j) = points[i][j];
	//		pointsData.at<float>(i, j + 3) = normals[i][j];
	//	}
	//}

	cv::kmeans(pointsData, numSegments, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0), 1, cv::KMEANS_PP_CENTERS);

//	mLabelImage = cv::Mat(mNumRows, mNumCols, CV_8U);
	mSegmentation.mSegmentedImg = MatrixXi::Zero(mNumRows, mNumCols);
	mSegmentation.mSegmentedPixelIdx.clear();
	mSegmentation.mSegmentedPixelIdx.resize(numSegments);

	int count = 0;
	for (int i = 0; i < mNumRows; ++i)
	{
		for (int j = 0; j < mNumCols; ++j)
		{
			int label = labels.at<int>(count);
			//mLabelImage.at<uchar>(i, j) = static_cast<uchar>(label);
			mSegmentation.mSegmentedImg(i, j) = label;
			mSegmentation.mSegmentedPixelIdx[label].insert(count);
			count++;
		}
	}
	
	mergeSegments();
	//imshow("clustered", mLabelImage);
	return mSegmentation;
}

void DepthSegmentation::initializeAdjacency()
{
	int numSegments = NumSegments();
	mSegmentAdjacency = MatrixXi::Constant(numSegments, numSegments, 0);
	mBoundaryIdx.resize(numSegments);
	int count = 0;

	for (int i = 0; i < mNumRows; ++i)
	{
		for (int j = 0; j < mNumCols; ++j)
		{
			int currentLabel = mSegmentation.mSegmentedImg(i, j);
			for (int l = -1; l <= 1; ++l)
			{
				for (int m = -1; m <= 1; ++m)
				{
					int adjI = i + l;
					int adjJ = j + m;
					if (adjI < 0 || adjI >= mNumRows || adjJ < 0 || adjJ >= mNumCols)
						continue;

					int adjLabel = mSegmentation.mSegmentedImg(adjI, adjJ);
					if (currentLabel != adjLabel)
					{
						mSegmentAdjacency(currentLabel, adjLabel) = 1;
						mSegmentAdjacency(adjLabel, currentLabel) = 1;
						mBoundaryIdx[currentLabel].insert(count);
					}
				}
			}
			count++;
		}
	}

}
void DepthSegmentation::initializeSimilarity()
{
	int numSegments = NumSegments();
	mSegmentSimilarity = MatrixXf::Constant(numSegments, numSegments, -1);
	for (int i = 0; i < numSegments; ++i)
	{
		for (int j = 0; j < numSegments; ++j)
		{
			if (i >= j) continue;
			if (mSegmentAdjacency(i, j))
			{
				double similarity = computeSimilarity(i, j);
				mSegmentSimilarity(i, j) = similarity;
				mSegmentSimilarity(j, i) = similarity;
			}
		}
	}
}

VectorXd DepthSegmentation::normalizeHistogram(const VectorXi& hist)
{
	int numPixels = hist.sum();
	VectorXd ret = hist.cast<double>();
	ret /= numPixels;
	return ret;
}

void DepthSegmentation::initializeNormalHistogram()
{
	int numSegments = NumSegments();
	mNormalHistogram.resize(numSegments);
	mNormalHistogramNormalized.resize(numSegments);
	for (int i = 0; i < numSegments; ++i)
	{
		VectorXi hist = computeNormalHistogram(i);
		VectorXd histNormalized = normalizeHistogram(hist);
		mNormalHistogram[i] = hist;
		mNormalHistogramNormalized[i] = histNormalized;
	}
}

VectorXi DepthSegmentation::computeNormalHistogram(int ithSegment)
{
	const double phi = (1 + sqrt(5)) / 2.0;
	Vector3d faceNormalIcosahedron[NUM_ICOSAGEDRON_FACES] =
	{
		Vector3d(-1, -1, -1),
		Vector3d(-1, -1, 1),
		Vector3d(-1, 1, -1),
		Vector3d(-1, 1, 1),
		Vector3d(1, -1, -1),
		Vector3d(1, -1, 1),
		Vector3d(1, 1, -1),
		Vector3d(1, 1, 1),

		Vector3d(0, -1 / phi, -phi),
		Vector3d(0, -1 / phi, phi),
		Vector3d(0, 1 / phi, -phi),
		Vector3d(0, 1 / phi, phi),

		Vector3d(-1 / phi, -phi, 0),
		Vector3d(-1 / phi, phi, 0),
		Vector3d(1 / phi, -phi, 0),
		Vector3d(1 / phi, phi, 0),

		Vector3d(-phi, 0, -1 / phi),
		Vector3d(-phi, 0, 1 / phi),
		Vector3d(phi, 0, -1 / phi),
		Vector3d(phi, 0, 1 / phi),
	};
	for (int i = 0; i < NUM_ICOSAGEDRON_FACES; ++i)
	{
		faceNormalIcosahedron[i].normalize();
	}
	VectorXi hist = VectorXi::Zero(NUM_ICOSAGEDRON_FACES);
	const vector<Vector3d>& normals = mDepthImage->GetNormals();

	VectorXd normalClassificationBins = VectorXd::Zero(NUM_ICOSAGEDRON_FACES);

	for (set<int>::const_iterator it = mSegmentation.mSegmentedPixelIdx[ithSegment].begin(); it != mSegmentation.mSegmentedPixelIdx[ithSegment].end(); ++it)
	{
		int idx = *it;
		Vector3d n = normals[idx];
		for (int ithClass = 0; ithClass < NUM_ICOSAGEDRON_FACES; ++ithClass)
		{
			normalClassificationBins[ithClass] = faceNormalIcosahedron[ithClass].dot(n);
		}

		int normalClass;
		normalClassificationBins.maxCoeff(&normalClass);
		hist[normalClass]++;
	}
	return hist;
}

//void DepthSegmentation::preComputeBoundaryPixels()
//{
//	int numSegments = NumSegments();
//	mBoundaryIdx.resize(numSegments);
//	for (int i = 0; i < mNumRows; ++i)
//	{
//		for (int j = 0; j < mNumCols; ++j)
//		{
//			int idx = indexTo1d(i, j, mNumRows, mNumCols);
//			int currentLabel = mSegmentation.mSegmentedImg(i, j);
//
//			if (i > 0)
//			{
//				int leftLabel = mSegmentation.mSegmentedImg(i - 1, j);
//				if (currentLabel != leftLabel)
//					mBoundaryIdx[currentLabel].insert(idx);
//			}
//			if (i < mNumRows - 1)
//			{
//				int rightLabel = mSegmentation.mSegmentedImg(i + 1, j);
//				if (currentLabel != rightLabel)
//					mBoundaryIdx[currentLabel].insert(idx);
//			}
//			if (j > 0)
//			{
//				int upperLabel = mSegmentation.mSegmentedImg(i, j - 1);
//				if (currentLabel != upperLabel)
//					mBoundaryIdx[currentLabel].insert(idx);
//			}
//			if (j < mNumCols - 1)
//			{
//				int lowerLabel = mSegmentation.mSegmentedImg(i, j + 1);
//				if (currentLabel != lowerLabel)
//					mBoundaryIdx[currentLabel].insert(idx);
//			}
//		}
//	}
//}

double DepthSegmentation::computeBoundaryDepthDifference(int ithSegment, int jthSegment)
{
	int numBoundaryPixels = static_cast<int>(mBoundaryIdx[ithSegment].size());
	vector<float> depthDiff;

	for (set<int>::const_iterator it = mBoundaryIdx[ithSegment].begin(); it != mBoundaryIdx[ithSegment].end(); ++it)
	{
		int idx = *it;
		int i, j;
		indexTo2d(idx, i, j, mNumRows, mNumCols);

		for (int l = -1; l <= 1; ++l)
		{
			for (int m = -1; m <= 1; ++m)
			{
				int adjI = i + l;
				int adjJ = j + m;
				if (adjI < 0 || adjI >= mNumRows || adjJ < 0 || adjJ >= mNumCols)
					continue;

				int adjLabel = mSegmentation.mSegmentedImg(adjI, adjJ);
				if (adjLabel == jthSegment)
				{
					depthDiff.push_back(abs(mDepthImage->Depth(i, j) - mDepthImage->Depth(adjI, adjJ)));
				}
			}
		}
	}
	sort(depthDiff.begin(), depthDiff.end());
	return depthDiff[depthDiff.size() / 2];

	vector<float>::const_iterator maxIt = max_element(depthDiff.begin(), depthDiff.end());
	return *maxIt;
}

double DepthSegmentation::computeSimilarity(int ithSegment, int jthSegment)
{
	float maxDepthDiffAlongBoundary = computeBoundaryDepthDifference(ithSegment, jthSegment);

	if (maxDepthDiffAlongBoundary / 1000.0 > 0.05) return -1;
	const VectorXd& histI = mNormalHistogramNormalized[ithSegment];
	const VectorXd& histJ = mNormalHistogramNormalized[jthSegment];
	CHECK(histI.size() == histJ.size()) << "Histogram with different length in DepthSegmentation::computeSimilarity()";
	int histLen = histI.size();
	VectorXd minHist = VectorXd::Zero(histLen);
	for (int i = 0; i < histLen; ++i)
	{
		minHist[i] = histI[i] > histJ[i] ? histJ[i] : histI[i];
	}
	double ret = minHist.sum();
	CHECK(ret >= 0 && ret <= 1) << "minHist is not within [0, 1] in DepthSegmentation::computeSimilarity()";
	return ret;
}
void DepthSegmentation::mergeSegments(int ithSegment, int jthSegment)
{
	CHECK(mSegmentAdjacency(ithSegment, jthSegment)) << "Cannot merge segments that are not adjacent.";

	// union normal histogram
	mNormalHistogram[ithSegment] += mNormalHistogram[jthSegment];
	mNormalHistogramNormalized[ithSegment] = normalizeHistogram(mNormalHistogram[ithSegment]);

	// merge boundary pixels
	mBoundaryIdx[ithSegment].insert(mBoundaryIdx[jthSegment].begin(), mBoundaryIdx[jthSegment].end());
	mBoundaryIdx[jthSegment].clear();

	// change mSegmentation.mSegmentationImg;
	for (set<int>::const_iterator it = mSegmentation.mSegmentedPixelIdx[jthSegment].begin(); it != mSegmentation.mSegmentedPixelIdx[jthSegment].end(); ++it)
	{
		int idx = *it;
		int u, v;
		indexTo2d(idx, v, u, mNumRows, mNumCols);
		mSegmentation.mSegmentedImg(v, u) = ithSegment;
	}
	// change mSegmentation.mSegmentationPixelId;
	mSegmentation.mSegmentedPixelIdx[ithSegment].insert(mSegmentation.mSegmentedPixelIdx[jthSegment].begin(), mSegmentation.mSegmentedPixelIdx[jthSegment].end());
	mSegmentation.mSegmentedPixelIdx[jthSegment].clear();

	// merge adjacency table and recompute the similarity matrix
	mSegmentAdjacency(ithSegment, jthSegment) = 0; //jthSegment no longer exists
	for (int i = 0; i < mSegmentAdjacency.cols(); ++i)
	{
		if (mSegmentAdjacency(jthSegment, i))
		{
			mSegmentAdjacency(jthSegment, i) = mSegmentAdjacency(i, jthSegment) = 0; //jthSegment no longer exists
			mSegmentSimilarity(jthSegment, i) = mSegmentSimilarity(i, jthSegment) = -1;

			if (i != ithSegment)
			{
				mSegmentAdjacency(ithSegment, i) = mSegmentAdjacency(i, ithSegment) = 1; //segments adjacent to jthSegment is now adjacent to ithSegment
				mSegmentSimilarity(ithSegment, i) = mSegmentSimilarity(i, ithSegment) = computeSimilarity(i, ithSegment);
			}
		}
	}


}


void DepthSegmentation::mergeSegments()
{
	initializeAdjacency();
	initializeNormalHistogram();
	//preComputeBoundaryPixels();
	initializeSimilarity();
	
	int numSegments = NumSegments();
	for (int i = numSegments; i >= 2; --i)
	{
		//if (i == 14)
		//{
		//	printf("Hello");
		//	
		//	for (int testI = 0; testI < 200; ++testI)
		//	{
		//		for (int testJ = 0; testJ < 200; ++testJ)
		//		{
		//			if (mSegmentAdjacency(testI, testJ))
		//			{
		//				LOG(INFO) << testI << ": " << testJ << " " << computeBoundaryDepthDifference(testI, testJ);
		//			}
		//		}
		//	}
		//}
			
		int segmentI, segmentJ;
		float similarity = mSegmentSimilarity.maxCoeff(&segmentI, &segmentJ);
		if (similarity < 0.4)
			break;
		mergeSegments(segmentI, segmentJ);
		
		char testFileName[512];
		sprintf(testFileName, "testImages/DepthSegmentation%03d.png", i);
		SaveSegmentedImage(testFileName);
		LOG(INFO) << i << " segments remain.";
	}
	mSegmentation.Rebuild();

}
void DepthSegmentation::SaveSegmentedImage(const string& filename)
{
	cv::Mat labelImage(mNumRows, mNumCols, CV_8U);
	Matrix<uchar, Dynamic, Dynamic>labelImageEigen = mSegmentation.mSegmentedImg.cast<uchar>();
	cv::eigen2cv(labelImageEigen, labelImage);
	
	cv::Mat labelBoundaryImage;


	Laplacian(labelImage, labelBoundaryImage, CV_8U, 3, 1, 0, cv::BORDER_DEFAULT);
	float maxDepth = mDepthImage->MaxDepth();

	cv::Mat visualizationImage(mNumRows, mNumCols, CV_8U);
	for (int i = 0; i < mNumRows; ++i)
	{
		for (int j = 0; j < mNumCols; ++j)
		{
			visualizationImage.at<uchar>(i, j) = static_cast<uchar>(mDepthImage->Depth(i, j) / (1.2 * maxDepth) * 255);
			if (labelBoundaryImage.at<uchar>(i, j) != 0)
			{
				visualizationImage.at<uchar>(i, j) = 255;
			}
		}
	}
	imwrite(filename, visualizationImage);
	string opaqueFilename = filename;
	opaqueFilename += "opaque.png";
	imwrite(opaqueFilename, labelImage);
}

int DepthSegmentation::NumSegments() const
{
	return mSegmentation.NumSegments();
}