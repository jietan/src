#include "Part.h"
#include "utility/ConfigManager.h"
#include "MeshIO.h"

Part::Part() : mShape(NULL)
{
	mImageRes = 0.02f;
}
Part::Part(PrimitiveShape* shape, int shapeIdx, const vector<Eigen::Vector3f>& points, const vector<Eigen::Vector3f>& normals, sehoon::ann::KDTree* tree)
{
	mKDTreePoints = tree;
	mShape = shape;
	mPoints = points;
	mNormals = normals;
	mShapeIndex = shapeIdx;
	mImageRes = 0.02f;
	int numPoints = static_cast<int>(mPoints.size());
	if (mKDTreePoints)
	{
		mKDTreePoints->setDim(3);
		for (int i = 0; i < numPoints; ++i)
		{
			if (i % 10000 == 0)
				LOG(INFO) << "Finish adding " << i << "th points out of " << numPoints;
			mKDTreePoints->add(points[i]);
		}
		mKDTreePoints->initANN();
	}

}

PrimitiveShape* Part::GetShape() const
{
	return mShape;
}
const vector<Eigen::Vector3f>& Part::GetPoints() const
{
	return mPoints;
}
const vector<Eigen::Vector3f>& Part::GetNormals() const
{
	return mNormals;
}
bool Part::IsPointClose(const Eigen::Vector3f& pt)
{
	vector<int> nnId = mKDTreePoints->kSearch(pt, 1);
	Eigen::Vector3f nnPt = mPoints[nnId[0]];
	return (nnPt - pt).norm() < 0.05;
}

const cv::Mat& Part::ImagePart()
{
	CHECK(mShape->Identifier() == 0) << "Only plane primitive is supported.";
	int numPoints = static_cast<int>(mPoints.size());
	//Eigen::MatrixXf dataMat = Eigen::MatrixXf(numPoints, 3);
	//PlanePrimitiveShape* planePrimitive = static_cast<PlanePrimitiveShape*>(mShape);
	//Vec3f n = planePrimitive->Internal().getNormal();
	//cout << "Plane normals is :" << Eigen::Vector3f(n.getValue());
	//for (int i = 0; i < numPoints; ++i)
	//{
	//	dataMat.row(i) = mPoints[i];
	//}
	//mCenter = dataMat.colwise().mean();
	//Eigen::MatrixXf centered = dataMat.rowwise() - mCenter.transpose();
	//Eigen::MatrixXf cov = centered.transpose() * centered;

	//Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> es;

	//es.compute(cov);
	Eigen::Vector3f axes[3];
	PCAOnPoints(mPoints, mCenter, axes);
	mAxis1 = axes[2];
	mAxis2 = axes[1];

	mLocalCoords.resize(numPoints);
	mBBMin[0] = mAxis1.dot(mPoints[0] - mCenter);
	mBBMin[1] = mAxis2.dot(mPoints[0] - mCenter);
	mBBMax = mBBMin;
	for (int i = 1; i < numPoints; ++i)
	{
		float projAxis1 = mAxis1.dot(mPoints[i] - mCenter);
		if (projAxis1 < mBBMin[0])
			mBBMin[0] = projAxis1;
		if (projAxis1 > mBBMax[0])
			mBBMax[0] = projAxis1;
		float projAxis2 = mAxis2.dot(mPoints[i] - mCenter);
		if (projAxis2 < mBBMin[1])
			mBBMin[1] = projAxis2;
		if (projAxis2 > mBBMax[1])
			mBBMax[1] = projAxis2;
		mLocalCoords[i] = Eigen::Vector2f(projAxis1, projAxis2);
	}
	float extent1 = mBBMax[0] - mBBMin[0];
	float extent2 = mBBMax[1] - mBBMin[1];
	
	int height = extent1 / mImageRes + 1;
	int width = extent2 / mImageRes + 1;

	mImagePart = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
	int count = 0;
	for (int i = 0; i < numPoints; ++i)
	{
		Eigen::Vector2f gridId = (mLocalCoords[i] - mBBMin) / mImageRes;
		mImagePart.at<uchar>(static_cast<int>(gridId[0]), static_cast<int>(gridId[1])) = 255;
	}

	//cv::threshold(mImagePart, mImagePart, 127, 255, cv::THRESH_BINARY);
	//cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
	//cv::Mat temp;
	//cv::Mat eroded;
	//cv::erode(mImagePart, eroded, element);
	//cv::dilate(eroded, temp, element); // temp = open(img)
	//eroded.copyTo(mImagePart);

	for (int i = 0; i < height; ++i)
	{
		for (int j = 0; j < width; ++j)
		{
			if (mImagePart.at<uchar>(i, j) == 255)
				count++;
		}
	}
	mAreaRatio = static_cast<float>(count) / (height * width);
	LOG(INFO) << "Area ratio: " << mAreaRatio;
	return mImagePart;
}

const cv::Mat& Part::SkeletonPart()
{
	cv::Mat binaryImage;
	cv::threshold(mImagePart, binaryImage, 127, 255, cv::THRESH_BINARY);
	mSkeletonPart = cv::Mat(mImagePart.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat temp;
	cv::Mat eroded;

	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

	int imageArea = mImagePart.rows * mImagePart.cols;
	bool done;
	int count = 0;
	do
	{
		cv::erode(binaryImage, eroded, element);
		cv::dilate(eroded, temp, element); // temp = open(img)
		cv::subtract(binaryImage, temp, temp);
		cv::bitwise_or(mSkeletonPart, temp, mSkeletonPart);
		eroded.copyTo(binaryImage);
		count++;
		done = (cv::countNonZero(binaryImage) == 0 || count > imageArea);
	} while (!done);
	
	return mSkeletonPart;
}

void Part::DivideByNormalDirection(vector<Part>& newParts)
{
	int numPoints = static_cast<int>(mPoints.size());
	PlanePrimitiveShape* planePrimitive = static_cast<PlanePrimitiveShape*>(mShape);
	Vec3f n = planePrimitive->Internal().getNormal();
	vector<Eigen::Vector3f> reversedPoints;
	vector<Eigen::Vector3f> reversedNormals;

	vector<Eigen::Vector3f> correctPoints;
	vector<Eigen::Vector3f> correctNormals;
	for (int i = 0; i < numPoints; ++i)
	{
		if (mNormals[i].dot(Eigen::Vector3f(n.getValue())) < 0)
		{
			reversedPoints.push_back(mPoints[i]);
			reversedNormals.push_back(mNormals[i]);
		}
		else
		{
			correctPoints.push_back(mPoints[i]);
			correctNormals.push_back(mNormals[i]);
		}
	}
	if (reversedPoints.empty() || correctPoints.empty())
	{
		newParts.push_back(*this);
	}
	else
	{
		int numPointsThreshold;
		DecoConfig::GetSingleton()->GetInt("Image2Mesh", "PartMinPointThreshold", numPointsThreshold);

		if (correctPoints.size() > numPointsThreshold)
		{
			Part part1(mShape, mShapeIndex, correctPoints, correctNormals, NULL); 
			newParts.push_back(part1);
		}
		if (reversedPoints.size() > numPointsThreshold)
		{
			Part part2(mShape, mShapeIndex, reversedPoints, reversedNormals, NULL);
			newParts.push_back(part2);
		}
	}
}
const cv::Mat& Part::DivideByConnectivity(vector<Part>& newParts)
{
	mComponentPart = cv::Mat::zeros(mImagePart.size(), CV_8UC1);

	cv::Mat binary;
	std::vector < std::vector<cv::Point2i > > blobs;

	cv::threshold(mImagePart, binary, 0.0, 1.0, cv::THRESH_BINARY);

	findBlobs(binary, blobs);

	// Randomy color the blobs
	for (size_t i = 0; i < blobs.size(); i++) {


		for (size_t j = 0; j < blobs[i].size(); j++) {
			int x = blobs[i][j].x;
			int y = blobs[i][j].y;

			mComponentPart.at<uchar>(y, x) = i;
			//mComponentPart.at<cv::Vec3b>(y, x)[1] = g;
			//mComponentPart.at<cv::Vec3b>(y, x)[2] = r;
		}
	}
	int numNewParts = static_cast<int>(blobs.size());
	if (numNewParts == 1)
	{
		newParts.push_back(*this);
	}
	else
	{
		int numPoints = static_cast<int>(mPoints.size());
		vector<vector<Eigen::Vector3f> > splittedPoints;
		vector<vector<Eigen::Vector3f> > splittedNormals;
		splittedNormals.resize(numNewParts);
		splittedPoints.resize(numNewParts);

		for (int i = 0; i < numPoints; ++i)
		{
			Eigen::Vector2f gridId = (mLocalCoords[i] - mBBMin) / mImageRes;
			int label = mComponentPart.at<uchar>(gridId[0], gridId[1]);
			splittedPoints[label].push_back(mPoints[i]);
			splittedNormals[label].push_back(mNormals[i]);
		}
		int numPointsThreshold;
		DecoConfig::GetSingleton()->GetInt("Image2Mesh", "PartMinPointThreshold", numPointsThreshold);
		for (int i = 0; i < numNewParts; ++i)
		{
			if (splittedPoints[i].size() > numPointsThreshold)
			{
				newParts.push_back(Part(mShape, mShapeIndex, splittedPoints[i], splittedNormals[i], NULL));
			}
		}
	}
	return mComponentPart;
}
void Part::DivideBySkeleton(vector<Part>& newParts)
{
	if (mAreaRatio > 0.7)
	{
		newParts.push_back(*this);
		return;
	}
}

void Part::Process()
{
	voteForNormal();
	ImagePart();
	SkeletonPart();
}

void Part::findBlobs(const cv::Mat &binary, std::vector < std::vector<cv::Point2i> > &blobs)
{
	blobs.clear();

	// Fill the label_image with the blobs
	// 0  - background
	// 1  - unlabelled foreground
	// 2+ - labelled foreground

	cv::Mat label_image;
	binary.convertTo(label_image, CV_32SC1);

	int label_count = 2; // starts at 2 because 0,1 are used already

	for (int y = 0; y < label_image.rows; y++) {
		int *row = (int*)label_image.ptr(y);
		for (int x = 0; x < label_image.cols; x++) {
			if (row[x] != 1) {
				continue;
			}

			cv::Rect rect;
			cv::floodFill(label_image, cv::Point(x, y), label_count, &rect, 0, 0, 4);

			std::vector <cv::Point2i> blob;

			for (int i = rect.y; i < (rect.y + rect.height); i++) {
				int *row2 = (int*)label_image.ptr(i);
				for (int j = rect.x; j < (rect.x + rect.width); j++) {
					if (row2[j] != label_count) {
						continue;
					}

					blob.push_back(cv::Point2i(j, i));
				}
			}

			blobs.push_back(blob);

			label_count++;
		}
	}
}

const Eigen::Vector3f& Part::GetNormal() const
{
	return mPartNormal;
}

void Part::voteForNormal()
{
	PlanePrimitiveShape* planePrimitive = static_cast<PlanePrimitiveShape*>(mShape);
	Vec3f n = planePrimitive->Internal().getNormal();
	Eigen::Vector3f normal(n.getValue());

	int vote = 0;
	int numNormals = static_cast<int>(mNormals.size());
	for (int i = 0; i < numNormals; ++i)
	{
		if (mNormals[i].dot(normal) > 0)
			vote++;
	}
	if (vote > numNormals / 2)
	{
		mPartNormal = normal;
	}
	else
	{
		mPartNormal = -normal;
	}
}

void Part::Save(const string& filename)
{
	string fullFileName = filename;
	fullFileName += ".txt";
	ofstream out(fullFileName.c_str());
	out << mShapeIndex;
	fullFileName = filename;
	fullFileName += ".ply";
	vector<Eigen::Vector3i> colors;
	SavePointCloud(fullFileName.c_str(), mPoints, colors, mNormals);
}
void Part::Read(const string& filename, const vector<PrimitiveShape*> primitives)
{
	string fullFileName = filename + ".txt";
	ifstream in(fullFileName.c_str());
	in >> mShapeIndex;
	mShape = primitives[mShapeIndex];

	string plyFileName = filename + ".ply";
	ReadPointCloud(plyFileName, mPoints, mNormals);
	Process();
}
PartRectangle Part::GetRectangle() const
{
	PartRectangle ret;
	ret.mNormal = GetNormal();
	ret.mPoints = mPoints;
	ret.mTangent1 = mAxis1;
	ret.mTangent2 = mAxis2;
	if (mAxis1.cross(mAxis2).dot(ret.mNormal) < 0)
		ret.mTangent2 = -ret.mTangent2;
	ret.mExtent = (mBBMax - mBBMin) / 2.f;

	Eigen::Vector2f bbCenter = (mBBMax + mBBMin) / 2.f;
	ret.mCenter = mCenter + bbCenter[0] * mAxis1 + bbCenter[1] * mAxis2;

	return ret;
}
