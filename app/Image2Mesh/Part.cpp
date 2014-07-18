#include "Part.h"
#include <queue>
#include "utility/ConfigManager.h"
#include "MeshIO.h"

Part::Part() : mShape(NULL)
{
	mImageRes = 0.02f;
	mSkeletonResizeRatio = 8.f;
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
	mSkeletonResizeRatio = 8.f;
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
	vector<Eigen::Vector3f> axes;
	PCAOnPoints(mPoints, mCenter, axes);
	mAxis1 = axes[2];
	mAxis2 = axes[1];

	mLocalCoords.resize(numPoints);
	mBBMin[0] = mAxis1.dot(mPoints[0] - mCenter);
	mBBMin[1] = mAxis2.dot(mPoints[0] - mCenter);
	mBBMax = mBBMin;
	for (int i = 0; i < numPoints; ++i)
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
	int numPointsThreshold;
	DecoConfig::GetSingleton()->GetInt("Image2Mesh", "PartMinPointThreshold", numPointsThreshold);
	if (numPoints < numPointsThreshold)
		return;
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
	int numPointsThreshold;
	DecoConfig::GetSingleton()->GetInt("Image2Mesh", "PartMinPointThreshold", numPointsThreshold);

	mComponentPart = cv::Mat(mImagePart.size(), CV_8SC1, cv::Scalar(-1));

	if (mPoints.size() < numPointsThreshold)
		return mComponentPart;

	cv::Mat binary;
	std::vector < std::vector<cv::Point2i > > blobs;

	cv::threshold(mImagePart, binary, 0.0, 1.0, cv::THRESH_BINARY);

	findBlobs(binary, blobs);

	// Randomy color the blobs
	for (size_t i = 0; i < blobs.size(); i++) {


		for (size_t j = 0; j < blobs[i].size(); j++) {
			int x = blobs[i][j].x;
			int y = blobs[i][j].y;

			mComponentPart.at<char>(y, x) = i;
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
			if ((mPoints[i] - Eigen::Vector3f(0.447624, 1.40815, 0.992839)).norm() < 0.001)
			{
				imwrite("test.png", mComponentPart);
				Eigen::Vector2f localCoord;
				localCoord[0] = (mPoints[i] - mCenter).dot(mAxis1);
				localCoord[1] = (mPoints[i] - mCenter).dot(mAxis2);
				printf("Hello");
			}
			Eigen::Vector2f gridId = (mLocalCoords[i] - mBBMin) / mImageRes;
			int label = mComponentPart.at<char>(gridId[0], gridId[1]);
			splittedPoints[label].push_back(mPoints[i]);
			splittedNormals[label].push_back(mNormals[i]);
		}
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

void Part::seperatePartsInRectangle(const vector<cv::Vec2f>& lines, Part* splittedPart, Part* remainingPart)
{
	vector<vector<Eigen::Vector2i> > pxWithLines;
	findPixelAssociatedWithLines(lines, &pxWithLines);
	int numLines = static_cast<int>(lines.size());
	int maxPxNumber = 0;
	int maxLineId = -1;
	for (int i = 0; i < numLines; ++i)
	{
		if (pxWithLines[i].size() > maxPxNumber)
		{
			maxPxNumber = static_cast<int>(pxWithLines[i].size());
			maxLineId = i;
		}
	}
	Eigen::Vector2f axis[2];
	Eigen::Vector2f center, extent;
	constructRectangle(lines[maxLineId], pxWithLines[maxLineId], axis, &center, &extent);

	vector<int> inRectPointId, outRectPointId;
	classifyPointsAgainstRectangle(axis, center, extent, mPoints, &inRectPointId, &outRectPointId);
	CHECK(splittedPart && remainingPart) << "Parts should be pre-allocated in Part::seperatePartsInRectangle().";
	vector<Eigen::Vector3f> points;
	vector<Eigen::Vector3f> normals;
	int numPoints = static_cast<int>(inRectPointId.size());
	points.resize(numPoints);
	normals.resize(numPoints);
	for (int i = 0; i < numPoints; ++i)
	{
		points[i] = mPoints[inRectPointId[i]];
		normals[i] = mNormals[inRectPointId[i]];
	}
	*splittedPart = Part(mShape, mShapeIndex, points, normals, NULL);

	points.clear();
	normals.clear();
	numPoints = static_cast<int>(outRectPointId.size());
	points.resize(numPoints);
	normals.resize(numPoints);
	for (int i = 0; i < numPoints; ++i)
	{
		points[i] = mPoints[outRectPointId[i]];
		normals[i] = mNormals[outRectPointId[i]];
	}
	*remainingPart = Part(mShape, mShapeIndex, points, normals, NULL);
}

void Part::findPixelAssociatedWithLines(const vector<cv::Vec2f>& lines, vector<vector<Eigen::Vector2i> >* pxWithLines)
{
	int numLines = static_cast<int>(lines.size());
	int height = mImagePart.rows;
	int width = mImagePart.cols;
	pxWithLines->resize(numLines);
	for (int i = 0; i < height; ++i)
	{
		for (int j = 0; j < width; ++j)
		{
			if (mImagePart.at<uchar>(i, j) < 127) continue;
			for (int k = 0; k < numLines; ++k)
			{
				float rho = lines[k][0] / mSkeletonResizeRatio;
				float theta = lines[k][1];
				float error = sin(theta) * i - ((-cos(theta)) * j + rho);
				if (abs(error) <= 1)
				{
					(*pxWithLines)[k].push_back(Eigen::Vector2i(i, j));
				}
			}
		}
	}

}

void Part::computeCenterAndExtent(const vector<Eigen::Vector2f>& px, const Eigen::Vector2f& axis, float* center, float* extent)
{
	int numPxs = static_cast<int>(px.size());
	
	vector<float> projLens;
	projLens.resize(numPxs);
	for (int i = 0; i < numPxs; ++i)
	{
		projLens[i] = px[i].dot(axis);
	}
	float minValue = *(min_element(projLens.begin(), projLens.end()));
	float maxValue = *(max_element(projLens.begin(), projLens.end()));
	*center = (minValue + maxValue) / 2.f;
	*extent = (maxValue - minValue) / 2.f;
}

void Part::constructRectangle(const cv::Vec2f& line, const vector<Eigen::Vector2i>& pxWithLines, Eigen::Vector2f axis[2], Eigen::Vector2f* center, Eigen::Vector2f* extent)
{
	cv::Mat cdst, cdst0;
	cv::cvtColor(mImagePart, cdst, CV_GRAY2BGR);
	cv::cvtColor(mSkeletonPart, cdst0, CV_GRAY2BGR);
	float rho = line[0], theta = line[1];
	//cv::Point pt1, pt2;
	//double a = cos(theta), b = sin(theta);
	//double x0 = a*rho, y0 = b*rho;
	//pt1.x = cvRound(x0 + 1000 * (-b));
	//pt1.y = cvRound(y0 + 1000 * (a));
	//pt2.x = cvRound(x0 - 1000 * (-b));
	//pt2.y = cvRound(y0 - 1000 * (a));
	//cv::line(cdst0, pt1, pt2, cv::Scalar(0, 0, 255), 1, CV_AA);
	//cv::imshow("detected lines1", cdst0);

	//if (pxWithLines.empty())
	//{
	//	cv::waitKey();
	//}
	for (int j = 0; j < pxWithLines.size(); ++j)
		cdst.at<cv::Vec3b>(pxWithLines[j][0], pxWithLines[j][1]) = cv::Vec3b(0, 0, 255);

	//cv::imshow("detected lines", cdst);
	//cv::waitKey();
	//cv::destroyAllWindows();
	//float theta = line[1];
	axis[0] = Eigen::Vector2f(sin(theta), -cos(theta));
	axis[1] = Eigen::Vector2f(cos(theta), sin(theta));
	int numPxWithLines = static_cast<int>(pxWithLines.size());
	vector<Eigen::Vector2f> coords;
	Eigen::Vector2f centerLocal;
	for (int i = 0; i < numPxWithLines; ++i)
	{
		coords.push_back(Eigen::Vector2f(static_cast<float>(pxWithLines[i][1]), static_cast<float>(pxWithLines[i][0])));
	}
	computeCenterAndExtent(coords, axis[0], &(centerLocal[0]), &((*extent)[0]));
	vector<float> extentAxis2;
	vector<float> minAxis2;
	vector<float> maxAxis2;
	vector<float> centerAxis2;
	extentAxis2.resize(numPxWithLines);
	minAxis2.resize(numPxWithLines);
	maxAxis2.resize(numPxWithLines);
	centerAxis2.resize(numPxWithLines);
	int maxLen = Eigen::Vector2i(mImagePart.rows, mImagePart.cols).norm();
	int width = mImagePart.cols;
	int height = mImagePart.rows;
	for (int i = 0; i < numPxWithLines; ++i)
	{
		extentAxis2[i] = 0;
		int j = 0;
		float bound1, bound2;
		for (j = 0; j < maxLen; ++j)
		{
			int x = static_cast<int>(pxWithLines[i][1] + j * axis[1][0]);
			int y = static_cast<int>(pxWithLines[i][0] + j * axis[1][1]);
			if (x < 0 || y < 0 || x >= width || y >= height || mImagePart.at<uchar>(y, x) < 127)
			{
				bound1 = Eigen::Vector2f(x, y).dot(axis[1]);
				break;
			}
		}
		extentAxis2[i] += j;
		for (j = 0; j > -maxLen; --j)
		{
			int x = static_cast<int>(pxWithLines[i][1] + j * axis[1][0]);
			int y = static_cast<int>(pxWithLines[i][0] + j * axis[1][1]);
			if (x < 0 || y < 0 || x >= width || y >= height || mImagePart.at<uchar>(y, x) < 127)
			{
				bound2 = Eigen::Vector2f(x, y).dot(axis[1]);
				break;
			}
				
		}
		minAxis2[i] = bound1 < bound2 ? bound1 : bound2;
		maxAxis2[i] = bound1 < bound2 ? bound2 : bound1;
		centerAxis2[i] = (bound1 + bound2) / 2.f;
		extentAxis2[i] += abs(j);
	}
	sort(extentAxis2.begin(), extentAxis2.end());
	sort(centerAxis2.begin(), centerAxis2.end());
	(*extent)[1] = extentAxis2[numPxWithLines / 2] / 2;
	centerLocal[1] = centerAxis2[numPxWithLines / 2];
	Eigen::Matrix2f rot;
	rot.col(0) = axis[0];
	rot.col(1) = axis[1];
	//Eigen::Vector2f centerInCoord = rot * centerLocal;
	//(*center)[0] = centerInCoord[1];
	//(*center)[1] = centerInCoord[0];
	*center = rot * centerLocal;
}
void Part::classifyPointsAgainstRectangle(Eigen::Vector2f axis[2], const Eigen::Vector2f& center, const Eigen::Vector2f& extent, const vector<Eigen::Vector3f>& points, vector<int>* inRectPointId, vector<int>* outRectPointId)
{
	int numPoints = static_cast<int>(mPoints.size());
	for (int i = 0; i < numPoints; ++i)
	{
		if (isPointInRectangle(mLocalCoords[i], axis, center, extent))
		{
			inRectPointId->push_back(i);
		}
		else
		{
			outRectPointId->push_back(i);
		}
	}
}

bool Part::isPointInRectangle(const Eigen::Vector2f& localCoord, Eigen::Vector2f axis[2], const Eigen::Vector2f& center, const Eigen::Vector2f& extent)
{
	Eigen::Vector2f gridId = (localCoord - mBBMin) / mImageRes;
	Eigen::Vector2f gridIdCoord(gridId[1], gridId[0]);
	float projAxis1 = (gridIdCoord - center).dot(axis[0]);
	float projAxis2 = (gridIdCoord - center).dot(axis[1]);
	if (abs(projAxis1) < extent[0] && abs(projAxis2) < extent[1])
		return true;
	else
		return false;

}

void Part::DivideBySkeleton(vector<Part>& newParts, int level)
{
	int numPointsThreshold;
	DecoConfig::GetSingleton()->GetInt("Image2Mesh", "PartMinPointThreshold", numPointsThreshold);

	if (mPoints.size() < numPointsThreshold)
		return;

	if (mAreaRatio > 0.7)
	{
		newParts.push_back(*this);
		return;
	}
	cv::Mat dst, cdst, tmpImage;
	//Canny(mImagePart, mSkeletonPart, 50, 200, 3);
	
	cv::resize(mSkeletonPart, mSkeletonPart, cv::Size(mSkeletonPart.cols * mSkeletonResizeRatio, mSkeletonPart.rows * mSkeletonResizeRatio), 0, 0, cv::INTER_NEAREST);
	cv::cvtColor(mImagePart, cdst, CV_GRAY2BGR);
	vector<cv::Vec2f> lines;
	int higherBound = sqrt((mSkeletonPart.rows * mSkeletonPart.rows) + (mSkeletonPart.cols * mSkeletonPart.cols));
	int lowerBound = std::min(mSkeletonPart.rows, mSkeletonPart.cols) / 8;
	int defaultThreshold = higherBound;
	while (lines.empty() && defaultThreshold > lowerBound)
	{
		cv::HoughLines(mSkeletonPart, lines, 1, CV_PI / 18, defaultThreshold, 0, 0);
		defaultThreshold /= 2;
	}

	/* some visualization code */
	//const cv::Vec2f line = lines[0];
	//cv::Mat cdst0;
	//cv::cvtColor(mImagePart, cdst0, CV_GRAY2BGR);
	//float rho = line[0] / mSkeletonResizeRatio, theta = line[1];
	//cv::Point pt1, pt2;
	//double a = cos(theta), b = sin(theta);
	//double x0 = a*rho, y0 = b*rho;
	//pt1.x = cvRound(x0 + 1000 * (-b));
	//pt1.y = cvRound(y0 + 1000 * (a));
	//pt2.x = cvRound(x0 - 1000 * (-b));
	//pt2.y = cvRound(y0 - 1000 * (a));
	//cv::line(cdst0, pt1, pt2, cv::Scalar(0, 0, 255), 1, CV_AA);
	//cv::imshow("detected lines0", cdst0);
	//cv::waitKey();

	if (lines.empty())
	{
		DivideByConnectivity(newParts);
		return;
	}
	Part splittedPart, remainingPart;
	seperatePartsInRectangle(lines, &splittedPart, &remainingPart);
	//newParts.push_back(splittedPart);


	vector<Part> toProcess;

	splittedPart.Process();
	splittedPart.DivideByConnectivity(toProcess);


	remainingPart.Process();
	remainingPart.DivideByConnectivity(toProcess);

		//cv::Mat img = remainingPart.ImagePart();
		//char tmpFilename[512];
		//sprintf(tmpFilename, "../../../Tables/400000752.668193/partsImage/partSkeletonLevel%03d.png", level);
		//imwrite(tmpFilename, img);
	int numToProcess = static_cast<int>(toProcess.size());
	for (int i = 0; i < numToProcess; ++i)
	{
		toProcess[i].Process();
		toProcess[i].DivideBySkeleton(newParts, level + 1);
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
