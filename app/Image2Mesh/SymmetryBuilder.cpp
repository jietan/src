#include "SymmetryBuilder.h"
#include "MeshIO.h"
#include "utility/mathlib.h"


MirrorPlaneGenerator::MirrorPlaneGenerator(const Eigen::Vector3f& point, const Eigen::Vector3f& normal, int boxId) : mPoint(point), mNormal(normal), mBoxId(boxId)
{

}


UtilPlane MirrorPlaneGenerator::ToPlane() const
{
	return UtilPlane(mNormal, mPoint);
}

SymmetryBuilder::SymmetryBuilder() : mMaxScore(-1)
{

}

SymmetryBuilder::~SymmetryBuilder()
{
	int numTrees = static_cast<int>(mKDTreesPointCloud.size());
	for (int i = 0; i < numTrees; ++i)
	{
		delete mKDTreesPointCloud[i];
	}
	mKDTreesPointCloud.clear();
}
void SymmetryBuilder::SetUpDirection(const Eigen::Vector3f& up)
{
	mUp = up;
}
bool SymmetryBuilder::CheckSymmetry(const vector<BoxFromRectangles>& boxes)
{
	mBoxes = boxes;
	int numBoxes = static_cast<int>(mBoxes.size());
	int numCandidatePlanes = generateCandidatePlane(mBoxes, &mCandidateMirrorPlanes);
	mMirroredBoxes.resize(numCandidatePlanes);
	
	mCandidateSymmetryCorrespondences.resize(numCandidatePlanes);
	mSymmetryScore.resize(numCandidatePlanes);
	for (int i = 0; i < numCandidatePlanes; ++i)
	{
		generateMirroredBoxes(mBoxes, mCandidateMirrorPlanes[i], &(mMirroredBoxes[i]));
		mSymmetryScore[i] = evaluateSymmetry(mBoxes, mMirroredBoxes[i], &(mCandidateSymmetryCorrespondences[i]));
	}
	vector<float>::iterator scoreIt = max_element(mSymmetryScore.begin(), mSymmetryScore.end());
	//mSymmetryPlaneId = scoreIt - mSymmetryScore.begin();
	mMaxScore = *scoreIt;
	for (int i = 0; i < numCandidatePlanes; ++i)
	{
		float symmetryRatio = static_cast<float>(mSymmetryScore[i]) / numBoxes;
		if (symmetryRatio >= 0.95)
		{
			int numActualSymmetricPlanes = GetNumSymmetryPlanes();
			bool toAdd = true;
			for (int ithPlane = 0; ithPlane < numActualSymmetricPlanes; ++ithPlane)
			{
				if (isCorrespondenceSame(mCandidateSymmetryCorrespondences[i], mActualSymmetryCorrespondences[ithPlane]))
				{
					toAdd = false;
					break;
				}
			}
			if (toAdd)
			{
				mActualMirrorPlanes.push_back(mCandidateMirrorPlanes[i]);
				mActualSymmetryCorrespondences.push_back(mCandidateSymmetryCorrespondences[i]);
				mActualMirrorPlaneGenerators.push_back(mCandidateMirrorPlaneGenerators[i]);

			}
		}
	}
	
	return !mActualMirrorPlanes.empty();
}

bool SymmetryBuilder::isCorrespondenceSame(const vector<int>& correspondence1, const vector<int>& correspondence2)
{
	for (size_t i = 0; i < correspondence1.size(); ++i)
	{
		if (correspondence1[i] != -1 && correspondence2[i] != -1 && correspondence1[i] != correspondence2[i])
			return false;
	}
	return true;
}
int SymmetryBuilder::GetNumSymmetryPlanes() const
{
	return static_cast<int>(mActualMirrorPlanes.size());
}

void SymmetryBuilder::RefineSymmetry(int ithSymmetryPlane)
{
	buildKDTreeForPointsInBoxes();
	SetPlaneAsZeroXYTheta(mActualMirrorPlaneGenerators[ithSymmetryPlane].mPoint, mActualMirrorPlaneGenerators[ithSymmetryPlane].mNormal);
	const float xyOffsetBound = 0.03f;
	const float thetaOffsetBound = 5 * CV_PI / 180.f;
	const float xyResolution = 0.01f;
	const float thetaResolution = CV_PI / 180.f;
	float minScore = FLT_MAX;
	UtilPlane minPlane;
	//UtilPlane testPlane = fromXYThetaToPlane(Eigen::Vector3f::Zero());
	//float score = evaluateSymmetryScore(mBoxes, testPlane);


	for (float xOffset = -xyOffsetBound; xOffset <= xyOffsetBound; xOffset += xyResolution)
	{
		for (float yOffset = -xyOffsetBound; yOffset <= xyOffsetBound; yOffset += xyResolution)
		{
			for (float thetaOffset = -thetaOffsetBound; thetaOffset <= thetaOffsetBound; thetaOffset += thetaResolution)
			{
				UtilPlane testPlane = fromXYThetaToPlane(Eigen::Vector3f(xOffset, yOffset, thetaOffset));
				{
					float testOrthogonal = testPlane.GetNormal().dot(mUp);
					float score = evaluateSymmetryScore(mBoxes, ithSymmetryPlane, testPlane);
					LOG(INFO) << "(" << xOffset << ", " << yOffset << ", " << thetaOffset << "):" << score;
					if (score < minScore)
					{
						minScore = score;
						minPlane = testPlane;
					}
				}
			}
		}
	}
	LOG(INFO) << "MinScore: " << minScore;
	LOG(INFO) << "MinPlane: " << minPlane;
	mActualMirrorPlanes[ithSymmetryPlane] = minPlane;

}
vector<pair<int, int> > SymmetryBuilder::GetSymmetryCorrespondences(int ithSymmetricPlane) const
{
	vector<pair<int, int> > ret;
	int numBoxes = static_cast<int>(mBoxes.size());
	
	for (int i = 0; i < numBoxes; ++i)
	{
		if (i <= mActualSymmetryCorrespondences[ithSymmetricPlane][i])
		{
			ret.push_back(pair<int, int>(i, mActualSymmetryCorrespondences[ithSymmetricPlane][i]));
		}
	}
	return ret;
}

const UtilPlane& SymmetryBuilder::GetSymmetryPlane(int ithSymmetryPlane) const
{
	return mActualMirrorPlanes[ithSymmetryPlane];
}

int SymmetryBuilder::generateCandidatePlane(const vector<BoxFromRectangles>& boxes, vector<UtilPlane>* planes)
{
	planes->clear();
	mCandidateMirrorPlaneGenerators.clear();
	int numBoxes = static_cast<int>(boxes.size());
	for (int i = 0; i < numBoxes; ++i)
	{
		const Eigen::Vector3f& pt = boxes[i].GetCenter();
		for (int j = 0; j < 3; ++j)
		{
			Eigen::Vector3f n = mUp.cross(boxes[i].GetAxes()[j]);
			if (n.norm() < EPSILON_FLOAT) continue;
			n.normalize();
			MirrorPlaneGenerator generator(pt, n, i);
			mCandidateMirrorPlaneGenerators.push_back(generator);
			planes->push_back(generator.ToPlane());

		}
	}
	return static_cast<int>(planes->size());
}

void SymmetryBuilder::generateMirroredBoxes(const vector<BoxFromRectangles>& boxes, const UtilPlane& mirrorPlane, vector<BoxFromRectangles>* mirroredBoxes)
{
	int numBoxes = static_cast<int>(boxes.size());
	mirroredBoxes->resize(numBoxes);
	for (int i = 0; i < numBoxes; ++i)
	{
		(*mirroredBoxes)[i] = boxes[i].MirroredBox(mirrorPlane);
	}
	
}
float SymmetryBuilder::evaluateSymmetry(const vector<BoxFromRectangles>& boxes, const vector<BoxFromRectangles>& mirroredBoxes, vector<int>* correspondence)
{
	int numBoxes = static_cast<int>(boxes.size());
	correspondence->clear();
	correspondence->resize(numBoxes, -1);

	float score = 0;
	for (int i = 0; i < numBoxes; ++i)
	{
		for (int j = 0; j < numBoxes; ++j)
		{
			if (boxes[i].IsBoxSimilar(mirroredBoxes[j]) || mirroredBoxes[j].IsBoxSimilar(boxes[i]))
			{
				(*correspondence)[i] = j;
				score += 1;

				break;
			}
		}
	}
	return score;
}

void SymmetryBuilder::SaveVisualization(const string& folderName)
{
	vector<pair<int, int> > correspondence = GetSymmetryCorrespondences(0);
	int numCorrespondences = static_cast<int>(correspondence.size());
	char outputFileName[512];
	for (int i = 0; i < numCorrespondences; ++i)
	{
		unsigned char r = 255 * (rand() / (1.0 + RAND_MAX));
		unsigned char g = 255 * (rand() / (1.0 + RAND_MAX));
		unsigned char b = 255 * (rand() / (1.0 + RAND_MAX));
		int correspondenceFirst = correspondence[i].first;
		int correspondenceSecond = correspondence[i].second;
		vector<Eigen::Vector3f> vertices;
		vector<Eigen::Vector3i> faces;
		vector<Eigen::Vector3i> colors;
		mBoxes[correspondenceFirst].GetGeometry(vertices, faces);
		colors.resize(vertices.size(), Eigen::Vector3i(r, g, b));
		sprintf(outputFileName, "%s/Box_%02d.ply", folderName.c_str(), correspondenceFirst);
		SaveMesh(outputFileName, vertices, colors, faces);
		if (correspondenceFirst != correspondenceSecond)
		{
			mBoxes[correspondenceSecond].GetGeometry(vertices, faces);
			colors.resize(vertices.size(), Eigen::Vector3i(r, g, b));
			sprintf(outputFileName, "%s/Box_%02d.ply", folderName.c_str(), correspondenceSecond);
			SaveMesh(outputFileName, vertices, colors, faces);
		}
	}
}

void SymmetryBuilder::buildKDTreeForPointsInBoxes()
{
	int numBoxes = static_cast<int>(mBoxes.size());
	mKDTreesPointCloud.resize(numBoxes);
	for (int i = 0; i < numBoxes; ++i)
	{
		mKDTreesPointCloud[i] = new sehoon::ann::KDTree;
		mKDTreesPointCloud[i]->setDim(3);
		int numPoints = static_cast<int>(mBoxes[i].mPoints.size());
		for (int j = 0; j < numPoints; ++j)
		{
			mKDTreesPointCloud[i]->add(mBoxes[i].mPoints[j]);
		}
		mKDTreesPointCloud[i]->initANN();
	}
}
float SymmetryBuilder::evaluateSymmetryScoreForPointsInBoxes(const vector<BoxFromRectangles>& boxes, int originalId, int tobeReflectedId, const UtilPlane& plane)
{
	int numPointsToMirror = static_cast<int>(boxes[tobeReflectedId].mPoints.size());
	float score = 0;
	for (int i = 0; i < numPointsToMirror; ++i)
	{
		Eigen::Vector3f mirroredPoint = plane.MirrorPoint(boxes[tobeReflectedId].mPoints[i]);
		Eigen::Vector3f mirroredNormal = plane.MirrorVector(boxes[tobeReflectedId].mNormals[i]);
		vector<int> nnIdx = mKDTreesPointCloud[originalId]->kSearch(mirroredPoint, 1);
		const Eigen::Vector3f& nearestPoint = boxes[originalId].mPoints[nnIdx[0]];
		const Eigen::Vector3f& nearestNormal = boxes[originalId].mNormals[nnIdx[0]];
		float distance = (mirroredPoint - nearestPoint).norm();
		if (distance > 0.1 || nearestNormal.dot(mirroredNormal) < 0.9)
			score += 0.1;
		else
			score += distance;
	}
	return score;

}

float SymmetryBuilder::evaluateSymmetryScore(const vector<BoxFromRectangles>& boxes, int ithSymmetryPlane, const UtilPlane& plane)
{
	float score = 0;
	vector<pair<int, int> > correspondence = GetSymmetryCorrespondences(ithSymmetryPlane);
	int numCorrespondences = static_cast<int>(correspondence.size());
	for (int i = 0; i < numCorrespondences; ++i)
	{
		score += evaluateSymmetryScoreForPointsInBoxes(boxes, correspondence[i].first, correspondence[i].second, plane);
		//if (correspondence[i].first != correspondence[i].second)
		score += evaluateSymmetryScoreForPointsInBoxes(boxes, correspondence[i].second, correspondence[i].first, plane);
	}
	return score;
}
void SymmetryBuilder::SetPlaneAsZeroXYTheta(const Eigen::Vector3f& point, const Eigen::Vector3f& normal) // assume plane normal is the x, up is the z
{
	Eigen::Vector3f x = normal;
	Eigen::Vector3f z = mUp;
	Eigen::Vector3f y = z.cross(x);
	y.normalize();
	
	mXYThetaAxes.col(0) = x;
	mXYThetaAxes.col(1) = y;
	mXYThetaAxes.col(2) = z;
	
	mXYThetaOrigin = point;
}
UtilPlane SymmetryBuilder::fromXYThetaToPlane(const Eigen::Vector3f& xyTheta)
{
	Eigen::Vector3f p = mXYThetaOrigin + xyTheta[0] * mXYThetaAxes.col(0) + xyTheta[1] * mXYThetaAxes.col(1);
	Eigen::Vector3f localN(cos(xyTheta[2]), sin(xyTheta[2]), 0);
	Eigen::Vector3f n = mXYThetaAxes * localN;
	return UtilPlane(n, p);
}