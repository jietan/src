#include "SymmetryBuilder.h"
#include "MeshIO.h"

SymmetryBuilder::SymmetryBuilder() : mMaxScore(-1), mSymmetryPlaneId(-1)
{

}
void SymmetryBuilder::SetUpDirection(const Eigen::Vector3f& up)
{
	mUp = up;
}
bool SymmetryBuilder::CheckSymmetry(const vector<BoxFromRectangles>& boxes)
{
	mBoxes = boxes;
	int numBoxes = static_cast<int>(mBoxes.size());
	int numCandidatePlanes = generateCandidatePlane(mBoxes, &mMirrorPlanes);
	mMirroredBoxes.resize(numCandidatePlanes);
	mSymmetryCorrespondences.resize(numCandidatePlanes);
	mSymmetryScore.resize(numCandidatePlanes);
	for (int i = 0; i < numCandidatePlanes; ++i)
	{
		generateMirroredBoxes(mBoxes, mMirrorPlanes[i], &(mMirroredBoxes[i]));
		mSymmetryScore[i] = evaluateSymmetry(mBoxes, mMirroredBoxes[i], &(mSymmetryCorrespondences[i]));
	}
	vector<float>::iterator scoreIt = max_element(mSymmetryScore.begin(), mSymmetryScore.end());
	mSymmetryPlaneId = scoreIt - mSymmetryScore.begin();
	mMaxScore = *scoreIt;
	float symmetryRatio = static_cast<float>(mMaxScore) / numBoxes;
	return symmetryRatio > 0.8f;
}
vector<pair<int, int> > SymmetryBuilder::GetSymmetryCorrespondences() const
{
	vector<pair<int, int> > ret;
	int numBoxes = static_cast<int>(mBoxes.size());
	
	for (int i = 0; i < numBoxes; ++i)
	{
		if (i <= mSymmetryCorrespondences[mSymmetryPlaneId][i])
		{
			ret.push_back(pair<int, int>(i, mSymmetryCorrespondences[mSymmetryPlaneId][i]));
		}
	}
	return ret;
}

const UtilPlane& SymmetryBuilder::GetSymmetryPlane() const
{
	return mMirrorPlanes[mSymmetryPlaneId];
}

int SymmetryBuilder::generateCandidatePlane(const vector<BoxFromRectangles>& boxes, vector<UtilPlane>* planes)
{
	planes->clear();
	int numBoxes = static_cast<int>(boxes.size());
	for (int i = 0; i < numBoxes; ++i)
	{
		const Eigen::Vector3f& pt = boxes[i].GetCenter();
		for (int j = 0; j < 3; ++j)
		{
			Eigen::Vector3f n = mUp.cross(boxes[i].GetAxes()[j]);
			if (n.norm() < EPSILON_FLOAT) continue;
			n.normalize();
			planes->push_back(UtilPlane(n, pt));
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
	correspondence->resize(numBoxes);
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
	vector<pair<int, int> > correspondence = GetSymmetryCorrespondences();
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