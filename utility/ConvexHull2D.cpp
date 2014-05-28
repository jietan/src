#include "stdafx.h"
#include "ConvexHull2D.h"
#include "2dch.h"

double* ConvexHull2D::P[N+1];
double  ConvexHull2D::points[N][2];

vector<int> ConvexHull2D::GetConvexHull(const vector<vector3>& pts)
{
	vector<int> ret;
	int numPoints = static_cast<int>(pts.size());
	for (int i = 0; i < numPoints; ++i)
	{
		points[i][0] = pts[i].x;
		points[i][1] = pts[i].z;
		P[i] = points[i];
	}
	int u = ch2d(P, numPoints);
	for (int i = 0; i < u; ++i) 
		ret.push_back((P[i]-points[0])/2);
	return ret;
}
