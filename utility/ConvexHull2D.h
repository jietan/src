#ifndef CONVEX_HULL_2D_H
#define CONVEX_HULL_2D_H

#include "mathlib.h"
#include <vector>
using namespace std;

#define N 100000

class ConvexHull2D //assuming the y component is ignored
{
public:
	static vector<int> GetConvexHull(const vector<vector3>& pts);
private:
	static double* P[N+1]; /* an extra position is used */
	static double points[N][2];
};

#endif