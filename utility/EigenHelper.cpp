#include "stdafx.h"
#include "EigenHelper.h"

bool EigenVectorCompare(const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
    CHECK(a.size() == b.size()) << "Cannot compare two vectors with different sizes.";
    int dim = a.size();
    for (int i = 0; i < dim; ++i)
    {
        if (a[i] < b[i])
            return true;
        else if (a[i] > b[i])
            return false;
    }
    return false;
}

void EigenVectorBoundingBox(const vector<Eigen::VectorXd>& pts, Eigen::VectorXd& minPt, Eigen::VectorXd& maxPt)
{
    int numPoints = static_cast<int>(pts.size());
    if (!numPoints) return;
    minPt = maxPt = pts[0];
    int ptDim = minPt.size();
    for (int i = 1; i < numPoints; ++i)
    {
        for (int j = 0; j < ptDim; ++j)
        {
            if (pts[i][j] < minPt[j])
            {
                minPt[j] = pts[i][j];
            }
            else if (pts[i][j] > maxPt[j])
            {
                maxPt[j] = pts[i][j];
            }
        }
    }
}

bool operator >= (const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
    CHECK(a.size() == b.size()) << "Cannot compare two vectors with different sizes.";
    int dim = a.size();
    for (int i = 0; i < dim; ++i)
    {
        if (a[i] < b[i])
            return false;
    }
    return true;
}
bool operator <= (const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
    CHECK(a.size() == b.size()) << "Cannot compare two vectors with different sizes.";
    int dim = a.size();
    for (int i = 0; i < dim; ++i)
    {
        if (a[i] > b[i])
            return false;
    }
    return true;
}