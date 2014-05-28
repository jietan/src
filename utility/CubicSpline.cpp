#include "stdafx.h"
#include "CubicSpline.h"

CubicSpline::CubicSpline() :     
mStart(0),
mEnd(0),
mStartTangent(0),
mEndTangent(0),
mW1(0),
mW2(0),
mW3(0),
mW4(0)
{

}

void CubicSpline::SetControlPoints(double start, double end, double startTangent, double endTangent)
{
    mStart = start;
    mEnd = end;
    mStartTangent = startTangent;
    mEndTangent = endTangent;
    calculateWeights();
}
double CubicSpline::GetPoint(double t) const
{
    double ret = mW1 * t * t * t + mW2 * t * t + mW3 * t + mW4;
    return ret;
}

void CubicSpline::calculateWeights()
{
    mW1 = -2 * mEnd + mEndTangent + mStartTangent + 2 * mStart;
    mW2 = 3 * mEnd - mEndTangent - 2 * mStartTangent - 3 * mStart;
    mW3 = mStartTangent;
    mW4 = mStart;
}