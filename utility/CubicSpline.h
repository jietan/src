#ifndef _CUBIC_SPLINE_H
#define _CUBIC_SPLINE_H

#include "stdafx.h"

class CubicSpline
{
public:
    CubicSpline();
    void SetControlPoints(double start, double end, double startTangent, double endTangent);
    double GetPoint(double t) const;

private:
    void calculateWeights();

    double mStart;
    double mEnd;
    double mStartTangent;
    double mEndTangent;
    double mW1;
    double mW2;
    double mW3;
    double mW4;
};

#endif