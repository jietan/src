#include "stdafx.h"
#include "PiecewiseCubicSpline.h"
#include "utility/mathlib.h"



ControlPoint operator* (double alpha, const ControlPoint& rhs)
{
    ControlPoint ret = rhs;
    ret.t *= alpha;
    ret.p *= alpha;
    ret.v *= alpha;
    return ret;
}

bool ControlPointCompare(const ControlPoint& a, const ControlPoint& b)
{
    if (a.t < b.t)
        return true;
    else
        return false;
}

PiecewiseCubicSpline::PiecewiseCubicSpline() : 
mMinTime(0),
mMaxTime(0),
mbPeriod(false)
{

}
PiecewiseCubicSpline::~PiecewiseCubicSpline()
{

}
void PiecewiseCubicSpline::SetControlPoints(const vector<ControlPoint>& pts)
{
    mCtrlPoints = pts;
    if (mCtrlPoints.empty())
        return;
    sort(mCtrlPoints.begin(), mCtrlPoints.end(), ControlPointCompare);
    mMinTime = mCtrlPoints[0].t;
    mMaxTime = mCtrlPoints[static_cast<int>(mCtrlPoints.size()) - 1].t;
    calculateSplines();
}
void PiecewiseCubicSpline::AddControlPoint(const ControlPoint& pt)
{
    mCtrlPoints.push_back(pt);
    SetControlPoints(mCtrlPoints);
}
double PiecewiseCubicSpline::GetPoint(double t) const
{
    double duration = mMaxTime - mMinTime;
    if (mbPeriod)
    {
        while (t - mMinTime > duration)
        {
            t -= duration;
        }
    }
    else
    {
        t = Clamp(t, mMinTime, mMaxTime);
    }

    int numCtrlPoints = static_cast<int>(mCtrlPoints.size());

    for (int i = 0; i < numCtrlPoints - 1; ++i)
    {
        if (t >= mCtrlPoints[i].t && t <= mCtrlPoints[i + 1].t)
        {
            double scaledShiftedT = t - mCtrlPoints[i].t;
            scaledShiftedT /= (mCtrlPoints[i + 1].t - mCtrlPoints[i].t);
            return mSplines[i].GetPoint(scaledShiftedT);
        }
    }
    LOG(FATAL) << "Error Occured in PiecewiseSpline: Debugging!!!";
    return 0;
}
void PiecewiseCubicSpline::EnablePeriodicity(bool bEnable)
{
    mbPeriod = bEnable;
}

void PiecewiseCubicSpline::calculateSplines()
{
    mSplines.clear();
    int numCtrlPoints = static_cast<int>(mCtrlPoints.size());
    for (int i = 0; i < numCtrlPoints - 1; ++i)
    {
        CubicSpline spline;
        spline.SetControlPoints(mCtrlPoints[i].p, mCtrlPoints[i + 1].p, mCtrlPoints[i].v, mCtrlPoints[i + 1].v);
        mSplines.push_back(spline);
    }
}
void PiecewiseCubicSpline::Sample(int numSamples, vector<double>& t, vector<double>& h) const
{
    double duration = mMaxTime - mMinTime;
    double step = duration / numSamples;
    t.resize(numSamples);
    h.resize(numSamples);
    for (int i = 0; i < numSamples; ++i)
    {
        double currentTime = mMinTime + i * step;
        t[i] = currentTime;
        h[i] = GetPoint(currentTime);
    }
}

void PiecewiseCubicSpline::Sample(int numSamples, Eigen::VectorXd& t, Eigen::VectorXd& h) const
{
    vector<double> vecT, vecH;
    Sample(numSamples, vecT, vecH);
    t = Eigen::VectorXd::Zero(numSamples);
    h = Eigen::VectorXd::Zero(numSamples);
    for (int i = 0; i < numSamples; ++i)
    {
        t[i] = vecT[i];
        h[i] = vecH[i];
    }
}

const vector<ControlPoint>& PiecewiseCubicSpline::GetControlPoints() const
{
    return mCtrlPoints;
}