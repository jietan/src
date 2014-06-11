#ifndef _PIECEWISE_CUBIC_SPLINE
#define _PIECEWISE_CUBIC_SPLINE

#include "stdafx.h"
#include "CubicSpline.h"


class ControlPoint
{
public:
    double t;
    double p;
    double v;
    ControlPoint() : t(0), p(0), v(0)
    {

    }
    ControlPoint(double _t, double _p, double _v = 0) : t(_t), p(_p), v(_v)
    {

    }
    ControlPoint& operator+=(const ControlPoint& rhs)
    {
        t += rhs.t;
        p += rhs.p;
        v += rhs.v;
        return *this;
    }
    ControlPoint& operator-=(const ControlPoint& rhs)
    {
        t -= rhs.t;
        p -= rhs.p;
        v -= rhs.v;
        return *this;
    }
    ControlPoint operator+(const ControlPoint& rhs) const
    {
        ControlPoint ret = *this;
        ret += rhs;
        return ret;
    }

    ControlPoint operator- () const
    {
        ControlPoint ret;
        ret.t = -t;
        ret.p = -p;
        ret.v = -v;
        return ret;
    }
    ControlPoint operator- (const ControlPoint& rhs) const
    {
        ControlPoint ret = *this;
        ret -= rhs;
        return ret;
    }

};

ControlPoint operator* (double alpha, const ControlPoint& rhs);
bool ControlPointCompare(const ControlPoint& a, const ControlPoint& b);

class PiecewiseCubicSpline
{
public:
    PiecewiseCubicSpline();
    ~PiecewiseCubicSpline();
    void SetControlPoints(const vector<ControlPoint>& pts);
    void AddControlPoint(const ControlPoint& pt);
    double GetPoint(double t) const;
    void EnablePeriodicity(bool bEnable);
    void Sample(int numSamples, vector<double>& t, vector<double>& h) const;
    void Sample(int numSamples, Eigen::VectorXd& t, Eigen::VectorXd& h) const;
    const vector<ControlPoint>& GetControlPoints() const;
private:
    void calculateSplines();

    double mMinTime;
    double mMaxTime;
    vector<ControlPoint> mCtrlPoints;
    vector<CubicSpline> mSplines;
    bool mbPeriod;

};

#endif