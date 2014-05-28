#ifndef CONIC_SOLVER
#define CONIC_SOLVER

#include "stdafx.h"
#include "LinearConstraint.h"
#include "Cone.h"
#include <Eigen/Dense>
#include <vector>
using namespace Eigen;
using namespace std;

#ifdef _WIN32
#include <mosek.h>
#endif


class ConicSolver
{
public:
    ConicSolver();
    virtual ~ConicSolver();
    
    virtual void SetConstraints(const LinearConstraint& constraint);
    virtual void SetLinearTerm(const VectorXd& c);
    virtual void SetConstraints(const MatrixXd& A, const VectorXd& lb, const VectorXi& bLowerBounded, const VectorXd& ub, const VectorXi& bUpperBounded);
    virtual void SetLowerBounds(const VectorXd& lb, const VectorXi& bBounded);
    virtual void SetUpperBounds(const VectorXd& ub, const VectorXi &bBounded);
    virtual bool Solve(VectorXd& sol);
    virtual void AddCone(const Cone& cone);
protected:
    void convertMatrixVectorFormat();
    void convertAFormat();

    VectorXd mc;

    VectorXd mlb;
    VectorXd mub;
    VectorXi mbLowerBounded;
    VectorXi mbUpperBounded;

    MatrixXd mA;
    VectorXd mlbc;
    VectorXd mubc;
    VectorXi mbConstraintLowerBounded;
    VectorXi mbConstraintUpperBounded;

    int mNumCon;
    int mNumVar;
    int mNumCone;

    vector<Cone> mCones;
    vector<int> mAColumnStartIdx;
    vector<int> mARowIdx;
    vector<double> mAValues;

    double mConstant;

};
#endif
