#ifndef _QPPROBLEM_FROM_LCCQP_H
#define _QPPROBLEM_FROM_LCCQP_H

#include "QPSolver.h"

class LinearComplementarityConstraint;
class LinearConstraint;
class QPObjective;

class QPProblemFromLCCQP
{
public:
    QPProblemFromLCCQP();
    ~QPProblemFromLCCQP();
    void SetLinearConstraint(LinearConstraint* constraint);
    void SetLinearComplementarityConstraint(LinearComplementarityConstraint* constraint);
    void SetObjective(QPObjective* objective);
    void SetIsEqualityOnZ(const vector<int>& isEqualityOnZ);
    const vector<int>& GetInequalityConstraintIdOnBoundary() const;
    double GetMinValue() const;
    bool GetMinimizer(VectorXd& sol) const;
    const vector<int>& GetIsEqualityOnZ() const;
    bool Solve(VectorXd& sol);
    long long GetId() const;
private:
    LinearComplementarityConstraint* mLCConstraint;
    LinearConstraint* mLinearConstraint;
    QPObjective* mObjective;
    QPSolver mSolver;
    vector<int> mIsEqualityOnZ;
    vector<int> mInequalityOnBoundIdx;
    VectorXd mSol;
    VectorXd mGradient;
    double mMinValue;
    
    bool mIsMinimizationSucceed;
    long long mId;

    void calculateId();
};

#endif