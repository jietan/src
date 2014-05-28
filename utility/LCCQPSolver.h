#ifndef _LCC_QP_SOLVER_H
#define _LCC_QP_SOLVER_H

#include "stdafx.h"
#include "QuadraticObjective.h"
#include "LinearComplementarityConstraint.h"
#include "LinearConstraint.h"
#include "LCCQPGraph.h"

class LCCQPSolver
{
public:
    LCCQPSolver();
    virtual ~LCCQPSolver();
    virtual void SetObjective(QPObjective* obj);
    virtual void SetLinearConstraints(LinearConstraint* constraint);
    virtual void SetLinearComplementarityConstraint(LinearComplementarityConstraint* constraint);
    virtual bool Solve(VectorXd& sol);
    virtual void SetInitialGuess(const vector<int>& initialGuess);
    virtual int GetNumQPEvaluations() const;
    virtual vector<int> GetIsEqualityOnZ() const;
    virtual void SetMaxNumIterations(int maxIterations);
private:
    LinearComplementarityConstraint* mLCConstraint;
    LinearConstraint* mLinearConstraint;
    QPObjective* mObjective;
    vector<LCCQPGraph> mLCCQPGraphs;
    vector<vector<int> > mInitialGuess;
    LCCQPGraphNode* mBestNode;
    bool mbRandomRestart;
    int mMaxNumRestart;
    int mMaxNumIterations;
    void restart();
};


#endif