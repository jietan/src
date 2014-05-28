#ifndef _PRIORIZIZED_QP_SOLVER_H
#define _PRIORIZIZED_QP_SOLVER_H

#include "stdafx.h"
#include "QPSolver.h"

class PrioritizedQPSolver : public QPSolver
{
public:
    virtual ~PrioritizedQPSolver();
    virtual bool Solve(VectorXd& sol);

private:
    bool preSolve();
    bool solve();
    bool postSolve();
    bool getNullSpace(const MatrixXd& A, MatrixXd& nullSpace);
    void initializeObjectiveList();
    QPObjective* combineObjectives(const vector<QPObjective*>& objectivesToCombine);
    bool reparameterization();
    void reassemble(VectorXd& sol);

    MatrixXd mCBar;
    VectorXd mdBar;
    VectorXd mSol;
    vector<QPObjective*> mObjectives;
    vector<int> mNeedDeleteIdx;

    MatrixXd mNullSpace;
    VectorXd mMinimizer;

    MatrixXd mAEquality;
    VectorXd mbEquality;

    MatrixXd mAInEquality;
    VectorXd mlbcInEquality;
    VectorXd mubcInEquality;
    VectorXi mbGreaterThan;
    VectorXi mbLessThan;
};

#endif