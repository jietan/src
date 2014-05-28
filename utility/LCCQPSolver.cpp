#include "stdafx.h"
#include "LCCQPSolver.h"

LCCQPSolver::LCCQPSolver() : mbRandomRestart(true), mMaxNumRestart(1), mBestNode(NULL), mMaxNumIterations(10)
{

}
LCCQPSolver::~LCCQPSolver()
{

}
void LCCQPSolver::SetObjective(QPObjective* obj)
{
    mObjective = obj;
}
void LCCQPSolver::SetLinearConstraints(LinearConstraint* constraint)
{
    mLinearConstraint = constraint;
}

void LCCQPSolver::SetLinearComplementarityConstraint(LinearComplementarityConstraint* constraint)
{
    mLCConstraint = constraint;
}

void LCCQPSolver::SetInitialGuess(const vector<int>& initialGuess)
{
    mInitialGuess.push_back(initialGuess);
}

void LCCQPSolver::SetMaxNumIterations(int maxIterations)
{
    mMaxNumIterations = maxIterations;
}



bool LCCQPSolver::Solve(VectorXd& sol)
{
    mBestNode = NULL;
	mMaxNumRestart = static_cast<int>(mInitialGuess.size());
    if (mbRandomRestart)
        mLCCQPGraphs.resize(mMaxNumRestart);

    if (mMaxNumRestart > 1)
        mMaxNumRestart--; // to avoid use initialGuess from LCP


	set<long long> exploredId;
    
    for (int ithRestart = 0; ithRestart < mMaxNumRestart; ++ithRestart)
    {
        mLCCQPGraphs[ithRestart].SetMaxNumIterations(mMaxNumIterations);
        mLCCQPGraphs[ithRestart].SetObjective(mObjective);
        mLCCQPGraphs[ithRestart].SetLinearConstraints(mLinearConstraint);
        mLCCQPGraphs[ithRestart].SetLinearComplementarityConstraints(mLCConstraint);
        mLCCQPGraphs[ithRestart].Expand(mInitialGuess[ithRestart], exploredId);
        LCCQPGraphNode* node = mLCCQPGraphs[ithRestart].GetBestNode();

        if (node && mBestNode)
        {
            if (node->GetValue() < mBestNode->GetValue())
            {
                mBestNode = node;
            }
        }
        else if (!mBestNode)
        {
            mBestNode = node;
        }
		if (mBestNode)
        {
			LOG(INFO) << ithRestart << "th restart best obj function value: " << mBestNode->GetValue();
        }
        else
        {
            if (ithRestart == mMaxNumRestart - 1)
            {
                mMaxNumRestart = static_cast<int>(mInitialGuess.size());
                LOG(INFO) << "The initial guess is: ";
                for (int i = 0; i < static_cast<int>(mInitialGuess[mMaxNumRestart - 1].size()); ++i)
                {
                    LOG(INFO) << mInitialGuess[mMaxNumRestart - 1][i];
                }
            }
        }
    }

    if (!mBestNode)
        return false;
    else
    {
        mBestNode->GetMinimizer(sol);
		return true;
    }
}

int LCCQPSolver::GetNumQPEvaluations() const
{
    if (mLCCQPGraphs.empty())
        return 0;
    int ret = 0;
    for (int i = 0; i < mMaxNumRestart; ++i)
    {
        ret += mLCCQPGraphs[i].GetNumNodesInGraph();
    }
    return ret;
}

vector<int> LCCQPSolver::GetIsEqualityOnZ() const
{
    return mBestNode->GetIsEqualityOnZ();
}

