#include "stdafx.h"
#include "LCCQPGraphNode.h"
#include "QPProblemFromLCCQP.h"

LCCQPGraphNode::LCCQPGraphNode(LCCQPGraph* graph) : mGraph(graph), mbVisited(false)
{

}
LCCQPGraphNode::~LCCQPGraphNode()
{

}

bool LCCQPGraphNode::Evaluate()
{
    VectorXd sol;

    return mProblem->Solve(sol);
}

double LCCQPGraphNode::GetValue() const
{
    return mProblem->GetMinValue();
}

long long LCCQPGraphNode::GetId() const
{
    return mProblem->GetId();
}

bool LCCQPGraphNode::IsEvaluationSucceed() const
{
    VectorXd sol;
    return mProblem->GetMinimizer(sol);
}
void LCCQPGraphNode::GetMinimizer(VectorXd& sol) const
{
    mProblem->GetMinimizer(sol);
}
void LCCQPGraphNode::SetIsEqualityOnZ(const vector<int>& isEqualityOnZ)
{
    mProblem->SetIsEqualityOnZ(isEqualityOnZ);
}
const vector<int>& LCCQPGraphNode::GetIsEqualityOnZ() const
{
    return mProblem->GetIsEqualityOnZ();
}
const vector<int>& LCCQPGraphNode::GetInequalityConstraintIdOnBoundary() const
{
    return mProblem->GetInequalityConstraintIdOnBoundary();
}