#include "stdafx.h"
#include "LCCQPGraph.h"
#include "QuadraticObjective.h"
#include "LinearConstraint.h"
#include "LinearComplementarityConstraint.h"
#include "LCCQPGraphNode.h"
#include "QPProblemFromLCCQP.h"
//#include "controller/GroundContactLCConstraint.h"

LCCQPGraph::LCCQPGraph() : mCurrentNode(NULL), mObjective(NULL), mLinearConstraint(NULL), mLCCConstraint(NULL), mMaxNumIterations(10)
{

}
LCCQPGraph::~LCCQPGraph()
{
    int numNodes = GetNumNodes();
    for (int i = 0; i < numNodes; ++i)
    {
        delete mNodes[i]->mProblem;
        delete mNodes[i];
    }
    mNodes.clear();
}

void LCCQPGraph::SetObjective(QPObjective* objective)
{
    mObjective = objective;
}
void LCCQPGraph::SetLinearComplementarityConstraints(LinearComplementarityConstraint* constraint)
{
    mLCCConstraint = constraint;
}
void LCCQPGraph::SetLinearConstraints(LinearConstraint* constraint)
{
    mLinearConstraint = constraint;
}

void LCCQPGraph::SetMaxNumIterations(int num)
{
    mMaxNumIterations = num;
}

void LCCQPGraph::Expand(const vector<int>& initialGuess, set<long long>& exploredIdx)
{    
 //   int numIterations = 0;

 //   generateInitialNode(initialGuess, exploredIdx);

 //   while (mExploreQueue.size() != 0 && numIterations < mMaxNumIterations)
 //   {
 //       LCCQPGraphNode* nodeToExplore = mExploreQueue.top();
 //       mExploreQueue.pop();
 //       vector<int> inequalityOnBound = nodeToExplore->GetInequalityConstraintIdOnBoundary();
 //       const vector<int>& isEqualityOnZ = nodeToExplore->GetIsEqualityOnZ();
 //       VectorXd nodeToExploreSol;
 //       nodeToExplore->GetMinimizer(nodeToExploreSol);

 //       while(true)
 //       {
 //           GroundContactLCConstraint* groundLCCConstraint = dynamic_cast<GroundContactLCConstraint*>(mLCCConstraint);
 //           CHECK(groundLCCConstraint);
 //           vector<int> newIsEqualityOnZ;
 //           bool bPivotSucceed = groundLCCConstraint->PivotVariables(isEqualityOnZ, nodeToExploreSol, inequalityOnBound, newIsEqualityOnZ);
 //           if (!bPivotSucceed)
 //               break;

 //           QPProblemFromLCCQP* problem = new QPProblemFromLCCQP;
 //           problem->SetObjective(mObjective);
 //           problem->SetLinearComplementarityConstraint(mLCCConstraint);
 //           problem->SetLinearConstraint(mLinearConstraint);
 //           problem->SetIsEqualityOnZ(newIsEqualityOnZ);
 //           long long id = problem->GetId();
 //           if (exploredIdx.find(id) != exploredIdx.end())
 //           {
 //               delete problem;
 //           }
 //           else
 //           {
 //               LCCQPGraphNode* childNode = new LCCQPGraphNode(this);
 //               childNode->mParents.push_back(nodeToExplore);
 //               nodeToExplore->mChildren.push_back(childNode);
 //               childNode->mProblem = problem;

 //               mNodes.push_back(childNode);
 //               exploredIdx.insert(id);

 //               bool bSovleSucceeed = childNode->Evaluate();
 //               if (bSovleSucceeed)
 //                   mExploreQueue.push(childNode);
 //           }
 //       }
 //       numIterations++;
 //   }
	//LOG(INFO) << numIterations << " iterations are spent to expand the nodes.";
}

LCCQPGraphNode* LCCQPGraph::Next()
{
    return NULL;
}

int LCCQPGraph::GetNumNodes() const
{
    return static_cast<int>(mNodes.size());
}

void LCCQPGraph::generateInitialNode(const vector<int>& initialGuess, set<long long>& exploredIdx)
{
    CHECK(mObjective);
    CHECK(mLCCConstraint);
    CHECK(mLinearConstraint);

    LCCQPGraphNode* node = new LCCQPGraphNode(this);
    QPProblemFromLCCQP* problem = new QPProblemFromLCCQP;
    problem->SetObjective(mObjective);
    problem->SetLinearComplementarityConstraint(mLCCConstraint);
    problem->SetLinearConstraint(mLinearConstraint);

    if (initialGuess.empty())
    {
        LOG(WARNING) << "Initial Guess for LCCQP is not provided.";   
    }
    else
    {
        problem->SetIsEqualityOnZ(initialGuess);
    }

    mNodes.push_back(node);

    node->mProblem = problem;
    bool bSovleSucceeed = node->Evaluate();
    exploredIdx.insert(node->GetId());
    if (bSovleSucceeed)
        mExploreQueue.push(node);
}

LCCQPGraphNode* LCCQPGraph::GetBestNode() const
{
    int numNodes = GetNumNodes();
    double minValue = 1e30;
    LCCQPGraphNode* bestNode = NULL;
    for (int i = 0; i < numNodes; ++i)
    {
        LCCQPGraphNode* node = mNodes[i];
        if (node->GetValue() < minValue)
        {
            minValue = node->GetValue();
            bestNode = node;
        }
    }
    return bestNode;
}

int LCCQPGraph::GetNumNodesInGraph() const
{
    return static_cast<int>(mNodes.size());
}