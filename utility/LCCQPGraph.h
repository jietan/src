
#ifndef _LCC_QP_GRAPH_H
#define _LCC_QP_GRAPH_H
#include <queue>

#include "stdafx.h"
#include "LCCQPGraphNode.h"

class QPProblemFromLCCQP;
class LinearComplementarityConstraint;
class LinearConstraint;
class QPObjective;

class LCCQPGraph
{
public:
    LCCQPGraph();
    ~LCCQPGraph();
    void SetObjective(QPObjective* objective);
    void SetLinearComplementarityConstraints(LinearComplementarityConstraint* constraint);
    void SetLinearConstraints(LinearConstraint* constraint);
    void Expand(const vector<int>& initialGuess, set<long long>& exploredIdx);
    LCCQPGraphNode* Next();
    int GetNumNodes() const;
    LCCQPGraphNode* GetBestNode() const;
    int GetNumNodesInGraph() const;
    void SetMaxNumIterations(int num);
private:
    vector<LCCQPGraphNode*> mNodes;
    int mMaxNumIterations;
    //set<long long> mExploredIdx;
    priority_queue<LCCQPGraphNode*, vector<LCCQPGraphNode*>, CompareNode> mExploreQueue;
    LCCQPGraphNode* mCurrentNode;
    QPObjective* mObjective;
    LinearComplementarityConstraint* mLCCConstraint;
    LinearConstraint* mLinearConstraint;
    void generateInitialNode(const vector<int>& initialGuess, set<long long>& exploredIdx);
};

#endif
