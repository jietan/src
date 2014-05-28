#ifndef _LCC_QP_GRAPH_NODE_H
#define _LCC_QP_GRAPH_NODE_H

#include "stdafx.h"

class QPProblemFromLCCQP;
class LCCQPGraph;


class LCCQPGraphNode
{
    friend class LCCQPGraph;
public:
    LCCQPGraphNode(LCCQPGraph* graph);
    ~LCCQPGraphNode();
    double GetValue() const;
    bool Evaluate();
    long long GetId() const;
    bool IsEvaluationSucceed() const;
    void GetMinimizer(VectorXd& sol) const;
    void SetIsEqualityOnZ(const vector<int>& isEqualityOnZ);
    const vector<int>& GetIsEqualityOnZ() const;
    const vector<int>& GetInequalityConstraintIdOnBoundary() const;
    

    vector<LCCQPGraphNode*> mParents;
    vector<LCCQPGraphNode*> mChildren;
private:
    QPProblemFromLCCQP* mProblem;
    bool mbVisited;
    LCCQPGraph* mGraph;
};

struct CompareNode : public std::binary_function<LCCQPGraphNode*, LCCQPGraphNode*, bool>                                                                                     
{
    bool operator()(const LCCQPGraphNode* lhs, const LCCQPGraphNode* rhs) const
    {
        return lhs->GetValue() > rhs->GetValue();
    }
};

#endif
