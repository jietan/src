#ifndef _LINEAR_COMPLEMENTARITY_CONSTRAINT_H
#define _LINEAR_COMPLEMENTARITY_CONSTRAINT_H

#include "stdafx.h"

//q = mA * x + mb
//z = x[mVarOffset, ..., mVarOffset + mVarSize]
//q >= 0, z >= 0, q^Tz = 0

class LinearConstraint;

class LinearComplementarityConstraint
{
public:
    LinearComplementarityConstraint();
    virtual ~LinearComplementarityConstraint();
    
    virtual LinearConstraint GetLinearConstraint(const vector<int>& isEqualityOnZ);
    virtual void SetA(const MatrixXd& A);
    virtual void SetB(const MatrixXd& b);
    virtual void SetVariableInfo(int varOffset, int varSize);
    virtual int GetNumConstraints() const;
    virtual int GetNumVariables() const;
    
protected:
    int mVarOffset;
    int mVarSize;
    MatrixXd mA;
    VectorXd mb;
};

#endif