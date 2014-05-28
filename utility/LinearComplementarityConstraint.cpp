#include "stdafx.h"
#include "LinearComplementarityConstraint.h"
#include "LinearConstraint.h"

LinearComplementarityConstraint::LinearComplementarityConstraint() : mVarOffset(0), mVarSize(0)
{

}
LinearComplementarityConstraint::~LinearComplementarityConstraint()
{

}

LinearConstraint LinearComplementarityConstraint::GetLinearConstraint(const vector<int>& isEqualityOnZ)
{
    LinearConstraint ret;
    ret.SetSize(2 * mA.rows(), mVarOffset + mVarSize);
    for (int i = 0; i < mVarSize; ++i) //equality constraints
    {
        if (isEqualityOnZ[i])
        {
            ret.mA(i, mVarOffset + i) = 1.0;
            ret.mIsConstraintsLowerBounded[i] = ret.mIsConstraintsUpperBounded[i] = 1;
            ret.mlb[i] = ret.mub[i] = 0;
        }
        else
        {
            ret.mA.row(i) = mA.row(i);
            ret.mIsConstraintsLowerBounded[i] = ret.mIsConstraintsUpperBounded[i] = 1;
            ret.mlb[i] = ret.mub[i] = -mb[i];
        }
    }
    for (int i = 0; i < mVarSize; ++i) //inequality constraints
    {
        if (isEqualityOnZ[i])
        {
            ret.mA.row(mVarSize + i) = mA.row(i);
            ret.mIsConstraintsLowerBounded[mVarSize + i] = 1;
            ret.mIsConstraintsUpperBounded[mVarSize + i] = 0;
            ret.mlb[mVarSize + i] = -mb[i];
        }
        else
        {
            ret.mA(mVarSize + i, mVarOffset + i) = 1.0;
            ret.mIsConstraintsLowerBounded[mVarSize + i] = 1;
            ret.mIsConstraintsUpperBounded[mVarSize + i] = 0;
            ret.mlb[mVarSize + i] = 0;
        }
    }
    return ret;
}
void LinearComplementarityConstraint::SetA(const MatrixXd& A)
{
    mA = A;
}
void LinearComplementarityConstraint::SetB(const MatrixXd& b)
{
    mb = b;
}
void LinearComplementarityConstraint::SetVariableInfo(int varOffset, int varSize)
{
    mVarOffset = varOffset;
    mVarSize = varSize;
    
}

int LinearComplementarityConstraint::GetNumConstraints() const
{
    return mA.rows();
}
int LinearComplementarityConstraint::GetNumVariables() const
{
    return mVarSize;
}
