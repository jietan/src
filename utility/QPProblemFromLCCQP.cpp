#include "stdafx.h"
#include "QPProblemFromLCCQP.h"
#include "LinearComplementarityConstraint.h"
#include "LinearConstraint.h"
#include "QuadraticObjective.h"
#include "utility/EigenSerializer.h"

QPProblemFromLCCQP::QPProblemFromLCCQP() : mObjective(NULL), mLinearConstraint(NULL), mLCConstraint(NULL), mMinValue(1e30), mId(-1)
{
    
}
QPProblemFromLCCQP::~QPProblemFromLCCQP()
{

}
void QPProblemFromLCCQP::SetLinearConstraint(LinearConstraint* constraint)
{
    mLinearConstraint = constraint;
}
void QPProblemFromLCCQP::SetLinearComplementarityConstraint(LinearComplementarityConstraint* constraint)
{
    mLCConstraint = constraint;
    if (mLCConstraint)
    {
        int numLCCConstraint = mLCConstraint->GetNumConstraints();
        for (int i = 0; i < numLCCConstraint; ++i)
        {
            mIsEqualityOnZ.push_back(0);
        }
    }
}
void QPProblemFromLCCQP::SetObjective(QPObjective* objective)
{
    mObjective = objective;
}
void QPProblemFromLCCQP::SetIsEqualityOnZ(const vector<int>& isEqualityOnZ)
{
    mIsEqualityOnZ = isEqualityOnZ;
    calculateId();
}

const vector<int>& QPProblemFromLCCQP::GetIsEqualityOnZ() const
{
    return mIsEqualityOnZ;
}

double QPProblemFromLCCQP::GetMinValue() const
{
    return mMinValue;
}

bool QPProblemFromLCCQP::GetMinimizer(VectorXd& sol) const
{
    sol = mSol;
    return mIsMinimizationSucceed;
}

const vector<int>& QPProblemFromLCCQP::GetInequalityConstraintIdOnBoundary() const
{
    return mInequalityOnBoundIdx;
}

bool QPProblemFromLCCQP::Solve(VectorXd& sol)
{
    LinearConstraint ultimateConstraint = mLCConstraint->GetLinearConstraint(mIsEqualityOnZ);
   // LinearConstraint ultimateConstraint;
    QPSolver solver;
    solver.SetObjective(*mObjective);
    ultimateConstraint.Union(*mLinearConstraint);
    //ultimateConstraint = *mLinearConstraint;
    //EigenSerializer::SaveToFile("Linear.txt", ultimateConstraint.GetLhs());
    //EigenSerializer::SaveToFile("LCC.txt", ultimateConstraint.GetLhs());
    solver.SetConstraints(ultimateConstraint);
    int numLCConstraints = mLCConstraint->GetNumConstraints();

    mIsMinimizationSucceed = solver.Solve(mSol, SM_Cone);
    mInequalityOnBoundIdx.clear();

    if (mIsMinimizationSucceed)
    {
        mMinValue = mObjective->OutputObjectiveFuncValues(mSol, false);
        sol = mSol;
        bool constraintSatisfaction = ultimateConstraint.CheckValidation(mSol);

        const vector<ConstraintValidationT>& consSatisfactionTypes = ultimateConstraint.GetConstraintSatisfactionType();
        for (int i = 0; i < 2 * numLCConstraints; ++i)
        {
            CHECK(i == consSatisfactionTypes[i].first);
            CHECK(consSatisfactionTypes[i].second != CST_UpperBounded);
            if (consSatisfactionTypes[i].second == CST_LowerBounded)
            {
                mInequalityOnBoundIdx.push_back(i % numLCConstraints);
            }
        }
        mGradient = mObjective->EvaluateGradient(mSol);
    }
    return mIsMinimizationSucceed;
}

long long QPProblemFromLCCQP::GetId() const
{
    return mId;
}

void QPProblemFromLCCQP::calculateId()
{
    int numLCConstraints = mLCConstraint->GetNumConstraints();
    if (numLCConstraints > 63)
        LOG(WARNING) << "LCC has more than 63 constraints, Id could be duplicated.";
    mId = 0;
    for (int i = 0; i < numLCConstraints; ++i)
    {
        long long one  = 1;
        if (mIsEqualityOnZ[i])
            mId += one << i;
    }
}