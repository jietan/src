#include "stdafx.h"
#include "LinearConstraint.h"
#include "mathlib.h"

LinearConstraint::LinearConstraint()
{

}
LinearConstraint::~LinearConstraint()
{

}
void LinearConstraint::SetSize(int numCons, int numDofs)
{
	mA = Eigen::MatrixXd::Zero(numCons, numDofs);
	mlb = Eigen::VectorXd::Zero(numCons);
	mub = Eigen::VectorXd::Zero(numCons);
	mIsConstraintsLowerBounded = Eigen::VectorXi::Zero(numCons);
	mIsConstraintsUpperBounded = Eigen::VectorXi::Zero(numCons);
}

void LinearConstraint::Set(const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXi& bLowerBounded, const Eigen::VectorXd& ub, const Eigen::VectorXi& bUpperBounded)
{
    mA = A;
    mlb = lb;
    mIsConstraintsLowerBounded = bLowerBounded;
    mub = ub;
    mIsConstraintsUpperBounded = bUpperBounded;
}


const Eigen::MatrixXd& LinearConstraint::GetLhs() const
{
	return mA;
}
const Eigen::VectorXd& LinearConstraint::GetLowerBounds() const
{
	return mlb;
}
const Eigen::VectorXi& LinearConstraint::GetIsLowerBounded() const
{
	return mIsConstraintsLowerBounded;
}
const Eigen::VectorXd& LinearConstraint::GetUpperBounds() const
{
	return mub;
}
const Eigen::VectorXi& LinearConstraint::GetIsUpperBounded() const
{
	return mIsConstraintsUpperBounded;
}

void LinearConstraint::Union(LinearConstraint* rhs)
{
    Union(*rhs);
}

void LinearConstraint::Union(const LinearConstraint& rhs)
{
    if (GetNumConstraints() == 0)
    {
        *this = rhs;
        return;
    }
    else if (rhs.GetNumConstraints() == 0)
    {
        return;
    }

	CHECK(mA.cols() == rhs.mA.cols());
	int numRows = mA.rows() + rhs.mA.rows();
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(numRows, mA.cols());
	Eigen::VectorXd lb = Eigen::VectorXd::Zero(numRows);
	Eigen::VectorXd ub = Eigen::VectorXd::Zero(numRows);
	Eigen::VectorXi isConstraintsLowerBounded = Eigen::VectorXi::Zero(numRows);
	Eigen::VectorXi isConstraintsUpperBounded = Eigen::VectorXi::Zero(numRows);

	A.block(0, 0, mA.rows(), mA.cols()) = mA;
	A.block(mA.rows(), 0, rhs.mA.rows(), rhs.mA.cols()) = rhs.mA;
	lb.head(mlb.size()) = mlb;
	lb.tail(rhs.mlb.size()) = rhs.mlb;
	ub.head(mub.size()) = mub;
	ub.tail(rhs.mub.size()) = rhs.mub;
	isConstraintsLowerBounded.head(mIsConstraintsLowerBounded.size()) = mIsConstraintsLowerBounded;
	isConstraintsLowerBounded.tail(rhs.mIsConstraintsLowerBounded.size()) = rhs.mIsConstraintsLowerBounded;
	isConstraintsUpperBounded.head(mIsConstraintsUpperBounded.size()) = mIsConstraintsUpperBounded;
	isConstraintsUpperBounded.tail(rhs.mIsConstraintsUpperBounded.size()) = rhs.mIsConstraintsUpperBounded;

	mA = A; 
	mlb = lb;
	mub = ub;
	mIsConstraintsLowerBounded = isConstraintsLowerBounded;
	mIsConstraintsUpperBounded = isConstraintsUpperBounded;
}

int LinearConstraint::GetNumConstraints() const
{
    return mA.rows();
}
int LinearConstraint::GetNumVariables() const
{
    return mA.cols();
}

bool LinearConstraint::CheckValidation(const Eigen::VectorXd& sol)
{
    bool ret = true;
    mIsConstraintSatisfied.clear();
    Eigen::VectorXd reconstructedb = mA * sol;
    int numConstraints = mA.rows();
    for (int i = 0; i < numConstraints; ++i)
    {
        if (mIsConstraintsLowerBounded[i] && mIsConstraintsUpperBounded[i])
        {
            if (mlb[i] == mub[i])
            {
                if (NearlyEqual(reconstructedb[i], mlb[i], 1e-4))
                    mIsConstraintSatisfied.push_back(ConstraintValidationT(i, CST_Equal));
                else
                {
                    mIsConstraintSatisfied.push_back(ConstraintValidationT(i, CST_Invalid));
                    LOG(WARNING) << "Equality constraint " << i << " not satisfied: " << reconstructedb[i] << ", " << mlb[i];
                    ret = false;
                }
            }
            else
            {
                if (NearlyEqual(reconstructedb[i], mlb[i], 1e-4))
                    mIsConstraintSatisfied.push_back(ConstraintValidationT(i, CST_LowerBounded));
                else if (NearlyEqual(reconstructedb[i], mub[i]))
                    mIsConstraintSatisfied.push_back(ConstraintValidationT(i, CST_UpperBounded));
                else if (reconstructedb[i] <= mub[i] && reconstructedb[i] >= mlb[i])
                    mIsConstraintSatisfied.push_back(ConstraintValidationT(i, CST_InBetween));
                else
                {
                    mIsConstraintSatisfied.push_back(ConstraintValidationT(i, CST_Invalid));
                    LOG(WARNING) << "Inbetween constraint " << i << " not satisfied: " << reconstructedb[i] << ", " << mlb[i] << ", " << mub[i];
                    ret = false;
                }
            }
        }
        else if (mIsConstraintsLowerBounded[i] && !mIsConstraintsUpperBounded[i])
        {
            if (NearlyEqual(reconstructedb[i], mlb[i], 1e-4))
                mIsConstraintSatisfied.push_back(ConstraintValidationT(i, CST_LowerBounded));
            else if (reconstructedb[i] >= mlb[i])
                mIsConstraintSatisfied.push_back(ConstraintValidationT(i, CST_InBetween));
            else
            {
                mIsConstraintSatisfied.push_back(ConstraintValidationT(i, CST_Invalid));
                LOG(WARNING) << "Greater than constraint " << i << " not satisfied: " << reconstructedb[i] << ", " << mlb[i];
                ret = false;
            }
        }
        else if (!mIsConstraintsLowerBounded[i] && mIsConstraintsUpperBounded[i])
        {
            if (NearlyEqual(reconstructedb[i], mub[i], 1e-4))
                mIsConstraintSatisfied.push_back(ConstraintValidationT(i, CST_UpperBounded));
            else if (reconstructedb[i] <= mub[i])
                mIsConstraintSatisfied.push_back(ConstraintValidationT(i, CST_InBetween));
            else
            {
                mIsConstraintSatisfied.push_back(ConstraintValidationT(i, CST_Invalid));
                LOG(WARNING) << "Less than constraint " << i << " not satisfied: " << reconstructedb[i] << ", " << mub[i];
                ret = false;
            }
        }
        else
        {
            LOG(WARNING) << i << "th linear constraint should not be free";
        }
    }
    //LOG(INFO) << "--------------Constraint satisfaction information--------------------";
    //for (int i = 0; i < numConstraints; ++i)
    //{
    //    LOG(INFO) << i << ": " << mIsConstraintSatisfied[i].second;
    //}
    return ret;
}

const vector<ConstraintValidationT>& LinearConstraint::GetConstraintSatisfactionType() const
{
    CHECK(!mIsConstraintSatisfied.empty());
    return mIsConstraintSatisfied;
}