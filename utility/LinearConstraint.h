#ifndef _LINEAR_CONSTRAINT_H
#define _LINEAR_CONSTRAINT_H

#include "stdafx.h"

enum ConstraintSatisficationType
{
    CST_Equal,
    CST_InBetween,
    CST_LowerBounded,
    CST_UpperBounded,
    CST_Invalid
};

typedef pair<int, ConstraintSatisficationType> ConstraintValidationT;

class LinearConstraint
{
    friend class LinearComplementarityConstraint;
	friend class GroundContactConstraint;
	friend class MuscleRangeConstraint;
public:
	LinearConstraint();
	~LinearConstraint();
	void SetSize(int numCons, int numDofs);
    void Set(const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXi& bLowerBounded, const Eigen::VectorXd& ub, const Eigen::VectorXi& bUpperBounded);
	void Union (const LinearConstraint& rhs);
    void Union(LinearConstraint* rhs);
	const Eigen::MatrixXd& GetLhs() const;
	const Eigen::VectorXd& GetRhs() const;
	const Eigen::VectorXd& GetLowerBounds() const;
	const Eigen::VectorXi& GetIsLowerBounded() const;
	const Eigen::VectorXd& GetUpperBounds() const;
	const Eigen::VectorXi& GetIsUpperBounded() const;
    bool CheckValidation(const Eigen::VectorXd& sol);
    int GetNumConstraints() const;
    int GetNumVariables() const;
    const vector<ConstraintValidationT>& GetConstraintSatisfactionType() const;

private:
	Eigen::MatrixXd mA; //Linear inequality constraint
	Eigen::VectorXd mlb; 
	Eigen::VectorXd mub;
	Eigen::VectorXi mIsConstraintsLowerBounded;
	Eigen::VectorXi mIsConstraintsUpperBounded;

    vector<ConstraintValidationT> mIsConstraintSatisfied;
};

#endif