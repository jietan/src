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
    void Set(const MatrixXd& A, const VectorXd& lb, const VectorXi& bLowerBounded, const VectorXd& ub, const VectorXi& bUpperBounded);
	void Union (const LinearConstraint& rhs);
    void Union(LinearConstraint* rhs);
	const MatrixXd& GetLhs() const;
	const VectorXd& GetRhs() const;
	const VectorXd& GetLowerBounds() const;
	const VectorXi& GetIsLowerBounded() const;
	const VectorXd& GetUpperBounds() const;
	const VectorXi& GetIsUpperBounded() const;
    bool CheckValidation(const VectorXd& sol);
    int GetNumConstraints() const;
    int GetNumVariables() const;
    const vector<ConstraintValidationT>& GetConstraintSatisfactionType() const;

private:
	MatrixXd mA; //Linear inequality constraint
	VectorXd mlb; 
	VectorXd mub;
	VectorXi mIsConstraintsLowerBounded;
	VectorXi mIsConstraintsUpperBounded;

    vector<ConstraintValidationT> mIsConstraintSatisfied;
};

#endif