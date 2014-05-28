#ifndef _QP_Objective_H
#define _QP_Objective_H

#include "stdafx.h"
//#include "controller/COMTarget.h"

class QPObjective
{
public:
	QPObjective();
	virtual ~QPObjective();
	virtual void AddChild(QPObjective* objective, int priority);
	virtual const MatrixXd& GetQuadraticTerm();
	virtual const VectorXd& GetLinearTerm();
    virtual void SetLhsAndRhs(const MatrixXd& lhs, const VectorXd& rhs);//||mLhs * x + mRhs||^2
	virtual double GetConstantTerm();
	virtual double GetWeight() const;
	virtual int GetDim() const;
    virtual const MatrixXd& GetLhs();
    virtual const VectorXd& GetRhs();
	virtual void SetWeight(double weight);
	virtual void SetDim(int dim);
    virtual double GetObjectiveFuncValue(const VectorXd& sol, int ithChild);
    virtual double OutputObjectiveFuncValues(const VectorXd& sol, bool bOutputLog = true);
    virtual VectorXd EvaluateGradient(const VectorXd& sol);
    virtual void SetName(const string& name);
    virtual const string& GetName() const;
    virtual void PadZeros(int numDimToPad);
    virtual const vector<vector<QPObjective*> >& GetPrioritizedObjectives();
protected:
    void appendAllObjectives(vector<QPObjective*>& allObjectives);
    virtual void SetQuadraticTerm(const MatrixXd& quadratic);
    virtual void SetLinearTerm(const VectorXd& linear);
    virtual void SetConstantTerm(double constant);

	MatrixXd mQuadraticTerm;
	VectorXd mLinearTerm;
	double mConstantTerm;

    MatrixXd mLhs;
    VectorXd mRhs; 
	int mDim;
	double mWeight;
	QPObjective* mParentObjective;
	vector<QPObjective*> mChildObjectives;
    string mName;
    int mPriority;
    vector<vector<QPObjective*> > mPrioritizedObjectives;
private:
    QPObjective(const QPObjective& rhs);
    const QPObjective& operator= (const QPObjective& rhs);
};

#endif