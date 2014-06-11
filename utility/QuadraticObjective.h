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
	virtual const Eigen::MatrixXd& GetQuadraticTerm();
	virtual const Eigen::VectorXd& GetLinearTerm();
    virtual void SetLhsAndRhs(const Eigen::MatrixXd& lhs, const Eigen::VectorXd& rhs);//||mLhs * x + mRhs||^2
	virtual double GetConstantTerm();
	virtual double GetWeight() const;
	virtual int GetDim() const;
    virtual const Eigen::MatrixXd& GetLhs();
    virtual const Eigen::VectorXd& GetRhs();
	virtual void SetWeight(double weight);
	virtual void SetDim(int dim);
    virtual double GetObjectiveFuncValue(const Eigen::VectorXd& sol, int ithChild);
    virtual double OutputObjectiveFuncValues(const Eigen::VectorXd& sol, bool bOutputLog = true);
    virtual Eigen::VectorXd EvaluateGradient(const Eigen::VectorXd& sol);
    virtual void SetName(const string& name);
    virtual const string& GetName() const;
    virtual void PadZeros(int numDimToPad);
    virtual const vector<vector<QPObjective*> >& GetPrioritizedObjectives();
protected:
    void appendAllObjectives(vector<QPObjective*>& allObjectives);
    virtual void SetQuadraticTerm(const Eigen::MatrixXd& quadratic);
    virtual void SetLinearTerm(const Eigen::VectorXd& linear);
    virtual void SetConstantTerm(double constant);

	Eigen::MatrixXd mQuadraticTerm;
	Eigen::VectorXd mLinearTerm;
	double mConstantTerm;

    Eigen::MatrixXd mLhs;
    Eigen::VectorXd mRhs; 
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