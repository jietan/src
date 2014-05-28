#ifndef QP_SOLVER
#define QP_SOLVER

#include "stdafx.h"
#include "QuadraticObjective.h"
#include "LinearConstraint.h"
#include <Eigen/Dense>
#include <vector>
#include "ConicSolver.h"
using namespace Eigen;
using namespace std;

enum SolutionMethod
{
    SM_Normal,
    SM_SVD,
    SM_Separable,
    SM_Cone
};

class QPSolver
{
public:
	QPSolver();
	virtual ~QPSolver();
	virtual void SetObjective(QPObjective& obj);
	virtual void SetConstraints(const LinearConstraint& constraint);
	virtual void SetQuadraticTerm(const MatrixXd& Q);
	virtual void SetLinearTerm(const VectorXd& c);
	virtual void SetConstraints(const MatrixXd& A, const VectorXd& lb, const VectorXi& bLowerBounded, const VectorXd& ub, const VectorXi& bUpperBounded);
	virtual void SetLowerBounds(const VectorXd& lb, const VectorXi& bBounded);
	virtual void SetUpperBounds(const VectorXd& ub, const VectorXi &bBounded);
    virtual void ClearConstraints();
    virtual bool Solve(VectorXd& sol, SolutionMethod method = SM_Normal);
    virtual bool GetEqualityConstraints(MatrixXd& AEquality, VectorXd& bEquality);
    virtual bool GetInequalityConstraints(MatrixXd& AInequality, VectorXd& lbc, VectorXd& ubc, VectorXi& blb, VectorXi& bub);
    virtual int GetNumEqualityConstraints() const;
    virtual int GetNumInequalityConstraints() const;
    virtual int GetNumConstraints() const;
    virtual void DumpProblem(const string& fileName);
    virtual bool ReadProblemAndSolve(const string& fileName, VectorXd& sol, SolutionMethod method = SM_Normal);
    virtual double GetValue(const VectorXd& sol);
protected:
    void convertToConicProblem(ConicSolver& solver);
    void convertFromConicSolution(const VectorXd& conicSol, VectorXd& realSol);
	void convertMatrixVectorFormat();
	void convertQFormat();
	void convertAFormat();
	void preSolve();
	void postSolve(const VectorXd& sol, bool status, VectorXd& realSol);
    bool isEqualityConstraint(int ithConstraint) const;
    bool isInequalityConstraint(int ithConstraint) const;
    bool solve(VectorXd& sol);

    QPObjective* mObjective;

	MatrixXd mQ;
	VectorXd mc;
	
	VectorXd mlb;
	VectorXd mub;
	VectorXi mbLowerBounded;
	VectorXi mbUpperBounded;

	MatrixXd mA;
	VectorXd mlbc;
	VectorXd mubc;
	VectorXi mbConstraintLowerBounded;
	VectorXi mbConstraintUpperBounded;

    MatrixXd mLhs;
    VectorXd mRhs;

	int mNumCon;
	int mNumVar;
    int mNumVarOriginal;
    int mNumConOriginal;

	vector<int> mAColumnStartIdx;
	vector<int> mARowIdx;
	vector<double> mAValues;

	vector<int> mQSubi;
	vector<int> mQSubj;
	vector<double> mQValues;
    double mScale;

	SolutionMethod mMethodType;

	MatrixXd mU;
	VectorXd mCU;
	MatrixXd mSigma;
	MatrixXd mAU;
    double mConstant;

};
#endif