#include "stdafx.h"
#include "QPSolver.h"
#include "utility/EigenSerializer.h"
#include "ConicSolver.h"

#ifdef _WIN32
#include <mosek.h>
static void MSKAPI printstr(void* handle, char str[])
{
	//LOG(INFO) << str;
}

#endif



QPSolver::QPSolver() : mNumVar(0), mNumCon(0), mObjective(NULL), mScale(1), mMethodType(SM_Normal), mConstant(0)
{

}
QPSolver::~QPSolver()
{

}

void QPSolver::SetObjective(QPObjective& obj)
{
	const MatrixXd& Q = obj.GetQuadraticTerm();
	const VectorXd& c = obj.GetLinearTerm();
	SetQuadraticTerm(Q);
	SetLinearTerm(c);
    mConstant = mScale * obj.GetConstantTerm() / 2.0;

    mObjective = &obj;
    mLhs = mObjective->GetLhs();
    mRhs = mObjective->GetRhs();
}
void QPSolver::SetConstraints(const LinearConstraint& constraint)
{
	const MatrixXd& A = constraint.GetLhs();
	const VectorXd& lb = constraint.GetLowerBounds();
	const VectorXi& blb = constraint.GetIsLowerBounded();
	const VectorXd& ub = constraint.GetUpperBounds();
	const VectorXi& bub = constraint.GetIsUpperBounded();
	SetConstraints(A, lb, blb, ub, bub);
}

void QPSolver::SetQuadraticTerm(const MatrixXd& Q)
{
	mQ = mScale * Q;
	int numVar = Q.rows();
	mNumVar = numVar;
	mbLowerBounded = VectorXi::Zero(numVar);
	mbUpperBounded = VectorXi::Zero(numVar);
	mlb = VectorXd::Zero(numVar);
	mub = VectorXd::Zero(numVar);
}
void QPSolver::SetLinearTerm(const VectorXd& c)
{
	mc = mScale * c;
}
void QPSolver::SetConstraints(const MatrixXd& A, const VectorXd& lb, const VectorXi& bLowerBounded, const VectorXd& ub, const VectorXi& bUpperBounded)
{
	mA = A;
	int numCon = A.rows();
	mNumCon = numCon;
	mbConstraintLowerBounded = VectorXi::Zero(numCon);
	mbConstraintUpperBounded = VectorXi::Zero(numCon);
	mlbc = VectorXd::Zero(numCon);
	mubc = VectorXd::Zero(numCon);

    mlbc = lb;
    mbConstraintLowerBounded = bLowerBounded;

    mubc = ub;
    mbConstraintUpperBounded = bUpperBounded;

}

void QPSolver::SetLowerBounds(const VectorXd& lb, const VectorXi& bBounded)
{
	mlb = lb;
	mbLowerBounded = bBounded;
}

void QPSolver::SetUpperBounds(const VectorXd& ub, const VectorXi &bBounded)
{
	mub = ub;
	mbUpperBounded = bBounded;
}

int QPSolver::GetNumEqualityConstraints() const
{
    int count = 0;
    for (int i = 0; i < mNumCon; ++i)
    {
        if (isEqualityConstraint(i))
            count++;
    }
    return count;
}
int QPSolver::GetNumInequalityConstraints() const
{
    int count = 0;
    for (int i = 0; i < mNumCon; ++i)
    {
        if (isInequalityConstraint(i))
            count++;
    }
    return count;
}
int QPSolver::GetNumConstraints() const
{
    return mNumCon;
}

bool QPSolver::GetEqualityConstraints(MatrixXd& AEquality, VectorXd& bEquality)
{
    bool ret = false;
    int count = GetNumEqualityConstraints();
    if (count)
    {
        ret = true;
        AEquality = MatrixXd::Zero(count, mNumVar);
        bEquality = VectorXd::Zero(count);

        count = 0;
        for (int i = 0; i < mNumCon; ++i)
        {
            if (isEqualityConstraint(i))
            {
                AEquality.row(count) = mA.row(i);
                bEquality[count] = mlbc[i];
                count++;
            }
        }
    }
    return ret;
}
bool QPSolver::GetInequalityConstraints(MatrixXd& AInequality, VectorXd& lbc, VectorXd& ubc, VectorXi& blb, VectorXi& bub)
{
    bool ret = false;
    int count = GetNumInequalityConstraints();
    if (count)
    {
        ret = true;
        AInequality = MatrixXd::Zero(count, mNumVar);
        lbc = VectorXd::Zero(count);
        ubc = VectorXd::Zero(count);
        blb = VectorXi::Zero(count);
        bub = VectorXi::Zero(count);

        count = 0;
        for (int i = 0; i < mNumCon; ++i)
        {
            if (isInequalityConstraint(i))
            {
                AInequality.row(count) = mA.row(i);
                lbc[count] = mlbc[i];
                ubc[count] = mubc[i];
                blb[count] = mbConstraintLowerBounded[i];
                bub[count] = mbConstraintUpperBounded[i];
                count++;
            }
        }
    }
    return ret;
}

void QPSolver::ClearConstraints()
{
    mNumCon = 0;
}

void QPSolver::convertToConicProblem(ConicSolver& solver)
{
    const MatrixXd& F = mLhs;
    int newNumVar = mNumVar + 1 + 1 + F.rows(); //x,v,w,t
    int newNumCon = mNumCon + F.rows() + 1 + 2;
    
    VectorXd newC = VectorXd::Zero(newNumVar);
    newC.head(mNumVar) = mc;
    newC(mNumVar) = 1.0;
    solver.SetLinearTerm(newC);

    MatrixXd newA = MatrixXd::Zero(newNumCon, newNumVar);
    VectorXd newLbc = VectorXd::Zero(newNumCon);
    VectorXd newUbc = VectorXd::Zero(newNumCon);
    VectorXi newIsConstraintLowerBounded = VectorXi::Zero(newNumCon);
    VectorXi newIsConstraintUpperBounded = VectorXi::Zero(newNumCon);

    if (mNumCon)
    {
        newA.block(0, 0, mNumCon, mNumVar) = mA;
        newLbc.head(mNumCon) = mlbc;
        newUbc.head(mNumCon) = mubc;
        newIsConstraintLowerBounded.head(mNumCon) = mbConstraintLowerBounded;
        newIsConstraintUpperBounded.head(mNumCon) = mbConstraintUpperBounded;
    }

    newA.block(mNumCon, 0, F.rows(), mNumVar) = F;
    newA.block(mNumCon, newNumVar - F.rows(), F.rows(), F.rows()) = -MatrixXd::Identity(F.rows(), F.rows());
    newA(mNumCon + F.rows(), mNumVar + 1) = 1.0;
    newA(mNumCon + F.rows() + 1, mNumVar) = 1.0;
    newA(mNumCon + F.rows() + 2, mNumVar + 1) = 1.0;

    newLbc.segment(mNumCon, F.rows()) = VectorXd::Zero(F.rows());
    newLbc(mNumCon + F.rows()) = 1.0;

    newUbc.segment(mNumCon, F.rows()) = VectorXd::Zero(F.rows());
    newUbc(mNumCon + F.rows()) = 1.0;

    newIsConstraintLowerBounded.segment(mNumCon, F.rows()) = VectorXi::Constant(F.rows(), 1);
    newIsConstraintLowerBounded.tail(3) = VectorXi::Constant(3, 1);

    newIsConstraintUpperBounded.segment(mNumCon, F.rows()) = VectorXi::Constant(F.rows(), 1);
    newIsConstraintUpperBounded(mNumCon + F.rows()) = 1;

    solver.SetConstraints(newA, newLbc, newIsConstraintLowerBounded, newUbc, newIsConstraintUpperBounded);

    VectorXd newLb = VectorXd::Zero(newNumVar);
    VectorXd newUb = VectorXd::Zero(newNumVar);
    VectorXi newBLowerBounded = VectorXi::Zero(newNumVar);
    VectorXi newBUpperBounded = VectorXi::Zero(newNumVar);
    
    solver.SetLowerBounds(newLb, newBLowerBounded);
    solver.SetUpperBounds(newUb, newBUpperBounded);

    Cone cone;
    for (int i = mNumVar; i < newNumVar; ++i)
    {
        cone.mSubscripts.push_back(i);
    }
    solver.AddCone(cone);
}

void QPSolver::convertFromConicSolution(const VectorXd& conicSol, VectorXd& realSol)
{
    realSol = conicSol.head(mNumVar);
}

bool QPSolver::Solve(VectorXd& sol, SolutionMethod method)
{
    mMethodType = method;
    bool ret = false;
    if (mMethodType == SM_Cone)
    {
        ConicSolver solver;
        convertToConicProblem(solver);
        VectorXd solution;
        ret = solver.Solve(solution);
        if (ret)
        {
            convertFromConicSolution(solution, sol);
        }
    }
    else
    {   
        ret = solve(sol);
    }
    return ret;
}

bool QPSolver::solve(VectorXd& sol)
{
    bool ret = false;
#ifdef _WIN32
    VectorXd solution;
	preSolve();
	convertMatrixVectorFormat();
	MSKenv_t env;
	MSKtask_t task;
	MSKrescodee r;

	r = MSK_makeenv(&env, NULL, NULL, NULL, NULL);
	if (r == MSK_RES_OK)
	{
		r = MSK_linkfunctoenvstream(env, MSK_STREAM_LOG, NULL, printstr);
	}

	r = MSK_initenv(env);
	if (r == MSK_RES_OK)
	{
		r = MSK_maketask(env, mNumCon, mNumVar, &task);
		if (r == MSK_RES_OK)
		{
			r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);
		}

		if (r == MSK_RES_OK)
			r = MSK_putmaxnumvar(task, mNumVar);
		if (r == MSK_RES_OK)
			r = MSK_putmaxnumcon(task, mNumCon);

		/* Append ¡¯NUMCON ¡¯ empty constraints .
		 The constraints will initially have no bounds . */
		if (r == MSK_RES_OK)
			r = MSK_append(task, MSK_ACC_CON, mNumCon);
		/* Append ¡¯NUMVAR ¡¯ variables .
		 The variables will initially be fixed at zero (x =0). */
		if (r == MSK_RES_OK)
			r = MSK_append(task, MSK_ACC_VAR, mNumVar);

		/* Optionally add a constant term to the objective . */
		if (r == MSK_RES_OK)
			r = MSK_putcfix(task, mConstant);

		for (int j = 0; j < mNumVar && r == MSK_RES_OK; ++j)
		{
			/* Set the linear term c_j in the objective .*/
			if (r == MSK_RES_OK)
				r = MSK_putcj(task, j, mCU[j]);
			/* Set the bounds on variable j.*/
			if (r == MSK_RES_OK)
			{
				if (mbLowerBounded[j] && mbUpperBounded[j])
				{
					if (mlb[j] == mub[j])
						r = MSK_putbound(task, MSK_ACC_VAR, j, MSK_BK_FX, mlb[j], mub[j]);
					else
					{
						CHECK(mlb[j] < mub[j]);
						r = MSK_putbound(task, MSK_ACC_VAR, j, MSK_BK_RA, mlb[j], mub[j]);
					}
				}
				else if (mbLowerBounded[j])
				{
					r = MSK_putbound(task, MSK_ACC_VAR, j , MSK_BK_LO, mlb[j], +MSK_INFINITY);
				}
				else if (mbUpperBounded[j])
				{
					r = MSK_putbound(task, MSK_ACC_VAR, j, MSK_BK_UP, -MSK_INFINITY, mub[j]);
				}	
				else
				{
					r = MSK_putbound(task, MSK_ACC_VAR, j, MSK_BK_FR, -MSK_INFINITY, +MSK_INFINITY);
				}
			}
			/* Input column j of A */
			if (r == MSK_RES_OK && mNumCon)
			{
				int currentColumnIdx = mAColumnStartIdx[j];
				int nextColumnIdx = mAColumnStartIdx[j + 1];
				r = MSK_putavec(task, MSK_ACC_VAR, j, nextColumnIdx - currentColumnIdx, &(mARowIdx[currentColumnIdx]), &(mAValues[currentColumnIdx]));
			}
		}
		/* Set the bounds on constraints .
		 for i=1, ... , NUMCON : blc [i] <= constraint i <= buc [i] */
		for (int i = 0; i < mNumCon && r == MSK_RES_OK; ++i)
		{
			if (mbConstraintLowerBounded[i] && mbConstraintUpperBounded[i])
			{
				if (mlbc[i] == mubc[i])
				{
					r = MSK_putbound(task, MSK_ACC_CON, i, MSK_BK_FX, mlbc[i], mubc[i]);
				}
				else 
				{
					r = MSK_putbound(task, MSK_ACC_CON, i, MSK_BK_RA, mlbc[i], mubc[i]);
				}
			}
			else if (mbConstraintLowerBounded[i])
			{
				r = MSK_putbound(task, MSK_ACC_CON, i, MSK_BK_LO, mlbc[i], +MSK_INFINITY);
			}
			else if (mbConstraintUpperBounded[i])
			{
				r = MSK_putbound(task, MSK_ACC_CON, i, MSK_BK_UP, -MSK_INFINITY, mubc[i]);
			}
			else
			{
				LOG(WARNING) << "Every constraint should not be free.";
			}
		}
		if (r == MSK_RES_OK)
		{
			/* Input the Q for the objective . */
			r = MSK_putqobj(task, mQValues.size(), &(mQSubi[0]), &(mQSubj[0]), &(mQValues[0]));
		}

		if (r == MSK_RES_OK)
		{
			MSKrescodee trmcode;

			r = MSK_optimizetrm(task, &trmcode);
			MSK_solutionsummary(task, MSK_STREAM_LOG);

			if (r == MSK_RES_OK)
			{
				MSKsolstae solsta;
				MSK_getsolutionstatus(task, MSK_SOL_ITR, NULL, &solsta);
				double* result = new double[mNumVar];
				switch (solsta)
				{
				case MSK_SOL_STA_OPTIMAL:
				case MSK_SOL_STA_NEAR_OPTIMAL:
					MSK_getsolutionslice(task, MSK_SOL_ITR, MSK_SOL_ITEM_XX, 0, mNumVar, result);
					LOG(INFO) << "Optimal primal solution";
                    ret = true;
					solution = VectorXd::Zero(mNumVar);
					for (int k = 0; k < mNumVar; ++k)
						solution[k] = result[k];
					break;
				case MSK_SOL_STA_DUAL_INFEAS_CER:
				case MSK_SOL_STA_PRIM_INFEAS_CER:
				case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
				case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
					LOG(WARNING) << "Primal or dual infeasibility certificate found.";
					break;
				case MSK_SOL_STA_UNKNOWN:
					LOG(WARNING) << "The status of the solution could not be determined.";
					break;
				default:
					LOG(WARNING) << "Other solution status.";
					break;

				}
				delete[] result;

			}
		}
		else
		{
			LOG(WARNING) << "Error while optimizing.";
		}
		if (r != MSK_RES_OK)
		{
			char symname[MSK_MAX_STR_LEN];
			char desc[MSK_MAX_STR_LEN];
			LOG(WARNING) << "An error occurred while optimizing.";
			MSK_getcodedesc(r, symname, desc);
			LOG(WARNING) << "Error " << symname << " - " << desc;
		
		}
       
	}
	MSK_deletetask(&task);
	MSK_deleteenv(&env);
    
	postSolve(solution, ret, sol);
#endif
	return ret;
}

void QPSolver::preSolve()
{
	if (mMethodType == SM_Normal)
	{
		mU = MatrixXd::Identity(mNumVar, mNumVar);
		mCU = mc;
		mSigma = mQ;
		mAU = mA;
	}
	else if (mMethodType == SM_SVD)
	{
		JacobiSVD<MatrixXd> svd(mQ, ComputeThinU | ComputeThinV);
		mU = svd.matrixU();
		VectorXd singularValues = svd.singularValues();

		mSigma = MatrixXd::Zero(mNumVar, mNumVar);
		for (int i = 0; i < singularValues.size(); ++i)
		{
			mSigma(i, i) = singularValues[i];
		}
		MatrixXd uTrans = mU.transpose();
		mCU = uTrans * mc;
		mAU = mA * mU;
        //MatrixXd recoveredQ = mU * mSigma * uTrans;
        //for (int i = 0; i < recoveredQ.rows(); ++i)
        //{
        //    for (int j = 0; j < recoveredQ.cols(); ++j)
        //    {
        //        if (abs((recoveredQ(i, j) - mQ(i, j)) / mQ(i, j)) > 1e-6)
        //            LOG(INFO) << "(" << i << ", " << j << "): " << recoveredQ(i, j) << ": " << mQ(i, j);
        //    }
        //}
	}
    else if (mMethodType == SM_Separable)
    {
        const MatrixXd& lhs = mLhs;
        int numRows = lhs.rows();

        mNumVarOriginal = mNumVar;
        mNumConOriginal = mNumCon;

        mNumVar += numRows;
        mNumCon += numRows;

        mSigma = MatrixXd::Zero(mNumVar, mNumVar);
        mSigma.block(mNumVarOriginal, mNumVarOriginal, numRows, numRows) = MatrixXd::Identity(numRows, numRows);

        mCU = VectorXd::Zero(mNumVar);
        mCU.head(mNumVarOriginal) = mc.head(mNumVarOriginal);

        mAU = MatrixXd::Zero(mNumCon, mNumVar);
        if (mNumConOriginal)
            mAU.block(0, 0, mNumConOriginal, mNumVarOriginal) = mA;
        mAU.block(mNumConOriginal, 0, numRows, mNumVarOriginal) = lhs;
        mAU.block(mNumConOriginal, mNumVarOriginal, numRows, numRows) = -MatrixXd::Identity(numRows, numRows);

        VectorXd newLbc = VectorXd::Zero(mNumCon);
        VectorXd newUbc = VectorXd::Zero(mNumCon);
        VectorXi newBLowerBounded = VectorXi::Zero(mNumCon);
        VectorXi newBUpperBounded = VectorXi::Zero(mNumCon);

        if (mNumConOriginal)
        {
            newLbc.head(mNumConOriginal) = mlbc;
            newUbc.head(mNumConOriginal) = mubc;
            newBLowerBounded.head(mNumConOriginal) = mbConstraintLowerBounded;
            newBUpperBounded.head(mNumConOriginal) = mbConstraintUpperBounded;
        }

        newBLowerBounded.tail(numRows) = VectorXi::Constant(numRows, 1);
        newBUpperBounded.tail(numRows) = VectorXi::Constant(numRows, 1);

        mlbc = newLbc;
        mubc = newUbc;
        mbConstraintLowerBounded = newBLowerBounded;
        mbConstraintUpperBounded = newBUpperBounded;

        VectorXd newLb = VectorXd::Zero(mNumVar);
        VectorXd newUb = VectorXd::Zero(mNumVar);
        VectorXi newValueLowerBounded = VectorXi::Zero(mNumVar);
        VectorXi newValueUpperBounded = VectorXi::Zero(mNumVar);

        newLb.head(mNumVarOriginal) = mlb;
        newUb.head(mNumVarOriginal) = mub;
        newValueLowerBounded.head(mNumVarOriginal) = mbLowerBounded;
        newValueUpperBounded.head(mNumVarOriginal) = mbUpperBounded;

        mlb = newLb;
        mub = newUb;
        mbLowerBounded = newValueLowerBounded;
        mbUpperBounded = newValueUpperBounded;
    }

}

void QPSolver::postSolve(const VectorXd& sol, bool status, VectorXd& realSol)
{
	if (status)
	{
        if (mMethodType == SM_Normal)
        {
            realSol = sol;
        }
        else if (mMethodType == SM_SVD)
        {
		    realSol = mU * sol;
        }
        else if (mMethodType == SM_Separable)
        {
            realSol = sol.head(mNumVarOriginal);
        }
	}
}

void QPSolver::convertMatrixVectorFormat()
{
	convertQFormat();
	convertAFormat();
}
void QPSolver::convertQFormat()
{
	int numCols = mSigma.cols();
	int numRows = mSigma.rows();

	mQSubi.clear();
	mQSubj.clear();
	mQValues.clear();

	for (int ithRow = 0; ithRow < numRows; ++ithRow)
	{
		for (int ithCol = 0; ithCol < numCols; ++ithCol)
		{
			if (ithRow < ithCol || mSigma(ithRow, ithCol) == 0.0)
				continue;
			mQSubi.push_back(ithRow);
			mQSubj.push_back(ithCol);
			mQValues.push_back(mSigma(ithRow, ithCol));
		}
	}
}
void QPSolver::convertAFormat()
{
	if (!mNumCon) return;
	int numCols = mAU.cols();
	int numRows = mAU.rows();

	int numEntries = 0;
	mAColumnStartIdx.clear();
	mARowIdx.clear();
	mAValues.clear();

	for (int ithCol = 0; ithCol < numCols; ++ithCol)
	{
		mAColumnStartIdx.push_back(numEntries);
		for (int ithRow = 0; ithRow < numRows; ++ithRow)
		{
			double value = mAU(ithRow, ithCol);
			if (value != 0.0)
			{
				mAValues.push_back(value);
				mARowIdx.push_back(ithRow);
				numEntries++;
			}
		}
	}
	mAColumnStartIdx.push_back(numEntries);
}

bool QPSolver::isEqualityConstraint(int ithConstraint) const
{
    return mbConstraintLowerBounded[ithConstraint] && mbConstraintUpperBounded[ithConstraint] && mlbc[ithConstraint] == mubc[ithConstraint];
}
bool QPSolver::isInequalityConstraint(int ithConstraint) const
{
    return (!isEqualityConstraint(ithConstraint) && (mbConstraintLowerBounded[ithConstraint] || mbConstraintUpperBounded[ithConstraint]));
}

double QPSolver::GetValue(const VectorXd& sol)
{
    VectorXd result = mLhs * sol + mRhs;
    return result.norm() * result.norm();
}


void QPSolver::DumpProblem(const string& fileName)
{
    string realFileName = fileName;
    realFileName += "Lhs";
    EigenSerializer::SaveMatrixToFileDouble(realFileName, mLhs, true);
    realFileName += ".txt";
    EigenSerializer::SaveMatrixToFileDouble(realFileName, mLhs, false);

    realFileName = fileName;
    realFileName += "Rhs";
    EigenSerializer::SaveVectorToFileDouble(realFileName, mRhs, true);
    realFileName += ".txt";
    EigenSerializer::SaveVectorToFileDouble(realFileName, mRhs, false);

    realFileName = fileName;
    realFileName += "Q";
    EigenSerializer::SaveMatrixToFileDouble(realFileName, mQ, true);
    realFileName += ".txt";
    EigenSerializer::SaveMatrixToFileDouble(realFileName, mQ, false);

    realFileName = fileName;
    realFileName += "C";
    EigenSerializer::SaveVectorToFileDouble(realFileName, mc, true);
    realFileName += ".txt";
    EigenSerializer::SaveVectorToFileDouble(realFileName, mc, false);

    realFileName = fileName;
    realFileName += "A";
    EigenSerializer::SaveMatrixToFileDouble(realFileName, mA, true);
    realFileName += ".txt";
    EigenSerializer::SaveMatrixToFileDouble(realFileName, mA, false);

    realFileName = fileName;
    realFileName += "Lbc";
    EigenSerializer::SaveVectorToFileDouble(realFileName, mlbc, true);
    realFileName += ".txt";
    EigenSerializer::SaveVectorToFileDouble(realFileName, mlbc, false);

    realFileName = fileName;
    realFileName += "Ubc";
    EigenSerializer::SaveVectorToFileDouble(realFileName, mubc, true);
    realFileName += ".txt";
    EigenSerializer::SaveVectorToFileDouble(realFileName, mubc, false);

    realFileName = fileName;
    realFileName += "Blb";
    EigenSerializer::SaveVectorToFileInt(realFileName, mbConstraintLowerBounded, true);
    realFileName += ".txt";
    EigenSerializer::SaveVectorToFileInt(realFileName, mbConstraintLowerBounded, false);

    realFileName = fileName;
    realFileName += "Bub";
    EigenSerializer::SaveVectorToFileInt(realFileName, mbConstraintUpperBounded, true);
    realFileName += ".txt";
    EigenSerializer::SaveVectorToFileInt(realFileName, mbConstraintUpperBounded, false);

}

bool QPSolver::ReadProblemAndSolve(const string& fileName, VectorXd& sol, SolutionMethod method)
{
    string realFileName = fileName;
    realFileName += "Lhs";
    EigenSerializer::ReadMatrixFromFileDouble(realFileName, mLhs);
    
    realFileName = fileName;
    realFileName += "Rhs";
    EigenSerializer::ReadVectorFromFileDouble(realFileName, mRhs);

    realFileName = fileName;
    realFileName += "Q";
    EigenSerializer::ReadMatrixFromFileDouble(realFileName, mQ);

    realFileName = fileName;
    realFileName += "C";
    EigenSerializer::ReadVectorFromFileDouble(realFileName, mc);

    realFileName = fileName;
    realFileName += "A";
    EigenSerializer::ReadMatrixFromFileDouble(realFileName, mA);

    realFileName = fileName;
    realFileName += "Lbc";
    EigenSerializer::ReadVectorFromFileDouble(realFileName, mlbc);

    realFileName = fileName;
    realFileName += "Ubc";
    EigenSerializer::ReadVectorFromFileDouble(realFileName, mubc);

    realFileName = fileName;
    realFileName += "Blb";
    EigenSerializer::ReadVectorFromFileInt(realFileName, mbConstraintLowerBounded);

    realFileName = fileName;
    realFileName += "Bub";
    EigenSerializer::ReadVectorFromFileInt(realFileName, mbConstraintUpperBounded);

    //int startConId = 0;
    //int endConId = 48;
    //mNumVar = mA.cols();
    //mNumCon = mA.rows();
    //mNumCon = endConId - startConId;

    //double scale = 1;
    //MatrixXd newA = scale * mA.block(startConId, 0, mNumCon, mNumVar);
    //VectorXd newlbc = scale * mlbc.segment(startConId, mNumCon);
    //VectorXd newubc = scale * mubc.segment(startConId, mNumCon);
    //VectorXi newblb = mbConstraintLowerBounded.segment(startConId, mNumCon);
    //VectorXi newbub = VectorXi::Zero(mNumCon);//mbConstraintUpperBounded.segment(startConId, mNumCon);

    //mA = newA;
    //mlbc = newlbc;
    //mubc = newubc;
    //mbConstraintLowerBounded = newblb;
    //mbConstraintUpperBounded = newbub;


    mlb = VectorXd::Zero(mNumVar);
    mub = VectorXd::Zero(mNumVar);
    mbLowerBounded = VectorXi::Zero(mNumVar);
    mbUpperBounded = VectorXi::Zero(mNumVar);

    return Solve(sol, method);
}
