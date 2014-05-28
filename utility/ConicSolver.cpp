#include "stdafx.h"
#include "ConicSolver.h"
#include "utility/EigenSerializer.h"


#ifdef _WIN32
static void MSKAPI printstr(void* handle, char str[])
{
//	LOG(INFO) << str;
}
#endif _WIN32

ConicSolver::ConicSolver() : mNumVar(0), mNumCon(0), mConstant(0), mNumCone(0)
{

}
ConicSolver::~ConicSolver()
{

}

void ConicSolver::AddCone(const Cone& cone)
{
    mCones.push_back(cone);
    mNumCone++;
}


void ConicSolver::SetConstraints(const LinearConstraint& constraint)
{
	const MatrixXd& A = constraint.GetLhs();
	const VectorXd& lb = constraint.GetLowerBounds();
	const VectorXi& blb = constraint.GetIsLowerBounded();
	const VectorXd& ub = constraint.GetUpperBounds();
	const VectorXi& bub = constraint.GetIsUpperBounded();
	SetConstraints(A, lb, blb, ub, bub);
}


void ConicSolver::SetLinearTerm(const VectorXd& c)
{
	mc = c;
    mNumVar = mc.size();
}
void ConicSolver::SetConstraints(const MatrixXd& A, const VectorXd& lb, const VectorXi& bLowerBounded, const VectorXd& ub, const VectorXi& bUpperBounded)
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

void ConicSolver::SetLowerBounds(const VectorXd& lb, const VectorXi& bBounded)
{
	mlb = lb;
	mbLowerBounded = bBounded;
}

void ConicSolver::SetUpperBounds(const VectorXd& ub, const VectorXi &bBounded)
{
	mub = ub;
	mbUpperBounded = bBounded;
}





bool ConicSolver::Solve(VectorXd& sol)
{
    bool ret = false;
#ifdef _WIN32
    VectorXd solution;
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
				r = MSK_putcj(task, j, mc[j]);
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
                if (nextColumnIdx - currentColumnIdx > 0)
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
        for (int i = 0; i < mNumCone; ++i)
        {
            Cone& cone = mCones[i];
            r = MSK_appendcone(task, MSK_CT_RQUAD, 0.0, cone.mSubscripts.size(), cone.GetMosekConeSubId());
            //r = MSK_appendcone(task, MSK_CT_QUAD, 0.0, cone.mSubscripts.size(), cone.GetMosekConeSubId());
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
                    sol = VectorXd::Zero(mNumVar);
					for (int k = 0; k < mNumVar; ++k)
                    {
						solution[k] = result[k];
                        sol[k] = result[k];
                    }
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
#endif    
	return ret;
}


void ConicSolver::convertMatrixVectorFormat()
{
	convertAFormat();
}

void ConicSolver::convertAFormat()
{
	if (!mNumCon) return;
	int numCols = mA.cols();
	int numRows = mA.rows();

	int numEntries = 0;
	mAColumnStartIdx.clear();
	mARowIdx.clear();
	mAValues.clear();

	for (int ithCol = 0; ithCol < numCols; ++ithCol)
	{
		mAColumnStartIdx.push_back(numEntries);
		for (int ithRow = 0; ithRow < numRows; ++ithRow)
		{
			double value = mA(ithRow, ithCol);
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

