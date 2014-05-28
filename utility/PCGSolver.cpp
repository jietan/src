#include "stdafx.h"
#include "PCGSolver.h"
#ifdef _WIN32
#include "mkl_rci.h"
#include "mkl_blas.h"
#include "mkl_spblas.h"
#include "mkl_service.h"
#include <tbb/tbb.h>
#include <tbb/mutex.h>
using namespace tbb;
#endif

#include "DecoLogger.h"
#include <list>
#include <vector>
#include <math.h>

#ifdef _WIN32
#define NUM_CG_THREADS 8

class ParallelCGMultipleRHS
{
public:
	ParallelCGMultipleRHS(PCGSolver* solver, double* result, bool usePreconditioner) : mSolver(solver), mResult(result), mbUsePreconditioner(usePreconditioner)
	{

	}
	void operator()  (const blocked_range<size_t>& r)  const
	{
		int dim = mSolver->mMatrix->n;

		for (size_t i = r.begin(); i != r.end(); ++i)
		{
			mSolver->solveSingRhs(&(mResult[i * dim]), mbUsePreconditioner, &(mSolver->mRHS[i * dim]));
		}
	}

private:
	PCGSolver* mSolver;
	double* mResult;
	bool mbUsePreconditioner;
};
#endif

PCGSolver::PCGSolver() : mPrecon(NULL)
{
	//mkl_set_num_threads( 4 );
	//mkl_domain_set_num_threads( 16, MKL_ALL );
	//mkl_domain_set_num_threads( 8, MKL_BLAS );
}
PCGSolver::~PCGSolver()
{
	if (mPrecon)
		delete[] mPrecon;
}

void PCGSolver::Solve(double* result, bool usePreconditioner)
{
    if (usePreconditioner)
        calculatePreconditioner();
	if (mNumRhs == 1)
	{
		solveSingRhs(result, usePreconditioner);
	}
	else if (mNumRhs > 1)
	{
		solveMultipleRhs(result, usePreconditioner);
	}
	else
	{
		(*DecoLogger::GetSingleton()) << "PCG Solver requires at least one right hand side vector.\n";
	}
}

void PCGSolver::solveSingRhs(double* result, bool usePreconditioner, double* rhs)
{
#ifdef _WIN32
	if (!mMatrix->IsSymmetric())
	{
		(*DecoLogger::GetSingleton()) << "PCG solver can only solve symmetric matrix!\n";
	}

	char tr='l';
	char matdes[4];
	matdes[0] = 'd';
	matdes[1] = 'l';
	matdes[2] = 'n';
	matdes[3] = 'f';
	int dim = mMatrix->n;
	MKL_INT rci_request, itercount;
	MKL_INT ipar[128];
	double dpar[128];
	double* a = &(mMatrix->values[0]);
	int* ia = &(mMatrix->rowIndex[0]);
	int* ja = &(mMatrix->columns[0]);


	double *tmp = new double[4 * dim];
	double *trvec = new double[dim];
	double one = 1.0;

	/*---------------------------------------------------------------------------*/
	/* Initialize the initial guess                                              */
	/*---------------------------------------------------------------------------*/
	for(int i = 0; i < dim;i++) 
		result[i] = 0;
	/*---------------------------------------------------------------------------*/
	/* Initialize the solver                                                     */
	/*---------------------------------------------------------------------------*/
	if (!rhs)
		rhs = mRHS;
	dcg_init(&dim, result, rhs, &rci_request, ipar, dpar, tmp);
	if (rci_request != 0) goto failure;
	/*---------------------------------------------------------------------------*/
	/* Set the desired parameters:                                               */
	/* LOGICAL parameters:                                                       */
	/* do residual stopping test                                                 */
	/* do not request for the user defined stopping test                         */
	/* DOUBLE parameters                                                         */
	/*---------------------------------------------------------------------------*/
	ipar[1] = 6;
	ipar[4] = 1000;
	ipar[5] = 1;
	ipar[6] = 1;
	ipar[8] = 1;
	ipar[9] = 0;
	ipar[10] = usePreconditioner? 1 : 0;

	dpar[0] = mErrorThreshold;

	/*---------------------------------------------------------------------------*/
	/* Check the correctness and consistency of the newly set parameters         */
	/*---------------------------------------------------------------------------*/
	dcg_check(&dim, result, rhs, &rci_request, ipar, dpar, tmp);
	if (rci_request!=0) goto failure;
	/*---------------------------------------------------------------------------*/
	/* Compute the solution by RCI (P)CG solver without preconditioning          */
	/* Reverse Communications starts here                                        */
	/*---------------------------------------------------------------------------*/
rci: dcg(&dim, result, rhs, &rci_request, ipar, dpar, tmp);
	/*---------------------------------------------------------------------------*/
	/* If rci_request=0, then the solution was found with the required precision */
	/*---------------------------------------------------------------------------*/
	if (rci_request==0) goto getsln;
	/*---------------------------------------------------------------------------*/
	/* If rci_request=1, then compute the vector A*tmp[0]                 */
	/* and put the result in vector tmp[n]                                       */
	/*---------------------------------------------------------------------------*/

	if (rci_request==1)
	{
		mkl_dcsrsymv(&tr, &dim, a, ia, ja, tmp, &tmp[dim]);

		goto rci;
	}

	/*---------------------------------------------------------------------------*/
	/* If rci_request=3, then compute apply the preconditioner matrix C_inverse  */
	/* on vector TMP(:,3) and put the result in vector tmp[3*n]                  */
	/*---------------------------------------------------------------------------*/

	if (rci_request == 3)
	{
        for (int i = 0; i < dim; ++i)
        {
            tmp[3 * dim + i] = mPrecon[i] * tmp[2 * dim + i];
        }
		//char cvar1='L';
		//char cvar='T';
		//char cvar2='N';
		//mkl_dcsrtrsv(&cvar1,&cvar,&cvar2,&dim,mPrecon,ia,ja,&tmp[2*dim],trvec);
		//cvar1='L';
		//cvar='N';
		//cvar2='N';
		//mkl_dcsrtrsv(&cvar1,&cvar,&cvar2,&dim,mPrecon,ia,ja,trvec,&tmp[3*dim]);
		goto rci;
	}
	/*---------------------------------------------------------------------------*/
	/* If rci_request=anything else, then dcg subroutine failed                  */
	/* to compute the solution vector: solution[n]                               */
	/*---------------------------------------------------------------------------*/
	goto failure;
	/*---------------------------------------------------------------------------*/
	/* Reverse Communication ends here                                           */
	/* Get the current iteration number into itercount                           */
	/*---------------------------------------------------------------------------*/
getsln: dcg_get(&dim, result, rhs, &rci_request, ipar, dpar, tmp, &itercount);

	(*DecoLogger::GetSingleton()) << itercount << " number of iterations used to solve " << dim << " unknowns.\n";
	delete[] tmp;
	delete[] trvec;

	return;

failure: (*DecoLogger::GetSingleton()) << "This example FAILED with " << ipar[3] << " iterations and the solver has returned the ERROR code " << rci_request << "\n";
	delete[] tmp;
	delete[] trvec;

	return;
#endif

}


#if 0 //it seems that my MKL version do not support dcgmres
void PCGSolver::solveMultipleRhs(double* result, bool usePreconditioner)
{
	if (!mMatrix->IsSymmetric())
	{
		(*DecoLogger::GetSingleton()) << "PCG solver can only solve symmetric matrix!\n";
	}
	//mNumRhs = 1;
	char tr='l';
	
	int dim = mMatrix->n;
	MKL_INT rci_request;
	MKL_INT* itercount = new MKL_INT[mNumRhs];
	
	int parSize = 128 + mNumRhs * 2;
	int tmpSize = (3 + mNumRhs) * dim;
	MKL_INT* ipar = new MKL_INT[parSize];
	double* dpar = new double[parSize];
	double *tmp = new double[tmpSize];
	for (int i = 0; i < tmpSize; ++i)
	{
		tmp[i] = 0.0;
	}
	for (int i = 0; i < parSize; ++i)
	{
		ipar[i] = 0;
		dpar[i] = 0.0;
	}

	//memset(ipar, 0, (128 + mNumRhs * 2) * sizeof(MKL_INT));
	//memset(dpar, 0, (128 + mNumRhs * 2) * sizeof(double));

	double* a = &(mMatrix->values[0]);
	int* ia = &(mMatrix->rowIndex[0]);
	int* ja = &(mMatrix->columns[0]);



	int one = 1;


	/*---------------------------------------------------------------------------*/
	/* Initialize the initial guess                                              */
	/*---------------------------------------------------------------------------*/
	for(int i = 0; i < mNumRhs * dim;i++) 
		result[i] = 0;
	/*---------------------------------------------------------------------------*/
	/* Initialize the solver                                                     */
	/*---------------------------------------------------------------------------*/
	dcgmrhs_init(&dim, result, &mNumRhs, mRHS, &one, &rci_request, ipar, dpar, tmp);
	if (rci_request != 0) goto failure;
	/*---------------------------------------------------------------------------*/
	/* Set the desired parameters:                                               */
	/* LOGICAL parameters:                                                       */
	/* do residual stopping test                                                 */
	/* do not request for the user defined stopping test                         */
	/* DOUBLE parameters                                                         */
	/*---------------------------------------------------------------------------*/

	ipar[8] = 1;
	ipar[9] = 0;

	dpar[0] = mErrorThreshold;

	/*---------------------------------------------------------------------------*/
	/* Check the correctness and consistency of the newly set parameters         */
	/*---------------------------------------------------------------------------*/
//	dcgmrhs_check(&dim, result, &mNumRhs, mRHS, &rci_request, ipar, dpar, tmp);
	if (rci_request!=0) goto failure;
	/*---------------------------------------------------------------------------*/
	/* Compute the solution by RCI (P)CG solver without preconditioning          */
	/* Reverse Communications starts here                                        */
	/*---------------------------------------------------------------------------*/
rci: dcgmrhs(&dim, result, &mNumRhs, mRHS, &rci_request, ipar, dpar, tmp);
//	/*---------------------------------------------------------------------------*/
//	/* If rci_request=0, then the solution was found with the required precision */
//	/*---------------------------------------------------------------------------*/
	if (rci_request==0) 
		goto getsln;
//	/*---------------------------------------------------------------------------*/
//	/* If rci_request=1, then compute the vector A*tmp[0]                 */
//	/* and put the result in vector tmp[n]                                       */
//	/*---------------------------------------------------------------------------*/
//
	if (rci_request==1) 
	{
		mkl_dcsrsymv(&tr, &dim, a, ia, ja, tmp, &tmp[dim]);

		goto rci;
	}

	/*---------------------------------------------------------------------------*/
	/* If rci_request=anything else, then dcg subroutine failed                  */
	/* to compute the solution vector: solution[n]                               */
	/*---------------------------------------------------------------------------*/
	goto failure;
	/*---------------------------------------------------------------------------*/
	/* Reverse Communication ends here                                           */
	/* Get the current iteration number into itercount                           */
	/*---------------------------------------------------------------------------*/
//	MKL_FreeBuffers();
getsln: dcgmrhs_get(&dim, result, mRHS, &rci_request, ipar, dpar, tmp, itercount);

	(*DecoLogger::GetSingleton()) << itercount[0] << " number of iterations used to solve " << dim << " unknowns.\n";
	delete[] tmp;
	delete[] ipar;
	delete[] dpar;
	delete[] itercount;
	return;

failure: (*DecoLogger::GetSingleton()) << "This example FAILED with " << ipar[3] << " iterations and the solver has returned the ERROR code " << rci_request << "\n";
	delete[] tmp;
	delete[] ipar;
	delete[] dpar;
	delete[] itercount;
	return;
}
#else
void PCGSolver::solveMultipleRhs(double* result, bool usePreconditioner)
{
#ifdef _WIN32
	//int dim = mMatrix->n;
	//for (int i = 0; i < mNumRhs; ++i)
	//{
	//	solveSingRhs(&(result[i * dim]), usePreconditioner, &(mRHS[i * dim]));
	//}
	int grainSize = static_cast<int>(ceil(static_cast<double>(mNumRhs) / NUM_CG_THREADS));
	if (grainSize < 1)
		grainSize = 1;

	parallel_for(blocked_range<size_t>(0, mNumRhs, grainSize), ParallelCGMultipleRHS(this, result, usePreconditioner));	
#endif
}
#endif

void PCGSolver::calculatePreconditioner() 
{
#if 1
    if (mPrecon)
        delete[] mPrecon;
    int dim = mMatrix->n;
    mPrecon = new double[dim];
    for (int i = 0; i < dim; ++i)
    {
        int index;
        double value;
        mMatrix->GetDiagonalElement(i + 1, index, value);
        mPrecon[i] = 1.0 / value;
    }
#else //very slow MIC(0)
	if (mPrecon)
		delete[] mPrecon;
	
	int numEntries = mMatrix->GetNumNonZeroEntries();
	mPrecon = new double[numEntries];	
	memset(mPrecon, 0, numEntries * sizeof(double));

	double* a = &(mMatrix->values[0]);
	int* ia = &(mMatrix->rowIndex[0]);
	int* ja = &(mMatrix->columns[0]);

	int dim = mMatrix->n;

	for (int ithRow = 1; ithRow <= dim; ++ithRow)
	{
		int ithRowStartIdx = ia[ithRow - 1] - 1;
		int ithRowEndIdx = ia[ithRow] - 1;

		for (int idx = ithRowStartIdx; idx < ithRowEndIdx; ++idx)
		{
			int jthCol = ja[idx];
			mPrecon[idx] = a[idx];
			if (ithRow == jthCol)
			{
				for (int k = ithRowStartIdx; k < idx; ++k)
				{
					mPrecon[idx] -= mPrecon[k] * mPrecon[k];
				}
				mPrecon[idx] = sqrt(mPrecon[idx]);
			}
			else
			{
				int jthRowStartIdx = ia[jthCol - 1] - 1;
				int jthRowEndIdx = ia[jthCol] - 1;
				
				for (int k1 = ithRowStartIdx; k1 < ithRowEndIdx; ++k1)
				{
					for (int k2 = jthRowStartIdx; k2 < jthRowEndIdx; ++k2)
					{
						if (ja[k1] == ja[k2] && ja[k1] < jthCol)
						{
							mPrecon[idx] -= mPrecon[k1] * mPrecon[k2];
						}
					}
				}
				double Ljj = 1.0;
				for (int k2 = jthRowStartIdx; k2 < jthRowEndIdx; ++k2)
				{
					if (ja[k2] == jthCol)
						Ljj = mPrecon[k2];
				}
				mPrecon[idx] /= Ljj;
			}
		}
	}
#endif
}
