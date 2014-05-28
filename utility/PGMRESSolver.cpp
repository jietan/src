#include "stdafx.h"
#include "PGMRESSolver.h"

#ifdef _WIN32
#include "mkl_rci.h"
#include "mkl_blas.h"
#include "mkl_spblas.h"
#include "mkl_service.h"
#endif

#include "DecoLogger.h"

PGMRESSolver::PGMRESSolver() : mPrecon(NULL)
{
	//mkl_domain_set_num_threads( 8, MKL_ALL );
}
PGMRESSolver::~PGMRESSolver()
{
	if (mPrecon)
		delete mPrecon;
}

void PGMRESSolver::Solve(double* result, bool usePreconditioner)
{
#ifdef _WIN32
	/*---------------------------------------------------------------------------
	/* Define arrays for the upper triangle of the coefficient matrix
	/* Compressed sparse row storage is used for sparse representation
	/*---------------------------------------------------------------------------*/
	double* A = &(mMatrix->values[0]);
	int* ia = &(mMatrix->rowIndex[0]);
	int* ja = &(mMatrix->columns[0]);
	int N = mMatrix->n;
	int numNonZeroEntries = mMatrix->GetNumNonZeroEntries();
	/*---------------------------------------------------------------------------
	/* Allocate storage for the ?par parameters and the solution/rhs/residual vectors
	/*---------------------------------------------------------------------------*/
	MKL_INT ipar[128];
	double dpar[128];
	int restart = 2;

	double* tmp = new double[N*(2*restart+1)+(restart*(restart+9))/2+1];
	double* trvec = new double[N];
	double* bilu0 = new double[numNonZeroEntries];
	double* b = new double[N];
	double* residual = new double[N];

	/*---------------------------------------------------------------------------
	/* Some additional variables to use with the RCI (P)FGMRES solver
	/*---------------------------------------------------------------------------*/
	MKL_INT itercount,ierr=0;
	MKL_INT RCI_request, i, ivar;
	double dvar;
	char cvar,cvar1,cvar2;
	ivar=N;
	cvar='N';
	/*---------------------------------------------------------------------------
	/* Save the right-hand side in vector b for future use
	/*---------------------------------------------------------------------------*/
	i=1;
	dcopy(&ivar, mRHS, &i, b, &i);
	/*---------------------------------------------------------------------------
	/* Initialize the initial guess
	/*---------------------------------------------------------------------------*/
	for(i=0;i<N;i++)
	{
		result[i]=0.0;
	}
	//computed_solution[0]=100.0;

	/*---------------------------------------------------------------------------
	/* Initialize the solver
	/*---------------------------------------------------------------------------*/
	dfgmres_init(&ivar, result, mRHS, &RCI_request, ipar, dpar, tmp);
	if (RCI_request!=0) goto FAILED;

	/*---------------------------------------------------------------------------
	/* Calculate ILU0 preconditioner.
	/*                      !ATTENTION!
	/* DCSRILU0 routine uses some IPAR, DPAR set by DFGMRES_INIT routine.
	/* Important for DCSRILU0 default entries set by DFGMRES_INIT are
	/* ipar[1] = 6 - output of error messages to the screen,
	/* ipar[5] = 1 - allow output of errors,
	/* ipar[30]= 0 - abort DCSRILU0 calculations if routine meets zero diagonal element.
	/*
	/* If ILU0 is going to be used out of MKL FGMRES context, than the values
	/* of ipar[1], ipar[5], ipar[30], dpar[30], and dpar[31] should be user
	/* provided before the DCSRILU0 routine call.
	/*
	/* In this example, specific for DCSRILU0 entries are set in turn:
	/* ipar[30]= 1 - change small diagonal value to that given by dpar[31],
	/* dpar[30]= 1.E-20 instead of the default value set by DFGMRES_INIT.
	/*                  It is a small value to compare a diagonal entry with it.
	/* dpar[31]= 1.E-16 instead of the default value set by DFGMRES_INIT.
	/*                  It is the target value of the diagonal value if it is
	/*                  small as compared to dpar[30] and the routine should change
	/*                  it rather than abort DCSRILU0 calculations.
	/*---------------------------------------------------------------------------*/

	ipar[30]=1;
	dpar[30]=1.E-20;
	dpar[31]=1.E-16;

	dcsrilu0(&ivar, A, ia, ja, bilu0, ipar, dpar, &ierr);


	if (ierr!=0)
	{
		(*DecoLogger::GetSingleton()) << "Preconditioner dcsrilu0 has returned the ERROR code " << ierr << "\n";
		goto FAILED1;
	}

	/*---------------------------------------------------------------------------
	/* Set the desired parameters:
	/* do the restart after 2 iterations
	/* LOGICAL parameters:
	/* do not do the stopping test for the maximal number of iterations
	/* do the Preconditioned iterations of FGMRES method
	/* Set parameter ipar[10] for preconditioner call. For this example,
	/* it reduces the number of iterations.
	/* DOUBLE PRECISION parameters
	/* set the relative tolerance to 1.0D-3 instead of default value 1.0D-6
	/* NOTE. Preconditioner may increase the number of iterations for an
	/* arbitrary case of the system and initial guess and even ruin the
	/* convergence. It is user's responsibility to use a suitable preconditioner
	/* and to apply it skillfully.
	/*---------------------------------------------------------------------------*/
	ipar[14]=restart;
	ipar[7]=0;
	ipar[10]=1;
	dpar[0]=1.0E-3;

	/*---------------------------------------------------------------------------
	/* Check the correctness and consistency of the newly set parameters
	/*---------------------------------------------------------------------------*/
	dfgmres_check(&ivar, result, mRHS, &RCI_request, ipar, dpar, tmp);
	if (RCI_request!=0) goto FAILED;

	/*---------------------------------------------------------------------------
	/* Compute the solution by RCI (P)FGMRES solver with preconditioning
	/* Reverse Communication starts here
	/*---------------------------------------------------------------------------*/
ONE:  dfgmres(&ivar, result, mRHS, &RCI_request, ipar, dpar, tmp);
	/*---------------------------------------------------------------------------
	/* If RCI_request=0, then the solution was found with the required precision
	/*---------------------------------------------------------------------------*/
	if (RCI_request==0) goto COMPLETE;
	/*---------------------------------------------------------------------------
	/* If RCI_request=1, then compute the vector A*tmp[ipar[21]-1]
	/* and put the result in vector tmp[ipar[22]-1]
	/*---------------------------------------------------------------------------
	/* NOTE that ipar[21] and ipar[22] contain FORTRAN style addresses,
	/* therefore, in C code it is required to subtract 1 from them to get C style
	/* addresses
	/*---------------------------------------------------------------------------*/
	if (RCI_request==1)
	{
		mkl_dcsrgemv(&cvar, &ivar, A, ia, ja, &tmp[ipar[21]-1], &tmp[ipar[22]-1]);
		goto ONE;
	}
	/*---------------------------------------------------------------------------
	/* If RCI_request=2, then do the user-defined stopping test
	/* The residual stopping test for the computed solution is performed here
	/*---------------------------------------------------------------------------
	/* NOTE: from this point vector b[N] is no longer containing the right-hand
	/* side of the problem! It contains the current FGMRES approximation to the
	/* solution. If you need to keep the right-hand side, save it in some other
	/* vector before the call to dfgmres routine. Here we saved it in vector
	/* rhs[N]. The vector b is used instead of rhs to preserve the
	/* original right-hand side of the problem and guarantee the proper
	/* restart of FGMRES method. Vector b will be altered when computing the
	/* residual stopping criterion!
	/*---------------------------------------------------------------------------*/
	if (RCI_request==2)
	{
		/* Request to the dfgmres_get routine to put the solution into b[N] via ipar[12]
		/*---------------------------------------------------------------------------
		/* WARNING: beware that the call to dfgmres_get routine with ipar[12]=0 at this stage may
		/* destroy the convergence of the FGMRES method, therefore, only advanced users should
		/* exploit this option with care */
		ipar[12]=1;
		/* Get the current FGMRES solution in the vector b[N] */
		dfgmres_get(&ivar, result, b, &RCI_request, ipar, dpar, tmp, &itercount);
		/* Compute the current true residual via MKL (Sparse) BLAS routines */
		mkl_dcsrgemv(&cvar, &ivar, A, ia, ja, b, residual);
		dvar=-1.0E0;
		i=1;
		daxpy(&ivar, &dvar, mRHS, &i, residual, &i);
		dvar=dnrm2(&ivar,residual,&i);
		if (dvar<1.0E-3) goto COMPLETE;

		else goto ONE;
	}
	/*---------------------------------------------------------------------------
	/* If RCI_request=3, then apply the preconditioner on the vector
	/* tmp[ipar[21]-1] and put the result in vector tmp[ipar[22]-1]
	/*---------------------------------------------------------------------------
	/* NOTE that ipar[21] and ipar[22] contain FORTRAN style addresses,
	/* therefore, in C code it is required to subtract 1 from them to get C style
	/* addresses
	/* Here is the recommended usage of the result produced by ILU0 routine
	/* via standard MKL Sparse Blas solver routine mkl_dcsrtrsv.
	/*---------------------------------------------------------------------------*/
	if (RCI_request==3)
	{
		cvar1='L';
		cvar='N';
		cvar2='U';
		mkl_dcsrtrsv(&cvar1,&cvar,&cvar2,&ivar,bilu0,ia,ja,&tmp[ipar[21]-1],trvec);
		cvar1='U';
		cvar='N';
		cvar2='N';
		mkl_dcsrtrsv(&cvar1,&cvar,&cvar2,&ivar,bilu0,ia,ja,trvec,&tmp[ipar[22]-1]);
		goto ONE;
	}
	/*---------------------------------------------------------------------------
	/* If RCI_request=4, then check if the norm of the next generated vector is
	/* not zero up to rounding and computational errors. The norm is contained
	/* in dpar[6] parameter
	/*---------------------------------------------------------------------------*/
	if (RCI_request==4)
	{
		if (dpar[6]<1.0E-12) goto COMPLETE;
		else goto ONE;
	}
	/*---------------------------------------------------------------------------
	/* If RCI_request=anything else, then dfgmres subroutine failed
	/* to compute the solution vector: computed_solution[N]
	/*---------------------------------------------------------------------------*/
	else
	{
		goto FAILED;
	}
	/*---------------------------------------------------------------------------
	/* Reverse Communication ends here
	/* Get the current iteration number and the FGMRES solution (DO NOT FORGET to
	/* call dfgmres_get routine as computed_solution is still containing
	/* the initial guess!). Request to dfgmres_get to put the solution
	/* into vector computed_solution[N] via ipar[12]
	/*---------------------------------------------------------------------------*/
COMPLETE:   ipar[12]=0;
	dfgmres_get(&ivar, result, mRHS, &RCI_request, ipar, dpar, tmp, &itercount);
	/*---------------------------------------------------------------------------
	/* Print solution vector: computed_solution[N] and the number of iterations: itercount
	/*---------------------------------------------------------------------------*/

	(*DecoLogger::GetSingleton()) << itercount << " number of iterations used to solve " << N << " unknowns.\n";
	goto END;

FAILED:
	(*DecoLogger::GetSingleton()) << "The solver has returned the ERROR code " << RCI_request << "\n";
	goto END;

FAILED1:
	(*DecoLogger::GetSingleton()) << "-------------------------------------------------------------------\n";
	(*DecoLogger::GetSingleton()) << "Unfortunately, FGMRES+ILU0 C example has FAILED\n";
	(*DecoLogger::GetSingleton()) << "-------------------------------------------------------------------\n";
	goto END;

END:
	delete[] tmp;
	delete[] trvec;
	delete[] bilu0;
	delete[] b;
	delete[] residual;
#endif
}

void PGMRESSolver::calculatePreconditioner()
{

}
