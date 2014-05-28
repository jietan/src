#include "stdafx.h"
#include "PrioritizedQPSolver.h"
#include "EigenSerializer.h"
#include "mathlib.h"

PrioritizedQPSolver::~PrioritizedQPSolver()
{

}

bool PrioritizedQPSolver::Solve(VectorXd& sol)
{
    bool bSucceed = true;
    bSucceed &= preSolve();
    bSucceed &= solve();
    bSucceed &= postSolve();
    if (bSucceed)
    {
        sol = mSol; 
    }
    return bSucceed;

    //bool equalityExist = GetEqualityConstraints(mAEquality, mbEquality);
    //if (equalityExist)
    //{
    //    if (!reparameterization())
    //    {
    //        LOG(WARNING) << "Equality constraints cannot be satisfied.";
    //    }
    //}
    //bool ret = QPSolver::Solve(sol);
    //if (ret && equalityExist)
    //    reassemble(sol);
    //return ret;
}

void PrioritizedQPSolver::initializeObjectiveList()
{
    mObjectives.clear();
    mNeedDeleteIdx.clear();

    bool equalityExist = GetEqualityConstraints(mAEquality, mbEquality);
    if (equalityExist)
    {
        QPObjective* equalityObjective = new QPObjective;
        //EigenSerializer::SaveMatrixToFile("mA.txt", mAEquality, true);
        equalityObjective->SetLhsAndRhs(mAEquality, -mbEquality);
        equalityObjective->SetName("EqualityConstraint");
        mObjectives.push_back(equalityObjective);
        mNeedDeleteIdx.push_back(static_cast<int>(mObjectives.size()) - 1);
    }
    bool inequalityExist = GetInequalityConstraints(mAInEquality, mlbcInEquality, mubcInEquality, mbGreaterThan, mbLessThan);
    CHECK(mObjective);
    const vector<vector<QPObjective*> >& priorityObjectiveList = mObjective->GetPrioritizedObjectives();
    int numPriorityLevels = static_cast<int>(priorityObjectiveList.size());
    for (int i = 0; i < numPriorityLevels; ++i)
    {
        const vector<QPObjective*> currentLevelObjs = priorityObjectiveList[i];
        CHECK(!currentLevelObjs.empty());
        if (currentLevelObjs.size() == 1)
        {
            mObjectives.push_back(currentLevelObjs[0]);
        }
        else
        {
            QPObjective* combinedObj = combineObjectives(currentLevelObjs);
            mObjectives.push_back(combinedObj);
            mNeedDeleteIdx.push_back(static_cast<int>(mObjectives.size()) - 1);
        }
    }

}

QPObjective* PrioritizedQPSolver::combineObjectives(const vector<QPObjective*>& objectivesToCombine)
{
    QPObjective* ret = new QPObjective;
    string name = "Combined";
    int numToCombine = static_cast<int>(objectivesToCombine.size());

    int numRows = 0;
    int numCols = objectivesToCombine[0]->GetDim();
    for (int i = 0; i < numToCombine; ++i)
    {
        name += objectivesToCombine[i]->GetName();
        numRows += objectivesToCombine[i]->GetLhs().rows();
    }
    ret->SetName(name);
    MatrixXd newLhs = MatrixXd::Zero(numRows, numCols);
    VectorXd newRhs = VectorXd::Zero(numRows);

    int currentRowNum = 0;
    for (int i = 0; i < numToCombine; ++i)
    {
        const MatrixXd& lhs = objectivesToCombine[i]->GetLhs();
        const VectorXd& rhs = objectivesToCombine[i]->GetRhs();
        double weight = objectivesToCombine[i]->GetWeight();

        newLhs.block(currentRowNum, 0, lhs.rows(), lhs.cols()) = weight * lhs;
        newRhs.segment(currentRowNum, rhs.rows()) = weight * rhs;

        currentRowNum += rhs.rows();
    }
    ret->SetLhsAndRhs(newLhs, newRhs);
    return ret;
}

bool PrioritizedQPSolver::preSolve()
{
    initializeObjectiveList();
    int numObjectives = static_cast<int>(mObjectives.size());
    if (!numObjectives)
    {
        LOG(WARNING) << "Zero objectives in PriorizedQPSolver.";
        return false;
    }
    int dim = mObjectives[0]->GetDim();
    mCBar = MatrixXd::Identity(dim, dim);
    mdBar = VectorXd::Zero(dim);
    return true;
}

bool PrioritizedQPSolver::solve()
{
    int numObjectives = static_cast<int>(mObjectives.size());
    for (int i = 0; i < numObjectives; ++i)
    {
        const MatrixXd& A = mObjectives[i]->GetLhs();
        MatrixXd Abar = A * mCBar;
        VectorXd bBar = mObjectives[i]->GetRhs() +  A * mdBar;
        QPObjective newObjective;
        newObjective.SetLhsAndRhs(Abar, bBar);
        newObjective.SetName("Reparameterized " + mObjectives[i]->GetName());

        QPSolver solver;
        solver.SetObjective(newObjective);
        LinearConstraint constraint;
        MatrixXd newAInEquality = mAInEquality * mCBar;
        VectorXd newlbcInEquality = mlbcInEquality - mAInEquality * mdBar;
        VectorXd newubcInEquality = mubcInEquality - mAInEquality * mdBar;
        VectorXi newGreaterThan = mbGreaterThan;
        VectorXi newLessThan = mbLessThan;

        //int constraintStart = 0;
        //int constraintEnd = 1;

        //MatrixXd newAInEquality1 = newAInEquality.block(constraintStart, 0, constraintEnd - constraintStart, newAInEquality.cols());
        //VectorXd newlbcInEquality1 = newlbcInEquality.segment(constraintStart, constraintEnd - constraintStart);
        //VectorXd newubcInEquality1 = newubcInEquality.segment(constraintStart, constraintEnd - constraintStart);
        //VectorXi newGreaterThan1 = newGreaterThan.segment(constraintStart, constraintEnd - constraintStart);
        //VectorXi newLessThan1 = newLessThan.segment(constraintStart, constraintEnd - constraintStart);

        //EigenSerializer::SaveMatrixToFile("Abar.txt", Abar, true);
        //EigenSerializer::SaveVectorToFile("bBar.txt", bBar, true);
        //EigenSerializer::SaveMatrixToFile("newAInEquality1.txt", newAInEquality1, true);
        //EigenSerializer::SaveVectorToFile("newlbcInEquality1.txt", newlbcInEquality1, true);
        //EigenSerializer::SaveVectorToFile("newubcInEquality1.txt", newubcInEquality1, true);

        //constraint.Set(newAInEquality1, newlbcInEquality1, newGreaterThan1, newubcInEquality1, newLessThan1);
        constraint.Set(newAInEquality, newlbcInEquality, newGreaterThan, newubcInEquality, newLessThan);
        solver.SetConstraints(constraint);

        char fileName[256];
        memset(fileName, 0, 256 * sizeof(char));
        sprintf(fileName, "../../../problem/Problem%d", i);
        solver.DumpProblem(fileName);

        VectorXd d;
        if (!solver.Solve(d, SM_Cone))
        {
            mSol = mdBar;
            LOG(WARNING) << "Problem infeasible at " << i << "th objective: " << mObjectives[i]->GetName();
            return true;
        }


        double value = newObjective.OutputObjectiveFuncValues(d);
        LOG(INFO) << "The optimal value of " << i << "th objective is: " << value;
        mdBar += mCBar * d;
        MatrixXd nullSpace;
        bool isNullSpaceExist = getNullSpace(Abar, nullSpace);
        LOG(INFO) << "Null space dimension after " << i << "th objective: " << nullSpace.cols();
        if (!isNullSpaceExist)
        {
            mSol = mdBar;
            LOG(WARNING) << "A is full rank at " << i << "th objective: " << mObjectives[i]->GetName();
            return true;
        }
        mCBar *= nullSpace;
    }
    mSol = mdBar;
    return true;
}

bool PrioritizedQPSolver::postSolve()
{
    int numObjectiveToDelete = static_cast<int>(mNeedDeleteIdx.size());
    for (int i = 0; i < numObjectiveToDelete; ++i)
    {
        delete mObjectives[mNeedDeleteIdx[i]];
    }
    return true;
}

bool PrioritizedQPSolver::getNullSpace(const MatrixXd& A, MatrixXd& nullSpace)
{
    JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeFullV);
    MatrixXd U = svd.matrixU();
    MatrixXd V = svd.matrixV();
    VectorXd singularValues = svd.singularValues();

    int ithCol = singularValues.size();
    for (int i = 0; i < singularValues.size(); ++i)
    {
        if (NearlyEqual(singularValues[i], 0))
        //if (singularValues[i] == 0)
        {
            if (ithCol == singularValues.size())
                ithCol = i;
        }
    }
    if (ithCol == V.cols())
        return false;
    else
    {
        nullSpace = V.block(0, ithCol, V.rows(), V.cols() - ithCol);
        MatrixXd zeroMat = A * nullSpace;
        for (int i = 0; i < zeroMat.rows(); ++i)
            for (int j = 0; j < zeroMat.cols(); ++j)
            {
                if (!NearlyEqual(zeroMat(i, j), 0))
                    LOG(WARNING) << "Null space not null: " << i << ", " << j << ": " << zeroMat(i, j);
            }
        JacobiSVD<MatrixXd> svd1(nullSpace, ComputeThinU | ComputeThinV);
        VectorXd singularValues2 = svd1.singularValues();

        for (int i = 0; i < singularValues2.size(); ++i)
        {
            if (NearlyEqual(singularValues2[i], 0))
            {
                LOG(WARNING) << "V only has zero singular values.";
            }
        }
        return true;
    }
}

bool PrioritizedQPSolver::reparameterization()
{
    JacobiSVD<MatrixXd> svd(mAEquality, ComputeThinU | ComputeThinV);
    MatrixXd U = svd.matrixU();
    MatrixXd V = svd.matrixV();
    VectorXd singularValues = svd.singularValues();
    MatrixXd sigmaInv = MatrixXd::Zero(singularValues.size(), singularValues.size());
    int ithCol = -1;
    for (int i = 0; i < singularValues.size(); ++i)
    {
        if (singularValues[i] < EPSILON_FLOAT)
        {
            if (ithCol == -1)
                ithCol = i;
            sigmaInv(i, i) = 0;
        }
        else
        {
            sigmaInv(i, i) = 1.0 / singularValues[i];
        }
    }
    mMinimizer = V * (sigmaInv * (U.transpose() * mbEquality));


    bool ret = false;
    VectorXd verification = mAEquality * mMinimizer - mbEquality;

    if (verification.norm() < EPSILON_FLOAT)
        ret = true;

    if (ithCol != -1)
    {
        mNullSpace = V.block(0, ithCol, V.rows(), V.cols() - ithCol);
    }
    else
    {
        return false; // no null space to explore...
    }

    bool inequalityExist = GetInequalityConstraints(mAInEquality, mlbcInEquality, mubcInEquality, mbGreaterThan, mbLessThan);
    ClearConstraints();

    MatrixXd originalQ = mQ;
    VectorXd originalc = mc;
    SetQuadraticTerm(mNullSpace.transpose() * originalQ * mNullSpace);
    SetLinearTerm(mNullSpace.transpose() * (originalQ * mMinimizer + originalc));

    if (inequalityExist)
    {
        mlbcInEquality -= mAInEquality * mMinimizer;
        mubcInEquality -= mAInEquality * mMinimizer;
        mAInEquality = mAInEquality * mNullSpace;
        SetConstraints(mAInEquality, mlbcInEquality, mbGreaterThan, mubcInEquality, mbLessThan);
    }
    return ret;
}

void PrioritizedQPSolver::reassemble(VectorXd& sol)
{
    sol = mMinimizer + mNullSpace * sol;
}
