#include "stdafx.h"
#include "QuadraticObjective.h"

QPObjective::QPObjective() : mWeight(0), mParentObjective(NULL), mDim(0)
{

}
QPObjective::~QPObjective()
{
	int numChildren = static_cast<int>(mChildObjectives.size());
	for (int i = 0; i < numChildren; ++i)
	{
		delete mChildObjectives[i];
	}
	mChildObjectives.clear();
}

void QPObjective::AddChild(QPObjective* objective, int priority)
{
	if (!objective)
	{
		LOG(WARNING) << "Failed to add a child objective whose value is NULL.";
	}
	objective->mParentObjective = this;
    objective->mPriority = priority;
	mChildObjectives.push_back(objective);
}

const MatrixXd& QPObjective::GetQuadraticTerm()
{
	int numChildren = static_cast<int>(mChildObjectives.size());
	if (numChildren) //intermediate objective node
	{
		mQuadraticTerm = MatrixXd::Zero(mDim, mDim);
		for (int i = 0; i < numChildren; ++i)
		{
			mQuadraticTerm += mChildObjectives[i]->GetWeight() * mChildObjectives[i]->GetQuadraticTerm();
		}
	}
	return mQuadraticTerm;
}
const VectorXd& QPObjective::GetLinearTerm()
{
	int numChildren = static_cast<int>(mChildObjectives.size());
	if (numChildren) //intermediate objective node
	{
		mLinearTerm = VectorXd::Zero(mDim);
		for (int i = 0; i < numChildren; ++i)
		{
			mLinearTerm += mChildObjectives[i]->GetWeight() * mChildObjectives[i]->GetLinearTerm();
		}
	}
	return mLinearTerm;
}

double QPObjective::GetConstantTerm()
{
	int numChildren = static_cast<int>(mChildObjectives.size());
	if (numChildren) //intermediate objective node
	{
		mConstantTerm = 0;
		for (int i = 0; i < numChildren; ++i)
		{
			mConstantTerm += mChildObjectives[i]->GetWeight() * mChildObjectives[i]->GetConstantTerm();
		}
	}
	return mConstantTerm;

}
double QPObjective::GetWeight() const
{
	return mWeight;
}
void QPObjective::SetWeight(double weight)
{
	mWeight = weight;
}

void QPObjective::SetDim(int dim)
{
	mDim = dim;
}

int QPObjective::GetDim() const
{
	return mDim;
}

void QPObjective::SetQuadraticTerm(const MatrixXd& quadratic)
{
	mQuadraticTerm = quadratic;
}
void QPObjective::SetLinearTerm(const VectorXd& linear)
{
	mLinearTerm = linear;
}
void QPObjective::SetConstantTerm(double constant)
{
	mConstantTerm = constant;
}

VectorXd QPObjective::EvaluateGradient(const VectorXd& sol)
{
    VectorXd ret = VectorXd::Zero(sol.size());
    ret = 0.5 * mQuadraticTerm * sol + mLinearTerm;
    return ret;
}

void QPObjective::PadZeros(int numDimToPad)
{
    int numChildren = static_cast<int>(mChildObjectives.size());
    if (numChildren) //intermediate objective node
    {
        for (int i = 0; i < numChildren; ++i)
            mChildObjectives[i]->PadZeros(numDimToPad);
        mDim += numDimToPad;
    }
    else
    {
        int newDim = mDim + numDimToPad;
        MatrixXd newQuadratic = MatrixXd::Zero(newDim, newDim);
        VectorXd newLinear = VectorXd::Zero(newDim);
        MatrixXd newLhs = MatrixXd::Zero(mLhs.rows(), mLhs.cols() + numDimToPad);

        newQuadratic.block(0, 0, mDim, mDim) = mQuadraticTerm;
        newLinear.segment(0, mDim) = mLinearTerm;
        newLhs.block(0, 0, mLhs.rows(), mLhs.cols()) = mLhs;

        mQuadraticTerm = newQuadratic;
        mLinearTerm = newLinear;
        mLhs = newLhs;
        mDim = newDim;
    }
}

void QPObjective::SetLhsAndRhs(const MatrixXd& lhs, const VectorXd& rhs)
{
    CHECK(mChildObjectives.empty());

    mLhs = lhs;
    mRhs = rhs; 

    mQuadraticTerm = lhs.transpose() * lhs;
    mLinearTerm = lhs.transpose() * rhs;
    mConstantTerm = rhs.transpose() * rhs;

    mDim = lhs.cols();
    mWeight = 1.0;
    mParentObjective = NULL;
}


const MatrixXd& QPObjective::GetLhs()
{
    if (mChildObjectives.empty())
        return mLhs;
    else
    {
        int numChildren = static_cast<int>(mChildObjectives.size());
        int numRows = 0;
        int numCols = 0;
        for (int i = 0; i < numChildren; ++i)
        {
            const MatrixXd& childLhs = mChildObjectives[i]->GetLhs();
            numRows += childLhs.rows();
            numCols = childLhs.cols();
        }
        mLhs = MatrixXd::Zero(numRows, numCols);
        int currentRow = 0;
        for (int i = 0; i < numChildren; ++i)
        {
            const MatrixXd& childLhs = mChildObjectives[i]->GetLhs();
            double weight = mChildObjectives[i]->GetWeight();
            mLhs.block(currentRow, 0, childLhs.rows(), numCols) = weight * childLhs;
            currentRow += childLhs.rows();
        }
    }
    return mLhs;
}
const VectorXd& QPObjective::GetRhs()
{
    if (mChildObjectives.empty())
        return mRhs;
    else
    {
        int numChildren = static_cast<int>(mChildObjectives.size());
        int numRows = 0;
        for (int i = 0; i < numChildren; ++i)
        {
            const VectorXd& childRhs = mChildObjectives[i]->GetRhs();
            numRows += childRhs.rows();

        }
        mRhs = VectorXd::Zero(numRows);
        int currentRow = 0;
        for (int i = 0; i < numChildren; ++i)
        {
            const VectorXd& childRhs = mChildObjectives[i]->GetRhs();
            double weight = mChildObjectives[i]->GetWeight();
            mRhs.segment(currentRow, childRhs.rows()) = weight * childRhs;
            currentRow += childRhs.rows();
        }
    }
    return mRhs;
}

const string& QPObjective::GetName() const
{
    return mName;
}

double QPObjective::GetObjectiveFuncValue(const VectorXd& sol, int ithChild)
{
    int numChildren = static_cast<int>(mChildObjectives.size());
    if (ithChild >= numChildren)
        return -1.0;
    else
        return mChildObjectives[ithChild]->OutputObjectiveFuncValues(sol, false);
}

double QPObjective::OutputObjectiveFuncValues(const VectorXd& sol, bool bOutputLog)
{
    if (mChildObjectives.empty())
    {
        VectorXd valuePart1 = mQuadraticTerm * sol;
        double value = sol.dot(valuePart1) + 2 * mLinearTerm.dot(sol) + mConstantTerm;

        if (bOutputLog)
            LOG(INFO) << "Objective " << mName << ": " << value;

        return value;
    }
    else
    {
        int numChildren = static_cast<int>(mChildObjectives.size());
        double value = 0;
        for (int i = 0; i < numChildren; ++i)
        {
            value += mChildObjectives[i]->GetWeight() * mChildObjectives[i]->OutputObjectiveFuncValues(sol);
        }

        if (bOutputLog)
            LOG(INFO) << "Objective " << mName << ": " << value;
        return value;
    }
}

const vector<vector<QPObjective*> >& QPObjective::GetPrioritizedObjectives()
{
    mPrioritizedObjectives.clear();

    vector<QPObjective*> allObjectives;
    appendAllObjectives(allObjectives);

    int numObjectives = static_cast<int>(allObjectives.size());

    int processedCounter = 0;
    while(true)
    {
        if (processedCounter == numObjectives)
            break;
        double minPriority = 1e30;
        double lowThreshold = -1;
        for (int i = 0; i < numObjectives; ++i)
        {
            if (allObjectives[i]->mPriority < minPriority && allObjectives[i]->mPriority > lowThreshold)
            {
                minPriority = allObjectives[i]->mPriority;
            }
        }
        vector<QPObjective*> currentPriorityObjectives;
        for (int i = 0; i < numObjectives; ++i)
        {
            if (allObjectives[i]->mPriority == minPriority)
            {
                currentPriorityObjectives.push_back(allObjectives[i]);
                processedCounter++;
            }
        }
        lowThreshold = minPriority;
        mPrioritizedObjectives.push_back(currentPriorityObjectives);
    }
    return mPrioritizedObjectives;
}

void QPObjective::appendAllObjectives(vector<QPObjective*>& allObjectives)
{
    if (mChildObjectives.empty())
    {
        allObjectives.push_back(this);
    }
    else
    {
        int numChildren = static_cast<int>(mChildObjectives.size());
        double value = 0;
        for (int i = 0; i < numChildren; ++i)
        {
            mChildObjectives[i]->appendAllObjectives(allObjectives);
        }
    }
}


void QPObjective::SetName(const string& name)
{
    mName = name;
}

QPObjective::QPObjective(const QPObjective& rhs)
{

}
const QPObjective& QPObjective::operator= (const QPObjective& rhs)
{
    return *this;
}