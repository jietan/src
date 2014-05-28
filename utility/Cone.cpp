#include "stdafx.h"
#include "Cone.h"


Cone::Cone() : mMosekSubId(NULL)
{

}
Cone::~Cone()
{
    if (mMosekSubId)
        delete[] mMosekSubId;
}
Cone::Cone(const Cone& rhs)
{
    mSubscripts = rhs.mSubscripts;
    mMosekSubId = NULL;
}

Cone& Cone::operator= (const Cone& rhs)
{
    if (this == &rhs)
        return *this;

    mSubscripts = rhs.mSubscripts;
    if (mMosekSubId)
        delete[] mMosekSubId;
    mMosekSubId = NULL;
    return *this;
}
MSKidxt* Cone::GetMosekConeSubId()
{
    CHECK(!mSubscripts.empty());

    if (mMosekSubId)
        delete[] mMosekSubId;
    int numSubscripts = static_cast<int>(mSubscripts.size());
    mMosekSubId = new MSKidxt[numSubscripts];

    for (int i = 0; i < numSubscripts; ++i)
    {
        mMosekSubId[i] = mSubscripts[i];
    }
    return mMosekSubId;
}
