#ifndef _CONE_H
#define _CONE_H

#include "stdafx.h"
#include <Eigen/Dense>
#include <vector>
using namespace Eigen;
using namespace std;

#ifdef  _WIN32
#include <mosek.h>
#else
#define MSKidxt int
#endif

class Cone
{
public:
    Cone();
    ~Cone();
    Cone(const Cone& rhs);
    Cone& operator= (const Cone& rhs);

    MSKidxt* GetMosekConeSubId();

    vector<int> mSubscripts;
private:
    MSKidxt* mMosekSubId;

};

#endif
