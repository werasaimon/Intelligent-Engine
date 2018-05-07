#ifndef ITIMESTEP_H
#define ITIMESTEP_H


#include "../common/math/IMatematical.h"

namespace IPhysics
{

struct ILocationsAndRotation
{
    IVector3    x;
    IQuaternion q;
};


struct IVelocity
{
   IVector3 v;
   IVector3 w;
};

}


#endif // ITIMESTEP_H
