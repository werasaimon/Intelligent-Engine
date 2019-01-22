#ifndef ITIMESTEP_H
#define ITIMESTEP_H


#include "../ICommon/IMaths.h"

namespace IPhysics
{

    struct IMovement
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
