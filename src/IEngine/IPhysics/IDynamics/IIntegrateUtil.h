#ifndef IINTEGRATEUTIL_H
#define IINTEGRATEUTIL_H

#include "../ICommon/IMaths.h"

namespace IPhysics
{


    class IIntegrateUtil
    {
        public:

            IIntegrateUtil();

            static  ITransform IntegrateTransform( const ITransform& curTrans , const IVector3& linvel, const IVector3& angvel, scalar timeStep);

            static void Damping( IVector3& Velocity , const scalar& min_damping , const scalar& damping );
    };
}





#endif // IINTEGRATEUTIL_H
