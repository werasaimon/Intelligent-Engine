#ifndef IREALTIVITYWORLD_H
#define IREALTIVITYWORLD_H


#include "IDynamicsWorld.h"
#include "../IBody/IRigidRealtivity.h"

namespace IPhysics
{

    class IDynamicsWorldRealtivity : public IDynamicsWorld
    {
            //-------------------- Attributes --------------------//

            const scalar mLightSpeed;

            IIndependentObserver *mObserverSysytem;// = nullptr;



            // -------------------- Methods -------------------- //
            /// Private copy-constructor
            IDynamicsWorldRealtivity(const IDynamicsWorldRealtivity& world);

            /// Private assignment operator
            IDynamicsWorldRealtivity& operator=(const IDynamicsWorldRealtivity& world);



        public:

            IDynamicsWorldRealtivity(const IVector3& gravity , IIndependentObserver *_ObserverSysytem , const scalar _LightSpeed = DEFAUL_LIGHT_MAX_VELOCITY_C);


            virtual ~IDynamicsWorldRealtivity();

            /// Create a rigid body into the physics world.
            IRigidRealtivity* CreateRigidBody(const ITransform& transform);


            /// Integrate the Velocity .
            virtual void IntegrateVelocities(scalar TimeStep);


            /// Integrate the motion .
            virtual void IntegratePositions(scalar TimeStep);


            virtual void UpdateBodiesState( scalar TimeStep );



    };
}



#endif // IREALTIVITYWORLD_H
