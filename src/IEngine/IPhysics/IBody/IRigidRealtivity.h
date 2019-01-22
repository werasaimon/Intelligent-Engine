#ifndef IRIGIDREALTIVITY_H
#define IRIGIDREALTIVITY_H


#include "IRigidBody.h"


namespace IPhysics
{


	/// Loretz factor
	static scalar GammaFactor(const IVector3& v , const scalar vSpeeLigth )
	{
	   float gamma = sqrt(1.0 - v.Dot(v)/(vSpeeLigth*vSpeeLigth));
	   gamma = (gamma > MACHINE_EPSILON) ? gamma : MACHINE_EPSILON;
	   return gamma;
	}

        class IIndependentObserver
	{

            public:

		IVector3 pos;
		IVector3 vel;

                IIndependentObserver(const IVector3 _pos , const IVector3 _vel )
                    : pos(_pos) ,
                      vel(_vel)
                {

                }


                IIndependentObserver(IIndependentObserver* src)
                    : pos(src->pos),
                      vel(src->vel)
                {
                }

                void Derivative()
                {
                    //                    pos = IVector3(0,0,0);
                    //                    vel = IVector3(0,0,0);
                }

	};


	class IRigidRealtivity : public IRigidBody
	{
		private:

                //-------------------- Attributes --------------------//

                const IIndependentObserver *mObserverSysytem = nullptr;

                const scalar mLightSpeed;

                scalar mGammaL;
                scalar mGammaR;
                scalar mSumGamma;

                IMatrix3x3 mLoretzMatrix;

                //-------------------- method ---------------------//

                /// Private copy-constructor
                IRigidRealtivity(const  IRigidRealtivity& body);

                /// Private assignment operator
                IRigidRealtivity& operator=(const  IRigidRealtivity& body);


		public:

                IRigidRealtivity(const ITransform& transform, IContactManager *ContactMeneger, bodyindex id ,
                                 const IIndependentObserver *_ObserverSysytem , const scalar _LightSpeed = DEFAUL_LIGHT_MAX_VELOCITY_C);


                void UpdateInterpolationRealtivityAttributes();

                //============================== Apply =====================================//

                /// apply force
                virtual void ApplyForce(const IVector3& force, const IVector3& point);
                virtual void ApplyTorque(const IVector3& torque);
                virtual void ApplyForceToCenterOfMass(const IVector3& force);



                /// apply impulse
                virtual void ApplyImpulse(const IVector3& impuls , const IVector3& point);
                virtual void ApplyImpulseAngular(const IVector3&  impuls );
                virtual void ApplyImpulseLinear(const IVector3&  impuls );

                //==========================================================================//
        };

}

#endif // IRIGIDREALTIVITY_H
