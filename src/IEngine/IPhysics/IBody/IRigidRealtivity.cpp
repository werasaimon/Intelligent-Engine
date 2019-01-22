#include "IRigidRealtivity.h"


namespace IPhysics
{



	IRigidRealtivity::IRigidRealtivity(const ITransform &transform, IContactManager *ContactMeneger, bodyindex id,
									   const IIndependentObserver *_ObserverSysytem,
									   const scalar _LightSpeed)
	: IRigidBody(transform, ContactMeneger, id , TypeBody::RIGIDREALTIVITY_TYPE) ,
	  mObserverSysytem(_ObserverSysytem),
	  mLightSpeed(_LightSpeed),
	  mGammaL(0),
	  mGammaR(0),
	  mSumGamma(0)
	{
		mLoretzMatrix.SetToIdentity();
	}


	void IRigidRealtivity::UpdateInterpolationRealtivityAttributes()
	{
		mGammaL = GammaFactor(mObserverSysytem->vel - mLinearVelocity  , mLightSpeed);
		mGammaR = GammaFactor(mAngularVelocity , mLightSpeed / (M_PI * 2));
	}

	void IRigidRealtivity::ApplyForce(const IVector3 &force, const IVector3 &point)
	{

	}

	void IRigidRealtivity::ApplyTorque(const IVector3 &torque)
	{

	}

	void IRigidRealtivity::ApplyForceToCenterOfMass(const IVector3 &force)
	{

	}

	void IRigidRealtivity::ApplyImpulse(const IVector3 &impuls, const IVector3 &point)
	{
		// If it is not a dynamic body, we do nothing
		if (mType != DYNAMIC || mIsSleeping ) return;

		 /**
		 scalar maxSpeedR = mLightSpeed / ((point - mCenterOfMassWorld).Cross(impuls)).Length();

		 scalar R = (1.0 + ((impuls).Dot(mAngularVelocity)/(maxSpeedR*maxSpeedR)));
		 scalar L = (1.0 + ((impuls).Dot(mLinearVelocity)/(mLightSpeed*mLightSpeed)));

		 IVector3 impulsL =  (mMassInverse * impuls);
		 IVector3 impulsR =  ((GetInertiaTensorInverseWorld() * (point - mCenterOfMassWorld).Cross(impuls)));

		 mLinearVelocity  += impulsL / L;
		 mAngularVelocity += impulsR / R;
		 **/

		ApplyImpulseLinear(impuls);
		ApplyImpulseAngular((point - mCenterOfMassWorld).Cross(impuls));

	}

	void IRigidRealtivity::ApplyImpulseAngular(const IVector3 &impuls)
	{
		// If it is not a dynamic body or sleeping , we do nothing
		if (mType != DYNAMIC || mIsSleeping ) return;

		// scalar maxSpeedR = (mLightSpeed / 2.0 * M_PI);
		// scalar R = (1.0 + ((impuls).Dot(mAngularVelocity)/(maxSpeedR*maxSpeedR)));

		 mAngularVelocity += (GetInertiaTensorInverseWorld() * impuls);/// R;
	}

	void IRigidRealtivity::ApplyImpulseLinear(const IVector3 &impuls)
	{
		// If it is not a dynamic body or sleeping , we do nothing
		if (mType != DYNAMIC || mIsSleeping ) return;

		 mLinearVelocity += (mMassInverse * impuls) / (1.0 + ((impuls).Dot(mLinearVelocity)/(mLightSpeed*mLightSpeed)));;
	}




}
