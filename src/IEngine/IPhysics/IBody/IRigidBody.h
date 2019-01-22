#ifndef IRIGIDBODY_H
#define IRIGIDBODY_H


#include "ICollisionBody.h"
#include "IMaterial/IMaterial.h"
#include "../ICollision/ICollisionContact/IContactManager.h"
#include "../ICommon/IMemory/IList.h"



namespace IPhysics
{

//// Class declarations
class   IJoint;

///List element joint
typedef IListElement<IJoint> JointListElement;


enum TypeBody { RIGIDBODY_TYPE , RIGIDREALTIVITY_TYPE };


class IRigidBody : public ICollisionBody
{

protected:


    //-------------------- Attributes --------------------//

	TypeBody mTypeBody;


    /// Material properties of the rigid body
    IMaterial mMaterial;


    /// First element of the linked list of joints involving this body
    JointListElement*  mJointsList;



    /// Intial mass of the body
    scalar mInitMass;




    /// Linear velocity damping factor
    scalar mLinearDamping;

    /// Angular velocity damping factor
    scalar mAngularDamping;


	/// Linear velocity of the body
	IVector3 mLinearVelocity;

	/// Angular velocity of the body
	IVector3 mAngularVelocity;


    /// Linear velocity of the body
	IVector3 mSplitLinearVelocity;

    /// Angular velocity of the body
	IVector3 mSplitAngularVelocity;


    /// Current external force on the body
    IVector3 mExternalForce;

    /// Current external torque on the body
    IVector3 mExternalTorque;




    /// Center of mass of the body in local-space coordinates.
    /// The center of mass can therefore be different from the body origin
    IVector3 mCenterOfMassLocal;

    /// Center of mass of the body in world-space coordinates
    IVector3 mCenterOfMassWorld;



    /// Inverse of the mass of the body
    scalar     mMassInverse;


    /// Local inertia tensor of the body (in local-space) with respect to the
    /// center of mass of the body
    IMatrix3x3 mInertiaTensorLocal;

    /// Inverse of the inertia tensor of the body
    IMatrix3x3 mInertiaTensorLocalInverse;

    /// Inertia tensor of the body
    IMatrix3x3 mInertiaTensorWorldInverse;





    /// True if the gravity needs to be applied to this rigid body
    bool mIsGravityEnabled;






    /// Private copy-constructor
    IRigidBody(const  IRigidBody& body);

    /// Private assignment operator
    IRigidBody& operator=(const  IRigidBody& body);



public:

	IRigidBody( const ITransform& transform, IContactManager *ContactManager, bodyindex id , TypeBody _typeBody = TypeBody::RIGIDBODY_TYPE );


    virtual ~IRigidBody();



    /// Add a collision shape to the body.
    virtual IProxyShape *AddCollisionShape(ICollisionShape* collisionShape , scalar mass ,  const ITransform& transform = ITransform::Identity() );

    /// Remove a collision shape from the body
    virtual void RemoveCollisionShape(const IProxyShape* proxyShape);



    //============================== Physics ==================================//

    /// Set the type of the body (static, kinematic or dynamic)
    void SetType(BodyType type);


    /// Set the variable to know whether or not the body is sleeping
    void SetIsSleeping(bool isSleeping);



    /// Update the broad-phase state for this body (because it has moved for instance)
    virtual void UpdateBroadPhaseStatee(scalar _timeStep) const;


    /// remove of the list joints
    virtual void RemoveJointFromJointsList( const IJoint* joint);


    /// Recompute the center of mass, total mass and inertia tensor of the body using all
    /// the collision shapes attached to the body.
    virtual void RecomputeMassInformation();


    /// Update the transform of the body after a change of the center of mass
    void UpdateTransformWithCenterOfMass();


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


    IMaterial GetMaterial() const;

    /// Inertia to of mass
    scalar GetMass() const;
    scalar GetInverseMass() const;


    /// Inertia tensor to Body
    IMatrix3x3  GetInertiaTensorLocal() const;
    IMatrix3x3  GetInertiaTensorInverseLocal() const;


    IMatrix3x3  GetInertiaTensorWorld() const;
    IMatrix3x3  GetInertiaTensorInverseWorld() const;


    IVector3 GetLinearVelocity() const;
    IVector3 GetAngularVelocity() const;


    scalar GetLinearDamping() const;
    scalar GetAngularDamping() const;


    // -------------------- Friendships -------------------- //
	friend class IDynamicsWorldRealtivity;
    friend class IDynamicsWorld;
    friend class IIsland;
    friend class IContactSolver;


    friend class IFixedJoint;
    friend class IBallAndSocketJoint;
    friend class IHingeJoint;
    friend class ISliderJoint;



};


//void IRigidBody::updateBroadPhaseStatee(scalar _timeStep) const
//{
////    //DynamicsWorld& world = static_cast<DynamicsWorld&>(mWorld);
////    const IVector3 displacement =  mLinearVelocity * _timeStep;

////    // For all the proxy collision shapes of the body
////    for (IProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
////    {
////        // Recompute the world-space AABB of the collision shape
////        IAABB aabb;
////        shape->getCollisionShape()->computeAABB(aabb, mTransform , shape->getLocalToBodyTransform());

////        // Update the broad-phase state for the proxy collision shape
////        mCollisionDetection->updateProxyCollisionShape(shape, aabb, displacement);
////    }
//}


SIMD_INLINE IVector3 IRigidBody::GetLinearVelocity() const
{
    return mLinearVelocity;
}

SIMD_INLINE IVector3 IRigidBody::GetAngularVelocity() const
{
    return mAngularVelocity;
}


SIMD_INLINE scalar IRigidBody::GetLinearDamping() const
{
    return mLinearDamping;
}

SIMD_INLINE scalar IRigidBody::GetAngularDamping() const
{
    return mAngularDamping;
}


}

#endif // IRIGIDBODY_H
