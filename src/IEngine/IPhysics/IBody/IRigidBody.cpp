#include "IRigidBody.h"



namespace IPhysics
{


	namespace
	{
		/// Helper function for daping vector*s
		////////////////////////////////////////////////////////////////
		/// Helper function for daping vector*s
		static void Damping( IVector3& Velocity , const scalar& min_damping , const scalar& damping )
		{
			if( Velocity.LengthSquare()  < min_damping )
			{
				if( Velocity.Length() > damping )
				{
					Velocity -= ( Velocity.GetUnit() * damping);
				}
				else
				{
					Velocity  = IVector3(0,0,0);
				}
			}
		}
		////////////////////////////////////////////////////////////////

	}

IRigidBody::IRigidBody(const ITransform &transform, IContactManager *ContactManager, bodyindex id, TypeBody _typeBody)
: ICollisionBody( transform , ContactManager , id ) ,
mInitMass(scalar(1.0)),
mCenterOfMassLocal(0, 0, 0),
mCenterOfMassWorld(transform.GetPosition()),
mIsGravityEnabled(true),
mLinearDamping(scalar(0.004)),
mAngularDamping(scalar(0.004)),
mLinearVelocity(IVector3(0,0,0)),
mAngularVelocity(IVector3(0,0,0)),
mJointsList(nullptr),
mTypeBody(_typeBody)
{
    /// World transform
    mTransform = (transform);

    /// Compute the inverse mass
    mMassInverse = (mInitMass > scalar(0))? scalar(1.0) / mInitMass : scalar(0);

}

IRigidBody::~IRigidBody()
{
    // Remove all the proxy collision shapes of the body
    ICollisionBody::RemoveAllCollisionShapes();
}

IProxyShape *IRigidBody::AddCollisionShape(ICollisionShape *collisionShape, scalar mass, const ITransform &transform)
{
    assert(mass > scalar(0.0));

    // Create a new proxy collision shape to attach the collision shape to the body
    IProxyShape* proxyShape = new IProxyShape(this, collisionShape, transform , mass);


    // Add it to the list of proxy collision shapes of the body
    if (mProxyCollisionShapes == nullptr)
    {
        mProxyCollisionShapes = proxyShape;
    }
    else
    {
        proxyShape->mNext = mProxyCollisionShapes;
        mProxyCollisionShapes = proxyShape;
    }

    // Compute the world-space AABB of the new collision shape
    IAABB aabb;
    collisionShape->ComputeAABB(aabb, mTransform , transform);

    // Notify the collision detection about this new collision shape
    mCollisionDetection->AddProxyCollisionShape(proxyShape, aabb);

    mNbCollisionShapes++;

    // Recompute the center of mass, total mass and inertia tensor of the body with the new
    // collision shape
    RecomputeMassInformation();

    // Return a pointer to the proxy collision shape
    return proxyShape;
}

void IRigidBody::RemoveCollisionShape(const IProxyShape *proxyShape)
{
    // Remove the collision shape
    ICollisionBody::RemoveCollisionShape(proxyShape);

    // Recompute the total mass, center of mass and inertia tensor
    RecomputeMassInformation();
}


void IRigidBody::UpdateTransformWithCenterOfMass()
{
	// Translate the body according to the translation of the center of mass position
	mTransform.SetPosition(mCenterOfMassWorld - mTransform.GetBasis() * mCenterOfMassLocal);
}



void IRigidBody::SetType(BodyType type)
{
	//if (mType == type) return;
	ICollisionBody::SetType(type);

	mType = type;

	// Recompute the total mass, center of mass and inertia tensor
	RecomputeMassInformation();

	// If it is a static body
	if (mType == STATIC)
	{
		// Reset the velocity to zero
		mLinearVelocity=IVector3(0,0,0);
		mAngularVelocity=IVector3(0,0,0);
	}

	// If it is a static or a kinematic body
	if (mType == STATIC || mType == KINEMATIC)
	{
		// Reset the inverse mass and inverse inertia tensor to zero
		mMassInverse = scalar(0.0);
		mInertiaTensorLocal.SetToZero();
		mInertiaTensorLocalInverse.SetToZero();

	}
	else
	{
		// If it is a dynamic body
		mMassInverse = scalar(1.0) / mInitMass;
		mInertiaTensorLocalInverse = mInertiaTensorLocal.GetInverse();
	}

	// Awake the body
	SetIsSleeping(false);


	//UpdateMatrices();

	// Remove all the contacts with this body
	ResetContactManifoldsList();

	// Ask the broad-phase to test again the collision shapes of the body for collision
	// detection (as if the body has moved)
	AskForBroadPhaseCollisionCheck();

	// Reset the force and torque on the body
	mExternalForce=IVector3(0,0,0);
	mExternalTorque=IVector3(0,0,0);

}

void IRigidBody::SetIsSleeping(bool isSleeping)
{
	if (isSleeping)
	{
		// Absolutely Stop motion
		mLinearVelocity=IVector3(0,0,0);
		mAngularVelocity=IVector3(0,0,0);
		mExternalForce=IVector3(0,0,0);
		mExternalTorque=IVector3(0,0,0);
	}

	IBody::SetIsSleeping(isSleeping);
}

void IRigidBody::UpdateBroadPhaseStatee(scalar _timeStep) const
{
	//DynamicsWorld& world = static_cast<DynamicsWorld&>(mWorld);
	const IVector3 displacement =  mLinearVelocity * _timeStep;

	// For all the proxy collision shapes of the body
	for (IProxyShape* shape = mProxyCollisionShapes; shape != nullptr; shape = shape->mNext)
	{
		// Recompute the world-space AABB of the collision shape
		IAABB aabb;
		shape->GetCollisionShape()->ComputeAABB(aabb, mTransform , shape->GetLocalToBodyTransform());

		// Update the broad-phase state for the proxy collision shape
		mCollisionDetection->UpdateProxyCollisionShape(shape, aabb, displacement);
	}
}



void IRigidBody::RemoveJointFromJointsList(const IJoint *joint)
{

	assert(joint != NULL);
	assert(mJointsList != NULL);

	// Remove the joint from the linked list of the joints of the first body
	if (mJointsList->GetPointer() == joint)
	{   // If the first element is the one to remove
		JointListElement* elementToRemove = mJointsList;
		mJointsList = elementToRemove->GetNext();
		delete elementToRemove;
	}
	else
	{  // If the element to remove is not the first one in the list
		JointListElement* currentElement = mJointsList;
		while (currentElement->GetNext() != nullptr)
		{
			if (currentElement->GetNext()->GetPointer() == joint)
			{
				JointListElement* elementToRemove = currentElement->GetNext();
                currentElement/*->next */ = elementToRemove->GetNext();
                delete elementToRemove;
                break;
            }
            currentElement = currentElement->GetNext();
        }
    }


}

void IRigidBody::RecomputeMassInformation()
{
    mInitMass = scalar(0.0);
    mMassInverse = scalar(0.0);
    mInertiaTensorLocal.SetToZero();
    mInertiaTensorLocalInverse.SetToZero();
    mCenterOfMassLocal.SetToZero();

    // If it is STATIC or KINEMATIC body
    if (mType == STATIC || mType == KINEMATIC)
    {
        mCenterOfMassWorld = mTransform.GetPosition();
        return;
    }

    assert(mType == DYNAMIC);

    // Compute the total mass of the body
    for (IProxyShape* shape = mProxyCollisionShapes; shape != nullptr; shape = shape->mNext)
    {
        mInitMass += shape->GetMass();
        mCenterOfMassLocal += shape->GetLocalToBodyTransform().GetPosition() * shape->GetMass();
    }

    if (mInitMass > scalar(0.0))
    {
        mMassInverse = scalar(1.0) / mInitMass;
    }
    else
    {
        mInitMass = scalar(1.0);
        mMassInverse = scalar(1.0);
    }

    // Compute the center of mass
    const IVector3 oldCenterOfMass = mCenterOfMassWorld;
    mCenterOfMassLocal *= mMassInverse;
    mCenterOfMassWorld  =  mTransform * mCenterOfMassLocal;

    // Compute the total mass and inertia tensor using all the collision shapes
    for (IProxyShape* shape = mProxyCollisionShapes; shape != nullptr; shape = shape->mNext)
    {
        // Get the inertia tensor of the collision shape in its local-space
        IMatrix3x3 inertiaTensor;
        shape->GetCollisionShape()->ComputeLocalInertiaTensor(inertiaTensor, shape->GetMass());

        // Convert the collision shape inertia tensor into the local-space of the body
        const ITransform& shapeTransform = shape->GetLocalToBodyTransform();
        IMatrix3x3 rotationMatrix = shapeTransform.GetBasis();
        inertiaTensor = rotationMatrix * inertiaTensor * rotationMatrix.GetTranspose();


        // Use the parallel axis theorem to convert the inertia tensor w.r.t the collision shape
        // center into a inertia tensor w.r.t to the body origin.
        IVector3   offset = shapeTransform.GetPosition() - mCenterOfMassLocal;
        scalar     offsetSquare = offset.LengthSquare();
        IMatrix3x3 offsetMatrix;
        offsetMatrix[0].SetAllValues(offsetSquare, scalar(0.0), scalar(0.0));
        offsetMatrix[1].SetAllValues(scalar(0.0), offsetSquare, scalar(0.0));
        offsetMatrix[2].SetAllValues(scalar(0.0), scalar(0.0), offsetSquare);
        offsetMatrix[0] += offset * (-offset.x);
        offsetMatrix[1] += offset * (-offset.y);
        offsetMatrix[2] += offset * (-offset.z);
        offsetMatrix *= shape->GetMass();

        mInertiaTensorLocal += inertiaTensor + offsetMatrix;
    }

    // Compute the local inverse inertia tensor
    mInertiaTensorLocalInverse = mInertiaTensorLocal.GetInverse();

    // Update the linear velocity of the center of mass
    mLinearVelocity += mAngularVelocity.Cross(mCenterOfMassWorld - oldCenterOfMass);

}




//============================== Apply =====================================//

void IRigidBody::ApplyForce(const IVector3 &force, const IVector3 &point)
{
    // If it is not a dynamic body, we do nothing
    if (mType != DYNAMIC || mIsSleeping ) return;


    // Add the force
    mExternalForce += mMassInverse * force;
    // Add the torque
    mExternalTorque += GetInertiaTensorInverseWorld() * ((point - mCenterOfMassWorld).Cross(force));

}

void IRigidBody::ApplyTorque(const IVector3 &torque)
{
    // If it is not a dynamic body, we do nothing
    if (mType != DYNAMIC || mIsSleeping ) return;

    // Add the torque
    mExternalTorque += GetInertiaTensorInverseWorld() * torque;
}

void IRigidBody::ApplyForceToCenterOfMass(const IVector3 &force)
{
    // If it is not a dynamic body, we do nothing
    if (mType != DYNAMIC || mIsSleeping ) return;

    // Add the force
    mExternalForce += mMassInverse * force;
}

void IRigidBody::ApplyImpulse(const IVector3 &impuls, const IVector3 &point)
{
    // If it is not a dynamic body, we do nothing
    if (mType != DYNAMIC || mIsSleeping ) return;

    mLinearVelocity  += mMassInverse * impuls;
    mAngularVelocity += GetInertiaTensorInverseWorld() * (point - mCenterOfMassWorld).Cross(impuls);
}

void IRigidBody::ApplyImpulseAngular(const IVector3 &impuls)
{
    // If it is not a dynamic body, we do nothing
    if (mType != DYNAMIC || mIsSleeping ) return;

    // Awake the body if it was sleeping
    mAngularVelocity += GetInertiaTensorInverseWorld() * impuls;

}

void IRigidBody::ApplyImpulseLinear(const IVector3 &impuls)
{
    // If it is not a dynamic body or sleeping , we do nothing
    if (mType != DYNAMIC || mIsSleeping ) return;

    mLinearVelocity += impuls * mMassInverse;
}

//=======================================================================//


IMaterial IRigidBody::GetMaterial() const
{
    return mMaterial;
}

/// Massa of the body
SIMD_INLINE scalar IRigidBody::GetMass() const
{
    return mInitMass;// * gamma
}

/// Inverse massa of the body
SIMD_INLINE scalar IRigidBody::GetInverseMass() const
{
    return mMassInverse;// * gamma
}



// Return the local inertia tensor of the body (in local-space coordinates)
/**
 * @return The 3x3 inertia tensor matrix of the body (in local-space coordinates)
 */
SIMD_INLINE  IMatrix3x3 IRigidBody::GetInertiaTensorLocal() const
{
    return mInertiaTensorLocal;// * gamma
}


// Return the local inertia tensor of the body (in local-space coordinates)
/**
 * @return The 3x3 inverse inertia tensor matrix of the body (in local-space coordinates)
 */
SIMD_INLINE  IMatrix3x3 IRigidBody::GetInertiaTensorInverseLocal() const
{
    return mInertiaTensorLocalInverse;// * gamma
}



// Return the inertia tensor in world coordinates.
/// The inertia tensor I_w in world coordinates is computed
/// with the local inertia tensor I_b in body coordinates
/// by I_w = R * I_b * R^T
/// where R is the rotation matrix (and R^T its transpose) of
/// the current orientation quaternion of the body
/**
 * @return The 3x3 inertia tensor matrix of the body in world-space coordinates
 */
SIMD_INLINE IMatrix3x3 IRigidBody::GetInertiaTensorWorld() const
{
   // Compute and return the inertia tensor in world coordinates
   return mTransform.GetBasis() * GetInertiaTensorLocal() * mTransform.GetBasis().GetTranspose();
}




/*SIMD_INLINE */IMatrix3x3 IRigidBody::GetInertiaTensorInverseWorld() const
{
    // TODO : DO NOT RECOMPUTE THE MATRIX MULTIPLICATION EVERY TIME. WE NEED TO STORE THE
    //        INVERSE WORLD TENSOR IN THE CLASS AND UPLDATE IT WHEN THE ORIENTATION OF THE BODY CHANGES

    // Compute and return the inertia tensor in world coordinates
    return (mTransform.GetBasis() * GetInertiaTensorInverseLocal() * mTransform.GetBasis().GetTranspose());
}






}
