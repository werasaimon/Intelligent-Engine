#include "IRigidBody.h"



namespace IPhysics
{



IRigidBody::IRigidBody(const ITransform &transform, IContactManager *CollideWorld, bodyindex id)
    : ICollisionBody( transform , CollideWorld , id ) ,
      mInitMass(scalar(1.0)),
      mCenterOfMassLocal(0, 0, 0),
      mCenterOfMassWorld(transform.getPosition()),
      mIsGravityEnabled(true),
      mLinearDamping(scalar(0.004)),
      mAngularDamping(scalar(0.004)),
      mLinearVelocity(IVector3(0,0,0)),
      mAngularVelocity(IVector3(0,0,0)),
      mJointsList(NULL)
{
    /// World transform
    mTransform = (transform);

    /// Compute the inverse mass
    mMassInverse = (mInitMass > scalar(0))? scalar(1.0) / mInitMass : scalar(0);

}

IRigidBody::~IRigidBody()
{
    // Remove all the proxy collision shapes of the body
    ICollisionBody::removeAllCollisionShapes();
}

IProxyShape *IRigidBody::addCollisionShape(ICollisionShape *collisionShape, scalar mass, const ITransform &transform)
{
    assert(mass > scalar(0.0));

    // Create a new proxy collision shape to attach the collision shape to the body
    IProxyShape* proxyShape = new IProxyShape(this, collisionShape, transform , mass);


    // Add it to the list of proxy collision shapes of the body
    if (mProxyCollisionShapes == NULL)
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
    collisionShape->computeAABB(aabb, mTransform , transform);

    // Notify the collision detection about this new collision shape
    mCollisionDetection->addProxyCollisionShape(proxyShape, aabb);

    mNbCollisionShapes++;

    // Recompute the center of mass, total mass and inertia tensor of the body with the new
    // collision shape
    recomputeMassInformation();

    // Return a pointer to the proxy collision shape
    return proxyShape;
}

void IRigidBody::removeCollisionShape(const IProxyShape *proxyShape)
{
    // Remove the collision shape
    ICollisionBody::removeCollisionShape(proxyShape);

    // Recompute the total mass, center of mass and inertia tensor
    recomputeMassInformation();
}



void IRigidBody::setType(BodyType type)
{
    //if (mType == type) return;
    ICollisionBody::setType(type);

    mType = type;

    // Recompute the total mass, center of mass and inertia tensor
    recomputeMassInformation();

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
        mInertiaTensorLocal.setToZero();
        mInertiaTensorLocalInverse.setToZero();

    }
    else
    {
        // If it is a dynamic body
        mMassInverse = scalar(1.0) / mInitMass;
        mInertiaTensorLocalInverse = mInertiaTensorLocal.getInverse();
    }

    // Awake the body
    setIsSleeping(false);


    //UpdateMatrices();

    // Remove all the contacts with this body
    resetContactManifoldsList();

    // Ask the broad-phase to test again the collision shapes of the body for collision
    // detection (as if the body has moved)
    askForBroadPhaseCollisionCheck();

    // Reset the force and torque on the body
    mExternalForce=IVector3(0,0,0);
    mExternalTorque=IVector3(0,0,0);

}

void IRigidBody::setIsSleeping(bool isSleeping)
{
    if (isSleeping)
    {
        // Absolutely Stop motion
        mLinearVelocity=IVector3(0,0,0);
        mAngularVelocity=IVector3(0,0,0);
        mExternalForce=IVector3(0,0,0);
        mExternalTorque=IVector3(0,0,0);
    }

    IBody::setIsSleeping(isSleeping);
}

void IRigidBody::updateBroadPhaseStatee(scalar _timeStep) const
{
    //DynamicsWorld& world = static_cast<DynamicsWorld&>(mWorld);
    const IVector3 displacement =  mLinearVelocity * _timeStep;

    // For all the proxy collision shapes of the body
    for (IProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
    {
        // Recompute the world-space AABB of the collision shape
        IAABB aabb;
        shape->getCollisionShape()->computeAABB(aabb, mTransform , shape->getLocalToBodyTransform());

        // Update the broad-phase state for the proxy collision shape
        mCollisionDetection->updateProxyCollisionShape(shape, aabb, displacement);
    }
}



void IRigidBody::removeJointFromJointsList(const IJoint *joint)
{

    assert(joint != NULL);
    assert(mJointsList != NULL);

    // Remove the joint from the linked list of the joints of the first body
    if (mJointsList->getPointer() == joint)
    {   // If the first element is the one to remove
        JointListElement* elementToRemove = mJointsList;
        mJointsList = elementToRemove->getNext();
        delete elementToRemove;
    }
    else
    {  // If the element to remove is not the first one in the list
        JointListElement* currentElement = mJointsList;
        while (currentElement->getNext() != NULL)
        {
            if (currentElement->getNext()->getPointer() == joint)
            {
                JointListElement* elementToRemove = currentElement->getNext();
                currentElement/*->next */ = elementToRemove->getNext();
                delete elementToRemove;
                break;
            }
            currentElement = currentElement->getNext();
        }
    }


}

void IRigidBody::recomputeMassInformation()
{
    mInitMass = scalar(0.0);
    mMassInverse = scalar(0.0);
    mInertiaTensorLocal.setToZero();
    mInertiaTensorLocalInverse.setToZero();
    mCenterOfMassLocal.setToZero();

    // If it is STATIC or KINEMATIC body
    if (mType == STATIC || mType == KINEMATIC)
    {
        mCenterOfMassWorld = mTransform.getPosition();
        return;
    }

    assert(mType == DYNAMIC);

    // Compute the total mass of the body
    for (IProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
    {
        mInitMass += shape->getMass();
        mCenterOfMassLocal += shape->getLocalToBodyTransform().getPosition() * shape->getMass();
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
    for (IProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
    {
        // Get the inertia tensor of the collision shape in its local-space
        IMatrix3x3 inertiaTensor;
        shape->getCollisionShape()->computeLocalInertiaTensor(inertiaTensor, shape->getMass());

        // Convert the collision shape inertia tensor into the local-space of the body
        const ITransform& shapeTransform = shape->getLocalToBodyTransform();
        IMatrix3x3 rotationMatrix = shapeTransform.getBasis();
        inertiaTensor = rotationMatrix * inertiaTensor * rotationMatrix.getTranspose();


        // Use the parallel axis theorem to convert the inertia tensor w.r.t the collision shape
        // center into a inertia tensor w.r.t to the body origin.
        IVector3   offset = shapeTransform.getPosition() - mCenterOfMassLocal;
        scalar     offsetSquare = offset.lengthSquare();
        IMatrix3x3 offsetMatrix;
        offsetMatrix[0].setAllValues(offsetSquare, scalar(0.0), scalar(0.0));
        offsetMatrix[1].setAllValues(scalar(0.0), offsetSquare, scalar(0.0));
        offsetMatrix[2].setAllValues(scalar(0.0), scalar(0.0), offsetSquare);
        offsetMatrix[0] += offset * (-offset.x);
        offsetMatrix[1] += offset * (-offset.y);
        offsetMatrix[2] += offset * (-offset.z);
        offsetMatrix *= shape->getMass();

        mInertiaTensorLocal += inertiaTensor + offsetMatrix;
    }

    // Compute the local inverse inertia tensor
    mInertiaTensorLocalInverse = mInertiaTensorLocal.getInverse();

    // Update the linear velocity of the center of mass
    mLinearVelocity += mAngularVelocity.cross(mCenterOfMassWorld - oldCenterOfMass);

}

void IRigidBody::updateTransformWithCenterOfMass()
{
    // Translate the body according to the translation of the center of mass position
    mTransform.setPosition(mCenterOfMassWorld - mTransform.getBasis() * mCenterOfMassLocal);
}


//============================== Apply =====================================//

void IRigidBody::ApplyForce(const IVector3 &force, const IVector3 &point)
{
    // If it is not a dynamic body, we do nothing
    if (mType != DYNAMIC || mIsSleeping ) return;


    // Add the force
    mExternalForce += mMassInverse * force;
    // Add the torque
    mExternalTorque += getInertiaTensorInverseWorld() * ((point - mCenterOfMassWorld).cross(force));

}

void IRigidBody::ApplyTorque(const IVector3 &torque)
{
    // If it is not a dynamic body, we do nothing
    if (mType != DYNAMIC || mIsSleeping ) return;

    // Add the torque
    mExternalTorque += getInertiaTensorInverseWorld() * torque;
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
    mAngularVelocity += getInertiaTensorInverseWorld() * (point - mCenterOfMassWorld).cross(impuls);
}

void IRigidBody::ApplyImpulseAngular(const IVector3 &impuls)
{
    // If it is not a dynamic body, we do nothing
    if (mType != DYNAMIC || mIsSleeping ) return;

    // Awake the body if it was sleeping
    mAngularVelocity += getInertiaTensorInverseWorld() * impuls;

}

void IRigidBody::ApplyImpulseLinear(const IVector3 &impuls)
{
    // If it is not a dynamic body or sleeping , we do nothing
    if (mType != DYNAMIC || mIsSleeping ) return;

    mLinearVelocity += impuls * mMassInverse;
}

//=======================================================================//


IMaterial IRigidBody::getMaterial() const
{
    return mMaterial;
}

/// Massa of the body
SIMD_INLINE scalar IRigidBody::getMass() const
{
    return mInitMass;// * gamma
}

/// Inverse massa of the body
SIMD_INLINE scalar IRigidBody::getInverseMass() const
{
    return mMassInverse;// * gamma
}



// Return the local inertia tensor of the body (in local-space coordinates)
/**
 * @return The 3x3 inertia tensor matrix of the body (in local-space coordinates)
 */
SIMD_INLINE  IMatrix3x3 IRigidBody::getInertiaTensorLocal() const
{
    return mInertiaTensorLocal;// * gamma
}


// Return the local inertia tensor of the body (in local-space coordinates)
/**
 * @return The 3x3 inverse inertia tensor matrix of the body (in local-space coordinates)
 */
SIMD_INLINE  IMatrix3x3 IRigidBody::getInertiaTensorInverseLocal() const
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
SIMD_INLINE IMatrix3x3 IRigidBody::getInertiaTensorWorld() const
{
   // Compute and return the inertia tensor in world coordinates
   return mTransform.getBasis() * getInertiaTensorLocal() * mTransform.getBasis().getTranspose();
}




SIMD_INLINE IMatrix3x3 IRigidBody::getInertiaTensorInverseWorld() const
{
    // TODO : DO NOT RECOMPUTE THE MATRIX MULTIPLICATION EVERY TIME. WE NEED TO STORE THE
    //        INVERSE WORLD TENSOR IN THE CLASS AND UPLDATE IT WHEN THE ORIENTATION OF THE BODY CHANGES

    // Compute and return the inertia tensor in world coordinates
    return (mTransform.getBasis() * getInertiaTensorInverseLocal() * mTransform.getBasis().getTranspose()) ;
}






}
