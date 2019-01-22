#include "IProxyShape.h"

namespace IPhysics
{

// Constructor
/**
 * @param body Pointer to the parent body
 * @param shape Pointer to the collision shape
 * @param transform Transformation from collision shape local-space to body local-space
 * @param mass Mass of the collision shape (in kilograms)
 */
IProxyShape::IProxyShape( ICollisionBody* body , ICollisionShape* shape, const ITransform& transform, scalar mass)
 : mBody(body) ,
   mCollisionShape(shape) ,
   mLocalToBodyTransform(transform),
   mMass(mass),
   mNext(nullptr),
   mBroadPhaseID(-1),
   mCachedCollisionData(nullptr),
   mUserData(nullptr),
   mCollisionCategoryBits(0x0001),
   mCollideWithMaskBits(0xFFFF)
{

}

// Destructor
IProxyShape::~IProxyShape()
{
    // Release the cached collision data memory
    if (mCachedCollisionData != nullptr)
    {
        IFree(mCachedCollisionData);
    }
}

// Return true if a point is inside the collision shape
/**
 * @param worldPoint Point to test in world-space coordinates
 * @return True if the point is inside the collision shape
 */
bool IProxyShape::TestPointInside(const IVector3& worldPoint)
{
    const ITransform localToWorld = mLocalToBodyTransform;
    const IVector3 localPoint = localToWorld.GetInverse() * worldPoint;
    return mCollisionShape->TestPointInside(localPoint, this);
}

// Raycast method with feedback information
/**
 * @param ray Ray to use for the raycasting
 * @param[out] raycastInfo Result of the raycasting that is valid only if the
 *             methods returned true
 * @return True if the ray hit the collision shape
 */
bool IProxyShape::Raycast(const IRay& ray, IRaycastInfo& raycastInfo)
{
    // If the corresponding body is not active, it cannot be hit by rays
    if (!mBody->IsActive()) return false;

    // Convert the ray into the local-space of the collision shape
    const ITransform localToWorldTransform = GetWorldTransform();
    const ITransform worldToLocalTransform = localToWorldTransform.GetInverse();

    IRay rayLocal(worldToLocalTransform * ray.Origin,
                  worldToLocalTransform * ray.Direction,
                  ray.maxFraction);

    bool isHit = mCollisionShape->Raycast(rayLocal, raycastInfo, this);

    // Convert the raycast info into world-space
    raycastInfo.worldPoint  = localToWorldTransform * raycastInfo.worldPoint;
    raycastInfo.worldNormal = localToWorldTransform.GetBasis() * raycastInfo.worldNormal;
    raycastInfo.worldNormal.Normalize();

    return isHit;
}






}
