#ifndef ICOLLISIONSHAPEINFO_H
#define ICOLLISIONSHAPEINFO_H


#include "../IProxyShape.h"
#include "../ICollisionShapes/ICollisionShape.h"

namespace IPhysics
{


class IOverlappingPair;


// Class rpCollisionShapeInfo
/**
 * This structure regroups different things about a collision shape. This is
 * used to pass information about a collision shape to a collision algorithm.
 */
struct ICollisionShapeInfo
{

    public:

    /// Broadphase overlapping pair
    //IOverlappingPair* overlappingPair;

    /// Proxy shape
    //IProxyShape* proxyShape;

    /// Pointer to the collision shape
    const ICollisionShape* collisionShape;

    /// Transform that maps from collision shape local-space to world-space
    const ITransform        shapeToWorldTransform;

    /// Cached collision data of the proxy shape
    void** cachedCollisionData;

    /// Constructor
    ICollisionShapeInfo(const ICollisionShape* _CollisionShape,
                        const ITransform& shapeLocalToWorldTransform,
                        void** cachedData)
        : collisionShape(_CollisionShape),
          shapeToWorldTransform(shapeLocalToWorldTransform) ,
          cachedCollisionData(cachedData)
    {


    }


    // Return a local support point in a given direction with the object margin
    IVector3 GetLocalSupportPointWithMargin(const IVector3 &direction ) const
    {
        return collisionShape->GetLocalSupportPointWithMargin(direction);
    }

    /**

    // Return a local support point in a given direction with the object margin
    Vector3 getLocalSupportPointWithoutMargin(const Vector3 &direction ) const
    {
          Vector3 supportPoint = collisionShape->getLocalSupportPointWithoutMargin(direction,NULL);

          // Add the margin to the support point
          Vector3 unitVec(0.0, -1.0, 0.0);
          if (direction.lengthSquare() > MACHINE_EPSILON * MACHINE_EPSILON)
          {
              unitVec = direction.getUnit();
          }
          supportPoint += unitVec * 0.08;


          return collisionShape->getLocalSupportPointWithoutMargin(direction,NULL);
    }

    /**/


     // Return a world support point in a given direction
    IVector3 GetWorldSupportPointWithMargin(const IVector3 &direction) const
    {
        return shapeToWorldTransform * GetLocalSupportPointWithMargin(shapeToWorldTransform.GetBasis().GetTranspose() * direction );
    }


    const ITransform& GetWorldTransform() const
    {
        return shapeToWorldTransform;
    }


    IVector3 getScale() const
    {
        return collisionShape->GetScaling();
    }
};


}


#endif // ICOLLISIONSHAPEINFO_H
