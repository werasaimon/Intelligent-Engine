#include "ICollisionShape.h"

namespace IPhysics
{


// Constructor
ICollisionShape::ICollisionShape(CollisionShapeType type)
: mType(type),
  mScaling(1.0, 1.0, 1.0)
{
//    mNbMaxPeturberationIteration = 10;
//    mEpsilonPeturberation = 0.001;
}

// Destructor
ICollisionShape::~ICollisionShape()
{

}



// Compute the world-space AABB of the collision shape given a transform
/**
 * @param[out] aabb The axis-aligned bounding box (AABB) of the collision shape
 *                  computed in world-space coordinates
 * @param transform Transform used to compute the AABB of the collision shape
 */
void ICollisionShape::computeAABB(IAABB& aabb, const ITransform &transform0 , const ITransform &transform1  ) const
{

    ITransform transform = transform0 * transform1;


    // Get the local bounds in x,y and z direction
    IVector3 minBounds;
    IVector3 maxBounds;
    getLocalBounds(minBounds, maxBounds);


    /// Scale size AABB local
    minBounds *= 1.04;
    maxBounds *= 1.04;


  //      minBounds = minBounds * mScaling;
  //      maxBounds = maxBounds * mScaling;



    // Rotate the local bounds according to the orientation of the body
    IMatrix3x3 worldAxis =  (transform.getBasis()).getAbsoluteMatrix();
    IVector3   worldMinBounds(worldAxis.getRow(0).dot(minBounds),
                              worldAxis.getRow(1).dot(minBounds),
                              worldAxis.getRow(2).dot(minBounds));
    IVector3   worldMaxBounds(worldAxis.getRow(0).dot(maxBounds),
                              worldAxis.getRow(1).dot(maxBounds),
                              worldAxis.getRow(2).dot(maxBounds));


    // Compute the minimum and maximum coordinates of the rotated extents
     IVector3 minCoordinates = transform.getPosition() + worldMinBounds;
     IVector3 maxCoordinates = transform.getPosition() + worldMaxBounds;


    // Update the AABB with the new minimum and maximum coordinates
    aabb.setMin(minCoordinates);
    aabb.setMax(maxCoordinates);
}


}
