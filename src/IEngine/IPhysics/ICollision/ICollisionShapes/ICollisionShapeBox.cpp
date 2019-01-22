#include "ICollisionShapeBox.h"
#include "../IProxyShape.h"

namespace IPhysics
{

// Default initilization
#define DEFAULT_BOX_MAX_PETURBERATION_ITERATIONS	      4
#define DEFAULT_BOX_EPS_PETURBERATION_ANGLES_COFFICIENT   0.08

// Constructor
/**
 * @param extent The vector with the three extents of the box (in meters)
 * @param margin The collision margin (in meters) around the collision shape
 */

ICollisionShapeBox::ICollisionShapeBox(const IVector3 &extent)
    : ICollisionShapeConvex(BOX, OBJECT_MARGIN) ,
      mExtent(extent - IVector3(OBJECT_MARGIN,OBJECT_MARGIN,OBJECT_MARGIN))
{

    assert(extent.x > scalar(0.0) && extent.x > OBJECT_MARGIN);
    assert(extent.y > scalar(0.0) && extent.y > OBJECT_MARGIN);
    assert(extent.z > scalar(0.0) && extent.z > OBJECT_MARGIN);

    mNbMaxPeturberationIteration = DEFAULT_BOX_MAX_PETURBERATION_ITERATIONS; // maximum iteration  for Axis Peturberation
    mEpsilonPeturberation =  DEFAULT_BOX_EPS_PETURBERATION_ANGLES_COFFICIENT;// epsilon for Peturberation
}


// Constructor
/**
 * @param extent The vector with the three extents of the box (in meters)
 * @param margin The collision margin (in meters) around the collision shape
 */
ICollisionShapeBox::ICollisionShapeBox(const IVector3& extent, scalar margin)
: ICollisionShapeConvex(BOX, margin) ,
  mExtent(extent - IVector3(margin, margin, margin))
{

    assert(extent.x > scalar(0.0) && extent.x > margin);
    assert(extent.y > scalar(0.0) && extent.y > margin);
    assert(extent.z > scalar(0.0) && extent.z > margin);

    mNbMaxPeturberationIteration = DEFAULT_BOX_MAX_PETURBERATION_ITERATIONS; // maximum iteration  for Axis Peturberation
    mEpsilonPeturberation =  DEFAULT_BOX_EPS_PETURBERATION_ANGLES_COFFICIENT;// epsilon for Peturberation
}

// Destructor
ICollisionShapeBox::~ICollisionShapeBox()
{

}

// Return the local inertia tensor of the collision shape
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *                    coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
void ICollisionShapeBox::ComputeLocalInertiaTensor(IMatrix3x3 &tensor, scalar mass) const
{
    scalar factor = (scalar(1.0) / scalar(3.0)) * mass;
    IVector3 realExtent = mExtent + IVector3(mMargin, mMargin, mMargin);
    scalar xSquare = realExtent.x * realExtent.x;
    scalar ySquare = realExtent.y * realExtent.y;
    scalar zSquare = realExtent.z * realExtent.z;

    tensor = IMatrix3x3( factor * (ySquare + zSquare), 0.0, 0.0,
                         0.0, factor * (xSquare + zSquare), 0.0,
                         0.0, 0.0, factor * (xSquare + ySquare) );
}



bool ICollisionShapeBox::Raycast(const IRay& ray, IRaycastInfo& raycastInfo, IProxyShape* proxyShape) const
{

    IVector3 rayDirection = ray.Direction;
    scalar tMin = DECIMAL_SMALLEST;
    scalar tMax = DECIMAL_LARGEST;
    IVector3 normalDirection(scalar(0), scalar(0), scalar(0));
    IVector3 currentNormal;

    // For each of the three slabs
    for (i32 i=0; i<3; i++)
    {
        // If ray is parallel to the slab
        if (IAbs(rayDirection[i]) < MACHINE_EPSILON)
        {
            // If the ray's origin is not inside the slab, there is no hit
            if (ray.Origin[i] > mExtent[i] || ray.Origin[i] < -mExtent[i]) return false;
        }
        else
        {

            // Compute the intersection of the ray with the near and far plane of the slab
            scalar oneOverD = scalar(1.0) / rayDirection[i];
            scalar t1 = (-mExtent[i] - ray.Origin[i]) * oneOverD;
            scalar t2 = (mExtent[i] - ray.Origin[i]) * oneOverD;
            currentNormal[0] = (i == 0) ? -mExtent[i] : scalar(0.0);
            currentNormal[1] = (i == 1) ? -mExtent[i] : scalar(0.0);
            currentNormal[2] = (i == 2) ? -mExtent[i] : scalar(0.0);

            // Swap t1 and t2 if need so that t1 is intersection with near plane and
            // t2 with far plane
            if (t1 > t2)
            {
                ISwap(t1, t2);
                currentNormal = -currentNormal;
            }

            // Compute the intersection of the of slab intersection interval with previous slabs
            if (t1 > tMin)
            {
                tMin = t1;
                normalDirection = currentNormal;
            }
            tMax = IMin(tMax, t2);

            // If tMin is larger than the maximum raycasting fraction, we return no hit
            if (tMin > ray.maxFraction) return false;

            // If the slabs intersection is empty, there is no hit
            if (tMin > tMax) return false;
        }
    }

    // If tMin is negative, we return no hit
    if (tMin < scalar(0.0) || tMin > ray.maxFraction) return false;

    // The ray intersects the three slabs, we compute the hit point
    IVector3 localHitPoint = ray.Origin + tMin * rayDirection;


    raycastInfo.body = proxyShape->GetBody();
    raycastInfo.proxyShape = proxyShape;
    raycastInfo.hitFraction = tMin;
    raycastInfo.worldPoint = localHitPoint;
    raycastInfo.worldNormal = normalDirection;

    return true;
}



#undef DEFAULT_BOX_MAX_PETURBERATION_ITERATIONS
#undef DEFAULT_BOX_EPS_PETURBERATION_ANGLES_COFFICIENT

}
