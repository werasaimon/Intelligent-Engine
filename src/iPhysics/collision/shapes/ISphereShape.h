#ifndef ISPHERESHAPE_H
#define ISPHERESHAPE_H


#include "IConvexShape.h"

namespace IPhysics
{


// Class SphereShape
/**
 * This class represents a sphere collision shape that is centered
 * at the origin and defined by its radius. This collision shape does not
 * have an explicit object margin distance. The margin is implicitly the
 * radius of the sphere. Therefore, no need to specify an object margin
 * for a sphere shape.
 */
class ISphereShape : public IConvexShape
{

    protected :


        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ISphereShape(const ISphereShape& shape );

        /// Private assignment operator
        ISphereShape& operator=(const ISphereShape& shape);

        /// Return a local support point in a given direction without the object margin
        virtual IVector3 getLocalSupportPointWithMargin(const IVector3& direction) const;

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const IVector3& localPoint, IProxyShape* proxyShape) const;

        /// Raycast method with feedback information
        virtual bool raycast(const IRay& ray, IRaycastInfo& raycastInfo, IProxyShape* proxyShape) const;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const;

    public :


        // -------------------- Methods -------------------- //

        /// Constructor
        ISphereShape(scalar radius);

        /// Destructor
        virtual ~ISphereShape();

        /// Return the radius of the sphere
        scalar getRadius() const;

        /// Set the scaling vector of the collision shape
        virtual void setLocalScaling(const IVector3& scaling);

        /// Return the local bounds of the shape in x, y and z directions.
        virtual void getLocalBounds(IVector3& min, IVector3& max) const;

        /// Return the local inertia tensor of the collision shape
        virtual void computeLocalInertiaTensor(IMatrix3x3& tensor, scalar mass) const;

        /// Update the AABB of a body using its collision shape
        virtual void computeAABB(IAABB& aabb, const ITransform& transform) const;
};

// Get the radius of the sphere
/**
 * @return Radius of the sphere (in meters)
 */
SIMD_INLINE scalar ISphereShape::getRadius() const
{
    return mMargin;
}

// Set the scaling vector of the collision shape
SIMD_INLINE void ISphereShape::setLocalScaling(const IVector3& scaling)
{
    mMargin = (mMargin / mScaling.x) * scaling.x;
    ICollisionShape::setLocalScaling(scaling);
}

// Return the number of bytes used by the collision shape
SIMD_INLINE size_t ISphereShape::getSizeInBytes() const
{
    return sizeof(ISphereShape);
}

// Return a local support point in a given direction without the object margin
SIMD_INLINE IVector3 ISphereShape::getLocalSupportPointWithMargin(const IVector3& direction) const
{
//    // Return the center of the sphere (the radius is taken into account in the object margin)
    return  direction.getUnit() * mMargin;

    // Return the center of the sphere (the radius is taken into account in the object margin)
  //  return IVector3(0.0, 0.0, 0.0);
}

// Return the local bounds of the shape in x, y and z directions.
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
SIMD_INLINE void ISphereShape::getLocalBounds(IVector3& min, IVector3& max) const
{

    // Maximum bounds
    max.x = mMargin;
    max.y = mMargin;
    max.z = mMargin;

    // Minimum bounds
    min.x = -mMargin;
    min.y = -mMargin;
    min.z = -mMargin;
}

// Return the local inertia tensor of the sphere
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *                    coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
SIMD_INLINE void ISphereShape::computeLocalInertiaTensor(IMatrix3x3& tensor, scalar mass) const
{
    scalar diag = scalar(0.4) * mass * mMargin * mMargin;

    tensor = IMatrix3x3( diag, 0.f, 0.f,
                         0.f, diag, 0.f,
                         0.f, 0.f, diag );
}

// Update the AABB of a body using its collision shape
/**
 * @param[out] aabb The axis-aligned bounding box (AABB) of the collision shape
 *                  computed in world-space coordinates
 * @param transform Transform used to compute the AABB of the collision shape
 */
SIMD_INLINE void ISphereShape::computeAABB(IAABB& aabb, const ITransform& transform) const
{
    // Get the local extents in x,y and z direction
    IVector3 extents(mMargin, mMargin, mMargin);

    // Update the AABB with the new minimum and maximum coordinates
    aabb.setMin(transform.getPosition() - extents);
    aabb.setMax(transform.getPosition() + extents);
}

// Return true if a point is inside the collision shape
SIMD_INLINE bool ISphereShape::testPointInside(const IVector3& localPoint, IProxyShape* proxyShape) const
{
    return (localPoint.lengthSquare() < mMargin * mMargin);
}


}

#endif // ISPHERESHAPE_H
