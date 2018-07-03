#ifndef IAABB_H
#define IAABB_H

#include "../common/math/IMatematical.h"

namespace IPhysics
{


// Class IAABB
/**
 * This class represents a bounding volume of type "Axis Aligned
 * Bounding Box". It's a box where all the edges are always aligned
 * with the world coordinate system. The AABB is defined by the
 * minimum and maximum world coordinates of the three axis.
 */
class IAABB
{
    private :

        // -------------------- Attributes -------------------- //

        /// Minimum world coordinates of the IAABB on the x,y and z axis
        IVector3 mMinCoordinates;

        /// Maximum world coordinates of the IAABB on the x,y and z axis
        IVector3 mMaxCoordinates;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        IAABB();

        /// Constructor
        IAABB(const IVector3& minCoordinates, const IVector3& maxCoordinates);

        /// Copy-constructor
        IAABB(const IAABB& aabb);

        /// Destructor
        ~IAABB();

        /// Return the center point
        IVector3 getCenter() const;

        /// Return the minimum coordinates of the IAABB
        const IVector3& getMin() const;

        /// Set the minimum coordinates of the IAABB
        void setMin(const IVector3& min);

        /// Return the maximum coordinates of the IAABB
        const IVector3& getMax() const;

        /// Set the maximum coordinates of the IAABB
        void setMax(const IVector3& max);

        /// Return the size of the IAABB in the three dimension x, y and z
        IVector3 getExtent() const;

        /// Inflate each side of the IAABB by a given size
        void inflate(scalar dx, scalar dy, scalar dz);

        /// Return true if the current IAABB is overlapping with the IAABB in argument
        bool testCollision(const IAABB& aabb) const;

        /// Return the volume of the IAABB
        scalar getVolume() const;

        /// Merge the IAABB in parameter with the current one
        void mergeWithAABB(const IAABB& aabb);

        /// Replace the current IAABB with a new IAABB that is the union of two IAABBs in parameters
        void mergeTwoAABBs(const IAABB& aabb1, const IAABB& aabb2);

        /// Return true if the current IAABB contains the IAABB given in parameter
        bool contains(const IAABB& aabb) const;

        /// Return true if a point is inside the IAABB
        bool contains(const IVector3& point) const;

        /// Return true if the IAABB of a triangle intersects the IAABB
        bool testCollisionTriangleAABB(const IVector3* trianglePoints) const;

        /// Return true if the ray intersects the IAABB
        bool testRayIntersect(const IRay& ray) const;

        /// Create and return an IAABB for a triangle
        static IAABB createAABBForTriangle(const IVector3* trianglePoints);

        /// Assignment operator
        IAABB& operator=(const IAABB& aabb);

        // -------------------- Friendship -------------------- //

        friend class IDynamicAABBTree;
};

// Return the center point of the IAABB in world coordinates
SIMD_INLINE IVector3 IAABB::getCenter() const
{
    return (mMinCoordinates + mMaxCoordinates) * scalar(0.5);
}

// Return the minimum coordinates of the IAABB
SIMD_INLINE const IVector3& IAABB::getMin() const
{
    return mMinCoordinates;
}

// Set the minimum coordinates of the IAABB
SIMD_INLINE void IAABB::setMin(const IVector3& min)
{
    mMinCoordinates = min;
}

// Return the maximum coordinates of the IAABB
SIMD_INLINE const IVector3& IAABB::getMax() const
{
    return mMaxCoordinates;
}

// Set the maximum coordinates of the IAABB
SIMD_INLINE void IAABB::setMax(const IVector3& max)
{
    mMaxCoordinates = max;
}

// Return the size of the IAABB in the three dimension x, y and z
SIMD_INLINE IVector3 IAABB::getExtent() const
{
  return  mMaxCoordinates - mMinCoordinates;
}

// Inflate each side of the IAABB by a given size
SIMD_INLINE void IAABB::inflate(scalar dx, scalar dy, scalar dz)
{
    mMaxCoordinates += IVector3(dx, dy, dz);
    mMinCoordinates -= IVector3(dx, dy, dz);
}

// Return true if the current IAABB is overlapping with the IAABB in argument.
/// Two IAABBs overlap if they overlap in the three x, y and z axis at the same time
SIMD_INLINE bool IAABB::testCollision(const IAABB& aabb) const
{
    if (mMaxCoordinates.x < aabb.mMinCoordinates.x || aabb.mMaxCoordinates.x < mMinCoordinates.x) return false;
    if (mMaxCoordinates.y < aabb.mMinCoordinates.y || aabb.mMaxCoordinates.y < mMinCoordinates.y) return false;
    if (mMaxCoordinates.z < aabb.mMinCoordinates.z || aabb.mMaxCoordinates.z < mMinCoordinates.z) return false;
    return true;
}

// Return the volume of the AABB
SIMD_INLINE scalar IAABB::getVolume() const
{
    const IVector3 diff = mMaxCoordinates - mMinCoordinates;
    return (diff.x * diff.y * diff.z);
}

// Return true if the IAABB of a triangle intersects the IAABB
SIMD_INLINE bool IAABB::testCollisionTriangleAABB(const IVector3* trianglePoints) const
{

    if (IMin3(trianglePoints[0].x, trianglePoints[1].x, trianglePoints[2].x) > mMaxCoordinates.x) return false;
    if (IMin3(trianglePoints[0].y, trianglePoints[1].y, trianglePoints[2].y) > mMaxCoordinates.y) return false;
    if (IMin3(trianglePoints[0].z, trianglePoints[1].z, trianglePoints[2].z) > mMaxCoordinates.z) return false;

    if (IMax3(trianglePoints[0].x, trianglePoints[1].x, trianglePoints[2].x) < mMinCoordinates.x) return false;
    if (IMax3(trianglePoints[0].y, trianglePoints[1].y, trianglePoints[2].y) < mMinCoordinates.y) return false;
    if (IMax3(trianglePoints[0].z, trianglePoints[1].z, trianglePoints[2].z) < mMinCoordinates.z) return false;

    return true;
}

// Return true if a point is inside the IAABB
SIMD_INLINE bool IAABB::contains(const IVector3& point) const
{

    return (point.x >= mMinCoordinates.x - MACHINE_EPSILON && point.x <= mMaxCoordinates.x + MACHINE_EPSILON &&
            point.y >= mMinCoordinates.y - MACHINE_EPSILON && point.y <= mMaxCoordinates.y + MACHINE_EPSILON &&
            point.z >= mMinCoordinates.z - MACHINE_EPSILON && point.z <= mMaxCoordinates.z + MACHINE_EPSILON);
}



// Assignment operator
SIMD_INLINE IAABB& IAABB::operator=(const IAABB& aabb)
{
    if (this != &aabb)
    {
        mMinCoordinates = aabb.mMinCoordinates;
        mMaxCoordinates = aabb.mMaxCoordinates;
    }
    return *this;
}


}

#endif // IAABB_H
