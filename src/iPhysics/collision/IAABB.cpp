#include "IAABB.h"

namespace IPhysics
{

// Constructor
IAABB::IAABB()
{

}

// Constructor
IAABB::IAABB(const IVector3& minCoordinates, const IVector3& maxCoordinates)
     :mMinCoordinates(minCoordinates),
      mMaxCoordinates(maxCoordinates)
{

}

// Copy-constructor
IAABB::IAABB(const IAABB& aabb)
     : mMinCoordinates(aabb.mMinCoordinates),
       mMaxCoordinates(aabb.mMaxCoordinates)
{

}

// Destructor
IAABB::~IAABB()
{

}

// Merge the IAABB in parameter with the current one
void IAABB::mergeWithAABB (const IAABB& aabb)
{

    mMinCoordinates.x = IMin(mMinCoordinates.x, aabb.mMinCoordinates.x);
    mMinCoordinates.y = IMin(mMinCoordinates.y, aabb.mMinCoordinates.y);
    mMinCoordinates.z = IMin(mMinCoordinates.z, aabb.mMinCoordinates.z);

    mMaxCoordinates.x = IMax(mMaxCoordinates.x, aabb.mMaxCoordinates.x);
    mMaxCoordinates.y = IMax(mMaxCoordinates.y, aabb.mMaxCoordinates.y);
    mMaxCoordinates.z = IMax(mMaxCoordinates.z, aabb.mMaxCoordinates.z);
}

// Replace the current AABB with a new AABB that is the union of two AABBs in parameters
void IAABB::mergeTwoAABBs(const IAABB& aabb1, const IAABB& aabb2)
{
    mMinCoordinates.x = IMin(aabb1.mMinCoordinates.x, aabb2.mMinCoordinates.x);
    mMinCoordinates.y = IMin(aabb1.mMinCoordinates.y, aabb2.mMinCoordinates.y);
    mMinCoordinates.z = IMin(aabb1.mMinCoordinates.z, aabb2.mMinCoordinates.z);

    mMaxCoordinates.x = IMax(aabb1.mMaxCoordinates.x, aabb2.mMaxCoordinates.x);
    mMaxCoordinates.y = IMax(aabb1.mMaxCoordinates.y, aabb2.mMaxCoordinates.y);
    mMaxCoordinates.z = IMax(aabb1.mMaxCoordinates.z, aabb2.mMaxCoordinates.z);
}

// Return true if the current AABB contains the AABB given in parameter
bool IAABB::contains(const IAABB& aabb) const
{

    bool isInside = true;
    isInside = isInside && mMinCoordinates.x <= aabb.mMinCoordinates.x;
    isInside = isInside && mMinCoordinates.y <= aabb.mMinCoordinates.y;
    isInside = isInside && mMinCoordinates.z <= aabb.mMinCoordinates.z;

    isInside = isInside && mMaxCoordinates.x >= aabb.mMaxCoordinates.x;
    isInside = isInside && mMaxCoordinates.y >= aabb.mMaxCoordinates.y;
    isInside = isInside && mMaxCoordinates.z >= aabb.mMaxCoordinates.z;
    return isInside;
}

// Create and return an AABB for a triangle
IAABB IAABB::createAABBForTriangle(const IVector3* trianglePoints)
{

    IVector3 minCoords(trianglePoints[0].x, trianglePoints[0].y, trianglePoints[0].z);
    IVector3 maxCoords(trianglePoints[0].x, trianglePoints[0].y, trianglePoints[0].z);

    if (trianglePoints[1].x < minCoords.x) minCoords.x = trianglePoints[1].x;
    if (trianglePoints[1].y < minCoords.y) minCoords.y = trianglePoints[1].y;
    if (trianglePoints[1].z < minCoords.z) minCoords.z = trianglePoints[1].z;

    if (trianglePoints[2].x < minCoords.x) minCoords.x = trianglePoints[2].x;
    if (trianglePoints[2].y < minCoords.y) minCoords.y = trianglePoints[2].y;
    if (trianglePoints[2].z < minCoords.z) minCoords.z = trianglePoints[2].z;

    if (trianglePoints[1].x > maxCoords.x) maxCoords.x = trianglePoints[1].x;
    if (trianglePoints[1].y > maxCoords.y) maxCoords.y = trianglePoints[1].y;
    if (trianglePoints[1].z > maxCoords.z) maxCoords.z = trianglePoints[1].z;

    if (trianglePoints[2].x > maxCoords.x) maxCoords.x = trianglePoints[2].x;
    if (trianglePoints[2].y > maxCoords.y) maxCoords.y = trianglePoints[2].y;
    if (trianglePoints[2].z > maxCoords.z) maxCoords.z = trianglePoints[2].z;

    return IAABB(minCoords, maxCoords);
}

// Return true if the ray intersects the AABB
/// This method use the line vs AABB raycasting technique described in
/// Real-time Collision Detection by Christer Ericson.
bool IAABB::testRayIntersect(const IRay &ray) const
{

    const IVector3 point2 = ray.point1 + ray.maxFraction * (ray.point2 - ray.point1);
    const IVector3 e = mMaxCoordinates - mMinCoordinates;
    const IVector3 d = point2 - ray.point1;
    const IVector3 m = ray.point1 + point2 - mMinCoordinates - mMaxCoordinates;

    // Test if the AABB face normals are separating axis
    scalar adx = IAbs(d.x);
    if (IAbs(m.x) > e.x + adx) return false;
    scalar ady = IAbs(d.y);
    if (IAbs(m.y) > e.y + ady) return false;
    scalar adz = IAbs(d.z);
    if (IAbs(m.z) > e.z + adz) return false;

    // Add in an epsilon term to counteract arithmetic errors when segment is
    // (near) parallel to a coordinate axis (see text for detail)
    const scalar epsilon = 0.00001;
    adx += epsilon;
    ady += epsilon;
    adz += epsilon;

    // Test if the cross products between face normals and ray direction are
    // separating axis
    if (IAbs(m.y * d.z - m.z * d.y) > e.y * adz + e.z * ady) return false;
    if (IAbs(m.z * d.x - m.x * d.z) > e.x * adz + e.z * adx) return false;
    if (IAbs(m.x * d.y - m.y * d.x) > e.x * ady + e.y * adx) return false;

    // No separating axis has been found
    return true;
}


}
