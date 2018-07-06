#ifndef AABB_H
#define AABB_H


#include "../math/IMatematical.h"


namespace IGeometry
{

// Class AABB
/**
 * This class represents a bounding volume of type "Axis Aligned
 * Bounding Box". It's a box where all the edges are always aligned
 * with the world coordinate system. The AABB is defined by the
 * minimum and maximum world coordinates of the three axis.
 */
class gAABB
{
private :

    // -------------------- Attributes -------------------- //

    /// Minimum world coordinates of the gAABB on the x,y and z axis
    IVector3 mMinCoordinates;

    /// Maximum world coordinates of the gAABB on the x,y and z axis
    IVector3 mMaxCoordinates;

  public:



    // Constructor
    gAABB()
    {

    }

    // Constructor
    gAABB(const IVector3& minCoordinates,
          const IVector3& maxCoordinates)
         :mMinCoordinates(minCoordinates),
          mMaxCoordinates(maxCoordinates)
    {

    }

    // Copy-constructor
    gAABB(const gAABB& aabb)
         : mMinCoordinates(aabb.mMinCoordinates),
           mMaxCoordinates(aabb.mMaxCoordinates)
    {

    }

    // Destructor
    ~gAABB()
    {

    }

    /// Return the center point
    IVector3 getCenter() const;

    /// Return the minimum coordinates of the gAABB
    const IVector3& getMin() const;

    /// Set the minimum coordinates of the gAABB
    void setMin(const IVector3& min);

    /// Return the maximum coordinates of the gAABB
    const IVector3& getMax() const;

    /// Set the maximum coordinates of the gAABB
    void setMax(const IVector3& max);

    /// Return the size of the gAABB in the three dimension x, y and z
    IVector3 getExtent() const;


    // Return true if the ray intersects the AABB
    /// This method use the line vs AABB raycasting technique described in
    /// Real-time Collision Detection by Christer Ericson.
    bool testRayIntersect(const IRay &ray) const;



};




// Return the center point of the gAABB in world coordinates
inline IVector3 gAABB::getCenter() const
{
    return (mMinCoordinates + mMaxCoordinates) * float(0.5);
}

// Return the minimum coordinates of the gAABB
inline const IVector3& gAABB::getMin() const
{
    return mMinCoordinates;
}

// Set the minimum coordinates of the gAABB
inline void gAABB::setMin(const IVector3& min)
{
    mMinCoordinates = min;
}

// Return the maximum coordinates of the gAABB
inline const IVector3& gAABB::getMax() const
{
    return mMaxCoordinates;
}

// Set the maximum coordinates of the gAABB
inline void gAABB::setMax(const IVector3& max)
{
    mMaxCoordinates = max;
}

// Return the size of the gAABB in the three dimension x, y and z
inline IVector3 gAABB::getExtent() const
{
  return  mMaxCoordinates - mMinCoordinates;
}

inline bool gAABB::testRayIntersect(const IRay &ray) const
{

    const IVector3 point2 = ray.point1 + ray.maxFraction * (ray.point2 - ray.point1);
    const IVector3 e = mMaxCoordinates - mMinCoordinates;
    const IVector3 d = point2 - ray.point1;
    const IVector3 m = ray.point1 + point2 - mMinCoordinates - mMaxCoordinates;

    // Test if the AABB face normals are separating axis
    float adx = IAbs(d.x);
    if (IAbs(m.x) > e.x + adx) return false;
    float ady = IAbs(d.y);
    if (IAbs(m.y) > e.y + ady) return false;
    float adz = IAbs(d.z);
    if (IAbs(m.z) > e.z + adz) return false;

    // Add in an epsilon term to counteract arithmetic errors when segment is
    // (near) parallel to a coordinate axis (see text for detail)
    const float epsilon = 0.00001;
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

#endif // AABB_H
