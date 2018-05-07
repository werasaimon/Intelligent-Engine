#ifndef ICOLLISIONSHAPE_H
#define ICOLLISIONSHAPE_H

#include "../IAABB.h"
#include "../IRaycastInfo.h"

namespace IPhysics
{

/// Type of the collision shape
enum CollisionShapeType {TRIANGLE,
                         BOX,
                         SPHERE,
                         CONE,
                         CYLINDER,
                         CAPSULE,
                         CONVEX_MESH ,
                         CONVEX_HULL_MESH ,
                         CONCAVE_MESH,
                         HEIGHTFIELD};


//Extern declarations
class IProxyShape;

// Class CollisionShape
/**
 * This abstract class represents the collision shape associated with a
 * body that is used during the narrow-phase collision detection.
 */
class ICollisionShape
{

    protected :


        /// Max iterration axis peturbiration
        i32     mNbMaxPeturberationIteration;
        /// Eppsiolon for axis peturbiration
        scalar  mEpsilonPeturberation;

        // -------------------- Attributes ----------------- //

        /// Type of the collision shape
        CollisionShapeType mType;

        /// Scaling vector of the collision shape
        IVector3 mScaling;


        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ICollisionShape(const ICollisionShape& shape);

        /// Private assignment operator
        ICollisionShape& operator=(const ICollisionShape& shape);

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const IVector3& worldPoint, IProxyShape* proxyShape) const = 0;

        /// Raycast method with feedback information
        virtual bool raycast(const IRay& ray, IRaycastInfo& raycastInfo, IProxyShape* proxyShape) const = 0;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const = 0;




        /// Return a local support point in a given direction with the object margin
        virtual IVector3 getLocalSupportPointWithMargin(const IVector3& direction) const = 0;


        /// Return a local support point in a given direction without the object margin
        virtual IVector3 getLocalSupportPointWithoutMargin(const IVector3& direction) const = 0;

    public:


        // -------------------- Methods -------------------- //

        /// Constructor
        ICollisionShape(CollisionShapeType type);

        /// Destructor
        virtual ~ICollisionShape();

        /// Return the type of the collision shapes
        CollisionShapeType getType() const;

        /// Return true if the collision shape is convex, false if it is concave
        virtual bool isConvex() const = 0;

        /// Return the local bounds of the shape in x, y and z directions
        virtual void getLocalBounds(IVector3& min, IVector3& max) const = 0;

        /// Return the scaling vector of the collision shape
        IVector3 getScaling() const;

        /// Set the local scaling vector of the collision shape
        virtual void setLocalScaling(const IVector3& scaling);

        /// Return the local inertia tensor of the collision shapes
        virtual void computeLocalInertiaTensor(IMatrix3x3& tensor, scalar mass) const = 0;

        /// Compute the world-space AABB of the collision shape given a transform
        virtual void computeAABB(IAABB& aabb, const ITransform& transform0 , const ITransform& transform1 ) const;

        /// Return true if the collision shape type is a convex shape
        static bool isConvex(CollisionShapeType shapeType);

        /// Return the maximum number of contact manifolds in an overlapping pair given two shape types
        static i32 computeNbMaxContactManifolds(CollisionShapeType shapeType1,
                                                CollisionShapeType shapeType2);

        // -------------------- Friendship -------------------- //

        friend class IProxyShape;
        friend class ICollisionWorld;
        friend class ICollisionBody;
        friend class ICollisionShapeInfo;

        friend class IGenerationContactPoints;
};


// Return the type of the collision shape
/**
 * @return The type of the collision shape (box, sphere, cylinder, ...)
 */
SIMD_INLINE CollisionShapeType ICollisionShape::getType() const
{
    return mType;
}

// Return true if the collision shape type is a convex shape
SIMD_INLINE bool ICollisionShape::isConvex(CollisionShapeType shapeType)
{
    return shapeType != CONCAVE_MESH && shapeType != HEIGHTFIELD;
}

// Return the scaling vector of the collision shape
SIMD_INLINE IVector3 ICollisionShape::getScaling() const
{
    return mScaling;
}

// Set the scaling vector of the collision shape
SIMD_INLINE void ICollisionShape::setLocalScaling(const IVector3& scaling)
{
    mScaling = scaling;
}

// Return the maximum number of contact manifolds allowed in an overlapping
// pair wit the given two collision shape types
SIMD_INLINE i32 ICollisionShape::computeNbMaxContactManifolds(CollisionShapeType shapeType1,
                                                              CollisionShapeType shapeType2)
{
    // If both shapes are convex
    if (isConvex(shapeType1) && isConvex(shapeType2))
    {
        return NB_MAX_CONTACT_MANIFOLDS_CONVEX_SHAPE;
    }   // If there is at least one concave shape
    else
    {
        return NB_MAX_CONTACT_MANIFOLDS_CONCAVE_SHAPE;
    }
}


}

#endif // ICOLLISIONSHAPE_H
