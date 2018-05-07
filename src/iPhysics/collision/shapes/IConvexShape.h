#ifndef ICONVEXSHAPE_H
#define ICONVEXSHAPE_H

#include "ICollisionShape.h"

namespace IPhysics
{


/// Object margin for collision detection in meters (for the GJK-EPA Algorithm)
const scalar OBJECT_MARGIN = scalar(0.002);

// Class ConvexShape
/**
* This abstract class represents a convex collision shape associated with a
* body that is used during the narrow-phase collision detection.
*/
class IConvexShape : public ICollisionShape
{

   protected :

       // -------------------- Attributes -------------------- //

       /// Margin used for the GJK collision detection algorithm
       scalar mMargin;

       // -------------------- Methods -------------------- //

       /// Private copy-constructor
       IConvexShape(const IConvexShape& shape);

       /// Private assignment operator
       IConvexShape& operator=(const IConvexShape& shape);


   public:

       /// Return a local support point in a given direction with the object margin
       virtual IVector3 getLocalSupportPointWithMargin(const IVector3& direction) const = 0;

       /// Return a local support point in a given direction without the object margin
       IVector3 getLocalSupportPointWithoutMargin(const IVector3& direction) const;

       /// Return true if a point is inside the collision shape
       virtual bool testPointInside(const IVector3& worldPoint, IProxyShape* proxyShape) const=0;

   public :


       // -------------------- Methods -------------------- //

       /// Constructor
       IConvexShape(CollisionShapeType type, scalar margin);

       /// Destructor
       virtual ~IConvexShape();

       /// Return the current object margin
       scalar getMargin() const;

       /// Return true if the collision shape is convex, false if it is concave
       virtual bool isConvex() const;

       // -------------------- Friendship -------------------- //

       friend class  ICollisionBody;
};

/// Return true if the collision shape is convex, false if it is concave
SIMD_INLINE bool IConvexShape::isConvex() const
{
   return true;
}

// Return the current collision shape margin
/**
* @return The margin (in meters) around the collision shape
*/
SIMD_INLINE scalar IConvexShape::getMargin() const
{
   return mMargin;
}


}

#endif // ICONVEXSHAPE_H
