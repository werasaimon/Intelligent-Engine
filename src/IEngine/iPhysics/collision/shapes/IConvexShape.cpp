#include "IConvexShape.h"

namespace IPhysics
{


// Constructor
IConvexShape::IConvexShape(CollisionShapeType type, scalar margin)
: ICollisionShape(type), mMargin(margin)
{

}

// Destructor
IConvexShape::~IConvexShape()
{

}

// Return a local support point in a given direction with the object margin
SIMD_INLINE IVector3 IConvexShape::getLocalSupportPointWithoutMargin(const IVector3& direction) const
{
    // Get the support point without margin
    IVector3 supportPoint = getLocalSupportPointWithMargin(direction);

    /**/
    if (mMargin != scalar(0.0))
    {
        // Add the margin to the support point
        IVector3 unitVec(0.0, -1.0, 0.0);
        if (direction.lengthSquare() > MACHINE_EPSILON * MACHINE_EPSILON)
        {
            unitVec = direction.getUnit();
        }
        supportPoint += unitVec * mMargin;
    }
    /**/

    return supportPoint;
}

}
