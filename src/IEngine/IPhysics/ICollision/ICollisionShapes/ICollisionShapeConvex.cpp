#include "ICollisionShapeConvex.h"

namespace IPhysics
{


// Constructor
ICollisionShapeConvex::ICollisionShapeConvex(CollisionShapeType type, scalar margin)
: ICollisionShape(type), mMargin(margin)
{

}

// Destructor
ICollisionShapeConvex::~ICollisionShapeConvex()
{

}

// Return a local support point in a given direction with the object margin
SIMD_INLINE IVector3 ICollisionShapeConvex::GetLocalSupportPointWithoutMargin(const IVector3& direction) const
{
    // Get the support point without margin
    IVector3 supportPoint = GetLocalSupportPointWithMargin(direction);

    /**/
    if (mMargin != scalar(0.0))
    {
        // Add the margin to the support point
        IVector3 unitVec(0.0, -1.0, 0.0);
        if (direction.LengthSquare() > MACHINE_EPSILON * MACHINE_EPSILON)
        {
            unitVec = direction.GetUnit();
        }
        supportPoint += unitVec * mMargin;
    }
    /**/

    return supportPoint;
}

}
