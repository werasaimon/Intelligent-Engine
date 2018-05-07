#ifndef ICOLLISIONALGORITHM_H
#define ICOLLISIONALGORITHM_H


#include "../../common/math/IMatematical.h"
#include "ICollisionShapeInfo.h"


namespace IPhysics
{


class OutputCollisionInfo
{

public:

    //-------- Attribute --------//
    scalar   Penetration;
    IVector3 Normal;
    IVector3 Point1;
    IVector3 Point2;

    OutputCollisionInfo()
    {

    }

    OutputCollisionInfo( const IVector3& _normal ,  const scalar&  _penetration ,
                         const IVector3& _point1 ,
                         const IVector3& _point2 )
      :Normal(_normal),
       Penetration(_penetration),
       Point1(_point1),
       Point2(_point2)
    {

    }
};




// Class NarrowPhaseAlgorithm
/**
 * This abstract class is the base class for a  narrow-phase collision
 * detection algorithm. The goal of the narrow phase algorithm is to
 * compute information about the contact between two proxy shapes.
 */
class ICollisionAlgorithm
{

protected :

    // -------------------- Attributes -------------------- //

    OutputCollisionInfo mOutputCollisionInfo;


    // -------------------- Methods -------------------- //

    /// Private copy-constructor
    ICollisionAlgorithm(const ICollisionAlgorithm& algorithm);

    /// Private assignment operator
    ICollisionAlgorithm& operator=(const ICollisionAlgorithm& algorithm);

public :

    // -------------------- Methods -------------------- //

    /// Constructor
    ICollisionAlgorithm();

    /// Destructor
    virtual ~ICollisionAlgorithm();


    /// Compute a contact info if the two bounding volume collide
    virtual bool testCollision(const ICollisionShapeInfo& shape1Info,
                               const ICollisionShapeInfo& shape2Info) = 0;

    OutputCollisionInfo outputCollisionInfo() const;

    //---------------------- friendship --------------------------//
    friend class IGenerationContactPoints;


};

}

#endif // ICOLLISIONALGORITHM_H
