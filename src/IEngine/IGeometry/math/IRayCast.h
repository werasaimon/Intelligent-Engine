#ifndef IRAYCAST_H
#define IRAYCAST_H

#include "IMatematical.h"

namespace IGeometry
{

class IRayCast
{

 public:

    IVector3 point1;
    IVector3 point2;
    float    maxFraction;

    IVector3 hit;


    // -------------------- Methods -------------------- //


    IRayCast(){}

    /// Constructor with arguments
    IRayCast(const IVector3& _point1, const IVector3& _point2, float maxFrac = float(1.0))
     : point1(_point1),
       point2(_point2),
       maxFraction(maxFrac)
    {
    }

    /// Copy-constructor
    IRayCast(const IRayCast& ray)
     : point1(ray.point1),
       point2(ray.point2),
       maxFraction(ray.maxFraction)
    {

    }

    /// Destructor
    ~IRayCast()
    {

    }

    /// Overloaded assignment operator
    IRayCast& operator=(const IRayCast& ray)
    {
        if (&ray != this)
        {
            point1 = ray.point1;
            point2 = ray.point2;
            maxFraction = ray.maxFraction;
        }
        return *this;
    }
};

}

#endif // IRAYCAST_H
