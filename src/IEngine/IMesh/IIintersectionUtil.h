#ifndef IIINTERSECTIONUTIL_H
#define IIINTERSECTIONUTIL_H

#include "../ICommon/IMatematical.h"

namespace IEngine
{


static bool IRayIntersectionToTriangle(const IVector3& vertA , const IVector3& vertB , const IVector3& vertC ,const  IVector3 &o, const IVector3 &d)
{
    IVector3 e1 = vertB - vertA;
    IVector3 e2 = vertC - vertA;
    IVector3 p = d.Cross(e2);
    float a = e1.Dot(p);

    if(a == 0)
        return false;

    float f = 1.0f/a;

    IVector3 s = o - vertA;
    float u = f * s.Dot(p);
    if(u < 0.0f || u > 1.0f)
        return false;

    IVector3 q = s.Cross(e1);
    float v = f * d.Dot(q);

    if(v < 0.0f || u+v > 1.0f)
        return false;

    float t = f * e2.Dot(q);

    bool success = (t >= 0.0f && t <= 1.0f);
    return !success;

}

}

#endif // IIINTERSECTIONUTIL_H
