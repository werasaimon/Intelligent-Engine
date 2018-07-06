#include "IGizmo.h"


namespace IGeometry
{

std::map<int, IMatrix4x4> IGizmo::outputTransforms() const
{
    return mOutputTransforms;
}

IGizmo::IGizmo()
{
}

bool IGizmo::IntersectRayToSphere(IVector3 rayOrigin, IVector3 rayDir, IVector3 center, float radius, float &t, IVector3 &q)
    {
        IVector3 m = rayOrigin - center;
        float b = dot(m, rayDir);
        float c = dot(m, m) - radius * radius;

        // Exit if r’s origin outside s (c > 0) and r pointing away from s (b > 0)
        if (c > 0.0f && b > 0.0f) return false;
        float discr = b*b - c;

        // A negative discriminant corresponds to ray missing sphere
        if (discr < 0.0f) return false;

        // Ray now found to intersect sphere, compute smallest t value of intersection
        t = -b - ISqrt(discr);

        // If t is negative, ray started inside sphere so clamp t to zero
        if (t < 0.0f) t = 0.0f;
        q = rayOrigin + t * rayDir;

        return true;
    }



    bool IGizmo::IntersectLineToLine(IVector3 p1, IVector3 p2, IVector3 p3, IVector3 p4, IVector3 *pa, IVector3 *pb, bool is_realse)
    {

        const float lengthErrorThreshold = 1e-3;

        IVector3 p13,p43,p21;

        p13 = p1 - p3;
        p43 = p4 - p3;
        p21 = p2 - p1;


        if (IAbs(p43.x) < EPS &&
            IAbs(p43.y) < EPS &&
            IAbs(p43.z) < EPS)
            return false;


        if (IAbs(p21.x) < EPS &&
            IAbs(p21.y) < EPS &&
            IAbs(p21.z) < EPS)
            return false;


        float d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
        float d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
        float d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
        float d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
        float d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

        float denom = d2121 * d4343 - d4321 * d4321;

        if (IAbs(denom) < EPS)
            return false;

        float numer = d1343 * d4321 - d1321 * d4343;
        float mua = numer / denom;
        float mub = (d1343 + d4321 * (mua)) / d4343;

        IVector3 ip_a =  p1 + mua * p21;
        IVector3 ip_b =  p3 + mub * p43;



        if( pa != NULL) *pa = ip_a;
        if( pb != NULL) *pb = ip_b;


        // Means we have an intersection
        if (mua >= 0.0 && mua <= 1.0 &&
            mub >= 0.0 && mub <= 1.0 )
        {
            // See if this lies on the segment
            if((ip_a - p3).lengthSquare() + (ip_a - p4).lengthSquare() <= (p3-p4).lengthSquare() + lengthErrorThreshold &&
               (ip_b - p1).lengthSquare() + (ip_b - p2).lengthSquare() <= (p1-p2).lengthSquare() + lengthErrorThreshold)
            {
                if( (ip_b - ip_a).length() < 0.03 )
                {
                    return true;
                }
            }
        }

        return is_realse;
    }

}

