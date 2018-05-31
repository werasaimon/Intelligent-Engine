#ifndef IMESHCREATEBOX_H
#define IMESHCREATEBOX_H

#include "IMeshModel.h"
#include "../math/IMatematical.h"


namespace IGeometry
{


namespace
{
    //Helper function for Line/AABB test.  Tests collision on a single dimension
    //Param:    Start of line, Direction/length of line,
    //          Min value of AABB on plane, Max value of AABB on plane
    //          Enter and Exit "timestamps" of intersection (OUT)
    //Return:   True if there is overlap between Line and AABB, False otherwise
    //Note:     Enter and Exit are used for calculations and are only updated in case of intersection
    bool Line_AABB_1d(float start, float dir, float min, float max, float& enter, float& exit)
    {
        //If the line segment is more of a point, just check if it's within the segment
        if(fabs(dir) < 1.0E-8)
            return (start >= min && start <= max);

        //Find if the lines overlap
        float   ooDir = 1.0f / dir;
        float   t0 = (min - start) * ooDir;
        float   t1 = (max - start) * ooDir;

        //Make sure t0 is the "first" of the intersections
        if(t0 > t1)
            ISwap(t0, t1);

        //Check if intervals are disjoint
        if(t0 > exit || t1 < enter)
            return false;

        //Reduce interval based on intersection
        if(t0 > enter)
            enter = t0;
        if(t1 < exit)
            exit = t1;

        return true;
    }

    //Check collision between a line segment and an AABB
    //Param:    Start point of line segement, End point of line segment,
    //          One corner of AABB, opposite corner of AABB,
    //          Location where line hits the AABB (OUT)
    //Return:   True if a collision occurs, False otherwise
    //Note:     If no collision occurs, OUT param is not reassigned and is not considered useable
    bool Line_intersection_AABB(const IVector3& s, const IVector3& e, const IVector3& min, const IVector3& max, IVector3& hitPoint)
    {
        float       enter = 0.0f;
        float       exit = 1.0f;
        IVector3    dir = e - s;

        //Check each dimension of Line/AABB for intersection
        if(!Line_AABB_1d(s.x, dir.x, min.x, max.x, enter, exit)) return false;
        if(!Line_AABB_1d(s.y, dir.y, min.y, max.y, enter, exit)) return false;
        if(!Line_AABB_1d(s.z, dir.z, min.z, max.z, enter, exit)) return false;

        //If there is intersection on all dimensions, report that point
        hitPoint = s + dir * enter;
        return true;
    }
}





class IMeshCreateBox : public IMeshModel
{

  private:

     IVector3 mExtentSize;

  public:

     IMeshCreateBox(const IVector3& halfSize);

    bool raycast( IRayCast& ray ) const
    {

        /**

//        ray.point1 = mTransformMatrix.getInverse() * ray.point1;
//        ray.point2 = mTransformMatrix.getInverse() * ray.point2;

        float tMin = -std::numeric_limits<float>::max();
        float tMax =  std::numeric_limits<float>::max();

        IVector3 rayDirection = ray.point2 - ray.point1;

        IVector3 normalDirection(float(0), float(0), float(0));
        IVector3 currentNormal;

        IVector3 Extented = mExtentSize;// * 0.5f;

        // For each of the three slabs
        for (int i=0; i<3; i++)
        {
            // If ray is parallel to the slab
            if (fabs(rayDirection[i]) < MACHINE_EPSILON)
            {
                // If the ray's origin is not inside the slab, there is no hit
                if (ray.point1[i] > Extented[i] || ray.point1[i] < -Extented[i]) return false;
            }
            else
            {
                // Compute the intersection of the ray with the near and far plane of the slab
                float oneOverD = float(1.0) / rayDirection[i];
                float t1 = (-Extented[i] - ray.point1[i]) * oneOverD;
                float t2 = ( Extented[i] - ray.point1[i]) * oneOverD;
                currentNormal[0] = (i == 0) ? -Extented[i] : float(0.0);
                currentNormal[1] = (i == 1) ? -Extented[i] : float(0.0);
                currentNormal[2] = (i == 2) ? -Extented[i] : float(0.0);

                // Swap t1 and t2 if need so that t1 is intersection with near plane and
                // t2 with far plane
                if (t1 > t2)
                {
                    ISwap(t1, t2);
                    currentNormal = -currentNormal;
                }

                // Compute the intersection of the of slab intersection interval with previous slabs
                if (t1 > tMin)
                {
                    tMin = t1;
                    normalDirection = currentNormal;
                }
                tMax = IMin(tMax, t2);

                // If tMin is larger than the maximum raycasting fraction, we return no hit
                if (tMin > ray.maxFraction) return false;

                // If the slabs intersection is empty, there is no hit
                if (tMin > tMax) return false;
            }
        }

        // If tMin is negative, we return no hit
        if (tMin < float(0.0) || tMin > ray.maxFraction) return false;

        /**/


        ITransform t;
        t.setFromOpenGL((float*)(mTransformMatrix.getData()));

        IVector3 hit;

       IVector3 rayDirection = (ray.point2 - ray.point1).normalized();

       IVector3 max =  mExtentSize;
       IVector3 min = -mExtentSize;

       IVector3 point1 = t.getInverse() * ray.point1;
       IVector3 point2 = t.getInverse() * ray.point2;

       bool collide = Line_intersection_AABB(point1,point2,min,max,hit);

      // ray.maxFraction = (mTransformMatrix * point1 - mTransformMatrix * hit).dot(rayDirection);
       ray.hit = t * hit;

       ray.maxFraction = (ray.hit - ray.point1).dot(rayDirection);

       return collide;
       /**/
    }

  protected:

     void init(const IVector3& halfSize);


};


}

#endif // IMESHCREATEBOX_H
