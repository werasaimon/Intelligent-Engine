#include "ISphereShape.h"
#include "../IProxyShape.h"

namespace IPhysics
{

#define SPHERE_MAX_PETURBERATION_ITERATIONS	    1
#define SPHERE_EPS_PETURBERATION_ANGLES_COFFICIENT  0.00001

// Constructor
/**
 * @param radius Radius of the sphere (in meters)
 */
ISphereShape::ISphereShape( scalar radius )
: IConvexShape(SPHERE, radius)
{
    assert(radius > scalar(0.0));
    mNbMaxPeturberationIteration = SPHERE_MAX_PETURBERATION_ITERATIONS; // maximum iteration  for Axis Peturberation
    mEpsilonPeturberation        = SPHERE_EPS_PETURBERATION_ANGLES_COFFICIENT;// epsilon for Peturberation
}

// Destructor
ISphereShape::~ISphereShape()
{

}

// Raycast method with feedback information
bool ISphereShape::raycast(const IRay& ray, IRaycastInfo& raycastInfo, IProxyShape* proxyShape) const
{

        scalar radius = mMargin;

        const IVector3 m = ray.point1;
        scalar c = m.dot(m) - radius * radius;

        // If the origin of the ray is inside the sphere, we return no intersection
        if (c < scalar(0.0)) return false;

        const IVector3 rayDirection = ray.point2 - ray.point1;
        scalar b = m.dot(rayDirection);

        // If the origin of the ray is outside the sphere and the ray
        // is pointing away from the sphere, there is no intersection
        if (b > scalar(0.0)) return false;

        scalar raySquareLength = rayDirection.lengthSquare();

        // Compute the discriminant of the quadratic equation
        scalar discriminant = b * b - raySquareLength * c;

        // If the discriminant is negative or the ray length is very small, there is no intersection
        if (discriminant < scalar(0.0) || raySquareLength < MACHINE_EPSILON) return false;

        // Compute the solution "t" closest to the origin
        scalar t = -b - ISqrt(discriminant);

        assert(t >= scalar(0.0));

        // If the hit point is withing the segment ray fraction
        if (t < ray.maxFraction * raySquareLength) {

            // Compute the intersection information
            t /= raySquareLength;

            raycastInfo.body = proxyShape->getBody();
            raycastInfo.proxyShape = proxyShape;
            raycastInfo.hitFraction = t;
            raycastInfo.worldPoint = ray.point1 + t * rayDirection;
            raycastInfo.worldNormal = raycastInfo.worldPoint;

            return true;
        }

        return false;

}

}
