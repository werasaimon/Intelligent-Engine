#include "ICollisionShapeSphere.h"
#include "../IProxyShape.h"

namespace IPhysics
{

// Default initilization
#define DEFAULT_SPHERE_MAX_PETURBERATION_ITERATIONS	    1
#define DEFAULT_SPHERE_EPS_PETURBERATION_ANGLES_COFFICIENT  0.00001

// Constructor
/**
 * @param radius Radius of the sphere (in meters)
 */
ICollisionShapeSphere::ICollisionShapeSphere( scalar radius )
: ICollisionShapeConvex(SPHERE, radius)
{
    assert(radius > scalar(0.0));
    mNbMaxPeturberationIteration = DEFAULT_SPHERE_MAX_PETURBERATION_ITERATIONS; // maximum iteration  for Axis Peturberation
    mEpsilonPeturberation        = DEFAULT_SPHERE_EPS_PETURBERATION_ANGLES_COFFICIENT;// epsilon for Peturberation
}

// Destructor
ICollisionShapeSphere::~ICollisionShapeSphere()
{

}

// Raycast method with feedback information
bool ICollisionShapeSphere::Raycast(const IRay& ray, IRaycastInfo& raycastInfo, IProxyShape* proxyShape) const
{

        scalar radius = mMargin;

        const IVector3 m = ray.Origin;
        scalar c = m.Dot(m) - radius * radius;

        // If the origin of the ray is inside the sphere, we return no intersection
        if (c < scalar(0.0)) return false;

        const IVector3 rayDirection = ray.Direction;
        scalar b = m.Dot(rayDirection);

        // If the origin of the ray is outside the sphere and the ray
        // is pointing away from the sphere, there is no intersection
        if (b > scalar(0.0)) return false;

        scalar raySquareLength = rayDirection.LengthSquare();

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

            raycastInfo.body = proxyShape->GetBody();
            raycastInfo.proxyShape = proxyShape;
            raycastInfo.hitFraction = t;
            raycastInfo.worldPoint = ray.Origin + t * rayDirection;
            raycastInfo.worldNormal = raycastInfo.worldPoint;

            return true;
        }

        return false;

}

#undef DEFAULT_SPHERE_MAX_PETURBERATION_ITERATIONS
#undef DEFAULT_SPHERE_EPS_PETURBERATION_ANGLES_COFFICIENT

}
