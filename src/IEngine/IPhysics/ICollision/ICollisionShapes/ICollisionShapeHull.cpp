#include "ICollisionShapeHull.h"


namespace IPhysics
{



namespace
{
   //IGJKAlgorithm GJKAlgorithm;
}


// Default initilization
#define DEFAULT_HULL_MAX_PETURBERATION_ITERATIONS	       10
#define DEFAULT_HULL_EPS_PETURBERATION_ANGLES_COFFICIENT   0.055


ICollisionShapeHull::ICollisionShapeHull(const IHullDescriptor &_HullDescriptor , scalar margin)
: ICollisionShapeConvex( CONVEX_HULL_MESH , margin ) ,
  mNbVertices(_HullDescriptor.NbVertices) , mVertices(_HullDescriptor.Vertices) ,
  mNbIndexes(_HullDescriptor.NbIndexes) , mIndexes(_HullDescriptor.Indexes)
{
    mNbMaxPeturberationIteration = DEFAULT_HULL_MAX_PETURBERATION_ITERATIONS; // maximum iteration  for Axis Peturberation
    mEpsilonPeturberation = DEFAULT_HULL_EPS_PETURBERATION_ANGLES_COFFICIENT;// epsilon for Peturberation

}



ICollisionShapeHull::~ICollisionShapeHull()
{
    mNbVertices = 0;
    mNbIndexes  = 0;

    if(mVertices != NULL )
    {
        delete mVertices;
        mVertices = NULL;
    }

    if(mIndexes != NULL )
    {
       delete mIndexes;
       mIndexes = NULL;
    }

}

size_t ICollisionShapeHull::GetSizeInBytes() const
{
   return sizeof (ICollisionShapeHull);
}



IVector3 ICollisionShapeHull::GetLocalSupportPointWithMargin(const IVector3 &direction) const
{
    uint index = 0;
    scalar max = (mVertices[0].Dot(direction));

    for (uint i = 1; i < mNbVertices; i++)
    {
        scalar d = (mVertices[i].Dot(direction));
        if (d > max)
        {
            max = d;
            index = i;
        }
    }

    return mVertices[index];
}




bool ICollisionShapeHull::TestPointInside(const IVector3& localPoint, IProxyShape* proxyShape) const
{
   // return GJKAlgorithm.testPointInside(localPoint, proxyShape);
}


bool ICollisionShapeHull::Raycast(const IRay& ray, IRaycastInfo& raycastInfo, IProxyShape* proxyShape) const
{
  //  return GJKAlgorithm.raycast(ray, raycastInfo, proxyShape);
}



void ICollisionShapeHull::SetLocalScaling(const IVector3& scaling)
{
  mScaling = scaling;
}


void ICollisionShapeHull::GetIntervalLocal(const IVector3& xAxis, scalar& min, scalar& max) const
{

    IVector3 s_p0 = GetLocalSupportPointWithMargin( xAxis );
    IVector3 s_p1 = GetLocalSupportPointWithMargin(-xAxis );
    min = s_p1.Dot(xAxis);
    max = s_p0.Dot(xAxis);
}




void ICollisionShapeHull::GetLocalBounds(IVector3& min, IVector3& max) const
{
    GetIntervalLocal( IVector3::X , min.x , max.x);
    GetIntervalLocal( IVector3::Y , min.y , max.y);
    GetIntervalLocal( IVector3::Z , min.z , max.z);
}



void ICollisionShapeHull::ComputeLocalInertiaTensor(IMatrix3x3& tensor, scalar mass) const
{
    IVector3 min;
    IVector3 max;
    GetLocalBounds( min , max );

    IVector3 halfSize;
    halfSize.x = IAbs(min.x - max.x) * 0.5;
    halfSize.y = IAbs(min.y - max.y) * 0.5;
    halfSize.z = IAbs(min.z - max.z) * 0.5;


    scalar  factor = (scalar(1.0) / scalar(3.0)) * mass;
    IVector3 realExtent = halfSize + IVector3(mMargin, mMargin, mMargin);
    scalar  xSquare = realExtent.x * realExtent.x;
    scalar  ySquare = realExtent.y * realExtent.y;
    scalar  zSquare = realExtent.z * realExtent.z;
    tensor.SetAllValues(factor * (ySquare + zSquare), 0.0, 0.0,
                        0.0, factor * (xSquare + zSquare), 0.0,
                        0.0, 0.0, factor * (xSquare + ySquare));

}


#undef DEFAULT_DEFAULYHULL_MAX_PETURBERATION_ITERATIONS
#undef DEFAULT_HULL_EPS_PETURBERATION_ANGLES_COFFICIENT


}
