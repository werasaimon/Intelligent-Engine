#ifndef RPMATEMATICAL_H
#define RPMATEMATICAL_H


#include "../../../IMath/IMaths.h"
#include "../ISettings.h"


namespace IPhysics
{

using namespace IMath;

typedef IVector2D<scalar>       IVector2;
typedef IVector3D<scalar>       IVector3;
typedef IVector4D<scalar>       IVector4;
typedef ILorentzVector<scalar>  ILorentzVector4;
typedef IMatrix2x2<scalar>      IMatrix2x2;
typedef IMatrix3x3<scalar>      IMatrix3x3;
typedef IMatrix4x4<scalar>      IMatrix4x4;
typedef IQuaternion<scalar>     IQuaternion;
typedef IRay<scalar>            IRay;
typedef ITransform<scalar>      ITransform;

typedef ILine3D<scalar>         ILine3;
typedef ILineSegment3D<scalar>  ILineSegment3;
typedef IPlane<scalar>          IPlane;

const scalar DECIMAL_SMALLEST = -std::numeric_limits<scalar>::max();
const scalar DECIMAL_LARGEST  =  std::numeric_limits<scalar>::max();

}

#endif // RPMATEMATICAL_H
