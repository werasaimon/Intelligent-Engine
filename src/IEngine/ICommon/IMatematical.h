#ifndef IMATEMATICAL_H
#define IMATEMATICAL_H

#include "../IMath/IMaths.h"

namespace IEngine
{

using namespace IMath;

typedef IVector2D<float>       IVector2;
typedef IVector3D<float>       IVector3;
typedef IVector4D<float>       IVector4;
typedef IMatrix2x2<float>      IMatrix2x2;
typedef IMatrix3x3<float>      IMatrix3x3;
typedef IMatrix4x4<float>      IMatrix4x4;
typedef IQuaternion<float>     IQuaternion;
typedef ITransform<float>      ITransform;
typedef IRay<float>            IRay;

typedef IVector2D<int> Index2i;
typedef IVector3D<int> Index3i;
typedef IVector4D<int> Index4i;

typedef ILine3D<float>         ILine3;
typedef ILineSegment3D<float>  ILineSegment3;
//typedef IPlane<float>          IIPlane;

typedef IAxisAlignedBox3D<float> IAxisAlignedBox3D;




}


#endif // IMATEMATICAL_H
