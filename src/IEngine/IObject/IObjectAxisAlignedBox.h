#ifndef IOBJECTAXISALIGNEDBOX_H
#define IOBJECTAXISALIGNEDBOX_H


#include "../ICommon/IMatematical.h"


namespace IEngine
{
class IObjectAxisAlignedBox
{

protected:

    IAxisAlignedBox3D mAxisAlignedBox;

public:

    IObjectAxisAlignedBox();
    IObjectAxisAlignedBox( const IVector3& vMin ,
                           const IVector3& vMax );


    void InsertCalculate(const IVector3& vPoint);

    IAxisAlignedBox3D& AxisAlignedBox();
    IAxisAlignedBox3D  GetAxisAlignedBox() const;
    IAxisAlignedBox3D  GetAxisAlignedBoxTransform(IMatrix4x4 _TransformMatrix) const;
};

}

#endif // IOBJECTAXISALIGNEDBOX_H
