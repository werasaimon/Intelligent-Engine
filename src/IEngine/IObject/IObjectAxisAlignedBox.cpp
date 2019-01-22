#include "IObjectAxisAlignedBox.h"

namespace IEngine
{

    IObjectAxisAlignedBox::IObjectAxisAlignedBox()
        : mAxisAlignedBox(false)
    {

    }

    IObjectAxisAlignedBox::IObjectAxisAlignedBox(const IVector3 &vMin,
                                                 const IVector3 &vMax)
        : mAxisAlignedBox(vMin , vMax)
    {

    }

    void IObjectAxisAlignedBox::InsertCalculate(const IVector3 &vPoint)
    {
        mAxisAlignedBox.Insert(vPoint);
    }

    IAxisAlignedBox3D &IObjectAxisAlignedBox::AxisAlignedBox()
    {
        return mAxisAlignedBox;
    }

    IAxisAlignedBox3D IObjectAxisAlignedBox::GetAxisAlignedBox() const
    {
        return mAxisAlignedBox;
    }

    IAxisAlignedBox3D IObjectAxisAlignedBox::GetAxisAlignedBoxTransform(IMatrix4x4 _TransformMatrix) const
    {
        // Update the AABB with the new minimum and maximum coordinates
        return mAxisAlignedBox.GetAxisAlignedBoxTransform(_TransformMatrix);

    }


}

