#ifndef ICAMERA_H
#define ICAMERA_H

#include "../IObject/IObjectTransform.h"

namespace IEngine
{

class ICamera : public IObjectTransform
{
    public:

        ICamera();


        void ProjectionOrthoMatrix(float left, float  right , float buttom , float top , float NearPlane, float FarPlane);
        void ProjectionPerspectiveMatrix(float  FieldOfView, float  aspect, float  NearPlane, float  FarPlane);
        void LookAt(const IVector3& eye, const IVector3& center, const IVector3& up);


        IMatrix4x4 getProjectionMatrix() const;
        IMatrix4x4 getViewMatrix() const;

        float  getAspect() const;
        float  getNearPlane() const;
        float  getFarPlane() const;/////


        IVector3 getEye() const;
        IVector3 getCenter() const;
        IVector3 getUp() const;

protected:

        IMatrix4x4 mProjectionMatrix;

        float  mAspect;
        float  mNearPlane;
        float  mFarPlane;

        IVector3 mEye;
        IVector3 mCenter;
        IVector3 mUp;
};




}


#endif // ICAMERA_H