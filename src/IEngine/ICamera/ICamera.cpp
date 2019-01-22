#include "ICamera.h"

namespace IEngine
{

ICamera::ICamera()
:IObjectTransform()
{
  mAspect    = 0.f;
  mNearPlane = 0.f;
  mFarPlane  = 0.f;

  mEye    = IVector3(0,0,0);
  mCenter = IVector3(0,0,0);
  mUp     = IVector3(0,0,0);

  mTransformMatrix.SetToIdentity();
  mProjectionMatrix.SetToIdentity();
}

void ICamera::ProjectionOrthoMatrix(float left, float  right , float buttom , float top , float NearPlane, float FarPlane)
{
  mAspect    = 1.0;
  mNearPlane = NearPlane;
  mFarPlane  = FarPlane;

  mProjectionMatrix.SetToIdentity();
  mProjectionMatrix = IMatrix4x4::CreateOrtho(left,right,buttom,top,NearPlane,FarPlane);
}

void ICamera::ProjectionPerspectiveMatrix(float  FieldOfView, float  aspect, float  NearPlane, float  FarPlane)
{
  mAspect    = aspect;
  mNearPlane = NearPlane;
  mFarPlane  = FarPlane;

  mProjectionMatrix.SetToIdentity();
  mProjectionMatrix = IMatrix4x4::CreatePerspective( FieldOfView , aspect , NearPlane , FarPlane );
  //mProjectionMatrix = IMatrix4x4::createPerspectivFOV(FieldOfView , aspect , NearPlane , FarPlane );
  //mProjectionMatrix = IMatrix4x4::createPerspective2( FieldOfView , aspect , NearPlane , FarPlane );
}

void ICamera::LookAt(const IVector3 &eye, const IVector3 &center, const IVector3 &up)
{
    mEye    = eye;
    mCenter = center;
    mUp     = up;

    mTransformMatrix.SetToIdentity();
    mTransformMatrix = IMatrix4x4::CreateLookAt( eye , center , up );
}

IMatrix4x4 ICamera::getProjectionMatrix() const
{
    return mProjectionMatrix;
}

IMatrix4x4 ICamera::getViewMatrix() const
{
    return mTransformMatrix;
}

float  ICamera::getAspect() const
{
  return mAspect;
}

float  ICamera::getNearPlane() const
{
  return mNearPlane;
}

float  ICamera::getFarPlane() const
{
  return mFarPlane;
}

IVector3 ICamera::getEye() const
{
  return mEye;
}

IVector3 ICamera::getCenter() const
{
  return mCenter;
}

IVector3 ICamera::getUp() const
{
    return mUp;
}


}

