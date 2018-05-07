#include "ICamera.h"

namespace IGeometry
{

ICamera::ICamera()
:IObject3D()
{
  mAspect    = 0.f;
  mNearPlane = 0.f;
  mFarPlane  = 0.f;

  mEye    = IVector3(0,0,0);
  mCenter = IVector3(0,0,0);
  mUp     = IVector3(0,0,0);

  mTransformMatrix.setToIdentity();
  mProjectionMatrix.setToIdentity();
}

void ICamera::ProjectionPerspectiveMatrix(float  FieldOfView, float  aspect, float  NearPlane, float  FarPlane)
{
  mAspect    = aspect;
  mNearPlane = NearPlane;
  mFarPlane  = FarPlane;

  mProjectionMatrix.setToIdentity();
  //mProjectionMatrix = IMatrix4x4::createPerspectiveFOV( FieldOfView , aspect , NearPlane , FarPlane);
  mProjectionMatrix = IMatrix4x4::createPerspective( FieldOfView , aspect , NearPlane , FarPlane );
}

void ICamera::LookAt(const IVector3 &eye, const IVector3 &center, const IVector3 &up)
{
    mEye    = eye;
    mCenter = center;
    mUp     = up;

    mTransformMatrix.setToIdentity();
    mTransformMatrix = IMatrix4x4::createLookAt( eye , center , up );
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

