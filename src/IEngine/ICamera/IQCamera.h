#ifndef IQCAMERA_H
#define IQCAMERA_H

#include "../ICommon/IMatematical.h"


#define IGL_CAMERA_MIN_ANGLE 5.0

namespace IEngine
{

class IQCamera
{
public:
    // On windows you might need: -fno-delayed-template-parsing
    //static constexpr float IGL_CAMERA_MIN_ANGLE = 5.;
    //  m_angle  Field of view angle in degrees {45}
    //  m_aspect  Aspect ratio {1}
    //  m_near  near clipping plane {1e-2}
    //  m_far  far clipping plane {100}
    //  m_at_dist  distance of looking at point {1}
    //  m_orthographic  whether to use othrographic projection {false}
    //  m_rotation_conj  Conjugate of rotation part of rigid transformation of
    //    camera {identity}. Note: we purposefully store the conjugate because
    //    this is what TW_TYPE_QUAT4D is expecting.
    //  m_translation  Translation part of rigid transformation of camera
    //    {(0,0,1)}
    float  mAngle;
    float  mAspect;
    float  mNear;
    float  mFar;
    float  mAtDist;
    float  mAngleOfViewSize;
    bool   mOrthographic;

    IQuaternion mRotationConj;
    IVector3    mTranslation;




public:

    IQCamera();
    virtual ~IQCamera(){}


    // Return projection matrix that takes relative camera coordinates and
    // transforms it to viewport coordinates
    //
    // Note:
    //
    //     if(m_angle > 0)
    //     {
    //       gluPerspective(m_angle,m_aspect,m_near,m_at_dist+m_far);
    //     }else
    //     {
    //       gluOrtho(-0.5*aspect,0.5*aspect,-0.5,0.5,m_at_dist+m_near,m_far);
    //     }
    //
    // Is equivalent to
    //
    //     glMultMatrixd(projection().data());
    //
    IMatrix4x4 ProjectionMatrix() const;


    // Transform Matrix4x4 o Camera
    IMatrix4x4 ViewMatrix() const;



    // Rotate and translate so that camera is situated at "eye" looking at "at"
    // with "up" pointing up.
    //
    // Inputs:
    //   eye  (x,y,z) coordinates of eye position
    //   at   (x,y,z) coordinates of at position
    //   up   (x,y,z) coordinates of up vector
    void LookAt(const IVector3& eye, const IVector3& at, const IVector3& up);







    // Return an Affine transformation (rigid actually) that
    // takes relative coordinates and tramsforms them into world 3d
    // coordinates: moves the camera into the scene.
    IMatrix4x4 GetAffine() const;

    // Return an Affine transformation (rigid actually) that puts the takes a
    // world 3d coordinate and transforms it into the relative camera
    // coordinates: moves the scene in front of the camera.
    //
    // Note:
    //
    //     gluLookAt(
    //       eye()(0), eye()(1), eye()(2),
    //       at()(0), at()(1), at()(2),
    //       up()(0), up()(1), up()(2));
    //
    // Is equivalent to
    //
    //     glMultMatrixd(camera.inverse().matrix().data());
    //
    // See also: affine, eye, at, up
    IMatrix4x4 GetInverse() const;

    // Returns world coordinates position of center or "eye" of camera.
    IVector3 GetEye() const;

    // Returns world coordinate position of a point "eye" is looking at.
    IVector3 GetAt() const;

    // Returns world coordinate unit vector of "up" vector
    IVector3 GetUp() const;

    // Return top right corner of unit plane in relative coordinates, that is
    // (w/2,h/2,1)
    IVector3 UnitPlane() const;

    // Move dv in the relative coordinate frame of the camera (move the FPS)
    //
    // Inputs:
    //   dv  (x,y,z) displacement vector
    //
    void Dolly(const IVector3 &dv);
    // "Scale zoom": Move `eye`, but leave `at`



    // Aka "Hitchcock", "Vertigo", "Spielberg" or "Trombone" zoom:
    // simultaneously dolly while changing angle so that `at` not only stays
    // put in relative coordinates but also projected coordinates. That is
    //
    // Inputs:
    //   da  change in angle in degrees
    void DollyZoom(const float &da);


    // Orbit around at so that rotation is now q
    //
    // Inputs:
    //   q  new rotation as quaternion
    void Orbit(const IQuaternion &q , const IVector3& origin = IVector3::ZERO);



};

}

#endif // IQCAMERA_H
