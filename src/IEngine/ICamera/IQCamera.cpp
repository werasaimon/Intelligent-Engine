#include "IQCamera.h"


namespace IEngine
{

    IQCamera::IQCamera()
        : mAngle(45.0),
          mAspect(1),
          mNear(1e-2),
          mFar(100),
          mAtDist(1),
          mAngleOfViewSize(1.0),
          mOrthographic(false),
          mRotationConj(1,0,0,0),
          mTranslation(0,0,1)
    {

    }

    IMatrix4x4 IQCamera::ProjectionMatrix() const
    {

        IMatrix4x4 P;
        using namespace std;
        const float far = mAtDist + mFar;
        const float near = mNear;

        // http://stackoverflow.com/a/3738696/148668
        if(mOrthographic)
        {
            const float f = 0.5;
            const float left = -f*mAspect;
            const float right = f*mAspect;
            const float bottom = -f;
            const float top = f;
            const float tx = (right+left)/(right-left);
            const float ty = (top+bottom)/(top-bottom);
            const float tz = (far+near)/(far-near);
            const float z_fix = 0.5 /mAtDist / tan(mAngle*0.5 * (M_PI/180.) );

            P = IMatrix4x4
                    (z_fix*2./(right-left), 0, 0, -tx,
                     0, z_fix*2./(top-bottom), 0, -ty,
                     0, 0, -z_fix*2./(far-near),  -tz,
                     0, 0, 0, 1);
        }
        else
        {
            const float yScale = ITan(M_PI*0.5 - 0.5*mAngle*M_PI/180.);
            // http://stackoverflow.com/a/14975139/148668
            const float xScale = yScale/mAspect;
            P = IMatrix4x4
                    (xScale, 0, 0, 0,
                     0, yScale, 0, 0,
                     0, 0, -(far+near)/(far-near), -1,
                     0, 0, -2.*near*far/(far-near), 0);
            P = P.GetTranspose();//.eval();
        }

        return P;
    }



    void IQCamera::LookAt(const IVector3 &eye, const IVector3 &at, const IVector3 &up)
    {

          // http://www.opengl.org/sdk/docs/man2/xhtml/gluLookAt.xml
          // Normalize vector from at to eye
          IVector3 F = eye-at;
          mAtDist = F.Length();
          F.Normalize();
          // Project up onto plane orthogonal to F and normalize
          assert(up.Cross(F).Length() > MACHINE_EPSILON && "(eye-at) x up ≈ 0");
          const IVector3 proj_up = (up-(up.Dot(F))*F).Normalized();
          IQuaternion a,b;
          a = a.LookRotation(IVector3(0,0,-1),-F);
          b = b.LookRotation(a.GetRotMatrix()*IVector3(0,1,0),proj_up);
          mRotationConj = (b*a).GetConjugate();


         //mRotationConj = IQuaternion::LookAtLH(eye,at,up);
         mRotationConj = IQuaternion::LookAtRH( eye , at , up ); //IQuaternion::LookAtLH(eye,at,up);

          mTranslation = /*mRotationConj.GetRotMatrix() **/ eye;




          //std::cout<<"m_at_dist: "<<mRotationConj<<std::endl;
          //cout<<"proj_up: "<<proj_up.transpose()<<endl;
          //cout<<"F: "<<F.transpose()<<endl;
          //cout<<"eye(): "<<this->eye().transpose()<<endl;
          //cout<<"at(): "<<this->at().transpose()<<endl;
          //cout<<"eye()-at(): "<<(this->eye()-this->at()).normalized().transpose()<<endl;
          //cout<<"eye-this->eye(): "<<(eye-this->eye()).squaredNorm()<<endl;
          //assert(           (eye-this->eye()).squaredNorm() < DOUBLE_EPS);
          //assert((F-(this->eye()-this->at()).normalized()).squaredNorm() <
          //  DOUBLE_EPS);
          //assert(           (at-this->at()).squaredNorm() < DOUBLE_EPS);
          //assert(        (proj_up-this->up()).squaredNorm() < DOUBLE_EPS);

    }



    IMatrix4x4 IQCamera::ViewMatrix() const
    {
        return (  IMatrix4x4::CreateRotation(mRotationConj).GetInverse() * IMatrix4x4::CreateTranslation(-mTranslation));
    }



    void IQCamera::DollyZoom(const float &da)
    {
#ifndef NDEBUG
        IVector3 old_at = GetAt();
#endif
        const float old_angle = mAngle;
        if(old_angle + da < IGL_CAMERA_MIN_ANGLE)
        {
            mOrthographic = true;
        }
        else if(old_angle + da > IGL_CAMERA_MIN_ANGLE)
        {
            mOrthographic = false;
        }

        if(!mOrthographic)
        {
            mAngle += da;
            mAngle = IMin(float(89.),IMax(float(IGL_CAMERA_MIN_ANGLE),mAngle));
            // change in distance
            const float s =  (2.*ITan(old_angle/2./180.*M_PI)) /
                             (2.*ITan(mAngle/2./180.*M_PI));

            mAngleOfViewSize = (4.*ITan(mAngle/2./180.*M_PI));//1.f/s;

            const float old_at_dist = mAtDist;
            mAtDist = old_at_dist * s;
            Dolly(IVector3(0,0,1)*(mAtDist - old_at_dist));
            assert((old_at.LengthSquare() < MACHINE_EPSILON));
        }

    }

    void IQCamera::Orbit(const IQuaternion &q , const IVector3& origin)
    {
        //IVector3 old_at = GetAt();
        // at should be fixed
        //
        // at_1 = R_1 * t_1 - R_1 * z = at_0
        // t_1 = R_1' * (at_0 + R_1 * z)
        //mRotationConj = q.GetConjugate();
        //mTranslation  =  (/*mRotationConj.GetRotMatrix() **/ IVector3(0,0,1) * mAtDist) * mRotationConj.GetRotMatrix().GetTranspose();
        //assert((old_at - GetAt()).squaredNorm() < DOUBLE_EPS);

        LookAt(q.GetRotMatrix() * (IVector3::Z * mAtDist) , origin , IVector3::Y );

    }



    IMatrix4x4 IQCamera::GetAffine() const
    {
        IMatrix4x4 t = IMatrix4x4::IDENTITY;
        t =     IMatrix4x4::CreateRotation(mRotationConj).GetInverse();
        t = t * IMatrix4x4::CreateTranslation(-mTranslation);
        return t;
    }

    IMatrix4x4 IQCamera::GetInverse() const
    {
        IMatrix4x4 t = IMatrix4x4::IDENTITY;
        t =     IMatrix4x4::CreateTranslation(-mTranslation);
        t = t * IMatrix4x4::CreateRotation(mRotationConj);
        return t;
    }

    IVector3 IQCamera::GetEye() const
    {
        return -(IVector3(0,0,0) * GetInverse());
    }

    IVector3 IQCamera::GetAt() const
    {
        return -(IVector3(0,0,-1)*mAtDist) * GetInverse();
    }

    IVector3 IQCamera::GetUp() const
    {
        return IMatrix4x4::CreateRotation(mRotationConj.GetConjugate()) * IVector3(0,1,0);
    }

    IVector3 IQCamera::UnitPlane() const
    {
        // Distance of center pixel to eye
        const float d = 1.0;
        const float a = mAspect;
        const float theta = mAngle*M_PI/180.;
        const float w =2.*ISqrt(-d*d/(a*a*IPow(tan(0.5*theta),2.)-1.))*a*ITan(0.5*theta);
        const float h = w/a;
        return IVector3(w*0.5,h*0.5,-d);
    }

    void IQCamera::Dolly(const IVector3 &dv)
    {
        mTranslation += dv;
    }









}
