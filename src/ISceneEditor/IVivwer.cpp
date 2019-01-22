#include "IVivwer.h"

IVivwer::IVivwer()
{

}

void IVivwer::ProjectionPerspectiv(float FieldOfView, float aspect, float NearPlane, float FarPlane)
{
    // mCamera.ProjectionPerspectiveMatrix(FieldOfView, aspect, NearPlane, FarPlane);

    (*mCamera)->mAngle  = FieldOfView;
    (*mCamera)->mAspect = aspect;
    (*mCamera)->mNear   = NearPlane;
    (*mCamera)->mFar    = FarPlane;
}

void IVivwer::BeginLookSceneOpenGL()
{
    glViewport(0, 0, mWidth , mHeight );

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glClearDepth(1.0);

    //(*mCamera).LookAt( mEye , mCenter , mUp  );

    (*mCamera)->LookAt( mEye , mCenter , mUp );


    IEngine::IMatrix4x4 M1;
    M1 = IEngine::IMatrix4x4::CreateRotationAxis( IEngine::IVector3::Y , AngleYawCamera  );
    M1 = IEngine::IMatrix4x4::CreateRotationAxis( IEngine::IVector3::X , AngleRollCamera ) * M1;
    // mEye = M1 * (IEngine::IVector3::Z * MoveLegth_FocusCamera);


    (*mCamera)->mAtDist = MoveLegth_FocusCamera;
    (*mCamera)->Orbit(IEngine::IQuaternion(M1),IEngine::IVector3::ZERO);



    //        glMatrixMode(GL_PROJECTION);
    //        glLoadMatrixf((*mCamera).getProjectionMatrix());

    //        glMatrixMode(GL_MODELVIEW);
    //        glLoadMatrixf((*mCamera).getTransformMatrix());

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf((*mCamera)->ProjectionMatrix());

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf((*mCamera)->ViewMatrix());

}

void IVivwer::Update()
{
    IEngine::IMatrix4x4 M1;
    M1 = IEngine::IMatrix4x4::CreateRotationAxis( IEngine::IVector3::Y , AngleYawCamera  );
    M1 = IEngine::IMatrix4x4::CreateRotationAxis( IEngine::IVector3::X , AngleRollCamera ) * M1;
    // mEye = M1 * (IEngine::IVector3::Z * MoveLegth_FocusCamera);

    (*mCamera)->Orbit(IEngine::IQuaternion(M1));

}
