#include "SceneCamera.h"
#include <QVector>

#include <GL/gl.h>


SceneCamera::SceneCamera()
{

}

bool SceneCamera::initialization()
{
    NullAllKey();

    mTimeStep = 1.0/60.0;

    float  width  = 600;
    float  height = 400;

    float aspect = width / height;
    float zNear  = 1.0;
    float zFar   = 1024;
    float fov    = 45.0;

    mCamera.ProjectionPerspectiveMatrix( fov , aspect , zNear , zFar );

    mEye    =  IGeometry::IVector3(0,1,-10);
    mCenter =  IGeometry::IVector3(0,0,0);
    mUp     =  IGeometry::IVector3(0,1,0);

    mDistanceCamera = 10.f;

    mWidth  = width;
    mHeight = height;

    mCamera.translateWorld( IGeometry::IVector3(0,0,-10) );
}

void SceneCamera::render(float FrameTime)
{
    glViewport(0, 0, mWidth , mHeight );


    mCamera.LookAt( mEye , mCenter , mUp  );

    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(mCamera.getProjectionMatrix().getData());

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(mCamera.getTransformMatrix().getData());


    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_CULL_FACE);
}

void SceneCamera::update()
{
  mCenter = mEye + mOrientCamera * IGeometry::IVector3::Z;
}

void SceneCamera::resize(float width, float height)
{
    mWidth  = width;
    mHeight = height;

    float aspect = mWidth / mHeight;
    float zNear  = 1.0;
    float zFar   = 1024;
    float fov    = 45.0;

    mCamera.ProjectionPerspectiveMatrix( fov , aspect , zNear , zFar );
}

void SceneCamera::mouseMove(float x, float y, int button)
{
    mMouseButton = button;

    float aspect = mWidth / mHeight;
    float m_x = ((x / mWidth ) - 0.5f) * aspect * 0.834;
    float m_y = ((y / mHeight) - 0.5f) * 0.834;

    float speedX = (m_x - oldX);
    float speedY = (m_y - oldY);

    float coff = 1.5f;

    angle_X += speedX * coff;
    angle_Y -= speedY * coff;

    oldX = m_x;
    oldY = m_y;

    mOrientCamera = IGeometry::IMatrix4x4::createRotationAxis( IGeometry::IVector3::Y , angle_X );
    mOrientCamera = IGeometry::IMatrix4x4::createRotationAxis( IGeometry::IVector3::X , angle_Y ) * mOrientCamera;
}

void SceneCamera::mousePress(float x, float y, int button)
{ 
    mMouseButton = button;

    float aspect = mWidth / mHeight;
    float m_x = ((x / mWidth ) - 0.5f) * aspect * 0.834;
    float m_y = ((y / mHeight) - 0.5f) * 0.834;

    oldX = m_x;
    oldY = m_y;
}

void SceneCamera::mouseReleasePress(float x, float y, int button)
{
    mMouseButton = button;

    oldX = x;
    oldY = y;
}

void SceneCamera::mouseWheel(float delta)
{

}

void SceneCamera::keyboard(int key)
{
        float coff = 0.91;

        if( key == Qt::Key_W)
        {
            mEye += mOrientCamera * IGeometry::IVector3::Z * coff;
        }

        if( key == Qt::Key_S)
        {
            mEye -= mOrientCamera * IGeometry::IVector3::Z * coff;
        }

        if( key == Qt::Key_A)
        {
            mEye += mOrientCamera * IGeometry::IVector3::X * coff;
        }

        if( key == Qt::Key_D)
        {
            mEye -= mOrientCamera * IGeometry::IVector3::X * coff;
        }
}

void SceneCamera::destroy()
{

}
