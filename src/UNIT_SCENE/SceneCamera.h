#ifndef SCENECAMERA_H
#define SCENECAMERA_H

#include "Scene.h"
#include "IGeometry/camera/ICamera.h"

class SceneCamera : public Scene
{
protected:

  IGeometry::ICamera mCamera;

  IGeometry::IVector3 mEye;
  IGeometry::IVector3 mCenter;
  IGeometry::IVector3 mUp;

  IGeometry::IMatrix4x4 mOrientCamera;

  /// width and height size window
  float mWidth;
  float mHeight;
  float mTimeStep;

  /// Distance
  float mDistanceCamera;


 //=========================//

  float oldX = 0;
  float oldY = 0;

  float angle_X = 0;
  float angle_Y = 0;


public:
    SceneCamera();

    bool initialization();
    void render(float FrameTime);
    void update();
    void resize( float width , float height );

    void mouseMove( float x , float y  , int button);
    void mousePress( float x , float y , int button );
    void mouseReleasePress( float x , float y , int button );
    void mouseWheel( float delta );

    void keyboard(int key );
    void destroy();
};

#endif // SCENECAMERA_H
