#ifndef SCENECOLLIDESHAPE_H
#define SCENECOLLIDESHAPE_H

#include "SceneCamera.h"
#include "IEngine/IPhysics/IPhysicsEngine.h"
#include "IEngine/IGeometry/IGeometry.h"

#include <map>

class SceneCollideShape : public SceneCamera
{

   IPhysics::ICollisionWorld      *mCollisionWorld;


   IPhysics::ITransform            mTransform[2];
   IPhysics::ICollisionBody       *mCollisionBody[2];


   IGeometry::IMeshModel           *mMeshes[2];

   std::map<IGeometry::IMeshModel* , int > mPhysIndexes;

  public:

      SceneCollideShape();

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

#endif // SCENECOLLIDESHAPE_H
