#ifndef SCENEDYNAMICBODY_H
#define SCENEDYNAMICBODY_H

#include <map>

#include "SceneCamera.h"
#include "IEngine/iPhysics/IPhysicsEngine.h"
#include "IEngine/IGeometry/IGeometry.h"




class SceneDynamicBody : public SceneCamera
{

    bool mPause;
    IPhysics::IDynamicsWorld *mDynamicWorld;

    std::map<IGeometry::IMeshModel* , int > mPhysMapIndexes;

    std::vector<IPhysics::IRigidBody*>  mPhysicsBodies;
    std::vector<IGeometry::IMeshModel*> mModels;


public:
    SceneDynamicBody();

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

#endif // SCENEDYNAMICBODY_H
