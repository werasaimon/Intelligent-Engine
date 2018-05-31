#ifndef ICREATORSCENE_H
#define ICREATORSCENE_H

#include "IScene.h"
#include "IEngine/IEngine.h"

class ICreatorScene : public IScene
{

 private:

    //------- camera --------//
    IGeometry::ICamera mCamera;

    IGeometry::IVector3 mEye;
    IGeometry::IVector3 mCenter;
    IGeometry::IVector3 mUp;

    float MoveLegth_FocusCamera;

    float AngleYawCamera  = 0;
    float AngleRollCamera = 0;

    bool  ControlCamera = false;

    //-----------------------//




    //=========================//

    float mWidth;
    float mHeight;

    float mouseOldX;
    float mouseOldY;

    float mouseX;
    float mouseY;

    //=========================//

    std::vector<IGeometry::IMeshModel*> mGMeshModels;

    int mSelectedIndexID;

    //======== User Interface ========//

//    IGeometry::UserRay UserRay;


public:

    IGeometry::Context   GizmoContext;
    IGeometry::IGizmo    GizmoManipulator;

    IGeometry::IDrawList GizmoDrawList;

 public:

     ICreatorScene();

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

#endif // ICREATORSCENE_H
