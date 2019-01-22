#ifndef IVIVWER_H
#define IVIVWER_H

#include "IEngineComponent/IComponentCamera.h"
#include <GL/gl.h>

class IVivwer
{
public:

    IVivwer();


    IVivwer(IEngineComponent::IComponentCamera *_Camera)
    : mCamera(_Camera)
    {

    }

    //-----------------------//
    float mWidth;
    float mHeight;

    IEngine::IVector3 mEye;
    IEngine::IVector3 mCenter;
    IEngine::IVector3 mUp;

    float MoveLegth_FocusCamera;

    float AngleYawCamera  = 0;
    float AngleRollCamera = 0;
    //-----------------------//

    IEngineComponent::IComponentCamera *mCamera;

    //-----------------------//

    void ProjectionPerspectiv(float  FieldOfView, float  aspect, float  NearPlane, float  FarPlane);

    void BeginLookSceneOpenGL();

    void Update();
};

#endif // IVIVWER_H
