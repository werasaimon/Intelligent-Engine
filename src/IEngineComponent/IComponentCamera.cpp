#include "IComponentCamera.h"

namespace IEngineComponent
{

IComponentCamera::IComponentCamera(IEngine::IQCamera *IQCamera , unsigned int id)
 : IComponent3D (id) ,
    mCamera(IQCamera)
{
   mType = ComponentType::COMPONENT_CAMERA;
}

IComponentCamera::~IComponentCamera()
{
    if(mCamera != nullptr)
    {
        delete mCamera;
        mCamera = nullptr;
    }
}

IEngine::IQCamera *IComponentCamera::GetCamera() const
{
    assert(mCamera);
    return mCamera;
}

IEngine::IQCamera *IComponentCamera::Camera()
{
    assert(mCamera);
    return mCamera;
}



}

