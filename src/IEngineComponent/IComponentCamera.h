#ifndef ICOMPONENTCAMERA_H
#define ICOMPONENTCAMERA_H

#include "IComponent3D.h"

namespace IEngineComponent
{


class IComponentCamera : public IComponent3D
{
 private:

    IEngine::IQCamera *mCamera;

    /// Private copy-constructor
    IComponentCamera(const  IComponentCamera& body) = delete;

    /// Private assignment operator
    IComponentCamera& operator=(const  IComponentCamera& body) = delete;

 public:

     IComponentCamera(IEngine::IQCamera *IQCamera , unsigned int id);
    ~IComponentCamera();



    IEngine::IQCamera* operator->()
    {
        assert(mCamera);
        return  mCamera;
    }
    IEngine::IQCamera& operator* ()
    {
        assert(mCamera);
        return *mCamera;
    }


    IEngine::IQCamera *GetCamera() const;
    IEngine::IQCamera *Camera();
};

}

#endif // ICOMPONENTCAMERA_H
