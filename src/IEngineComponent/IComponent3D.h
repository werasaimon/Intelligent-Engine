#ifndef ICOMPONENT_H
#define ICOMPONENT_H

#include "../IEngine/IEngine.h"
#include "IHierarchy/IHierarchyNode.h"

namespace IEngineComponent
{


enum ComponentType { COMPONENT_NODE ,
                     COMPONENT_CAMERA ,
                     COMPONENT_MESH3 };

class IComponent3D : public IHierarchyNode
{

        friend class IScene3D;

protected:


        //unsigned int mID;


    /// Type Component
    ComponentType mType;

    /// Transfomation from this global space
    IEngine::IMatrix4x4 mTransformMatrix;

    /// Axis Aligned Bouding Box from local space
    IEngine::IAxisAlignedBox3D mAABBox;

    /// constructor
    IComponent3D(unsigned int id);

//    /// Private copy-constructor
//    IComponent3D(const  IComponent3D& body) = delete;

//    /// Private assignment operator
//    IComponent3D& operator=(const  IComponent3D& body) = delete;


public:

    virtual ~IComponent3D();
    ComponentType GetType() const;


    IEngine::IMatrix4x4 GetTransformHierarchy() const
    {
        if(_parent != nullptr )
        {
            return static_cast<IComponent3D*>(_parent)->GetTransformHierarchy() * mTransformMatrix;
        }

        return mTransformMatrix;
    }


    IEngine::IMatrix4x4 GetTransform() const
    {
        return mTransformMatrix;
    }


    void SetTransform( const IEngine::IMatrix4x4& _transform )
    {
        mTransformMatrix = _transform;
    }


    IEngine::IAxisAlignedBox3D GetAxisAlignedBoxTransform(const IEngine::IMatrix4x4& _TransformMatrix) const
    {
        return mAABBox.GetAxisAlignedBoxTransform(_TransformMatrix);
    }

    IEngine::IAxisAlignedBox3D GetAxisAlignedBoxTransform() const
    {
        return mAABBox.GetAxisAlignedBoxTransform(GetTransformHierarchy());
        //return mAABBox.GetAxisAlignedBoxTransform(GetTransform());
    }

    IEngine::IAxisAlignedBox3D GetAxisAlignedBox() const
    {
        return mAABBox;
    }
};


}

#endif // ICOMPONENT_H
