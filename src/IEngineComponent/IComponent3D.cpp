#include "IComponent3D.h"



namespace IEngineComponent
{



IComponent3D::IComponent3D(unsigned int id)
  : IHierarchyNode(id)
{
    
}

IComponent3D::~IComponent3D()
{

}

ComponentType IComponent3D::GetType() const
{
    return mType;
}



}

