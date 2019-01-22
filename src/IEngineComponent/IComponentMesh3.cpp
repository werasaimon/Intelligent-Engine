#include "IComponentMesh3.h"


namespace IEngineComponent
{


//IComponentMesh3::IComponentMesh3(bool bWantNormals, bool bWantColors, bool bWantUVs, bool bWantTriGroups)
//    :mMesh(new IEngine::IMesh3(bWantNormals,bWantColors,bWantUVs,bWantTriGroups))
//{

//}

//IComponentMesh3::IComponentMesh3(IEngine::MeshComponents flags)
//    :mMesh(new IEngine::IMesh3(flags))
//{

//}

//IComponentMesh3::IComponentMesh3(IComponentMesh3 &copy, bool bCompact, bool bWantNormals, bool bWantColors, bool bWantUVs)
//    : mMesh(new IEngine::IMesh3(copy.mMesh,bCompact,bWantNormals,bWantColors,bWantUVs))
//{
//   mType = ComponentType::COMPONENT_MESH3;
//}

//IComponentMesh3::IComponentMesh3(const IComponentMesh3 &copy, bool bCompact, IEngine::MeshComponents flags)
// : mMesh(new IEngine::IMesh3(copy.mMesh,bCompact,flags))
//{

//}

    IComponentMesh3::IComponentMesh3(IEngine::IMesh3 *mesh, unsigned int id)
    : IComponent3D (id) , mMesh(mesh)
    {
         assert(mMesh);
         mAABBox = mMesh->GetAxisAlignedBox();
         mType = ComponentType::COMPONENT_MESH3;
    }

    IComponentMesh3::~IComponentMesh3()
    {
        std::cout << " Component Mesh Delete Destructor" << std::endl;
        if(mMesh != nullptr)
        {
            delete mMesh;
            mMesh = nullptr;
        }
    }

    IEngine::IMesh3 *IComponentMesh3::GetMesh() const
    {
        assert(mMesh);
        return mMesh;
    }

    IEngine::IMesh3 *IComponentMesh3::Mesh()
    {
        assert(mMesh);
        return mMesh;
    }

    IComponentMesh3 *IComponentMesh3::Clone()
    {
        IEngine::IMesh3 *clone_mesh = new IEngine::IMesh3();
        clone_mesh->Copy( *mMesh );
        IComponentMesh3 *Clone = new IComponentMesh3( clone_mesh , GetId() );
        Clone->mTransformMatrix = this->mTransformMatrix;
        Clone->mAABBox = this->mAABBox;
        //Clone->Copy(this);
        return Clone;
    }


    IComponentMesh3 *IComponentMesh3::CloneIstance()
    {
         IEngine::IMesh3 *clone_mesh = mMesh;
         IComponentMesh3 *Clone = new IComponentMesh3( clone_mesh , GetId() );
         Clone->mTransformMatrix = this->mTransformMatrix;
         Clone->mAABBox = this->mAABBox;
         Clone->Copy(this);
         return Clone;
    }



    void IComponentMesh3::CreatePramitiveToMesh(IEngine::MeshGenerator::CuboidDescriptor _cube_descriptor, IComponentMesh3 *_ComponentMesh )
    {
        assert(_ComponentMesh);
        _ComponentMesh->mMesh->ClearFullData();
        IEngine::MeshGenerators::GenerateMesh( _cube_descriptor , *_ComponentMesh->mMesh );
        _ComponentMesh->mAABBox = _ComponentMesh->mMesh->GetAxisAlignedBox();
    }

    void IComponentMesh3::CreatePramitiveToMesh(IEngine::MeshGenerator::EllipsoidDescriptor _ellipsoid_descriptor, IComponentMesh3 *_ComponentMesh)
    {
        assert(_ComponentMesh);
        _ComponentMesh->mMesh->ClearFullData();
        IEngine::MeshGenerators::GenerateMesh( _ellipsoid_descriptor , *_ComponentMesh->mMesh );
         _ComponentMesh->mAABBox = _ComponentMesh->mMesh->GetAxisAlignedBox();
    }

    void IComponentMesh3::CreatePramitiveToMesh(IEngine::MeshGenerator::ConeDescriptor _conus_descriptor, IComponentMesh3 *_ComponentMesh)
    {
        assert(_ComponentMesh);
        _ComponentMesh->mMesh->ClearFullData();
        IEngine::MeshGenerators::GenerateMesh( _conus_descriptor , *_ComponentMesh->mMesh );
         _ComponentMesh->mAABBox = _ComponentMesh->mMesh->GetAxisAlignedBox();
    }

    void IComponentMesh3::CreatePramitiveToMesh(IEngine::MeshGenerator::CylinderDescriptor _celinder_descriptor, IComponentMesh3 *_ComponentMesh)
    {
        assert(_ComponentMesh);
        _ComponentMesh->mMesh->ClearFullData();
        IEngine::MeshGenerators::GenerateMesh( _celinder_descriptor , *_ComponentMesh->mMesh );
        _ComponentMesh->mAABBox = _ComponentMesh->mMesh->GetAxisAlignedBox();
    }

    void IComponentMesh3::CreatePramitiveToMesh(IEngine::MeshGenerator::CapsuleDescriptor _capsule_descriptor, IComponentMesh3 *_ComponentMesh)
    {
        assert(_ComponentMesh);
        _ComponentMesh->mMesh->ClearFullData();
        IEngine::MeshGenerators::GenerateMesh( _capsule_descriptor , *_ComponentMesh->mMesh );
        _ComponentMesh->mAABBox = _ComponentMesh->mMesh->GetAxisAlignedBox();
    }

    void IComponentMesh3::CreatePramitiveToMesh(IEngine::MeshGenerator::CurveDescriptor _curve_descriptor, IComponentMesh3 *_ComponentMesh)
    {
        assert(_ComponentMesh);
        _ComponentMesh->mMesh->ClearFullData();
        IEngine::MeshGenerators::GenerateMesh( _curve_descriptor , *_ComponentMesh->mMesh );
        _ComponentMesh->mAABBox = _ComponentMesh->mMesh->GetAxisAlignedBox();
    }

    void IComponentMesh3::CreatePramitiveToMesh(IEngine::MeshGenerator::BezierPatchDescriptor _bezier_descriptor, IComponentMesh3 *_ComponentMesh)
    {
        assert(_ComponentMesh);
        _ComponentMesh->mMesh->ClearFullData();
        IEngine::MeshGenerators::GenerateMesh( _bezier_descriptor , *_ComponentMesh->mMesh );
        _ComponentMesh->mAABBox = _ComponentMesh->mMesh->GetAxisAlignedBox();
    }


}
