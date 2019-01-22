#ifndef ICOMPONENTMESH3_H
#define ICOMPONENTMESH3_H

#include "IComponent3D.h"

namespace IEngineComponent
{

class IComponentMesh3 : public IComponent3D
{

 private:

    IEngine::IMesh3 *mMesh;

//    /// Private copy-constructor
//    IComponentMesh3(const  IComponentMesh3& body) = delete;

//    /// Private assignment operator
//    IComponentMesh3& operator=(const  IComponentMesh3& body) = delete;


 public:

    /// constructor
     IComponentMesh3(IEngine::IMesh3* mesh, unsigned int id);
    ~IComponentMesh3();



    IEngine::IMesh3* operator->() { return  mMesh; }
    IEngine::IMesh3& operator* () { return *mMesh; }


    IEngine::IMesh3 *GetMesh() const;
    IEngine::IMesh3 *Mesh();


    IComponentMesh3 *Clone();
    IComponentMesh3 *CloneIstance();


    void DeleteRandomVertex()
    {
        assert(mMesh);

        mMesh->DeleteRandomVertex();
    }


    //======================================= Push primitive geometry to meshes ========================================//
    static void CreatePramitiveToMesh( IEngine::MeshGenerators::CuboidDescriptor _cube_descriptor , IComponentMesh3* _ComponentMesh );
    static void CreatePramitiveToMesh( IEngine::MeshGenerators::EllipsoidDescriptor _ellipsoid_descriptor , IComponentMesh3* _ComponentMesh );
    static void CreatePramitiveToMesh( IEngine::MeshGenerators::ConeDescriptor _conus_descriptor , IComponentMesh3* _ComponentMesh );
    static void CreatePramitiveToMesh( IEngine::MeshGenerators::CylinderDescriptor _celinder_descriptor , IComponentMesh3* _ComponentMesh );
    static void CreatePramitiveToMesh( IEngine::MeshGenerators::CapsuleDescriptor _capsule_descriptor , IComponentMesh3* _ComponentMesh );
    static void CreatePramitiveToMesh( IEngine::MeshGenerators::CurveDescriptor _curve_descriptor , IComponentMesh3* _ComponentMesh );
    static void CreatePramitiveToMesh( IEngine::MeshGenerators::BezierPatchDescriptor _bezier_descriptor , IComponentMesh3* _ComponentMesh );
};

}

#endif // ICOMPONENTMESH3_H
