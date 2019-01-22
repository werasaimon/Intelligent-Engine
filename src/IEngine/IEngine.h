#ifndef IENGINE_H
#define IENGINE_H


#include "IMath/IMaths.h"
#include "IPhysics/IPhysicsEngine.h"
//#include "IGeometry/IGeometryEngine.h"

#include "ICamera/ICamera.h"
#include "ICamera/IQCamera.h"
#include "ICommon/IColor.h"
#include "IObject/IObjectTransform.h"
#include "IObject/IObjectAxisAlignedBox.h"
#include "IMesh/IMeshGenerator.h"
#include "IMesh/IMeshModel.h"


namespace IEngine
{



   //=======================  IMath-Module ============================//

    using IAxisAlignedBox3D = IMath::IAxisAlignedBox3D<float>;
    using ILorentzVector4 = IMath::ILorentzVector<float>;
    using IQuaternion = IMath::IQuaternion<float>;
    using IVector2 = IMath::IVector2D<float>;
    using IVector3 = IMath::IVector3D<float>;
    using IVector4 = IMath::IVector4D<float>;
    using IMatrix2x2 = IMath::IMatrix2x2<float>;
    using IMatrix3x3 = IMath::IMatrix3x3<float>;
    using IMatrix4x4 = IMath::IMatrix4x4<float>;
    using IRay = IMath::IRay<float>;

    using IAxisAlignedBox3Di = IMath::IAxisAlignedBox3D<int>;
    using ILorentzVector4i = IMath::ILorentzVector<int>;
    using IQuaternioni = IMath::IQuaternion<int>;
    using IVector2i = IMath::IVector2D<int>;
    using IVector3i = IMath::IVector3D<int>;
    using IVector4i = IMath::IVector4D<int>;
    using IMatrix2x2i = IMath::IMatrix2x2<int>;
    using IMatrix3x3i = IMath::IMatrix3x3<int>;
    using IMatrix4x4i = IMath::IMatrix4x4<int>;
    using IRayi = IMath::IRay<int>;


   //======================= IGeometry-Module ===========================//


    namespace MeshGenerators = IEngine::MeshGenerator;

    using ICamera  = IEngine::ICamera;
    using IQCamera = IEngine::IQCamera;

    using IColor4 = IEngine::IMeshModel;
    using IObjectTransform = IEngine::IObjectTransform;
    using IObjectAxisAlignedBox = IEngine::IObjectAxisAlignedBox;
    using IMesh3 = IEngine::IMeshModel;


    using MeshResult = IEngine::MeshResult;
    using MeshComponents = IEngine::MeshComponents;
    using MeshHints = IEngine::MeshHints;




   //======================= IPhysics-Module ===========================//

    using IBody = IPhysics::IBody;
    using IRigidBody = IPhysics::IRigidBody;

    using IJointInfo = IPhysics::IJointInfo;
    using IJoint = IPhysics::IJoint;

    using IBallAndSocketJointInfo = IPhysics::IBallAndSocketJointInfo;
    using IBallAndSocketJoint = IPhysics::IBallAndSocketJoint;

    using IFixedJointInfo = IPhysics::IFixedJointInfo;
    using IFixedJoint = IPhysics::IFixedJoint;

    using IHingeJointInfo = IPhysics::IHingeJointInfo;
    using IHingeJoint = IPhysics::IHingeJoint;

    using ISliderJointInfo = IPhysics::ISliderJoint;
    using ISliderJoint = IPhysics::ISliderJoint;


    using ICollisionBody = IPhysics::ICollisionBody;
    using ICollisionShapeConvex = IPhysics::ICollisionShapeConvex;
    using ICollisionShape = IPhysics::ICollisionShape;
    using ICollisionShapeBox = IPhysics::ICollisionShapeBox;
    using ICollisionShapeSphere = IPhysics::ICollisionShapeSphere;
    using ICollisionShapeHull = IPhysics::ICollisionShapeHull;
    using ICollisionShapeInfo = IPhysics::ICollisionShapeInfo;

    using IAABB = IPhysics::IAABB;
    using IDynamicAABBTree = IPhysics::IDynamicAABBTree;
    using IDynamicAABBTreeOverlapCallback = IPhysics::IDynamicAABBTreeOverlapCallback;
    using IDynamicAABBTreeRaycastCallback = IPhysics::IDynamicAABBTreeRaycastCallback;

    using ICollisionWorld = IPhysics::ICollisionWorld;
    using IDynamicWorld   = IPhysics::IDynamicsWorld;



}


#endif // IENGINE_H
