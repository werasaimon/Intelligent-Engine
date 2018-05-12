#-------------------------------------------------
#
# Project created by QtCreator 2018-05-12T03:44:07
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = IntelligentEngineCreator
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES +=\
        widgetcreatorengine.cpp \
    main_IECreator.cpp \
    IEngine/IGeometry/camera/ICamera.cpp \
    IEngine/IGeometry/math/IColor.cpp \
    IEngine/IGeometry/mesh-utility/IMeshCreateBox.cpp \
    IEngine/IGeometry/mesh-utility/IMeshModel.cpp \
    IEngine/IGeometry/mesh-utility/ITexture2D.cpp \
    IEngine/IGeometry/transform/IObject3D.cpp \
    IEngine/IPhysics/body/IBody.cpp \
    IEngine/IPhysics/body/ICollisionBody.cpp \
    IEngine/IPhysics/body/IRigidBody.cpp \
    IEngine/IPhysics/collision/broad_phase/IBroadPhase.cpp \
    IEngine/IPhysics/collision/broad_phase/IDynamicAABBTree.cpp \
    IEngine/IPhysics/collision/contacts/IGenerationContactPoints.cpp \
    IEngine/IPhysics/collision/contacts/IQuickClipping.cpp \
    IEngine/IPhysics/collision/narrow_phase/gjk_epa/IGjkEpa.cpp \
    IEngine/IPhysics/collision/narrow_phase/ICollisionAlgorithm.cpp \
    IEngine/IPhysics/collision/narrow_phase/ICollisionAlgorithmGjkEpa.cpp \
    IEngine/IPhysics/collision/shapes/IBoxShape.cpp \
    IEngine/IPhysics/collision/shapes/ICollisionShape.cpp \
    IEngine/IPhysics/collision/shapes/IConvexShape.cpp \
    IEngine/IPhysics/collision/shapes/ISphereShape.cpp \
    IEngine/IPhysics/collision/IAABB.cpp \
    IEngine/IPhysics/collision/IProxyShape.cpp \
    IEngine/IPhysics/collision/IRaycastInfo.cpp \
    IEngine/IPhysics/common/timer/ITimer.cpp \
    IEngine/IPhysics/common/rpSettings.cpp \
    IEngine/IPhysics/dynamics/contacts/IContactManager.cpp \
    IEngine/IPhysics/dynamics/contacts/IContactManifold.cpp \
    IEngine/IPhysics/dynamics/contacts/IContactManifoldSet.cpp \
    IEngine/IPhysics/dynamics/contacts/IContactPoint.cpp \
    IEngine/IPhysics/dynamics/contacts/IContactSolver.cpp \
    IEngine/IPhysics/dynamics/joint/IBallAndSocketJoint.cpp \
    IEngine/IPhysics/dynamics/joint/IConstraintSolver.cpp \
    IEngine/IPhysics/dynamics/joint/IFixedJoint.cpp \
    IEngine/IPhysics/dynamics/joint/IHingeJoint.cpp \
    IEngine/IPhysics/dynamics/joint/IJoint.cpp \
    IEngine/IPhysics/dynamics/joint/ISliderJoint.cpp \
    IEngine/IPhysics/dynamics/material/IMaterial.cpp \
    IEngine/IPhysics/dynamics/ICollisionWorld.cpp \
    IEngine/IPhysics/dynamics/IDynamicsWorld.cpp \
    IEngine/IPhysics/dynamics/IIsland.cpp \
    IEngine/IPhysics/dynamics/IOverlappingPair.cpp

HEADERS  += widgetcreatorengine.h \
    IEngine/IGeometry/camera/ICamera.h \
    IEngine/IGeometry/math/IColor.h \
    IEngine/IGeometry/math/IMatematical.h \
    IEngine/IGeometry/mesh-utility/IMeshCreateBox.h \
    IEngine/IGeometry/mesh-utility/IMeshModel.h \
    IEngine/IGeometry/mesh-utility/ITexture2D.h \
    IEngine/IGeometry/transform/IObject3D.h \
    IEngine/IGeometry/IGeometry.h \
    IEngine/IMath/iFunc.h \
    IEngine/IMath/ILine3D.h \
    IEngine/IMath/ILineSegment3D.h \
    IEngine/IMath/ILorentzVector.h \
    IEngine/IMath/IMaths.h \
    IEngine/IMath/IMatrix2x2.h \
    IEngine/IMath/IMatrix3x3.h \
    IEngine/IMath/IMatrix4x4.h \
    IEngine/IMath/IPlane.h \
    IEngine/IMath/IQuaternion.h \
    IEngine/IMath/IRay.h \
    IEngine/IMath/ITransform.h \
    IEngine/IMath/IVector2D.h \
    IEngine/IMath/IVector3D.h \
    IEngine/IMath/IVector4D.h \
    IEngine/IPhysics/body/IBody.h \
    IEngine/IPhysics/body/ICollisionBody.h \
    IEngine/IPhysics/body/IRigidBody.h \
    IEngine/IPhysics/collision/broad_phase/IBroadPhase.h \
    IEngine/IPhysics/collision/broad_phase/IDynamicAABBTree.h \
    IEngine/IPhysics/collision/contacts/IGenerationContactPoints.h \
    IEngine/IPhysics/collision/contacts/IQuickClipping.h \
    IEngine/IPhysics/collision/narrow_phase/gjk_epa/IGjkEpa.h \
    IEngine/IPhysics/collision/narrow_phase/ICollisionAlgorithm.h \
    IEngine/IPhysics/collision/narrow_phase/ICollisionAlgorithmGjkEpa.h \
    IEngine/IPhysics/collision/narrow_phase/ICollisionShapeInfo.h \
    IEngine/IPhysics/collision/shapes/IBoxShape.h \
    IEngine/IPhysics/collision/shapes/ICollisionShape.h \
    IEngine/IPhysics/collision/shapes/IConvexShape.h \
    IEngine/IPhysics/collision/shapes/ISphereShape.h \
    IEngine/IPhysics/collision/IAABB.h \
    IEngine/IPhysics/collision/IProxyShape.h \
    IEngine/IPhysics/collision/IRaycastInfo.h \
    IEngine/IPhysics/common/math/IMatematical.h \
    IEngine/IPhysics/common/memory/IList.h \
    IEngine/IPhysics/common/memory/IStack.h \
    IEngine/IPhysics/common/timer/ITimer.h \
    IEngine/IPhysics/common/IScalar.h \
    IEngine/IPhysics/common/ISettings.h \
    IEngine/IPhysics/dynamics/contacts/IContactManager.h \
    IEngine/IPhysics/dynamics/contacts/IContactManifold.h \
    IEngine/IPhysics/dynamics/contacts/IContactManifoldSet.h \
    IEngine/IPhysics/dynamics/contacts/IContactPoint.h \
    IEngine/IPhysics/dynamics/contacts/IContactSolver.h \
    IEngine/IPhysics/dynamics/joint/IBallAndSocketJoint.h \
    IEngine/IPhysics/dynamics/joint/IConstraintSolver.h \
    IEngine/IPhysics/dynamics/joint/IFixedJoint.h \
    IEngine/IPhysics/dynamics/joint/IHingeJoint.h \
    IEngine/IPhysics/dynamics/joint/IJoint.h \
    IEngine/IPhysics/dynamics/joint/ISliderJoint.h \
    IEngine/IPhysics/dynamics/material/IMaterial.h \
    IEngine/IPhysics/dynamics/ICollisionWorld.h \
    IEngine/IPhysics/dynamics/IDynamicsWorld.h \
    IEngine/IPhysics/dynamics/IIsland.h \
    IEngine/IPhysics/dynamics/IOverlappingPair.h \
    IEngine/IPhysics/dynamics/ITimeStep.h \
    IEngine/IPhysics/IPhysicsEngine.h \
    IEngine/IEngine.h

FORMS    += widgetcreatorengine.ui
