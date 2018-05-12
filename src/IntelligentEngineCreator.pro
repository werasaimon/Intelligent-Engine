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
    IEngine/iPhysics/body/IBody.cpp \
    IEngine/iPhysics/body/ICollisionBody.cpp \
    IEngine/iPhysics/body/IRigidBody.cpp \
    IEngine/iPhysics/collision/broad_phase/IBroadPhase.cpp \
    IEngine/iPhysics/collision/broad_phase/IDynamicAABBTree.cpp \
    IEngine/iPhysics/collision/contacts/IGenerationContactPoints.cpp \
    IEngine/iPhysics/collision/contacts/IQuickClipping.cpp \
    IEngine/iPhysics/collision/narrow_phase/gjk_epa/IGjkEpa.cpp \
    IEngine/iPhysics/collision/narrow_phase/ICollisionAlgorithm.cpp \
    IEngine/iPhysics/collision/narrow_phase/ICollisionAlgorithmGjkEpa.cpp \
    IEngine/iPhysics/collision/shapes/IBoxShape.cpp \
    IEngine/iPhysics/collision/shapes/ICollisionShape.cpp \
    IEngine/iPhysics/collision/shapes/IConvexShape.cpp \
    IEngine/iPhysics/collision/shapes/ISphereShape.cpp \
    IEngine/iPhysics/collision/IAABB.cpp \
    IEngine/iPhysics/collision/IProxyShape.cpp \
    IEngine/iPhysics/collision/IRaycastInfo.cpp \
    IEngine/iPhysics/common/timer/ITimer.cpp \
    IEngine/iPhysics/common/rpSettings.cpp \
    IEngine/iPhysics/dynamics/contacts/IContactManager.cpp \
    IEngine/iPhysics/dynamics/contacts/IContactManifold.cpp \
    IEngine/iPhysics/dynamics/contacts/IContactManifoldSet.cpp \
    IEngine/iPhysics/dynamics/contacts/IContactPoint.cpp \
    IEngine/iPhysics/dynamics/contacts/IContactSolver.cpp \
    IEngine/iPhysics/dynamics/joint/IBallAndSocketJoint.cpp \
    IEngine/iPhysics/dynamics/joint/IConstraintSolver.cpp \
    IEngine/iPhysics/dynamics/joint/IFixedJoint.cpp \
    IEngine/iPhysics/dynamics/joint/IHingeJoint.cpp \
    IEngine/iPhysics/dynamics/joint/IJoint.cpp \
    IEngine/iPhysics/dynamics/joint/ISliderJoint.cpp \
    IEngine/iPhysics/dynamics/material/IMaterial.cpp \
    IEngine/iPhysics/dynamics/ICollisionWorld.cpp \
    IEngine/iPhysics/dynamics/IDynamicsWorld.cpp \
    IEngine/iPhysics/dynamics/IIsland.cpp \
    IEngine/iPhysics/dynamics/IOverlappingPair.cpp

HEADERS  += widgetcreatorengine.h \
    IEngine/IGeometry/camera/ICamera.h \
    IEngine/IGeometry/math/IColor.h \
    IEngine/IGeometry/math/IMatematical.h \
    IEngine/IGeometry/mesh-utility/IMeshCreateBox.h \
    IEngine/IGeometry/mesh-utility/IMeshModel.h \
    IEngine/IGeometry/mesh-utility/ITexture2D.h \
    IEngine/IGeometry/transform/IObject3D.h \
    IEngine/IGeometry/IGeometry.h \
    IEngine/iMath/iFunc.h \
    IEngine/iMath/ILine3D.h \
    IEngine/iMath/ILineSegment3D.h \
    IEngine/iMath/ILorentzVector.h \
    IEngine/iMath/IMaths.h \
    IEngine/iMath/IMatrix2x2.h \
    IEngine/iMath/IMatrix3x3.h \
    IEngine/iMath/IMatrix4x4.h \
    IEngine/iMath/IPlane.h \
    IEngine/iMath/IQuaternion.h \
    IEngine/iMath/IRay.h \
    IEngine/iMath/ITransform.h \
    IEngine/iMath/IVector2D.h \
    IEngine/iMath/IVector3D.h \
    IEngine/iMath/IVector4D.h \
    IEngine/iPhysics/body/IBody.h \
    IEngine/iPhysics/body/ICollisionBody.h \
    IEngine/iPhysics/body/IRigidBody.h \
    IEngine/iPhysics/collision/broad_phase/IBroadPhase.h \
    IEngine/iPhysics/collision/broad_phase/IDynamicAABBTree.h \
    IEngine/iPhysics/collision/contacts/IGenerationContactPoints.h \
    IEngine/iPhysics/collision/contacts/IQuickClipping.h \
    IEngine/iPhysics/collision/narrow_phase/gjk_epa/IGjkEpa.h \
    IEngine/iPhysics/collision/narrow_phase/ICollisionAlgorithm.h \
    IEngine/iPhysics/collision/narrow_phase/ICollisionAlgorithmGjkEpa.h \
    IEngine/iPhysics/collision/narrow_phase/ICollisionShapeInfo.h \
    IEngine/iPhysics/collision/shapes/IBoxShape.h \
    IEngine/iPhysics/collision/shapes/ICollisionShape.h \
    IEngine/iPhysics/collision/shapes/IConvexShape.h \
    IEngine/iPhysics/collision/shapes/ISphereShape.h \
    IEngine/iPhysics/collision/IAABB.h \
    IEngine/iPhysics/collision/IProxyShape.h \
    IEngine/iPhysics/collision/IRaycastInfo.h \
    IEngine/iPhysics/common/math/IMatematical.h \
    IEngine/iPhysics/common/memory/IList.h \
    IEngine/iPhysics/common/memory/IStack.h \
    IEngine/iPhysics/common/timer/ITimer.h \
    IEngine/iPhysics/common/IScalar.h \
    IEngine/iPhysics/common/ISettings.h \
    IEngine/iPhysics/dynamics/contacts/IContactManager.h \
    IEngine/iPhysics/dynamics/contacts/IContactManifold.h \
    IEngine/iPhysics/dynamics/contacts/IContactManifoldSet.h \
    IEngine/iPhysics/dynamics/contacts/IContactPoint.h \
    IEngine/iPhysics/dynamics/contacts/IContactSolver.h \
    IEngine/iPhysics/dynamics/joint/IBallAndSocketJoint.h \
    IEngine/iPhysics/dynamics/joint/IConstraintSolver.h \
    IEngine/iPhysics/dynamics/joint/IFixedJoint.h \
    IEngine/iPhysics/dynamics/joint/IHingeJoint.h \
    IEngine/iPhysics/dynamics/joint/IJoint.h \
    IEngine/iPhysics/dynamics/joint/ISliderJoint.h \
    IEngine/iPhysics/dynamics/material/IMaterial.h \
    IEngine/iPhysics/dynamics/ICollisionWorld.h \
    IEngine/iPhysics/dynamics/IDynamicsWorld.h \
    IEngine/iPhysics/dynamics/IIsland.h \
    IEngine/iPhysics/dynamics/IOverlappingPair.h \
    IEngine/iPhysics/dynamics/ITimeStep.h \
    IEngine/iPhysics/IPhysicsEngine.h \
    IEngine/IEngine.h

FORMS    += widgetcreatorengine.ui
