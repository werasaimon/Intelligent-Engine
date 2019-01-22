#-------------------------------------------------
#
# Project created by QtCreator 2018-05-12T03:44:07
#
#-------------------------------------------------

QT  += core gui qml quick quickwidgets


greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = IntelligentEngineCreator
TEMPLATE = app

CONFIG+=qml_debug
#---------------------------------------#
#QMAKE_CXXFLAGS += -m32
QMAKE_CXXFLAGS += -std=c++11

#LIBS += -lGL -lGLU -lglut
#LIBS += -lX11



#Linux
linux: {

#Android
 android: {
  LIBS +=  -lGLESv1_CM -lGLESv2
}

#Linux default
 !android: {
   LIBS += -lGL -lGLU -lglut #-lGLEW
}

}

#Windows
win32: {
   LIBS += -lopengl32 -lglu32 #-lglew32
}

#Windows
win64: {
   LIBS += -lopengl32 -lglu32 #-lglew32
}


QMAKE_RPATHDIR += $ORIGIN/lib
#---------------------------------------#

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
    ISceneEditor/IGizmo/IGizmoTransformScale.cpp \
    ISceneEditor/IGizmo/IGizmoTransformRotate.cpp \
    ISceneEditor/IGizmo/IGizmoTransformRender.cpp \
    ISceneEditor/IGizmo/IGizmoTransformMove.cpp \
    IEngine/IPhysics/IBody/IMaterial/IMaterial.cpp \
    IEngine/IPhysics/IBody/IBody.cpp \
    IEngine/IPhysics/IBody/ICollisionBody.cpp \
    IEngine/IPhysics/IBody/IRigidBody.cpp \
#    IEngine/IPhysics/IBody/IRigidRealtivity.cpp \
    IEngine/IPhysics/ICollision/ICollisionBroadPhase/IBroadPhase.cpp \
    IEngine/IPhysics/ICollision/ICollisionBroadPhase/IDynamicAABBTree.cpp \
    IEngine/IPhysics/ICollision/ICollisionContact/IContactGenerationPoints.cpp \
    IEngine/IPhysics/ICollision/ICollisionContact/IContactManager.cpp \
    IEngine/IPhysics/ICollision/ICollisionContact/IContactManifold.cpp \
    IEngine/IPhysics/ICollision/ICollisionContact/IContactManifoldSet.cpp \
    IEngine/IPhysics/ICollision/ICollisionContact/IContactPoint.cpp \
    IEngine/IPhysics/ICollision/ICollisionNarrowPhase/gjk_epa/IGjkEpa.cpp \
    IEngine/IPhysics/ICollision/ICollisionNarrowPhase/ICollisionAlgorithm.cpp \
    IEngine/IPhysics/ICollision/ICollisionNarrowPhase/ICollisionAlgorithmGjkEpa.cpp \
    IEngine/IPhysics/ICollision/ICollisionShapes/ICollisionShape.cpp \
    IEngine/IPhysics/ICollision/IAABB.cpp \
    IEngine/IPhysics/ICollision/IOverlappingPair.cpp \
    IEngine/IPhysics/ICollision/IProxyShape.cpp \
    IEngine/IPhysics/ICollision/IRaycastInfo.cpp \
    IEngine/IPhysics/ICommon/ISettings.cpp \
    IEngine/IPhysics/ICommon/ITimer.cpp \
    IEngine/IPhysics/IDynamics/IConstraintSolver/IBallAndSocketJoint.cpp \
    IEngine/IPhysics/IDynamics/IConstraintSolver/IConstraintSolver.cpp \
    IEngine/IPhysics/IDynamics/IConstraintSolver/IContactSolver.cpp \
    IEngine/IPhysics/IDynamics/IConstraintSolver/IFixedJoint.cpp \
    IEngine/IPhysics/IDynamics/IConstraintSolver/IHingeJoint.cpp \
    IEngine/IPhysics/IDynamics/IConstraintSolver/IJoint.cpp \
    IEngine/IPhysics/IDynamics/IConstraintSolver/ISliderJoint.cpp \
    IEngine/IPhysics/IDynamics/ICollisionWorld.cpp \
    IEngine/IPhysics/IDynamics/IDynamicsWorld.cpp \
#    IEngine/IPhysics/IDynamics/IDynamicsWorldRealtivity.cpp \
    IEngine/IPhysics/IDynamics/IIntegrateUtil.cpp \
    IEngine/IPhysics/IDynamics/IIsland.cpp \
    IEngine/IPhysics/IGeometry/QuickHull/QuickHull.cpp \
    IEngine/IPhysics/IGeometry/IQuickClipping.cpp \
    IEngine/IPhysics/ICollision/ICollisionShapes/ICollisionShapeBox.cpp \
    IEngine/IPhysics/ICollision/ICollisionShapes/ICollisionShapeSphere.cpp \
    IEngine/IPhysics/ICollision/ICollisionShapes/ICollisionShapeHull.cpp \
    IEngine/IPhysics/ICollision/ICollisionShapes/ICollisionShapeConvex.cpp \
    IEngine/ICommon/IColor.cpp \
    IEngine/ICamera/IQCamera.cpp \
    IEngine/ICamera/ICamera.cpp \
    IEngine/IObject/IObjectTransform.cpp \
    IEngine/IObject/IObjectAxisAlignedBox.cpp \
    IEngine/IMesh/IMeshModel.cpp \
    IEngine/IMesh/IMeshGeneratorTorusKnot.cpp \
    IEngine/IMesh/IMeshGeneratorTorus.cpp \
    IEngine/IMesh/IMeshGeneratorSpiral.cpp \
    IEngine/IMesh/IMeshGeneratorPipe.cpp \
    IEngine/IMesh/IMeshGeneratorPie.cpp \
    IEngine/IMesh/IMeshGeneratorEllipsoid.cpp \
    IEngine/IMesh/IMeshGeneratorDetails.cpp \
    IEngine/IMesh/IMeshGeneratorCylinder.cpp \
    IEngine/IMesh/IMeshGeneratorCurve.cpp \
    IEngine/IMesh/IMeshGeneratorCuboid.cpp \
    IEngine/IMesh/IMeshGeneratorCone.cpp \
    IEngine/IMesh/IMeshGeneratorCapsule.cpp \
    IEngine/IMesh/IMeshGeneratorBezierPatch.cpp \
    ISceneEditor/IVivwer.cpp \
    ISceneEditor/ISceneEditor.cpp \
    ISceneEditor/ISceneCompare.cpp \
    IEngineComponent/IScene3D.cpp \
    IEngineComponent/IComponentMesh3.cpp \
    IEngineComponent/IComponentCamera.cpp \
    IEngineComponent/IComponent3D.cpp \
    ieditglwidget.cpp \
    InterfaceShellCreatePrimitive.cpp \
    IEngineComponent/IHierarchy/IHierarchyScene.cpp \
    IEngineComponent/IHierarchy/IHierarchyNode.cpp \
    main.cpp \
    widgetcreatorengine.cpp





HEADERS  += \
    IEngine/IEngine.h \
    ISceneEditor/IGizmo/IUGizmo.h \
    ISceneEditor/IGizmo/IMathGizmo.h \
    ISceneEditor/IGizmo/IGizmoTransformScale.h \
    ISceneEditor/IGizmo/IGizmoTransformRotate.h \
    ISceneEditor/IGizmo/IGizmoTransformRender.h \
    ISceneEditor/IGizmo/IGizmoTransformMove.h \
    ISceneEditor/IGizmo/IGizmoTransform.h \
    IEngine/IMath/IAlgebra.h \
    IEngine/IMath/IAxisAlignedBox3D.h \
    IEngine/IMath/IComplex.h \
    IEngine/IMath/IFunc.h \
    IEngine/IMath/ILine3D.h \
    IEngine/IMath/ILineSegment3D.h \
    IEngine/IMath/ILorentzVector.h \
    IEngine/IMath/IMaths.h \
    IEngine/IMath/IMatrix2x2.h \
    IEngine/IMath/IMatrix3x3.h \
    IEngine/IMath/IMatrix4x4.h \
    IEngine/IMath/IOctonion.h \
    IEngine/IMath/IPlane.h \
    IEngine/IMath/IQuaternion.h \
    IEngine/IMath/IRay.h \
    IEngine/IMath/IReal.h \
    IEngine/IMath/IScalarType.h \
    IEngine/IMath/ISpherical.h \
    IEngine/IMath/ITransform.h \
    IEngine/IMath/IVector.h \
    IEngine/IMath/IVector2D.h \
    IEngine/IMath/IVector3D.h \
    IEngine/IMath/IVector4D.h \
    IEngine/IMath/IVectorType.h \
    IEngine/IMemory/dvector.h \
    IEngine/IMemory/iterator_util.h \
    IEngine/IMemory/refcount_vector.h \
    IEngine/IMemory/small_list_set.h \
    IEngine/IPhysics/IBody/IMaterial/IMaterial.h \
    IEngine/IPhysics/IBody/IBody.h \
    IEngine/IPhysics/IBody/ICollisionBody.h \
    IEngine/IPhysics/IBody/IRigidBody.h \
#    IEngine/IPhysics/IBody/IRigidRealtivity.h \
    IEngine/IPhysics/ICollision/ICollisionBroadPhase/IBroadPhase.h \
    IEngine/IPhysics/ICollision/ICollisionBroadPhase/IDynamicAABBTree.h \
    IEngine/IPhysics/ICollision/ICollisionContact/IContactGenerationPoints.h \
    IEngine/IPhysics/ICollision/ICollisionContact/IContactManager.h \
    IEngine/IPhysics/ICollision/ICollisionContact/IContactManifold.h \
    IEngine/IPhysics/ICollision/ICollisionContact/IContactManifoldSet.h \
    IEngine/IPhysics/ICollision/ICollisionContact/IContactPoint.h \
    IEngine/IPhysics/ICollision/ICollisionNarrowPhase/gjk_epa/IGjkEpa.h \
    IEngine/IPhysics/ICollision/ICollisionNarrowPhase/ICollisionAlgorithm.h \
    IEngine/IPhysics/ICollision/ICollisionNarrowPhase/ICollisionAlgorithmGjkEpa.h \
    IEngine/IPhysics/ICollision/ICollisionNarrowPhase/ICollisionShapeInfo.h \
    IEngine/IPhysics/ICollision/ICollisionShapes/ICollisionShape.h \
    IEngine/IPhysics/ICollision/IAABB.h \
    IEngine/IPhysics/ICollision/IOverlappingPair.h \
    IEngine/IPhysics/ICollision/IProxyShape.h \
    IEngine/IPhysics/ICollision/IRaycastInfo.h \
    IEngine/IPhysics/ICommon/IMemory/IList.h \
    IEngine/IPhysics/ICommon/IMemory/IStack.h \
    IEngine/IPhysics/ICommon/IMaths.h \
    IEngine/IPhysics/ICommon/IScalar.h \
    IEngine/IPhysics/ICommon/ISettings.h \
    IEngine/IPhysics/ICommon/ITimer.h \
    IEngine/IPhysics/IDynamics/IConstraintSolver/IBallAndSocketJoint.h \
    IEngine/IPhysics/IDynamics/IConstraintSolver/IConstraintSolver.h \
    IEngine/IPhysics/IDynamics/IConstraintSolver/IContactSolver.h \
    IEngine/IPhysics/IDynamics/IConstraintSolver/IFixedJoint.h \
    IEngine/IPhysics/IDynamics/IConstraintSolver/IHingeJoint.h \
    IEngine/IPhysics/IDynamics/IConstraintSolver/IJoint.h \
    IEngine/IPhysics/IDynamics/IConstraintSolver/ISliderJoint.h \
    IEngine/IPhysics/IDynamics/ICollisionWorld.h \
    IEngine/IPhysics/IDynamics/IDynamicsWorld.h \
#    IEngine/IPhysics/IDynamics/IDynamicsWorldRealtivity.h \
    IEngine/IPhysics/IDynamics/IIntegrateUtil.h \
    IEngine/IPhysics/IDynamics/IIsland.h \
    IEngine/IPhysics/IDynamics/ITimeStep.h \
    IEngine/IPhysics/IGeometry/QuickHull/Structs/Mesh.hpp \
    IEngine/IPhysics/IGeometry/QuickHull/Structs/Plane.hpp \
    IEngine/IPhysics/IGeometry/QuickHull/Structs/Pool.hpp \
    IEngine/IPhysics/IGeometry/QuickHull/Structs/Ray.hpp \
    IEngine/IPhysics/IGeometry/QuickHull/Structs/Vector3.hpp \
    IEngine/IPhysics/IGeometry/QuickHull/Structs/VertexDataSource.hpp \
    IEngine/IPhysics/IGeometry/QuickHull/ConvexHull.hpp \
    IEngine/IPhysics/IGeometry/QuickHull/HalfEdgeMesh.hpp \
    IEngine/IPhysics/IGeometry/QuickHull/MathUtils.hpp \
    IEngine/IPhysics/IGeometry/QuickHull/QuickHull.hpp \
    IEngine/IPhysics/IGeometry/QuickHull/Types.hpp \
    IEngine/IPhysics/IGeometry/IQuickClipping.h \
    IEngine/IPhysics/IGeometry/IQuickConvexHull.h \
    IEngine/IPhysics/IPhysicsEngine.h \
    IEngine/IEngine.h \
    IEngine/IPhysics/ICollision/ICollisionShapes/ICollisionShapeBox.h \
    IEngine/IPhysics/ICollision/ICollisionShapes/ICollisionShapeSphere.h \
    IEngine/IPhysics/ICollision/ICollisionShapes/ICollisionShapeHull.h \
    IEngine/IPhysics/ICollision/ICollisionShapes/ICollisionShapeConvex.h \
    IEngine/ICommon/IMatematical.h \
    IEngine/ICommon/IColor.h \
    IEngine/ICommon/IBezierPatch.h \
    IEngine/ICommon/IBernsteinPolynomial.h \
    IEngine/ICamera/IQCamera.h \
    IEngine/ICamera/ICamera.h \
    IEngine/IObject/IObjectTransform.h \
    IEngine/IObject/IObjectAxisAlignedBox.h \
    IEngine/IMesh/IMeshModel.h \
    IEngine/IMesh/IMeshGeneratorDetails.h \
    IEngine/IMesh/IMeshGenerator.h \
    IEngine/IMesh/IIndexUtil.h \
    IEngine/IMesh/IIintersectionUtil.h \
    ISceneEditor/IVivwer.h \
    ISceneEditor/ISceneEditor.h \
    ISceneEditor/ISceneCompare.h \
    ISceneEditor/ISceneConservationCheckPoint.h \
    IEngineComponent/glwidget.h \
    IEngineComponent/IScene3D.h \
    IEngineComponent/IComponentMesh3.h \
    IEngineComponent/IComponentLib.h \
    IEngineComponent/IComponentCamera.h \
    IEngineComponent/IComponent3D.h \
    ieditglwidget.h \
    InterfaceShellPanel.h \
    InterfaceShellCreatePrimitive.h \
    IEngineComponent/IHierarchy/IHierarchyScene.h \
    IEngineComponent/IHierarchy/IHierarchyNode.h \
    widgetcreatorengine.h \
    ISceneEditor/IUtilityOpenGLDraw.hpp






FORMS    += widgetcreatorengine.ui

DISTFILES +=

RESOURCES += \
    qrc.qrc


