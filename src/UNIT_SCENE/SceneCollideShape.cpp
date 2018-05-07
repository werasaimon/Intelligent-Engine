#include "SceneCollideShape.h"

#include <Qt>
#include "freeglut/GL/freeglut.h"

#include <iostream>
using namespace std;

namespace
{


 void DrawMesh( const IGeometry::IMeshModel *_model )
 {


     // For each triangular face
     for (uint i=0; i<_model->getIndicess().size(); i+=3)
     {

         // Get the three vertices index of the current face
         uint v1 = _model->getIndicess()[i+0];
         uint v2 = _model->getIndicess()[i+1];
         uint v3 = _model->getIndicess()[i+2];

         assert(v1 < _model->getNbVertices());
         assert(v2 < _model->getNbVertices());
         assert(v3 < _model->getNbVertices());

         // Compute the normal of the face
         IGeometry::IVector3  a = _model->getVertex(v1);
         IGeometry::IVector3  b = _model->getVertex(v2);
         IGeometry::IVector3  c = _model->getVertex(v3);


         glBegin(GL_LINE_LOOP);
           glVertex3fv(a);
           glVertex3fv(b);
           glVertex3fv(c);
         glEnd();

     }
 }

}

SceneCollideShape::SceneCollideShape()
{

}

bool SceneCollideShape::initialization()
{
     SceneCamera::initialization();


     mCollisionWorld = new IPhysics::ICollisionWorld();



    {
         int index = 0;

         mTransform[index].setPosition( IPhysics::IVector3(1,8,0) );
         mCollisionBody[index] = mCollisionWorld->createCollisionBody(mTransform[0]);

         //-------------------------------------------------//

         const IPhysics::IVector3 halfSize(1,1,1);

         mMeshes[0] = new IGeometry::IMeshCreateBox( halfSize );
         IGeometry::IMatrix4x4 position_and_orintation;
         //position_and_orintation = position_and_orintation.createTranslation(0,0,5);
         //position_and_orintation = position_and_orintation.createRotationAroundAxis(41.1,40.2,14.0);
         mMeshes[0]->setTransformMatrix(position_and_orintation);

         //-------------------------------------------------//

         IPhysics::ICollisionShape *shape_box = new IPhysics::IBoxShape( halfSize );

         IPhysics::ITransform local_transform;
         local_transform.setFromOpenGL(position_and_orintation.getData());

         mCollisionBody[index]->addCollisionShape( shape_box , 10.f , local_transform );

         //-------------------------------------------------//

         mCollisionBody[index]->setType( IPhysics::BodyType::DYNAMIC );

         mPhysIndexes[mMeshes[0]] = index;

     ///--------------------------------------------------------///

     }




     {
         //=========================================================//

         int index = 1;

         ///--------------------------------------------------------///

         mTransform[index].setPosition( IPhysics::IVector3(0,-1,3) );

         mCollisionBody[index] = mCollisionWorld->createCollisionBody(mTransform[1]);

         ///--------------------------------------------------------///

         const IPhysics::IVector3 halfSize(10,1,10);

         mMeshes[1] = new IGeometry::IMeshCreateBox( halfSize );
         IGeometry::IMatrix4x4 position_and_orintation;
         //position_and_orintation = position_and_orintation.createTranslation(0,0,5);
         //position_and_orintation = position_and_orintation.createRotationAroundAxis(41.1,40.2,14.0);
         mMeshes[1]->setTransformMatrix(position_and_orintation);


         IPhysics::ICollisionShape *shape_box = new IPhysics::IBoxShape(  halfSize );

         IPhysics::ITransform   local_transform;
         mCollisionBody[1]->addCollisionShape( shape_box , 10.f , local_transform );

         mCollisionBody[1]->setType( IPhysics::BodyType::DYNAMIC );

         mPhysIndexes[mMeshes[1]] = index;

         ///--------------------------------------------------------///

         //=========================================================//

     }

}

void SceneCollideShape::render(float FrameTime)
{
     SceneCamera::render(FrameTime);

     mCollisionWorld->FindNewContacts();

     {
         glPushMatrix();
         float m[16];
         mCollisionBody[mPhysIndexes[mMeshes[0]]]->getTransform().getOpenGLMatrix(m);
         glMultMatrixf(m);
         DrawMesh(mMeshes[0]);
         glPopMatrix();
     }


     {
         glPushMatrix();
         float m[16];
         mCollisionBody[mPhysIndexes[mMeshes[1]]]->getTransform().getOpenGLMatrix(m);
         glMultMatrixf(m);
         DrawMesh(mMeshes[1]);
         glPopMatrix();
     }

}

void SceneCollideShape::update()
{
     SceneCamera::update();
}

void SceneCollideShape::resize(float width, float height)
{
     SceneCamera::resize( width , height );
}

void SceneCollideShape::mouseMove(float x, float y, int button)
{
     SceneCamera::mouseMove( x , y , button );
}

void SceneCollideShape::mousePress(float x, float y, int button)
{
     SceneCamera::mousePress( x , y , button );
}

void SceneCollideShape::mouseReleasePress(float x, float y, int button)
{
     SceneCamera::mouseReleasePress( x , y , button );
}

void SceneCollideShape::mouseWheel(float delta)
{
     SceneCamera::mouseWheel(delta);
}


void SceneCollideShape::keyboard(int key)
{
     SceneCamera::keyboard( key );


     if( key == Qt::Key_8 )
     {
         mTransform[0].setPosition( mTransform[0].getPosition() + IPhysics::IVector3(0,0,0.1) );
         mCollisionBody[0]->setTransform(mTransform[0]);
     }

     if( key == Qt::Key_2 )
     {
         mTransform[0].setPosition( mTransform[0].getPosition() + IPhysics::IVector3(0,0,-0.1) );
         mCollisionBody[0]->setTransform(mTransform[0]);
     }


     if( key == Qt::Key_4 )
     {
         mTransform[0].setPosition( mTransform[0].getPosition() + IPhysics::IVector3(0.1,0,0) );
         mCollisionBody[0]->setTransform(mTransform[0]);
     }

     if( key == Qt::Key_6 )
     {
         mTransform[0].setPosition( mTransform[0].getPosition() + IPhysics::IVector3(-0.1,0,0) );
         mCollisionBody[0]->setTransform(mTransform[0]);
     }


     if( key == Qt::Key_7 )
     {
         mTransform[0].setPosition( mTransform[0].getPosition() + IPhysics::IVector3(0,-0.1,0) );
         mCollisionBody[0]->setTransform(mTransform[0]);
     }

     if( key == Qt::Key_9 )
     {
         mTransform[0].setPosition( mTransform[0].getPosition() + IPhysics::IVector3(0,0.1,0) );
         mCollisionBody[0]->setTransform(mTransform[0]);
     }


//     if( key == Qt::Key_1 )
//     {
//         Quaternion Q =  mTransform.getRotation() + Quaternion( IPhysics::IVector3(0,0.01,0.001) , 0 ) * mTransform.getRotation();
//         mTransform.setBasis(Q.getMatrix());
//         mCollisionBody->setTransform(mTransform);
//     }
}

void SceneCollideShape::destroy()
{
     SceneCamera::destroy();
}
