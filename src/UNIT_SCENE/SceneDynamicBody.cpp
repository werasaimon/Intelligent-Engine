#include "SceneDynamicBody.h"



#include <Qt>
#include "freeglut/GL/freeglut.h"

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

SceneDynamicBody::SceneDynamicBody()
{

}

bool SceneDynamicBody::initialization()
{
    SceneCamera::initialization();

    mPause = true;
    mDynamicWorld = new IPhysics::IDynamicsWorld(IPhysics::IVector3(0,-30,0));


   {
        int index_phys = 0;
        IPhysics::IVector3 _pos(0,-10,0);
        IPhysics::IVector3 _rot(0,0,0);
        IPhysics::ITransform transform_box(_pos , IPhysics::IQuaternion(_rot,0));
        IPhysics::IRigidBody *PhysicsBox = mDynamicWorld->createRigidBody(transform_box);

        IGeometry::IVector3 halfSize(10,1,10);
        IGeometry::IMeshModel *ModelBox = new IGeometry::IMeshCreateBox( halfSize );
        mModels.push_back(ModelBox);

        mPhysMapIndexes[ModelBox] = index_phys;

        PhysicsBox->addCollisionShape( new IPhysics::IBoxShape(halfSize) , 10 , IPhysics::ITransform::identity() );
        PhysicsBox->setType( IPhysics::BodyType::STATIC );

        mPhysicsBodies.push_back(PhysicsBox);
   }


    IPhysics::IRigidBody *PrevBody;

    for( int i = 0; i < 10; ++i )
    {

        //int index_phys = 1+i;
        IPhysics::IVector3 _pos(0,2+i * 5,i / 4.f);
        IPhysics::IVector3 _rot(0,0,0);
        IPhysics::ITransform transform(_pos , IPhysics::IQuaternion(_rot,0));
        IPhysics::IRigidBody *Physics = mDynamicWorld->createRigidBody(transform);

        IGeometry::IVector3 halfSize(1,1,1);
        IGeometry::IMeshModel *ModelBox1 = new IGeometry::IMeshCreateBox( halfSize );
        IGeometry::IMeshModel *ModelBox2 = new IGeometry::IMeshCreateBox( halfSize );

        IGeometry::IVector3 local_pos1 = IGeometry::IVector3::X *   2.0 + IGeometry::IVector3::Y  * 0.5;
        IGeometry::IVector3 local_pos2 = IGeometry::IVector3::X *  -2.0;

        ModelBox1->translateWorld(  local_pos1 );
        ModelBox2->translateWorld(  local_pos2 );

        mModels.push_back(ModelBox1);
        mModels.push_back(ModelBox2);


        mPhysicsBodies.push_back(Physics);

        mPhysMapIndexes[ModelBox1] = mPhysicsBodies.size()-1;
        mPhysMapIndexes[ModelBox2] = mPhysicsBodies.size()-1;



        IPhysics::ITransform transform_local1;
        IPhysics::ITransform transform_local2;

        transform_local1.setPosition( local_pos1 );
        transform_local2.setPosition( local_pos2 );

        Physics->addCollisionShape( new IPhysics::IBoxShape(halfSize) , 10 , transform_local1 );
        Physics->addCollisionShape( new IPhysics::IBoxShape(halfSize) , 10 , transform_local2 );


        Physics->setType( IPhysics::BodyType::DYNAMIC );


        if( i > 0 )
        {
            IPhysics::IVector3 anchor = (Physics->getTransform().getPosition() + PrevBody->getTransform().getPosition()) * 0.5f;
            IPhysics::IBallAndSocketJointInfo joint_info_init(Physics , PrevBody , anchor );
            mDynamicWorld->createJoint(joint_info_init);
        }

        PrevBody = Physics;

    }


}

void SceneDynamicBody::render(float FrameTime)
{
     SceneCamera::render(FrameTime);

     for (unsigned int i = 0; i < mModels.size(); ++i)
     {
         glPushMatrix();

         if( mPhysicsBodies[mPhysMapIndexes[mModels[i]]] != NULL)
         {
            float m[16];
            mPhysicsBodies[mPhysMapIndexes[mModels[i]]]->getTransform().getOpenGLMatrix(m);
            glMultMatrixf(m);
         }

         glMultMatrixf( mModels[i]->getTransformMatrix() );
         DrawMesh(mModels[i]);
         glPopMatrix();
     }
}

void SceneDynamicBody::update()
{
     SceneCamera::update();

     if( !mPause )
     {

         /**
         for (unsigned int i = 0; i < mPhysicsBodies.size(); ++i)
         {
            if( mPhysicsBodies[i]->getTransform().getPosition().length() > 50 )
            {
                mDynamicWorld->destroyBody(mPhysicsBodies[i]);
            }
         }
         /**/

         mDynamicWorld->update(1.f/60.f);


     }
}

void SceneDynamicBody::resize(float width, float height)
{
     SceneCamera::resize( width , height );
}

void SceneDynamicBody::mouseMove(float x, float y, int button)
{
     SceneCamera::mouseMove( x , y , button );
}

int num=0;
void SceneDynamicBody::mousePress(float x, float y, int button)
{
     SceneCamera::mousePress( x , y , button );

      if ( button == Qt::RightButton )
      {

          IGeometry::IMatrix4x4 M =  IGeometry::IMatrix4x4::createTranslation(mCamera.getEye()) * mOrientCamera;
          IPhysics::ITransform transform;
          transform.setFromOpenGL(M.getData());


          IPhysics::IRigidBody *Physics = mDynamicWorld->createRigidBody(transform);

          IGeometry::IVector3 halfSize(1,1,1);
          IGeometry::IMeshModel *ModelBox = new IGeometry::IMeshCreateBox( halfSize );
          mModels.push_back(ModelBox);


          Physics->addCollisionShape( new IPhysics::IBoxShape(halfSize) , 10 , IPhysics::ITransform::identity() );
          Physics->setType( IPhysics::BodyType::DYNAMIC );


          Physics->ApplyForceToCenterOfMass( (mCamera.getEye() - mCamera.getCenter()).getUnit() * -20000.0 );

          mPhysicsBodies.push_back(Physics);



          mPhysMapIndexes[ModelBox] = mPhysicsBodies.size() - 1.0;
      }
}

void SceneDynamicBody::mouseReleasePress(float x, float y, int button)
{
     SceneCamera::mouseReleasePress( x , y , button );
}

void SceneDynamicBody::mouseWheel(float delta)
{
     SceneCamera::mouseWheel(delta);
}

void SceneDynamicBody::keyboard(int key)
{
     SceneCamera::keyboard( key );

     if(key == Qt::Key_Space) mPause = !mPause;
}

void SceneDynamicBody::destroy()
{
     SceneCamera::destroy();

     delete mDynamicWorld;
}
