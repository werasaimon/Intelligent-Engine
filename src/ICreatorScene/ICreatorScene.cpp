#include "ICreatorScene.h"

#include <GL/gl.h>
#include <Qt>


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


     IGeometry::IVector3 mousePoint;

     IGeometry::IVector3 HitPoint;

    bool isSelected = false;


    bool m_mouse_is = false;

}

ICreatorScene::ICreatorScene()
{

}


//-----------------------------------------------------//

bool ICreatorScene::initialization()
{
    NullAllKey();

    float  width  = 600;
    float  height = 400;

    //GizmoContext.mWidth  = width;
    //GizmoContext.mHeight = height;

    float aspect = width / height;
    float zNear  = 1.0;
    float zFar   = 1024;
    float fov    = 45.0;

    mCamera.ProjectionPerspectiveMatrix( fov , aspect , zNear , zFar );

    mEye    =  IGeometry::IVector3(0,1,-10);
    mCenter =  IGeometry::IVector3(0,0,0);
    mUp     =  IGeometry::IVector3(0,1,0);

    MoveLegth_FocusCamera = 10;

    mWidth  = width;
    mHeight = height;

    mCamera.translateWorld( IGeometry::IVector3(0,0,-10) );

    //===================================================//

    mSelectedIndexID = -1;

    for (int i = 0; i < 10; ++i)
    {
        IGeometry::IVector3 pos( 5-rand()%10 ,
                                 5-rand()%10 ,
                                 5-rand()%10);

        IGeometry::IVector3 rot( 5-rand()%10 ,
                                 5-rand()%10 ,
                                 5-rand()%10);

        IGeometry::IVector3 halfSize(1,1,1);
        IGeometry::IMeshModel *ModelBox = new IGeometry::IMeshCreateBox( halfSize );
        ModelBox->translateWorld(pos);
        if(i >5 ) ModelBox->rotateWorld( rot.normalized() , rot.length() );


//        IGeometry::IVector3 origin = ModelBox->getOrigin();
//        IGeometry::IQuaternion Quat;
//        Quat.fromAxisRot( IGeometry::IVector3(0,1,0), rot.length() );
//        Quat.normalize();

//        IGeometry::IMatrix4x4 M_Quat = ModelBox->getTransformMatrix();
//        M_Quat.setRotation(Quat.getMatrix().getTranspose());

//        IGeometry::IMatrix4x4 MoveMatrix = ((ModelBox->getTransformMatrix() * IGeometry::IMatrix4x4::createTranslation(origin).getInverse()) *
//                                           (M_Quat * IGeometry::IMatrix4x4::createTranslation(origin)));
//       ModelBox->setTransformMatrix(MoveMatrix);

        mGMeshModels.push_back(ModelBox);
    }


    GizmoContext._CoordinatMode_ = IGeometry::Context::World;
    GizmoContext._TransformMode_ = GizmoContext.Move;

}



void ICreatorScene::render(float FrameTime)
{
    glViewport(0, 0, mWidth , mHeight );

    mCamera.LookAt( mEye , mCenter , mUp  );





    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(mCamera.getProjectionMatrix().getData());

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(mCamera.getTransformMatrix().getData());


    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_CULL_FACE);



    //----------------------------------------------//

    IGeometry::IVector3 point1 = GizmoContext.mRayInit.OriginPoint;
    IGeometry::IVector3 point2 = GizmoContext.mRayInit.ClosetPoint;

    glPushMatrix();
    glColor3f(1,0,0);
    glBegin(GL_LINES);
     glVertex3fv(point1);
     glVertex3fv(point2);
    glEnd();
    glColor3f(1,1,1);
    glPopMatrix();




//    if( mSelectedIndexID >= 0 )
//    {
//        GizmoContext.mbUsing = m_mouse_is;

//        //----------------------------------------------------------------------------------------//
//        GizmoManipulator.Manipulate(GizmoContext , &GizmoDrawList);

//        if(m_mouse_is)
//        {
//            mGMeshModels[mSelectedIndexID]->setTransformMatrix(GizmoManipulator.getTransformMatrix());
//        }
//        //----------------------------------------------------------------------------------------//
//    }

    // GizmoContext.mMousePressClick = false;
    //----------------------------------------------//



    glColor3f(1,1,1);

    for (unsigned int i = 0; i < mGMeshModels.size(); ++i)
    {
        glPushMatrix();

//        if( mPhysicsBodies[mPhysMapIndexes[mModels[i]]] != NULL)
//        {
//           float m[16];
//           mPhysicsBodies[mPhysMapIndexes[mModels[i]]]->getTransform().getOpenGLMatrix(m);
//           glMultMatrixf(m);
//        }

        IGeometry::IMatrix4x4 M;
        M.setPosition( mGMeshModels[i]->getTransformMatrix().getCoords() );
        M.setRotation( mGMeshModels[i]->getTransformMatrix().getRotMatrix().getTranspose() );

        glMultMatrixf( M );
        DrawMesh(mGMeshModels[i]);
        glPopMatrix();
    }








//    glPushMatrix();
//     glBegin(GL_POINTS);
//      glVertex3f(0,1,0);
//      glVertex3f(0,2,0);
//      glVertex3f(0,1,3);
//      glVertex3f(3,1,0);
//     glEnd();
//    glPopMatrix();


    glPushMatrix();
     glColor3f(1,1,0);
     glPointSize(2.0);
     glBegin(GL_POINTS);
     glVertex3f(mousePoint.x,mousePoint.y,mousePoint.z);
     glEnd();
     glColor3f(1,1,1);
    glPopMatrix();


//    glPushMatrix();
//    glColor3f(1,0,1);
//    glPointSize(10.0);
//    glBegin(GL_POINTS);
//    glVertex3fv(HitPoint);
//    glEnd();
//    glColor3f(1,1,1);
//    glPopMatrix();



    for ( auto it = GizmoDrawList.mLineSegments.begin(); it != GizmoDrawList.mLineSegments.end(); ++it )
    {
        glPushMatrix();
        glLineWidth(10);
        glBegin(GL_LINES);
         glColor4fv(it->mColor);
         glVertex3fv(it->mPoint1);
         glVertex3fv(it->mPoint2);
        glEnd();
        glLineWidth(1);
        glPopMatrix();
    }



}

void ICreatorScene::update()
{
    GizmoContext.mMatViewCamera = mCamera.getViewMatrix();

    IGeometry::IMatrix4x4 M1;
    M1 = IGeometry::IMatrix4x4::createRotationAxis( IGeometry::IVector3::Y , AngleYawCamera  );
    M1 = IGeometry::IMatrix4x4::createRotationAxis( IGeometry::IVector3::X , AngleRollCamera ) * M1;
    mEye = M1 * IGeometry::IVector3::Z * MoveLegth_FocusCamera;

    //GizmoContext.EyeCamera    = mEye;
    //GizmoContext.CenterCamera = mCenter;

    if( mSelectedIndexID >= 0 )
    {
        GizmoContext.mbUsing = m_mouse_is;

        //----------------------------------------------------------------------------------------//
        GizmoManipulator.Manipulate(GizmoContext , &GizmoDrawList);

        if(m_mouse_is)
        {
            mGMeshModels[mSelectedIndexID]->setTransformMatrix(GizmoManipulator.getTransformMatrix());
        }
        //----------------------------------------------------------------------------------------//
    }
}

void ICreatorScene::resize(float width, float height)
{
   // GizmoContext.mWidth  = width;
   // GizmoContext.mHeight = height;

    mWidth  = width;
    mHeight = height;

    float aspect = mWidth / mHeight;
    float zNear  = 1.0;
    float zFar   = 1024;
    float fov    = 45.0;

    mCamera.ProjectionPerspectiveMatrix( fov , aspect , zNear , zFar );
}

//-----------------------------------------------------//

void ICreatorScene::mouseMove(float x, float y, int button)
{
    mouseX = x;
    mouseY = y;

    if( mMouseButton == Qt::MouseButton::MidButton )
    {
        float speed_mouse_x = mouseX - mouseOldX;
        float speed_mouse_y = mouseY - mouseOldY;

        AngleYawCamera  -= speed_mouse_x * 0.005f;
        AngleRollCamera += speed_mouse_y * 0.005f;

        mouseOldX = x;
        mouseOldY = y;
    }
    else
    {


        if(  mMouseButton == Qt::MouseButton::LeftButton )
        {

            GizmoContext.mMousePressClick = false;

            //------------------------------------------//
            float aspect = mWidth / mHeight;
            float m_x = ((mouseX / mWidth ) - 0.5f) * aspect * 0.834;
            float m_y = ((mouseY / mHeight) - 0.5f) * 0.834;
            //GizmoContext.mMouseMoveX = m_x;
            //GizmoContext.mMouseMoveY = m_y;

            //GizmoContext.mMatViewCamera = mCamera.getViewMatrix();
            //GizmoContext.mMatViewCamera = mCamera.getViewMatrix();
            //GizmoManipulator.MouseMove(m_x,m_y,button,GizmoContext);



            //------------------------------------------//

            IGeometry::IVector3      Eye = mCamera.getEye();
            IGeometry::IMatrix3x3 RotCam = mCamera.getViewMatrix().getRotMatrix();

            mousePoint = Eye + IGeometry::IVector3(m_x,-m_y,-1.f) * RotCam;

//            //------------------------------------------//
//            UserRay.mRayMove.OriginPoint =  Eye;
//            UserRay.mRayMove.Direction   = (mousePoint - Eye).normalized();
//            UserRay.mRayMove.ClosetPoint =  Eye +  UserRay.mRayMove.Direction * 2000.0f;
//            //------------------------------------------//


            GizmoContext.mRayMove.OriginPoint =  Eye;
            GizmoContext.mRayMove.Direction   = (mousePoint - Eye).normalized();
            GizmoContext.mRayMove.ClosetPoint =  Eye +  GizmoContext.mRayMove.Direction * 2000.0f;

        }
    }



}


void ICreatorScene::mousePress(float x, float y, int button)
{
    mouseX = mouseOldX = x;
    mouseY = mouseOldY = y;

    mMouseButton = button;


    if(  mMouseButton == Qt::MouseButton::LeftButton )
    {

        m_mouse_is = true;

        GizmoContext.mMousePressClick = true;
        GizmoContext.mMousePressEnable = true;

        //------------------------------------------//
        float aspect = mWidth / mHeight;
        float m_x = ((mouseX / mWidth ) - 0.5f) * aspect * 0.834;
        float m_y = ((mouseY / mHeight) - 0.5f) * 0.834;
        //GizmoContext.mMousePressX = m_x;
        //GizmoContext.mMousePressY = m_y;




        //------------------------------------------//

        IGeometry::IVector3      Eye = mCamera.getEye();
        IGeometry::IMatrix3x3 RotCam = mCamera.getViewMatrix().getRotMatrix();

        GizmoContext.mMatViewCamera = mCamera.getViewMatrix();
        //GizmoManipulator.MousePress(m_x,m_y,button,GizmoContext);


        mousePoint = Eye + IGeometry::IVector3(m_x,-m_y,-1.f) * RotCam;

//        //------------------------------------------//
//        UserRay.mRayMove.OriginPoint  = UserRay.mRayInit.OriginPoint =  Eye;
//        UserRay.mRayMove.Direction    = UserRay.mRayInit.Direction   = (mousePoint - Eye).normalized();
//        UserRay.mRayMove.ClosetPoint  = UserRay.mRayInit.ClosetPoint =  Eye + UserRay.mRayMove.Direction * 2000.0f;
//        //------------------------------------------//



       GizmoContext.mRayMove.OriginPoint  = GizmoContext.mRayInit.OriginPoint =  Eye;
       GizmoContext.mRayMove.Direction    = GizmoContext.mRayInit.Direction   = (mousePoint - Eye).normalized();
       GizmoContext.mRayMove.ClosetPoint  = GizmoContext.mRayInit.ClosetPoint =  Eye + GizmoContext.mRayMove.Direction * 2000.0f;


//        mSelectedIndexID  = -1;

        //------------------------------------------//
        IGeometry::IVector3 a =  Eye;
        IGeometry::IVector3 b =  Eye + (mousePoint - Eye).normalized() * 2000.f;
        IGeometry::IRayCast RayCast( a , b );

        /**

    IGeometry::IVector3 Eye = mCamera.getEye();
    IGeometry::IVector3 mouse_point = IGeometry::IVector3(m_x,m_y,1);
    IGeometry::IVector3 cam_pos = Eye;
    IGeometry::IVector3 cam_ray = Eye + (mouse_point - Eye).normalized() * 2000.f;
    IGeometry::IRayCast RayCast( cam_pos , cam_ray );

    /**

    IGeometry::IVector3 rayOrigin;
    IGeometry::IVector3 rayDir;
    IGeometry::ComputeCameraRay( rayOrigin , rayDir , GizmoContext );

    IGeometry::IRayCast RayCast( rayOrigin + rayDir * 100.f , rayOrigin);

    /**/


//        IGeometry::IVector3 DirLookCam = RotCam * IGeometry::IVector3::Z;

        isSelected = false;
        float maxFriction = 0;
        for(int i = 0; i != mGMeshModels.size(); ++i )
        {
            IGeometry::IMeshModel* Mesh = mGMeshModels[i];

//            IGeometry::IMatrix4x4  M = Mesh->getTransformMatrix();


            //IGeometry::IMatrix4x4 Trans = IGeometry::IMatrix4x4::createTranslation(Mesh->getOrigin()).getInverse();

           // IGeometry::IVector3  offset_pos = M.getCoords();

//            IGeometry::IMatrix4x4 mm;
//            mm.setToIdentity();
//            mm.setRotation(M.getRotMatrix());
//            mm = IGeometry::IMatrix4x4::createTranslation(M.getCoords()) * mm.getInverse();

//            IGeometry::IMatrix4x4 inv_mat = IGeometry::IMatrix4x4::IDENTITY;
//            inv_mat.setRotation(   M.getRotMatrix() );
//            inv_mat.setPosition(   M.getCoords() );

//            IGeometry::IRayCast _ray_cast( M.getRotMatrix().getInverse() * ( UserRay.InitRayOrigin - M.getCoords() ),
//                                           M.getRotMatrix().getInverse() * ( UserRay.InitRayEnd    - M.getCoords() ) );

//            IPhysics::ITransform Tt;
//            Tt.setFromOpenGL(M);

//            IGeometry::IRayCast _ray_cast( ( /*Tt.getInverse() **/ UserRay.InitRayOrigin ) ,
//                                           ( /*Tt.getInverse() **/ UserRay.InitRayEnd    ) );

            if( Mesh->raycast(RayCast) )
            {

                //cout<< "ff   " << RayCast.maxFraction << "  :--:  " << (RayCast.hit - UserRay.InitRayOrigin).dot(UserRay.InitRayDir) <<endl;


                if( !isSelected )
                {
                    mSelectedIndexID = i;
                    maxFriction = RayCast.maxFraction;
                    HitPoint = RayCast.hit;
                }
                else
                {

                    if( RayCast.maxFraction < maxFriction )
                    {
                        mSelectedIndexID = i;
                        maxFriction = RayCast.maxFraction;
                        HitPoint = RayCast.hit;
                    }
                }

                 //cout << RayCast.maxFraction << " -- " <<  maxFriction << endl;

                isSelected = true;
            }
        }
        /**/

        if( isSelected && mSelectedIndexID > -1)
        {
            cout<< mSelectedIndexID <<endl;

            GizmoContext.mMatInitModel = mGMeshModels[mSelectedIndexID]->getTransformMatrix();
        }

    }

}

void ICreatorScene::mouseReleasePress(float x, float y, int button)
{
    mouseX = mouseOldX = x;
    mouseY = mouseOldY = y;



//    GizmoManipulator.Init();


      //************************************//

      if(  mMouseButton == Qt::MouseButton::LeftButton )
      {

          m_mouse_is = false;

          isSelected = false;

          GizmoContext.mMousePressClick = false;
          GizmoContext.mMousePressEnable = false;
          //------------------------------------------//
          float aspect = mWidth / mHeight;
          float m_x = ((mouseX / mWidth ) - 0.5f) * aspect * 0.834;
          float m_y = ((mouseY / mHeight) - 0.5f) * 0.834;
          //GizmoContext.mMouseMoveX = GizmoContext.mMousePressX = m_x;
          //GizmoContext.mMouseMoveY = GizmoContext.mMousePressY = m_y;


          GizmoContext.mMatViewCamera = mCamera.getViewMatrix();
          //GizmoManipulator.MouseRealase(m_x,m_y,button,GizmoContext);



          //------------------------------------------//

          IGeometry::IVector3      Eye = mCamera.getEye();
          IGeometry::IMatrix3x3 RotCam = mCamera.getViewMatrix().getRotMatrix();

          mousePoint = Eye + IGeometry::IVector3(m_x,-m_y,-1.f) * RotCam;


//          //------------------------------------------//
//          UserRay.mRayMove.OriginPoint  = UserRay.mRayInit.OriginPoint =  Eye;
//          UserRay.mRayMove.Direction    = UserRay.mRayInit.Direction   = (mousePoint - Eye).normalized();
//          UserRay.mRayMove.ClosetPoint  = UserRay.mRayInit.ClosetPoint =  Eye + UserRay.mRayMove.Direction * 2000.0f;
//          //------------------------------------------//



          GizmoContext.mRayMove.OriginPoint  = GizmoContext.mRayInit.OriginPoint =  Eye;
          GizmoContext.mRayMove.Direction    = GizmoContext.mRayInit.Direction   = (mousePoint - Eye).normalized();
          GizmoContext.mRayMove.ClosetPoint  = GizmoContext.mRayInit.ClosetPoint =  Eye + GizmoContext.mRayMove.Direction * 2000.0f;


          if( mSelectedIndexID >= 0 )
          {
             GizmoContext.mMatInitModel = mGMeshModels[mSelectedIndexID]->getTransformMatrix();
             // isSelected = false;
          }
      }

      //************************************//




}

void ICreatorScene::mouseWheel(float delta)
{
  MoveLegth_FocusCamera += (delta * 0.01f);
}

//-----------------------------------------------------//

void ICreatorScene::keyboard(int key)
{

}

void ICreatorScene::destroy()
{

}

