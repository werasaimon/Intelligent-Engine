/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtCore module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/


//#ifdef __ANDROID__
//#elif defined(WIN32) || defined(__linux__)
//#include <GL/glew.h>
//#endif


#include <QMouseEvent>
#include <QCoreApplication>
#include <QDir>
#include <QMenu>

#include <math.h>
#include "ieditglwidget.h"

float mouse_x = 0;
float mouse_y = 0;



IEditGLWidget::IEditGLWidget(QWidget *parent)
: QOpenGLWidget(parent)
{

        //--------------------------------------------------//

        float  width  = 600;
        float  height = 400;

        float aspect = width / height;
        float zNear  = 1.0;
        float zFar   = 1024;
        float fov    = 30;


        IEngine::IQCamera *iqCamera = new IEngine::IQCamera;
        mViewScene = new IVivwer( new IEngineComponent::IComponentCamera(iqCamera , 0) );

        mViewScene->ProjectionPerspectiv( fov , aspect , zNear , zFar );

        mViewScene->mEye    =  IEngine::IVector3(0,1,-10);
        mViewScene->mCenter =  IEngine::IVector3(0,0,0);
        mViewScene->mUp     =  IEngine::IVector3(0,1,0);

        mViewScene->MoveLegth_FocusCamera = 10;

        mViewScene->mWidth  = width;
        mViewScene->mHeight = height;

        //--------------------------------------------------//

        mGizmoMove = IuGizmo::CreateMoveGizmo();
        mGizmoRotate = IuGizmo::CreateRotateGizmo();
        mGizmoScale = IuGizmo::CreateScaleGizmo();

        mGizmoManipulator = mGizmoMove;
        mGizmoLocationMode = IuGizmo::IGizmo::LOCATE_WORLD;
        mGizmoManipulator->SetLocation( mGizmoLocationMode );
        mGizmoManipulator->SetEditMatrix( IEngine::IMatrix4x4::IDENTITY );
        mGizmoManipulator->SetScreenDimension( width , height );
        mGizmoManipulator->SetDisplayScale( 2.f );

        //--------------------------------------------------//


        /// initilisation scene
        mScene = new ISceneEditor(mViewScene);
        mScene->initialization();

        mScene->setGizmoManipulator(mGizmoManipulator);
        //mScene->initialization(mViewScene);

        // Use QBasicTimer because its faster than QTimer
        timer.start( 20 , this);

}



IEditGLWidget::~IEditGLWidget()
{
    // Make sure the context is current when deleting the texture
    // and the buffers.
    makeCurrent();
    doneCurrent();
}


//---------------------------------------------------//

IVivwer *IEditGLWidget::getViewScene() const
{
    return mViewScene;
}

ISceneEditor *IEditGLWidget::scene()
{
    return mScene;
}

//---------------------------------------------------//


void IEditGLWidget::initializeGL()
{

//#ifdef __ANDROID__
//#elif defined(WIN32) || defined(__linux__)

//    GLenum err = glewInit();
//    if(err != GLEW_OK)
//    {
//        printf("%s",glewGetErrorString(err));
//    }

//#endif


    initializeOpenGLFunctions();

    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);


}





void IEditGLWidget::resizeGL( int w , int h )
{
   mScene->resize( w , h );

  //-------------------------------------------------------------//


   float width = w;
   float height = h;
   mViewScene->mWidth  = width;
   mViewScene->mHeight = height;

   float aspect = width / height;
   float zNear  = 0.5;
   float zFar   = 1024;
   float fov    = 30;

   mViewScene->ProjectionPerspectiv( fov , aspect , zNear , zFar );

   if (mGizmoManipulator)
   {
       mGizmoManipulator->SetScreenDimension( mViewScene->mWidth,
                                              mViewScene->mHeight );
   }

   //-------------------------------------------------------------//
}


void IEditGLWidget::paintGL()
{
   mScene->render(1.f/60.f);
}


void IEditGLWidget::timerEvent(QTimerEvent *e)
{
    update();

    mViewScene->Update();
    mScene->update();

    //-------------------------------------------------------------------------------------------//
    if (mGizmoManipulator)
    {
        mGizmoManipulator->SetCameraMatrix(mViewScene->mCamera->GetCamera()->ViewMatrix() ,
                                           mViewScene->mCamera->GetCamera()->ProjectionMatrix() );
    }
    //-------------------------------------------------------------------------------------------//
}


void IEditGLWidget::keyPressEvent(QKeyEvent *keyEvent)
{
   mScene->keyboard( keyEvent->key() );
   mScene->SaveKeyPressed( keyEvent->key() );
   keyEvent->accept();

}

void IEditGLWidget::keyReleaseEvent(QKeyEvent *keyEvent)
{

   mScene->realaseKeyboard(keyEvent->key());
   mScene->SaveKeyReleased( keyEvent->key() );
   keyEvent->accept();
}



void IEditGLWidget::wheelEvent(QWheelEvent *event)
{
   mScene->mouseWheel(event->delta());
   event->accept();

   //----------------------------------------//
   float delta = event->delta();
   mViewScene->MoveLegth_FocusCamera += (delta * 0.02f);
   mViewScene->MoveLegth_FocusCamera = IMath::IClamp(mViewScene->MoveLegth_FocusCamera,5.f,1024.f);
   //----------------------------------------//

}



void IEditGLWidget::mouseMoveEvent(QMouseEvent *e)
{
  mouse_x = e->pos().x();
  mouse_y = e->pos().y();

  mouseX = mouse_x;
  mouseY = mouse_y;

  if( mMouseButton == Qt::MouseButton::MidButton )
  {
      float speed_mouse_x = mouseX - mouseOldX;
      float speed_mouse_y = mouseY - mouseOldY;

      mViewScene->AngleYawCamera  -= speed_mouse_x * 0.005f;
      mViewScene->AngleRollCamera += speed_mouse_y * 0.005f;

      mouseOldX = mouse_x;
      mouseOldY = mouse_y;
  }
  else
  {
     mScene->mouseMove( e->pos().x() , e->pos().y() , e->button() );

     if(  mMouseButton == Qt::MouseButton::LeftButton )
     {
         if (mGizmoManipulator)
         {
             mGizmoManipulator->OnMouseMove( mouseX, mouseY );
         }
     }
  }


    //----------------------------------------------------------//
}


void IEditGLWidget::mousePressEvent(QMouseEvent *e)
{

  mouse_x = e->pos().x();
  mouse_y = e->pos().y();

  mouseX = mouseOldX = mouse_x;
  mouseY = mouseOldY = mouse_y;
  mMouseButton = e->button();

// //---------------------------//
//  mouseX = mouseOldX = mouse_x;
//  mouseY = mouseOldY = mouse_y;
//  mMouseButton = e->button();
// //---------------------------//

   /**/
   if( e->button() == Qt::MouseButton::RightButton )
   {
       QMenu menu(this);
       menu.addSeparator();
       menu.addAction( "Move"   , this  , SLOT(Move())   );
       menu.addAction( "Scale"  ,  this , SLOT(Scale())  );
       menu.addAction( "Rotate" ,  this , SLOT(Rotate()) );
       menu.addSection("----------------------------");
       menu.addAction( "Delete" ,  this , SLOT(DeleteComponentSelected()) );




       menu.exec(e->globalPos());

       //e->pos().setX(mouse_x);
       //e->pos().setY(mouse_y);
       //menu.exec(e->pos());
   }
   else
   {
       if( mGizmoManipulator &&  e->button() == Qt::MouseButton::LeftButton )
       {
           mGizmoManipulator->OnMouseDown( mouseX, mouseY );
       }

       mScene->mousePress( e->pos().x() , e->pos().y() - 4.0 , e->button() );

   }
   /**/
}

void IEditGLWidget::mouseReleaseEvent(QMouseEvent *e)
{
    mouse_x = e->pos().x();
    mouse_y = e->pos().y();

    mouseX = mouseOldX = mouse_x;
    mouseY = mouseOldY = mouse_y;
    mMouseButton = e->button();

    if(  mMouseButton == Qt::MouseButton::LeftButton )
    {
        if (mGizmoManipulator)
        {
            mGizmoManipulator->OnMouseUp( mouseX, mouseY );
            mGizmoManipulator->InitEditMatrix();
        }
    }



   if( e->button() == Qt::RightButton )
   {
        //....
   }
   else
   {
       mScene->mouseReleasePress( e->pos().x() , e->pos().y() , e->button() );
   }

}


void IEditGLWidget::closeEvent(QCloseEvent *event)
{
   mScene->destroy();

//   if(mGizmoMove)
//   {
//       delete mGizmoMove;
//       mGizmoMove = nullptr;
//   }

//   if(mGizmoRotate)
//   {
//       delete mGizmoRotate;
//       mGizmoRotate = nullptr;
//   }

//   if(mGizmoScale)
//   {
//       delete mGizmoScale;
//       mGizmoScale = nullptr;
//   }

   event->accept();
}








