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
#include <math.h>

#include <QCoreApplication>
#include <QDir>

#include <QMenu>

#include "glwidget.h"
#include "IEngine/IMath/IMaths.h"

float mouse_x = 0;
float mouse_y = 0;

GLWidget::GLWidget(QWidget *parent)
  : QOpenGLWidget(parent)
{


}



GLWidget::~GLWidget()
{
    // Make sure the context is current when deleting the texture
    // and the buffers.
    makeCurrent();
    doneCurrent();
}



void GLWidget::initializeGL()
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

    glClearColor(0, 0, 0, 1);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    /// initilisation scene
    mScene = new SceneDynamicBody;
    mScene->initialization();


    // Use QBasicTimer because its faster than QTimer
    timer.start( 20 , this);

}





void GLWidget::resizeGL( int w , int h )
{

   mScene->resize( w , h );


    /**
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-1, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW);

    /**
    mScene->resize( w , h );

    /**
    glShadeModel(GL_SMOOTH);				// Enable Smooth Shading
    glClearColor(0.0f, 0.0f, 0.0f, 0.5f);		// Black Background
    glClearDepth(1.0f);					// Depth Buffer Setup
    glEnable(GL_DEPTH_TEST);				// Enables Depth Testing
    glDepthFunc(GL_LEQUAL);				// The Type Of Depth Testing To Do
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations


    /**
    glShadeModel(GL_SMOOTH);
    glViewport(0, 0, w , h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    //  glOrtho( 0.0 , world_size , 0.0 , world_size , -10.0 , 10.0);
    gluPerspective(45.0f, (GLfloat) w / (GLfloat) h , 0.01, 1000.0f);


    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    /**/

}


void GLWidget::paintGL()
{
    mScene->render(1.f/60.f);
}


void GLWidget::timerEvent(QTimerEvent *e)
{
    update();
    mScene->update();
}


void GLWidget::keyPressEvent(QKeyEvent *keyEvent)
{
   mScene->specialKeyboardDown( keyEvent->key() );
   mScene->keyboard( keyEvent->key() );
}

void GLWidget::keyReleaseEvent(QKeyEvent *keyEvent)
{
   mScene->specialKeyboardUp(  keyEvent->key() );
   keyEvent->accept();
}



void GLWidget::wheelEvent(QWheelEvent *event)
{
   mScene->mouseWheel(event->delta());
   event->accept();
}


void GLWidget::mouseMoveEvent(QMouseEvent *e)
{
  mouse_x = e->pos().x();
  mouse_x = e->pos().y();

  mScene->mouseMove( e->pos().x() , e->pos().y() , e->button() );
}

void GLWidget::mousePressEvent(QMouseEvent *e)
{

  mouse_x = e->pos().x();
  mouse_x = e->pos().y();

  mScene->mousePress( e->pos().x() , e->pos().y() , e->button() );

   /**
   if( e->button() == Qt::RightButton )
   {
       QMenu menu(this);
       menu.addSeparator();
       menu.addAction( "Move"   , this  , SLOT(Move())   );
       menu.addAction( "Scale"  ,  this , SLOT(Scale())  );
       menu.addAction( "Rotate" ,  this , SLOT(Rotate()) );
       e->pos().setX(mouse_x);
       e->pos().setY(mouse_y);
       menu.exec(e->pos());
   }
   else
   {
       mScene->mousePress( e->pos().x() , e->pos().y() - 4.0 , e->button() );
   }
   /**/
}

void GLWidget::mouseReleaseEvent(QMouseEvent *e)
{
   mScene->mouseReleasePress( e->pos().x() , e->pos().y() , e->button() );
}


void GLWidget::closeEvent(QCloseEvent *event)
{
   mScene->destroy();
   event->accept();
}

//Scene *GLWidget::scene()
//{
//    return mScene;
//}

//void GLWidget::setScene(Scene *scene)
//{
//    mScene = scene;
//}



