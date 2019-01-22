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

#ifndef MAINWIDGET_H
#define MAINWIDGET_H


#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QBasicTimer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QWheelEvent>
#include <QLayout>
#include <QLayoutItem>

#include <QFile>
#include <QDebug>
#include <QTextStream>


#include "ISceneEditor/ISceneCompare.h"
#include "ISceneEditor/ISceneEditor.h"

#include <iostream>
using namespace  std;

class IEditGLWidget : public QOpenGLWidget , protected QOpenGLFunctions
{
    Q_OBJECT

private:

   /// Base Timer Update
   QBasicTimer  timer;

   /// Scene Engine Editor
   ISceneEditor* mScene;

private:

    //---- Mouse Value ----//
    float mouseOldX;
    float mouseOldY;

    float mouseX;
    float mouseY;

    int  mMouseButton;
    //---------------------//

    IVivwer *mViewScene;

    IuGizmo::IGizmo::LOCATION mGizmoLocationMode;
    IuGizmo::IGizmo* mGizmoManipulator;

    IuGizmo::IGizmo* mGizmoMove;
    IuGizmo::IGizmo* mGizmoRotate;
    IuGizmo::IGizmo* mGizmoScale;

public:
    explicit IEditGLWidget(QWidget *parent = nullptr);
            ~IEditGLWidget();

    ISceneEditor *scene();

//protected:
public:

    QWidget *widget_interface;

    void initializeGL() Q_DECL_OVERRIDE;
    void resizeGL(int w, int h) Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;


    void mousePressEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void timerEvent(QTimerEvent *e) Q_DECL_OVERRIDE;

    void wheelEvent(QWheelEvent *event) Q_DECL_OVERRIDE;

    void keyPressEvent( QKeyEvent *keyEvent ) Q_DECL_OVERRIDE;
    void keyReleaseEvent( QKeyEvent *keyEvent ) Q_DECL_OVERRIDE;

    void closeEvent(QCloseEvent *event) Q_DECL_OVERRIDE;




private:


    void RemoveLayout(QLayout* layout)
    {
        QLayoutItem* child;
        while(layout->count()!=0)
        {
            child = layout->takeAt(0);
            if(child->layout() != 0)
            {
                RemoveLayout(child->layout());
            }
            else if(child->widget() != 0)
            {
                delete child->widget();
            }

            delete child;
            child = nullptr;
        }
    }


    void ClearLayout(QLayout *layout)
    {
        RemoveLayout(layout);
        // THIS IS THE SOLUTION!
        // Delete all existing widgets, if any.
        if ( layout != NULL )
        {
            QLayoutItem* item;
            while ( (item=layout->takeAt(0)) != NULL )
            {
                if(item->widget())
                {
                    item->widget()->deleteLater();
                    delete item->widget();
                }

                if(item->layout())
                {
                    ClearLayout(item->layout());
                }

                delete item;
                item = nullptr;
            }

            delete layout;
            layout = nullptr;
        }
    }



public slots:



    void Move()
    {
        //static_cast<ISceneEditor*>(mScene)->CheckMove();

        mGizmoManipulator = mGizmoMove;
        mGizmoManipulator->SetLocation( mGizmoLocationMode );
        mGizmoManipulator->SetEditMatrix( static_cast<ISceneEditor*>(mScene)->getGizmo_transform_matrix() );
        mGizmoManipulator->SetDisplayScale( 2.f );
        mGizmoManipulator->SetScreenDimension( mViewScene->mWidth, mViewScene->mHeight );
        static_cast<ISceneEditor*>(mScene)->setGizmoManipulator(mGizmoManipulator);
    }

    void Scale()
    {
        //static_cast<ISceneEditor*>(mScene)->CheckScale();

        std::cout << "Scale" << std::endl;

        mGizmoManipulator = mGizmoScale;
        mGizmoManipulator->SetLocation( mGizmoLocationMode );
        mGizmoManipulator->SetEditMatrix( static_cast<ISceneEditor*>(mScene)->getGizmo_transform_matrix() );
        mGizmoManipulator->SetDisplayScale( 2.f );
        mGizmoManipulator->SetScreenDimension( mViewScene->mWidth, mViewScene->mHeight );
        static_cast<ISceneEditor*>(mScene)->setGizmoManipulator(mGizmoManipulator);
    }

    void Rotate()
    {
        //static_cast<ISceneEditor*>(mScene)->CheckRotate();

        mGizmoManipulator = mGizmoRotate;
        mGizmoManipulator->SetLocation( mGizmoLocationMode );
        mGizmoManipulator->SetEditMatrix( static_cast<ISceneEditor*>(mScene)->getGizmo_transform_matrix() );
        mGizmoManipulator->SetDisplayScale( 2.f );
        mGizmoManipulator->SetScreenDimension( mViewScene->mWidth, mViewScene->mHeight );
        static_cast<ISceneEditor*>(mScene)->setGizmoManipulator(mGizmoManipulator);
    }




    void CheckLocal()
    {
       // static_cast<ISceneEditor*>(mScene)->CheckLocal();

        mGizmoManipulator->SetDisplayScale( 2.f );
        mGizmoManipulator->SetLocation( mGizmoLocationMode = IuGizmo::IGizmo::LOCATE_LOCAL );

    }

    void CheckWorld()
    {
        //static_cast<ISceneEditor*>(mScene)->CheckWorld();

        mGizmoManipulator->SetDisplayScale( 2.f );
        mGizmoManipulator->SetLocation( mGizmoLocationMode = IuGizmo::IGizmo::LOCATE_WORLD );
    }


    void DeleteComponentSelected()
    {
        mScene->RemoveSelectedComponent();
    }


    IVivwer *getViewScene() const;

};

#endif // MAINWIDGET_H
