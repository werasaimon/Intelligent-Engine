#include "widgetcreatorengine.h"
#include "ui_widgetcreatorengine.h"
#include "ISceneEditor/ISceneCompare.h"
#include "ieditglwidget.h"

#include <QLabel>



#include <iostream>

WidgetCreatorEngine::WidgetCreatorEngine(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::WidgetCreatorEngine)
{
    ui->setupUi(this);

    connect(ui->comboBox_ModeGizmo ,SIGNAL(activated(int)),this,SLOT(clickedaction(int)));
    connect(ui->comboBox, SIGNAL(currentIndexChanged(QString)), SLOT(showPosts()));


    connect(ui->pushButtonPrev , SIGNAL(released()), this , SLOT(Prev_PushButton()));
    connect(ui->pushButtonNext , SIGNAL(released()), this , SLOT(Next_PushButton()));


    connect(ui->pushButtonMove   , SIGNAL(released()), this , SLOT(Move_PushButton()));
    connect(ui->pushButtonRotate , SIGNAL(released()), this , SLOT(Rotate_PushButton()));
    connect(ui->pushButtonScale  , SIGNAL(released()), this , SLOT(Scale_PushButton()));

    connect(ui->pushButtonDelete , SIGNAL(released()), this , SLOT(DeleteComponent_PushButton()));


    PanelInetrface = nullptr;
    ui->comboBox->setCurrentText("CreatePrimitive");

}

WidgetCreatorEngine::~WidgetCreatorEngine()
{
    delete ui;
}

void WidgetCreatorEngine::keyPressEvent(QKeyEvent *keyEvent)
{
    if( !keyEvent->isAutoRepeat() )
    {
      ui->widget->keyPressEvent(keyEvent);
    }
}

void WidgetCreatorEngine::keyReleaseEvent(QKeyEvent *keyEvent)
{
    if( !keyEvent->isAutoRepeat() )
    {
       ui->widget->keyReleaseEvent(keyEvent);
    }

}


void WidgetCreatorEngine::closeEvent(QCloseEvent *event)
{
    ui->widget->closeEvent(event);
}



void WidgetCreatorEngine::showPosts()
{

    if(ui->groupBoxT->layout() != NULL)
    {
       ClearLayout(ui->groupBoxT->layout());
    }


    if( ui->comboBox->currentText() == "CreatePrimitive" )
    {
        if(PanelInetrface)
        {
            delete  PanelInetrface;
            PanelInetrface = nullptr;
        }

        PanelInetrface = new InterfaceShellCreatePrimitive(ui->widget->scene());
        ui->groupBoxT->setLayout( PanelInetrface->CreateLayout() );
    }
    else if( ui->comboBox->currentText() == "Test" )
    {
        //...
    }

}

void WidgetCreatorEngine::clickedaction(int index)
{
    if( index == 0 )
    {
       ui->widget->CheckWorld();
    }
    else if( index == 1 )
    {
       ui->widget->CheckLocal();
    }
}



void WidgetCreatorEngine::Prev_PushButton()
{
    qDebug() << "Prev Scene";
    (ui->widget->scene())->PrevScene();
}

void WidgetCreatorEngine::Next_PushButton()
{
    qDebug() << "Next Scene";
    (ui->widget->scene())->NextScene();
}


void WidgetCreatorEngine::DeleteComponent_PushButton()
{
   qDebug() << "Delete Component";
   (ui->widget->scene())->RemoveSelectedComponent();
}




void WidgetCreatorEngine::Move_PushButton()
{
    if(ui->widget->scene() != NULL)
    {
        ui->widget->Move();
    }
}

void WidgetCreatorEngine::Scale_PushButton()
{
    if(ui->widget->scene() != NULL)
    {
        ui->widget->Scale();
    }
}

void WidgetCreatorEngine::Rotate_PushButton()
{
    if(ui->widget->scene() != NULL)
    {
        ui->widget->Rotate();
    }

}











