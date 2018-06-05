#include "widgetcreatorengine.h"
#include "ui_widgetcreatorengine.h"
#include "ICreatorScene/IScene.h"
#include "ICreatorScene/ICreatorScene.h"
#include "ieditglwidget.h"

WidgetCreatorEngine::WidgetCreatorEngine(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::WidgetCreatorEngine)
{
    ui->setupUi(this);

    connect(ui->comboBox_ModeGizmo ,SIGNAL(activated(int)),this,SLOT(clickedaction(int)));
}

WidgetCreatorEngine::~WidgetCreatorEngine()
{
    delete ui;
}

void WidgetCreatorEngine::keyPressEvent(QKeyEvent *keyEvent)
{
     ui->widget1->keyPressEvent(keyEvent);
}

void WidgetCreatorEngine::keyReleaseEvent(QKeyEvent *keyEvent)
{
    ui->widget1->keyReleaseEvent(keyEvent);
}

void WidgetCreatorEngine::clickedaction(int index)
{
    if( index == 0 )
    {
       static_cast<ICreatorScene*>(ui->widget1->scene())->GizmoContext._CoordinatMode_ = IGeometry::Context::World;
    }
    else if( index == 1 )
    {
       static_cast<ICreatorScene*>(ui->widget1->scene())->GizmoContext._CoordinatMode_ = IGeometry::Context::Local;
    }
}

/**
void WidgetCreatorEngine::Move()
{
    if(ui->widget->scene() != NULL)
    {
        static_cast<ICreatorScene*>(ui->widget->scene())->GizmoContext._TransformMode_ = IGeometry::Context::Move;
    }
}

void WidgetCreatorEngine::Scale()
{
    if(ui->widget->scene() != NULL)
    {
        static_cast<ICreatorScene*>(ui->widget->scene())->GizmoContext._TransformMode_ = IGeometry::Context::Scale;
    }
}

void WidgetCreatorEngine::Rotate()
{
    if(ui->widget->scene() != NULL)
    {
        static_cast<ICreatorScene*>(ui->widget->scene())->GizmoContext._TransformMode_ = IGeometry::Context::Rotate;
    }

}
/**/


