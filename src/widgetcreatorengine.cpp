#include "widgetcreatorengine.h"
#include "ui_widgetcreatorengine.h"

WidgetCreatorEngine::WidgetCreatorEngine(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::WidgetCreatorEngine)
{
    ui->setupUi(this);
}

WidgetCreatorEngine::~WidgetCreatorEngine()
{
    delete ui;
}
