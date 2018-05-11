#include "maincreator.h"
#include "ui_maincreator.h"

MainCreator::MainCreator(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainCreator)
{
    ui->setupUi(this);
}

MainCreator::~MainCreator()
{
    delete ui;
}
