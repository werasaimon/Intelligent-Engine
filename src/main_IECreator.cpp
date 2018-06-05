#include "widgetcreatorengine.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    WidgetCreatorEngine w;
    w.show();

    return a.exec();
}
