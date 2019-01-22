#include "widgetcreatorengine.h"
#include <QApplication>

#include "IEngine/IMath/IMaths.h"

#include <QQuickView>

int main(int argc, char *argv[])
{


    QApplication a(argc, argv);
    WidgetCreatorEngine w;
    w.show();


    return a.exec();
}



