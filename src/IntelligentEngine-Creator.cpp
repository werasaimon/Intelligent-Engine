#include "maincreator.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainCreator w;
    w.show();

    return a.exec();
}
