#ifndef MAINCREATOR_H
#define MAINCREATOR_H

#include <QMainWindow>

namespace Ui {
class MainCreator;
}

class MainCreator : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainCreator(QWidget *parent = 0);
    ~MainCreator();

private:
    Ui::MainCreator *ui;
};

#endif // MAINCREATOR_H
