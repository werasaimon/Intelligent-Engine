#ifndef WIDGETCREATORENGINE_H
#define WIDGETCREATORENGINE_H

#include <QWidget>

namespace Ui {
class WidgetCreatorEngine;
}

class WidgetCreatorEngine : public QWidget
{
    Q_OBJECT

public:
    explicit WidgetCreatorEngine(QWidget *parent = 0);
    ~WidgetCreatorEngine();


    void keyPressEvent( QKeyEvent *keyEvent );
    void keyReleaseEvent( QKeyEvent *keyEvent );


private slots:

     void clickedaction(int index);

//    void Move();
//    void Scale();
//    void Rotate();



private:
    Ui::WidgetCreatorEngine *ui;


};

#endif // WIDGETCREATORENGINE_H
