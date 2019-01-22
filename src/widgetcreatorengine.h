#ifndef WIDGETCREATORENGINE_H
#define WIDGETCREATORENGINE_H

#include <QWidget>
#include <QTreeWidgetItem>
#include <QLayout>

#include <QQuickView>
#include <QQuickWidget>

#include <QBasicTimer>
#include <QtQml/QQmlApplicationEngine>
#include <QQmlContext>

#include "InterfaceShellPanel.h"
#include "InterfaceShellCreatePrimitive.h"

namespace Ui {
class WidgetCreatorEngine;
}


class WidgetCreatorEngine : public QWidget
{
    Q_OBJECT

private:

    /**
    * @brief PanelInetrface Right Panel Tools
    */
    InterfaceShellPanel *PanelInetrface;

    /**
     * @brief RemoveLayout
     * @param layout
     */
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

    /**
     * @brief ClearLayout
     * @param layout
     */
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


public:
    explicit WidgetCreatorEngine(QWidget *parent = 0);
    ~WidgetCreatorEngine();


    void keyPressEvent( QKeyEvent *keyEvent );
    void keyReleaseEvent( QKeyEvent *keyEvent );


    void closeEvent(QCloseEvent *event);


    //int getPageIndex(QTreeWidgetItem *item);

private slots:

    void showPosts();


     void clickedaction(int index);


     void Move_PushButton();
     void Scale_PushButton();
     void Rotate_PushButton();


     void Prev_PushButton();
     void Next_PushButton();

     void DeleteComponent_PushButton();




private:

    Ui::WidgetCreatorEngine *ui;



};

#endif // WIDGETCREATORENGINE_H
