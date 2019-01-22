#ifndef INTERFACE_PANEL_H
#define INTERFACE_PANEL_H

#include <QWidget>
#include <QLayout>
#include <QLayoutItem>
#include <QRadioButton>
#include <QPushButton>
#include <QLabel>

#include <QObject>

#include "ISceneEditor/ISceneEditor.h"




class InterfaceShellPanel : public QObject
{
    Q_OBJECT

public:

        explicit InterfaceShellPanel (QObject *parent = 0)
        : QObject(parent) {}

        virtual ~InterfaceShellPanel(){}

        /**
         * @brief CreateLayout
         * @return reloaded layout toolbar interface
         */
        virtual  QGridLayout *CreateLayout() = 0;



protected:


};

#endif // INTERFACE_PANEL_H
