#ifndef INTERFACESHELLPANELTEST_H
#define INTERFACESHELLPANELTEST_H

#include <QObject>
#include <QQuickWidget>
#include <QQmlContext>


#include "ISceneEditor/ISceneEditor.h"
#include "InterfaceShellPanel.h"




class InterfaceShellCreatePrimitive :  public InterfaceShellPanel
{
   Q_OBJECT

    public:

        explicit InterfaceShellCreatePrimitive(QObject *parent = nullptr);

        InterfaceShellCreatePrimitive( ISceneCompare *scene );

        /**
         * @brief CreateLayout
         * @return reloaded layout toolbar interface
         */
        QGridLayout *CreateLayout() override;


        Q_INVOKABLE void fFunctionCreateCuboid( const double& _width , const double& _height , const double& _length ,
                                                const int& _width_segment , const int _height_segment , const int& _length_segment);

        Q_INVOKABLE void fFunctionRecalculateCuboid( const double& _width , const double& _height , const double& _length ,
                                                     const int& _width_segment , const int _height_segment , const int& _length_segment);



        Q_INVOKABLE void fFunctionCreateEllipsoid( const double& _radius_X , const double& _radius_Y , const double& _radius_Z ,
                                                   const int& _U_segment , const int _V_segment );

        Q_INVOKABLE void fFunctionRecalculateEllipsoid( const double& _radius_X , const double& _radius_Y , const double& _radius_Z ,
                                                        const int& _U_segment , const int _V_segment );


        void CheckSaveSceneState()
        {
            ISceneEditor::ContexSaveScene _ContexSaveSceneInfo;
            _ContexSaveSceneInfo.isSaveHierarchyNode = false;
            _ContexSaveSceneInfo.isDeleteComponent = false;
            _ContexSaveSceneInfo.isSaveComponent = true;

            if(mSceneEngine) mSceneEngine->SaveSceneState(_ContexSaveSceneInfo);
        }


    signals:

    public slots:


    private:

        ISceneEditor *mSceneEngine;
        QQuickWidget *mQuickQMLWidget;
         QObject *mRoot;//корневой элемент QML модели


   IEngineComponent::IComponentMesh3 *m_pLastCreatedItemComponent;

};

#endif // INTERFACESHELLPANELTEST_H
