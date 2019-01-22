#include "InterfaceShellCreatePrimitive.h"




InterfaceShellCreatePrimitive::InterfaceShellCreatePrimitive(QObject *parent)
: InterfaceShellPanel(parent)
{
   qDebug() << " Create Primitive InterfacePanel " ;
}

InterfaceShellCreatePrimitive::InterfaceShellCreatePrimitive(ISceneCompare *scene)
 : mSceneEngine ( static_cast<ISceneEditor*>(scene) ) ,
   m_pLastCreatedItemComponent(nullptr)
{
   qDebug() << " Create Primitive InterfacePanel " ;
}



QGridLayout *InterfaceShellCreatePrimitive::CreateLayout()
{
    if(!mQuickQMLWidget)
    {
        delete mQuickQMLWidget;
        mQuickQMLWidget = nullptr;
    }



    mQuickQMLWidget = new QQuickWidget();
    mQuickQMLWidget->setSource(QUrl("qrc:/QMLSource/main_CreatePrimitive.qml"));

    // m_quickWidget->rootContext()->setContextProperty("myObj", myObj);
    mQuickQMLWidget->setResizeMode(QQuickWidget::SizeRootObjectToView);


    //Находим корневой элемент
    auto Root =  mQuickQMLWidget->rootObject();
    mRoot = (QObject*)(Root);


    //Соединяем C++ и QML, делая видимым функции С++ через элемент window
    mQuickQMLWidget->rootContext()->setContextProperty("InterfaceShellCreatePrimitive", this);

    QGridLayout *glayout = new QGridLayout;
    glayout->setHorizontalSpacing(10);
    glayout->setVerticalSpacing(10);

    glayout->addWidget(mQuickQMLWidget, 1, 0 );

    return glayout;
}



void InterfaceShellCreatePrimitive::fFunctionCreateCuboid(const double& _width , const double& _height , const double& _length ,
                                                          const int& _width_segment , const int  _height_segment , const int& _length_segment)
{
    qDebug() << "Create Primitive Cuboid " ;
    qDebug() << "Width:  " << _width  ;
    qDebug() << "Height: " << _height ;
    qDebug() << "Length: " << _length ;

    qDebug() << "Segment_Width:  " << _width_segment  ;
    qDebug() << "Segment_Height: " << _height_segment ;
    qDebug() << "Segment_Length: " << _length_segment ;

    CheckSaveSceneState();

    IEngine::MeshGenerators::CuboidDescriptor _cuboid_descriptor;

    _cuboid_descriptor.size.x = _width;
    _cuboid_descriptor.size.y = _height;
    _cuboid_descriptor.size.z = _length;

    _cuboid_descriptor.segments.x = _width_segment;
    _cuboid_descriptor.segments.y = _height_segment;
    _cuboid_descriptor.segments.z = _length_segment;

    IEngine::IMesh3 *NewMesh = new IEngine::IMesh3;
    IEngineComponent::IComponentMesh3 *ComponentMesh = new IEngineComponent::IComponentMesh3(NewMesh , 0);
    m_pLastCreatedItemComponent = ComponentMesh;
    IEngineComponent::IComponentMesh3::CreatePramitiveToMesh(_cuboid_descriptor,ComponentMesh);

    //mSceneEngine->Suka();


    mSceneEngine->AddComponent(ComponentMesh);

}

void InterfaceShellCreatePrimitive::fFunctionRecalculateCuboid(const double& _width , const double& _height , const double& _length ,
                                                               const int& _width_segment , const int _height_segment , const int& _length_segment )
{
    if(m_pLastCreatedItemComponent)
    {
        qDebug() << "Recalculate Primitive Cuboid " ;
        qDebug() << "Width:  " << _width  ;
        qDebug() << "Height: " << _height ;
        qDebug() << "Length: " << _length ;

        qDebug() << "Segment_Width:  " << _width_segment  ;
        qDebug() << "Segment_Height: " << _height_segment ;
        qDebug() << "Segment_Length: " << _length_segment ;

        CheckSaveSceneState();

        IEngine::MeshGenerators::CuboidDescriptor _cuboid_descriptor;

        _cuboid_descriptor.size.x = _width;
        _cuboid_descriptor.size.y = _height;
        _cuboid_descriptor.size.z = _length;

        _cuboid_descriptor.segments.x = _width_segment;
        _cuboid_descriptor.segments.y = _height_segment;
        _cuboid_descriptor.segments.z = _length_segment;

        IEngineComponent::IComponentMesh3::CreatePramitiveToMesh(_cuboid_descriptor,m_pLastCreatedItemComponent);
    }
}

void InterfaceShellCreatePrimitive::fFunctionCreateEllipsoid(const double &_radius_X, const double &_radius_Y, const double &_radius_Z,
                                                             const int &_U_segment, const int _V_segment)
{

    qDebug() << "Create Primitive Ellipsoid " ;

    qDebug() << "Radius X:  " << _radius_X  ;
    qDebug() << "Radius Y:  " << _radius_Y  ;
    qDebug() << "Radius Z:  " << _radius_Z  ;

    qDebug() << "Segment U:  " << _U_segment  ;
    qDebug() << "Segment V:  " << _V_segment  ;


    CheckSaveSceneState();

    IEngine::MeshGenerators::EllipsoidDescriptor _ellipsoid_descriptor;

    _ellipsoid_descriptor.radius.x = _radius_X;
    _ellipsoid_descriptor.radius.y = _radius_Y;
    _ellipsoid_descriptor.radius.z = _radius_Z;

    _ellipsoid_descriptor.segments.x  = _U_segment;
    _ellipsoid_descriptor.segments.y  = _V_segment;

    IEngine::IMesh3 *NewMesh = new IEngine::IMesh3;
    IEngineComponent::IComponentMesh3 *ComponentMesh = new IEngineComponent::IComponentMesh3(NewMesh , 0);
    m_pLastCreatedItemComponent = ComponentMesh;
    IEngineComponent::IComponentMesh3::CreatePramitiveToMesh(_ellipsoid_descriptor,ComponentMesh);


    mSceneEngine->AddComponent(ComponentMesh);
}

void InterfaceShellCreatePrimitive::fFunctionRecalculateEllipsoid(const double &_radius_X, const double &_radius_Y, const double &_radius_Z,
                                                                  const int &_U_segment, const int _V_segment)
{
    if(m_pLastCreatedItemComponent)
    {

        qDebug() << "Recalculate Primitive Ellipsoid " ;

        qDebug() << "Radius X:  " << _radius_X  ;
        qDebug() << "Radius Y:  " << _radius_Y  ;
        qDebug() << "Radius Z:  " << _radius_Z  ;

        qDebug() << "Segment U:  " << _U_segment  ;
        qDebug() << "Segment V:  " << _V_segment  ;

        CheckSaveSceneState();

        IEngine::MeshGenerators::EllipsoidDescriptor _ellipsoid_descriptor;

        _ellipsoid_descriptor.radius.x = _radius_X;
        _ellipsoid_descriptor.radius.y = _radius_Y;
        _ellipsoid_descriptor.radius.z = _radius_Z;

        _ellipsoid_descriptor.segments.x  = _U_segment;
        _ellipsoid_descriptor.segments.y  = _V_segment;

        IEngineComponent::IComponentMesh3::CreatePramitiveToMesh(_ellipsoid_descriptor,m_pLastCreatedItemComponent);


    }
}


