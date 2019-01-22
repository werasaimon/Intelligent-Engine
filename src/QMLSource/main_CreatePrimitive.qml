import QtQuick 2.5
import QtQuick.Controls 2.0


import "./create_primitive" as CreatePrimitiveSpaceName

Item
{

    //---------------------------------------------------------------------//

    CreatePrimitiveSpaceName.CreatePrimitiveCuboid
    {
        id: _DelegateCreateCuboid
    }


    // ListView для представления данных в виде списка
    ListView
    {
        id: listViewCreateCuboid

        visible: false


        function visibleOff()
        {
            listViewCreateCuboid.visible = 0;
        }

        function visibleOn()
        {
            listViewCreateCuboid.visible = 1;
        }


        // Размещаем его в оставшейся части окна приложения
        anchors.top: row.bottom
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.topMargin: 50

        /* в данном свойстве задаём вёрстку одного объекта
         * который будем отображать в списке в качестве одного элемента списка
         * */
        delegate: _DelegateCreateCuboid





        // Сама модель, в которой будут содержаться все элементы
        model: ListModel
        {
            id: listModelCreateCuboid // задаём ей id для обращения
        }
    }


    //---------------------------------------------------------------------//


   CreatePrimitiveSpaceName.CreatePrimitiveEllipsoid
   {
       id: _DelegateCreateEllipsoid
   }


   // ListView для представления данных в виде списка
   ListView
   {
       id: listViewCreateEllipsoid

       visible: false

       function visibleOff()
       {
           listViewCreateEllipsoid.visible = 0;
       }

       function visibleOn()
       {
           listViewCreateEllipsoid.visible = 1;
       }


       // Размещаем его в оставшейся части окна приложения
       anchors.top: row.bottom
       anchors.bottom: parent.bottom
       anchors.left: parent.left
       anchors.right: parent.right
       anchors.topMargin: 50


       /* в данном свойстве задаём вёрстку одного объекта
         * который будем отображать в списке в качестве одного элемента списка
         * */
       delegate: _DelegateCreateEllipsoid


       // Сама модель, в которой будут содержаться все элементы
       model: ListModel
       {
           id: listModelCreateEllipsoid // задаём ей id для обращения
       }
   }



    // ListView для представления данных в виде списка
    ListView
    {
        id: listViewCreateEllipsoid2

        function visibleOff()
        {
            listViewCreateEllipsoid2.visible = 0;
        }

        function visibleOn()
        {
            listViewCreateEllipsoid2.visible = 1;
        }

        visible: false
        // Размещаем его в оставшейся части окна приложения
        anchors.top: row.bottom
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.topMargin: 50


        /* в данном свойстве задаём вёрстку одного объекта
         * который будем отображать в списке в качестве одного элемента списка
         * */
        delegate: _DelegateCreateEllipsoid


        // Сама модель, в которой будут содержаться все элементы
        model: ListModel
        {
            id: listModelCreateEllipsoid2 // задаём ей id для обращения
        }
    }



   //---------------------------------------------------------------------//

    property int index_numer: 0

   //---------------------------------------------------------------------//

    Column
    {


        id: row
        // Задаём размеры строки и прибиваем к верхней части окна приложения
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        spacing: 10




        // Кнопка для создания динамических кнопок
        Button
        {
            id: button1
            anchors.left: parent.left
            anchors.right: parent.right
            // width: (parent.width / 8)*2
            height: 30

            text: qsTr("Create Cuboid")

            background: Rectangle
            {
                      color: button1.down ? "gray" : "#f6f6f6"
                      border.color: "#26282a"
                      radius: 4
            }

            onClicked:
            {
                if(!listViewCreateCuboid.visible)
                {
                    listViewCreateCuboid.visibleOn()
                    listViewCreateEllipsoid.visibleOff()
                    listViewCreateEllipsoid2.visibleOff()

                    listModelCreateCuboid.clear()
                    listModelCreateEllipsoid.clear()
                    listModelCreateEllipsoid2.clear()


                     //listModelCreateEllipsoid.append({})
                    listModelCreateCuboid.insert(0,{ "_init_cuboid_width" : 1 ,
                                                     "_init_cuboid_height" : 1 ,
                                                     "_init_cuboid_length" : 1 ,
                                                     "_init_cuboid_segment_width" : 1 ,
                                                     "_init_cuboid_segment_height" : 1 ,
                                                     "_init_cuboid_segment_length" : 1})

                }

            }


        }

        // Кнопка для удаления динамических кнопок
        Button {
            id: button2
            text: qsTr("Create Ellipsoid")

            anchors.left: parent.left
            anchors.right: parent.right

            //width: (parent.width / 8)*2
            height: 30

            background: Rectangle
            {
                      color: button2.down ? "gray" : "#f6f6f6"
                      border.color: "#26282a"
                      radius: 4
            }

            onClicked:
            {


               if(!listViewCreateEllipsoid.visible)
               {
                   listViewCreateEllipsoid.visibleOn()
                   listViewCreateCuboid.visibleOff()
                   listViewCreateEllipsoid2.visibleOff()


                   listModelCreateCuboid.clear()
                   listModelCreateEllipsoid.clear()
                   listModelCreateEllipsoid2.clear()

                   //listModelCreateEllipsoid.append({})
                   listModelCreateEllipsoid.insert(0,{"_init_radius_x" : 1 ,
                                                      "_init_radius_y" : 1 ,
                                                      "_init_radius_z" : 1 ,
                                                      "_init_u_segment" : 10 ,
                                                      "_init_v_segment" : 10})

               }


            }
        }

        Button {
            id: button3
            text: qsTr("Create Conus")

            anchors.left: parent.left
            anchors.right: parent.right

            // width: (parent.width / 8)*2
            height: 30

            background: Rectangle
            {
                      color: button3.down ? "gray" : "#f6f6f6"
                      border.color: "#26282a"
                      radius: 4
            }



            onClicked:
            {

            }
        }

        Button {
            id: button4
            text: qsTr("Create Cylinder")

            anchors.left: parent.left
            anchors.right: parent.right

            background: Rectangle
            {
                      color: button4.down ? "gray" : "#f6f6f6"
                      border.color: "#26282a"
                      radius: 4
            }

            // width: (parent.width / 8)*2
            height: 30
        }

        Button {
            id: button5
            text: qsTr("Create Capsule")

            anchors.left: parent.left
            anchors.right: parent.right

            background: Rectangle
            {
                      color: button5.down ? "gray" : "#f6f6f6"
                      border.color: "#26282a"
                      radius: 4
            }

            // width: (parent.width / 8)*2
            height: 30
        }

        Button {
            id: button6
            text: qsTr("Create Curve")

            anchors.left: parent.left
            anchors.right: parent.right

            background: Rectangle
            {
                      color: button6.down ? "gray" : "#f6f6f6"
                      border.color: "#26282a"
                      radius: 4
            }

            // width: (parent.width / 8)*2
            height: 30
        }

        Button {
            id: button7
            text: qsTr("Create Bezier")

            anchors.left: parent.left
            anchors.right: parent.right

            background: Rectangle
            {
                      color: button7.down ? "gray" : "#f6f6f6"
                      border.color: "#26282a"
                      radius: 4
            }

           // width: (parent.width / 8)*2
            height: 30
        }
    }


}



