import QtQuick 2.0
import QtQuick.Controls 2.0

import "./create_primitive" as CreatePrimitiveSpaceName


Item {
    width: 400
    height: 400

    Component {
        id: myComponent
        Text { text: "index" }    //fails
    }

    Component {
        id: myComponent2
        Text { text: "index Suka" }    //fails
    }




    Item
    {
        id: delegateComponent

        Text
        {
            id: pText
            text: "button"
            font.bold: true
            font.italic: true
            font.strikeout: true
            style: Text.Sunken
            font.pointSize: 10
        }

        ListModel {
            id: pModel

            ListElement { }
            ListElement { }
            ListElement { }
        }

        ListView {
            interactive: !currentItem.moving
            //interactive: true

            anchors.bottom: parent.bottom
            anchors.left: parent.left
            anchors.right: parent.right
            model: pModel
            delegate: pText
        }


    }





    Column
    {
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right

        Button
        {
            text : "Button"
        }

        Loader
        {
            id: loader
            sourceComponent: delegateComponent
        }
    }


}

//Item {
//    width: 640
//    height: 480

//    ListModel {
//        id: peopleModel
//        ListElement { }
//        ListElement { }
//        ListElement { }
//    }


//    Column {
//        anchors.fill: parent

//        Button
//        {
//            text:  "SukaButton"

//            onClicked:
//            {
//                background.sourceComponent = background_1
//            }
//        }



//        Repeater
//        {
//            anchors.fill: parent

//            model: peopleModel

//            Loader
//            {
//                id: background
//                sourceComponent: aDelegate3
//               // onLoaded: { item.name = name; item.age = age; }
//            }
//        }
//    }

//    Component {
//        id: aDelegate
//        Text {
//            property string name
//            property int age
//            x: 5
//            height: 15
//            text: name + "," + age
//        }
//    }

//    Component {
//        id: aDelegate2
//        Text {
//            id: wera
//            text: qsTr("text Suka")
//        }
//    }

//    Component {
//        id: aDelegate3
//        Text {
//            id: wera
//            text: qsTr("---Tesrt")
//        }
//    }


//    Component {
//           id: background_1

//           Rectangle {
//               color: main.backgroundColor1
//           }
//       }
//}
