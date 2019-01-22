import QtQuick 2.0
import QtQuick.Controls 2.0




Component
{
    id: _DelegateCreateCuboid


    Column
    {
        id : column_1
        anchors.left: parent.left
        anchors.right: parent.right
        spacing: 20



        property double cuboid_width: _init_cuboid_width
        property double cuboid_height: _init_cuboid_height
        property double cuboid_length: _init_cuboid_length


        property int cuboid_segment_width: _init_cuboid_segment_width
        property int cuboid_segment_height: _init_cuboid_segment_height
        property int cuboid_segment_length: _init_cuboid_segment_length

        Rectangle
        {

            anchors.left: parent.left
            anchors.right: parent.right

            color: "black"
            border.color: "black"
            height: 5
        }

        Row
        {
            anchors.left: parent.left
            anchors.right: parent.right
            spacing: 2


            Text
            {
                width: parent.width / 2
                height: 30

                text: " Width: "
            }

            SpinBox
            {
                id: spinbox_cub_w

                width: parent.width / 2
                height: 30

                editable: true
                wheelEnabled: true

                property int decimals: 2
                property real factor: Math.pow(10, decimals)
                property real realValue: value / factor

                from: factor
                value: cuboid_width * factor
                to: 999 * factor
                stepSize: 1 * factor


                validator: DoubleValidator
                {
                    bottom: Math.min(spinbox_cub_w.from, spinbox_cub_w.to)
                    top:  Math.max(spinbox_cub_w.from, spinbox_cub_w.to)
                }

                textFromValue: function(value, locale)
                {
                    value = value > factor ? value : factor
                    cuboid_width = value / factor
                    return Number(value / factor).toLocaleString(locale, 'f', spinbox_cub_w.decimals)
                }

                valueFromText: function(text, locale)
                {

                    return Number.fromLocaleString(locale, text) * factor
                }
            }

        }






        Row
        {
            id : row_2
            anchors.left: parent.left
            anchors.right: parent.right
            spacing: 2

            Text
            {
                width: parent.width / 2
                height: 30

                text: " Height: "
            }

            SpinBox
            {
                id: spinbox_cub_h

                width: parent.width / 2
                height: 30

                editable: true
                wheelEnabled: true

                property int decimals: 2
                property real factor: Math.pow(10, decimals)
                property real realValue: value / factor

                from: factor
                value: cuboid_height * factor
                to: 999 * factor
                stepSize: 1 * factor


                validator: DoubleValidator
                {
                    bottom: Math.min(spinbox_cub_h.from, spinbox_cub_h.to)
                    top:  Math.max(spinbox_cub_h.from, spinbox_cub_h.to)
                }

                textFromValue: function(value, locale)
                {
                    value = value > factor ? value : factor
                    cuboid_height = value / factor
                    return Number(value / factor).toLocaleString(locale, 'f', spinbox_cub_h.decimals)
                }

                valueFromText: function(text, locale)
                {

                    return Number.fromLocaleString(locale, text) * factor
                }
            }

        }


        Row
        {
            id : row_3
            anchors.left: parent.left
            anchors.right: parent.right
            spacing: 2

            Text
            {
                width: parent.width / 2
                height: 30
                text: " Length: "
            }

            SpinBox
            {
                id: spinbox_cub_l

                width: parent.width / 2
                height: 30

                editable: true
                wheelEnabled: true

                property int decimals: 2
                property real factor: Math.pow(10, decimals)
                property real realValue: value / factor

                from: factor
                value: cuboid_length * factor
                to: 999 * factor
                stepSize: 1 * factor


                validator: DoubleValidator
                {
                    bottom: Math.min(spinbox_cub_l.from, spinbox_cub_l.to)
                    top:  Math.max(spinbox_cub_l.from, spinbox_cub_l.to)
                }

                textFromValue: function(value, locale)
                {
                    value = value > factor ? value : factor
                    cuboid_length = value / factor
                    return Number(value / factor).toLocaleString(locale, 'f', spinbox_cub_l.decimals)
                }

                valueFromText: function(text, locale)
                {

                    return Number.fromLocaleString(locale, text) * factor
                }
            }

        }


        Rectangle
        {
            anchors.left: parent.left
            anchors.right: parent.right

            color: "black"
            border.color: "black"
            height: 5
        }





        Row
        {
            anchors.left: parent.left
            anchors.right: parent.right
            spacing: 2

            Text
            {
                width: parent.width / 2
                height: 30

                text: " Width Segment: "
            }

            SpinBox
            {
                id: spinbox_cub_seg_w

                width: parent.width / 2
                height: 30

                editable: true
                wheelEnabled: true

                from: 1
                value: cuboid_segment_width
                to: 99
                stepSize: 1

                validator: DoubleValidator
                {
                    bottom: Math.min(spinbox_cub_l.from, spinbox_cub_l.to)
                    top:  Math.max(spinbox_cub_l.from, spinbox_cub_l.to)
                }

                textFromValue: function(value, locale)
                {
                    cuboid_segment_width = value
                    return value
                }
            }
        }


        Row
        {
            anchors.left: parent.left
            anchors.right: parent.right
            spacing: 2

            Text
            {
                width: parent.width / 2
                height: 30

                text: " Height Segment: "
            }

            SpinBox
            {
                id: spinbox_cub_seg_h

                width: parent.width / 2
                height: 30

                editable: true
                wheelEnabled: true

                from: 1
                value: cuboid_segment_height
                to: 99
                stepSize: 1

                validator: DoubleValidator
                {
                    bottom: Math.min(spinbox_cub_h.from, spinbox_cub_h.to)
                    top:  Math.max(spinbox_cub_h.from, spinbox_cub_h.to)
                }

                textFromValue: function(value, locale)
                {
                    cuboid_segment_height = value
                    return value
                }
            }
        }


        Row
        {
            anchors.left: parent.left
            anchors.right: parent.right
            spacing: 2

            Text
            {
                width: parent.width / 2
                height: 30

                text: " Length Segment: "
            }

            SpinBox
            {
                id: spinbox_cub_seg_l

                width: parent.width / 2
                height: 30

                editable: true
                wheelEnabled: true

                from: 1
                value: cuboid_segment_length
                to: 99
                stepSize: 1

                validator: DoubleValidator
                {
                    bottom: Math.min(spinbox_cub_l.from, spinbox_cub_l.to)
                    top:  Math.max(spinbox_cub_l.from, spinbox_cub_l.to)
                }

                textFromValue: function(value, locale)
                {
                    cuboid_segment_length = value
                    return value
                }
            }
        }


        Rectangle
        {
            anchors.left: parent.left
            anchors.right: parent.right

            color: "black"
            border.color: "black"
            height: 5
        }


        // Кнопка для создания динамических кнопок
        Button {

            id: button_create
            anchors.left: parent.left
            anchors.right: parent.right

            background: Rectangle
            {
                color: button_create.down ? "gray" : "#f6f6f6"
                border.color: "#26282a"
                radius: 4
            }

            text: qsTr("Create Cuboid")
            height: 50

            onClicked:
            {
                console.log("suka")
                InterfaceShellCreatePrimitive.fFunctionCreateCuboid( cuboid_width , cuboid_height , cuboid_length ,
                                                                     cuboid_segment_width , cuboid_segment_height , cuboid_segment_length)
            }

        }

        // Кнопка для создания динамических кнопок
        Button {

            id: button_recalculate
            anchors.left: parent.left
            anchors.right: parent.right

            background: Rectangle
            {
                color: button_recalculate.down ? "gray" : "#f6f6f6"
                border.color: "#26282a"
                radius: 4
            }

            text: qsTr("Recalculate Cuboid")
            height: 50

            onClicked:
            {
                InterfaceShellCreatePrimitive.fFunctionRecalculateCuboid( cuboid_width , cuboid_height , cuboid_length ,
                                                                          cuboid_segment_width , cuboid_segment_height , cuboid_segment_length)
            }

        }


    }

}


