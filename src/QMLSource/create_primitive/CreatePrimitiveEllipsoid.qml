import QtQuick 2.0
import QtQuick.Controls 2.0

Component
{
    id: _DelegateCreateEllipsoid


    Column
    {
        id : column_1
        anchors.left: parent.left
        anchors.right: parent.right
        spacing: 20


        property double radius_X: _init_radius_x
        property double radius_Y: _init_radius_y
        property double radius_Z: _init_radius_Z


        property int u_segment: _init_u_segment
        property int v_segment: _init_v_segment



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
            id : row_1
            anchors.left: parent.left
            anchors.right: parent.right
            spacing: 2


            Text
            {
                width: parent.width / 2
                height: 30

                text: " Radius X: "
            }

            SpinBox
            {
                id: spinbox_ellipsoid_w

                width: parent.width / 2
                height: 30

                editable: true
                wheelEnabled: true

                property int decimals: 2
                property real factor: Math.pow(10, decimals)
                property real realValue: value / factor

                from: factor
                value: radius_X * factor
                to: 999 * factor
                stepSize: 1 * factor


                validator: DoubleValidator
                {
                    bottom: Math.min(spinbox_ellipsoid_w.from, spinbox_ellipsoid_w.to)
                    top:  Math.max(spinbox_ellipsoid_w.from, spinbox_ellipsoid_w.to)
                }

                textFromValue: function(value, locale)
                {
                    value = value > factor ? value : factor
                    radius_X = value / factor
                    return Number(value / factor).toLocaleString(locale, 'f', spinbox_ellipsoid_w.decimals)
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

                text: " Radius Y: "
            }

            SpinBox
            {
                id: spinbox_ellipsoid_h

                width: parent.width / 2
                height: 30

                editable: true
                wheelEnabled: true

                property int decimals: 2
                property real factor: Math.pow(10, decimals)
                property real realValue: value / factor

                from: factor
                value: radius_Y * factor
                to: 999 * factor
                stepSize: 1 * factor


                validator: DoubleValidator
                {
                    bottom: Math.min(spinbox_ellipsoid_h.from, spinbox_ellipsoid_h.to)
                    top:  Math.max(spinbox_ellipsoid_h.from, spinbox_ellipsoid_h.to)
                }

                textFromValue: function(value, locale)
                {
                    value = value > factor ? value : factor
                    radius_Y = value / factor
                    return Number(value / factor).toLocaleString(locale, 'f', spinbox_ellipsoid_h.decimals)
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
                text: " Radius Z "
            }

            SpinBox
            {
                id: spinbox_ellipsoid_l

                width: parent.width / 2
                height: 30

                editable: true
                wheelEnabled: true

                property int decimals: 2
                property real factor: Math.pow(10, decimals)
                property real realValue: value / factor

                from: factor
                value: radius_Z * factor
                to: 999 * factor
                stepSize: 1 * factor


                validator: DoubleValidator
                {
                    bottom: Math.min(spinbox_ellipsoid_l.from, spinbox_ellipsoid_l.to)
                    top:  Math.max(spinbox_ellipsoid_l.from, spinbox_ellipsoid_l.to)
                }

                textFromValue: function(value, locale)
                {
                    value = value > factor ? value : factor
                    radius_Z = value / factor
                    return Number(value / factor).toLocaleString(locale, 'f', spinbox_ellipsoid_l.decimals)
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




        //-------------------------------------------------//


        Row
        {
            anchors.left: parent.left
            anchors.right: parent.right
            spacing: 2

            Text
            {
                width: parent.width / 2
                height: 30

                text: " U Segment: "
            }

            SpinBox
            {
                id: spinbox_ellipsoid_seg_u

                width: parent.width / 2
                height: 30

                editable: true
                wheelEnabled: true

                from: 3
                value: u_segment
                to: 99
                stepSize: 1

                validator: DoubleValidator
                {
                    bottom: Math.min(spinbox_ellipsoid_seg_u.from, spinbox_ellipsoid_seg_u.to)
                    top:  Math.max(spinbox_ellipsoid_seg_u.from, spinbox_ellipsoid_seg_u.to)
                }

                textFromValue: function(value, locale)
                {
                    u_segment = value
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

                text: " V Segment: "
            }

            SpinBox
            {
                id: spinbox_ellipsoid_seg_v

                width: parent.width / 2
                height: 30

                editable: true
                wheelEnabled: true

                from: 2
                value: v_segment
                to: 99
                stepSize: 1

                validator: DoubleValidator
                {
                    bottom: Math.min(spinbox_ellipsoid_seg_v.from, spinbox_ellipsoid_seg_v.to)
                    top:  Math.max(spinbox_ellipsoid_seg_v.from, spinbox_ellipsoid_seg_v.to)
                }

                textFromValue: function(value, locale)
                {
                    v_segment = value
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

            text: qsTr("Create Ellipsoid")
            height: 50

            onClicked:
            {
                InterfaceShellCreatePrimitive.fFunctionCreateEllipsoid( radius_X , radius_Y , radius_Z , u_segment , v_segment);
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

            text: qsTr("Recalculate Ellipsoid")
            height: 50

            onClicked:
            {
                InterfaceShellCreatePrimitive.fFunctionRecalculateEllipsoid( radius_X , radius_Y , radius_Z , u_segment , v_segment);
            }

        }


    }

}
