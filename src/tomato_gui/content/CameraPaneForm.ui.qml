

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts
import QtMultimedia
import Tonelllo.ImageProvider 1.0

Rectangle {
    id: cameraPane
    width: 1200
    height: 700

    color: Universal.background

    property alias button: button
    property alias baseImage: baseImage
    property alias maskedImage: maskedImage

    property alias hue_min_spin: hue_min_spin
    property alias hue_min_slider: hue_min_slider
    property alias hue_max_spin: hue_max_spin
    property alias hue_max_slider: hue_max_slider

    property alias sat_min_spin: sat_min_spin
    property alias sat_min_slider: sat_min_slider
    property alias sat_max_spin: sat_max_spin
    property alias sat_max_slider: sat_max_slider

    property alias val_min_spin: val_min_spin
    property alias val_min_slider: val_min_slider
    property alias val_max_spin: val_max_spin
    property alias val_max_slider: val_max_slider

    property int sat_min: 0
    property int sat_max: 0

    property int val_min: 0
    property int val_max: 0

    Button {
        id: button
    }

    ColumnLayout {
        id: mainStack
        Layout.fillWidth: true
        Layout.fillHeight: true
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            id: upLayout
            VideoOutput {
                id: baseImage
                Layout.fillWidth: true
                Layout.fillHeight: true
            }
            VideoOutput {
                id: maskedImage
                Layout.fillWidth: true
                Layout.fillHeight: true
            }
        }
        Rectangle {
            id: controls
            ColumnLayout {
                Layout.fillHeight: true
                Layout.fillWidth: true
                Item {
                    Layout.preferredHeight: 50
                    Text {
                        id: hue_min_text
                        text: "Hue min"
                        color: "white"
                    }
                    Slider {
                        id: hue_min_slider
                        anchors.left: hue_min_text.right
                        value: ImageProvider.hue_min
                        from: 0
                        to: 255
                    }
                    SpinBox {
                        id: hue_min_spin
                        editable: true
                        anchors.left: hue_min_slider.right
                        value: ImageProvider.hue_min
                        from: 0
                        to: 255
                    }
                    Text {
                        id: hue_max_text
                        anchors.left: hue_min_spin.right
                        text: "Hue max"
                        color: "white"
                    }
                    Slider {
                        id: hue_max_slider
                        anchors.left: hue_max_text.right
                        value: ImageProvider.hue_max
                        from: 0
                        to: 255
                    }
                    SpinBox {
                        id: hue_max_spin
                        editable: true
                        anchors.left: hue_max_slider.right
                        value: ImageProvider.hue_max
                        from: 0
                        to: 255
                    }
                }
                Item {
                    Layout.preferredHeight: 50
                    Text {
                        id: sat_min_text
                        text: "Sat min"
                        color: "white"
                    }
                    Slider {
                        id: sat_min_slider
                        anchors.left: sat_min_text.right
                        value: ImageProvider.sat_min
                        from: 0
                        to: 255
                    }
                    SpinBox {
                        id: sat_min_spin
                        editable: true
                        anchors.left: sat_min_slider.right
                        value: ImageProvider.sat_min
                        from: 0
                        to: 255
                    }
                    Text {
                        id: sat_max_text
                        anchors.left: sat_min_spin.right
                        text: "Sat max"
                        color: "white"
                    }
                    Slider {
                        id: sat_max_slider
                        anchors.left: sat_max_text.right
                        value: ImageProvider.sat_max
                        from: 0
                        to: 255
                    }
                    SpinBox {
                        id: sat_max_spin
                        editable: true
                        anchors.left: sat_max_slider.right
                        value: ImageProvider.sat_max
                        from: 0
                        to: 255
                    }
                }
                Item {
                    Layout.preferredHeight: 50
                    Text {
                        id: val_min_text
                        text: "Val min"
                        color: "white"
                    }
                    Slider {
                        id: val_min_slider
                        anchors.left: val_min_text.right
                        value: ImageProvider.val_min
                        from: 0
                        to: 255
                    }
                    SpinBox {
                        id: val_min_spin
                        editable: true
                        anchors.left: val_min_slider.right
                        value: ImageProvider.val_min
                        from: 0
                        to: 255
                    }
                    Text {
                        id: val_max_text
                        anchors.left: val_min_spin.right
                        text: "Val max"
                        color: "white"
                    }
                    Slider {
                        id: val_max_slider
                        anchors.left: val_max_text.right
                        value: ImageProvider.val_max
                        from: 0
                        to: 255
                    }
                    SpinBox {
                        id: val_max_spin
                        editable: true
                        anchors.left: val_max_slider.right
                        value: ImageProvider.val_max
                        from: 0
                        to: 255
                    }
                }
            }
        }
    }
}
