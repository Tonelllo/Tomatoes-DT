
/*
  This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
  It is supposed to be strictly declarative and only uses a subset of QML. If you edit
  this file manually, you might introduce QML code that is not supported by Qt Design Studio.
  Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick 2.15
import QtQuick.Controls.Universal 2.15
import QtQuick.Layouts
import QtMultimedia
import tomato_gui
import Tonelllo.ImageProvider 1.0

Rectangle {
    id: cameraPane
    color: Universal.background

    property alias button: button
    property alias saver: saver
    property alias restorer: restorer
    property alias baseImage: baseImage
    property alias maskedImage: maskedImage
    property alias radImage: radImage

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

    property alias rad_min_spin: rad_min_spin
    property alias rad_min_slider: rad_min_slider
    property alias rad_max_spin: rad_max_spin
    property alias rad_max_slider: rad_max_slider

    Button {
        id: button
        text: qsTr("tester")
    }

    ColumnLayout {
        id: mainStack
        anchors.fill: parent
        ColumnLayout {
            id: tabLayout
            Layout.margins: 30
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 100
            TabBar {
                id: filteredBar
                Layout.alignment: Qt.AlignTop | Qt.AlignRight
                Layout.preferredWidth: 300
                TabButton {
                    text: qsTr("Mask")
                }
                TabButton {
                    text: qsTr("CircleFilter")
                }
                TabButton {
                    text: qsTr("Final")
                }
            }
            RowLayout {
                id: upLayout

                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredHeight: 100
                VideoOutput {
                    id: baseImage
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                }
                StackLayout {
                    id: filteredImages
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    currentIndex: filteredBar.currentIndex
                    VideoOutput {
                        id: maskedImage
                    }
                    VideoOutput {
                        id: radImage
                    }
                    VideoOutput {
                        id: finalImage
                    }
                }
            }
        }

        GridLayout {
            Layout.fillHeight: true
            Layout.fillWidth: true
            Layout.preferredHeight: 100
            columns: 2
            Item {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredHeight: 100
                Layout.preferredWidth: 500
                Text {
                    id: hue_min_text
                    anchors.left: parent.left
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
            }
            Item {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredHeight: 100
                Layout.preferredWidth: 500

                Text {
                    id: hue_max_text
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
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredHeight: 100
                Layout.preferredWidth: 500
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
            }
            Item {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredHeight: 100
                Layout.preferredWidth: 500
                Text {
                    id: sat_max_text
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
                Layout.fillHeight: true
                Layout.preferredHeight: 100
                Layout.preferredWidth: 500
                Layout.fillWidth: true
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
            }
            Item {
                Layout.fillHeight: true
                Layout.preferredHeight: 100
                Layout.preferredWidth: 500
                Layout.fillWidth: true
                Text {
                    id: val_max_text
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
            Item {
                Layout.fillHeight: true
                Layout.preferredHeight: 100
                Layout.preferredWidth: 500
                Layout.fillWidth: true
                Text {
                    id: rad_min_text
                    text: "Radius min"
                    color: "white"
                }
                Slider {
                    id: rad_min_slider
                    anchors.left: rad_min_text.right
                    value: ImageProvider.rad_min
                    from: 0
                    to: 1000
                }
                SpinBox {
                    id: rad_min_spin
                    editable: true
                    anchors.left: rad_min_slider.right
                    value: ImageProvider.rad_min
                    from: 0
                    to: 1000
                }
            }
            Item {
                Layout.fillHeight: true
                Layout.preferredHeight: 100
                Layout.preferredWidth: 500
                Layout.fillWidth: true
                Text {
                    id: rad_max_text
                    text: "Rad max"
                    color: "white"
                }
                Slider {
                    id: rad_max_slider
                    value: ImageProvider.rad_max
                    from: 0
                    to: 1000
                }
                SpinBox {
                    id: rad_max_spin
                    editable: true
                    anchors.left: rad_max_slider.right
                    value: ImageProvider.rad_max
                    from: 0
                    to: 1000
                }
            }
            RowLayout {
                Layout.fillHeight: true
                Layout.preferredHeight: 100
                Layout.preferredWidth: 500
                Layout.fillWidth: true
                id: saveRestoreRow
                Button {
                    id: saver
                    text: qsTr("Save")
                }
                Button {
                    id: restorer
                    text: qsTr("Restore")
                }
            }
        }
    }
}
