

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

Rectangle {
    id: cameraPane
    width: 640
    height: 480

    property alias button: button
    property alias baseImage: baseImage

    Button {
        id: button
    }

    VideoOutput {
        id: baseImage
        Layout.fillWidth: true
        Layout.fillHeight: true
        // width: 100
        // height: 100
    }
    // ColumnLayout {
    //     id: mainStack
    //     anchors.fill: parent
    //     RowLayout {
    //         id: upLayout
    //         anchors.fill: parent
    //         VideoOutput {
    //             id: baseImage
    //             Layout.fillWidth: true
    //             Layout.fillHeight: true
    //             // width: 100
    //             // height: 100
    //         }
    //     }
    // }
}
