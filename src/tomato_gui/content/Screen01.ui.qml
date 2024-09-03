/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick 6.2
import QtQuick.Controls.Universal 6.2
import tomato_gui

Rectangle {
    id: mainWindow
    anchors.fill: parent
    color: "black"
    SwipeView {
        id: mainSwipe
        interactive: true
        currentIndex: 1
        anchors.fill: parent

        ControlPane {
            id: controlPane
            // anchors.fill: parent causes bug with QT
        }
        CameraPane {
            id: cp
        }
    }
}
