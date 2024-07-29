/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/

import QtQuick 6.2
import QtQuick.Controls 6.2
import tomato_gui

Rectangle{
    width: 1200
    height: 700
    SwipeView {
        id: mainSwipe
        anchors.fill: parent

        CameraPane {
            id: cp
        }
    }
}