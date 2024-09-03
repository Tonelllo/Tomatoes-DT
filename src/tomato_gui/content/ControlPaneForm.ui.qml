

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick 6.2
import QtQuick.Controls.Universal 6.2
import QtQuick.Layouts
import tomato_gui

Rectangle {
    id: mainPane

    color: Universal.background

    property alias homeButton: homeButton
    property alias processOutput: processOutput

    ColumnLayout {
        anchors.fill: parent
        id: mainLayout
        Button {
            id: homeButton
            text: "HOME"

            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 500
            Layout.preferredWidth: 5000
        }

        Text{
            id: processOutput
            color: "white"

            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 500
            Layout.preferredWidth: 5000
        }
    }
}
