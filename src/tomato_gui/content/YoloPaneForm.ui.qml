/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick 6.2
import QtQuick.Controls.Universal 6.2
import QtQuick.Layouts
import QtMultimedia 6.2
import QtDataVisualization
import tomato_gui
import Tonelllo.ImageProvider 1.0

Rectangle {
    id: yoloWindow

    property alias yoloOut: yoloOut

    color: Universal.background

    RowLayout {
        id: mainLayout
        anchors.fill: parent
        VideoOutput {
            id: yoloOut

            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: 100
        }

        Scatter3D {
            id: tomatoVisualizer

            theme.backgroundColor: yoloWindow.color
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: 100
            Scatter3DSeries {
                baseColor: "red"
                ItemModelScatterDataProxy {
                    itemModel: redModel
                    // Mapping model roles to scatter series item coordinates.
                    xPosRole: "xPos"
                    yPosRole: "yPos"
                    zPosRole: "zPos"
                }
            }
            Scatter3DSeries {
                baseColor: "orange"
                ItemModelScatterDataProxy {
                    itemModel: orangeModel
                    // Mapping model roles to scatter series item coordinates.
                    xPosRole: "xPos"
                    yPosRole: "yPos"
                    zPosRole: "zPos"
                }
            }
        }

        ListModel {
            id: orangeModel
            ListElement {
                xPos: "2.754"
                yPos: "1.455"
                zPos: "3.362"
            }
            ListElement {
                xPos: "3.164"
                yPos: "2.022"
                zPos: "4.348"
            }
        }
        ListModel {
            id: redModel
            ListElement {
                xPos: "4.564"
                yPos: "1.865"
                zPos: "1.346"
            }
            ListElement {
                xPos: "1.068"
                yPos: "1.224"
                zPos: "2.983"
            }
            ListElement {
                xPos: "2.323"
                yPos: "2.502"
                zPos: "3.133"
            }
        }
    }
}
