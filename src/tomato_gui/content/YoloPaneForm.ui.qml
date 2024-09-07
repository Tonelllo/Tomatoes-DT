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
import Tonelllo.TomatoModels 1.0

Rectangle {
    id: yoloWindow

    property alias yoloOut: yoloOut
    property alias ripe: ripe
    property alias halfRipe: halfRipe
    property alias green: green

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

            theme.windowColor: yoloWindow.color
            theme.font.pointSize: 40

            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: 100

            aspectRatio: 1
            horizontalAspectRatio: 1

            axisX.title: "Y"
            axisX.titleVisible: true
            axisX.labelAutoRotation: 90

            axisY.title: "X"
            axisY.titleVisible: true
            axisY.labelAutoRotation: 90

            axisZ.title: "Z"
            axisZ.titleVisible: true
            axisZ.labelAutoRotation: 90

            Scatter3DSeries {
                baseColor: "red"
                ItemModelScatterDataProxy {
                    itemModel: ripe
                    // Mapping model roles to scatter series item coordinates.
                    xPosRole: "xPos"
                    yPosRole: "yPos"
                    zPosRole: "zPos"
                }
            }
            Scatter3DSeries {
                baseColor: "orange"
                ItemModelScatterDataProxy {
                    itemModel: halfRipe
                    // Mapping model roles to scatter series item coordinates.
                    xPosRole: "xPos"
                    yPosRole: "yPos"
                    zPosRole: "zPos"
                }
            }
            Scatter3DSeries {
                baseColor: "green"
                ItemModelScatterDataProxy {
                    itemModel: green
                    // Mapping model roles to scatter series item coordinates.
                    xPosRole: "xPos"
                    yPosRole: "yPos"
                    zPosRole: "zPos"
                }
            }

        }

        /* 0 -> 'fully_ripened' */
        /* 1 -> 'half_ripened' */
        /* 2 -> 'green' */
        TomatoListModel {
            id: ripe
            ripeness: 0
        }
        TomatoListModel {
            id: halfRipe
            ripeness: 1
        }
        TomatoListModel {
            id: green
            ripeness: 2
        }
    }
}
