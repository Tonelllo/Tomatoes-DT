import QtQuick 2.15
import Tonelllo.ImageProvider 1.0

CameraPaneForm {
    button.onClicked: console.log("Button Pressed")
    Component.onCompleted: {
        ImageProvider.sink = baseImage.videoSink
    }
}
