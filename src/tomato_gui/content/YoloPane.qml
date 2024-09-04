import QtQuick 6.2
import Tonelllo.ImageProvider 1.0

YoloPaneForm{
    Component.onCompleted: {
        ImageProvider.yoloSink = yoloOut.videoSink
    }
}
