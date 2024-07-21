import QtQuick 2.15
import Tonelllo.ImageProvider 1.0

CameraPaneForm {
    button.onClicked: console.log("Button Pressed")
    Component.onCompleted: {
        ImageProvider.origSink = baseImage.videoSink
        ImageProvider.maskedSink = maskedImage.videoSink
    }

    hue_min_spin.onValueChanged: {
        hue_min_slider.value = hue_min_spin.value
    }
    hue_min_slider.onValueChanged: {
        hue_min_spin.value = hue_min_slider.value
    }
    hue_max_spin.onValueChanged: {
        hue_max_slider.value = hue_max_spin.value
    }
    hue_max_slider.onValueChanged: {
        hue_max_spin.value = hue_max_slider.value
    }

    sat_min_spin.onValueChanged: {
        sat_min_slider.value = sat_min_spin.value
    }
    sat_min_slider.onValueChanged: {
        sat_min_spin.value = sat_min_slider.value
    }
    sat_max_spin.onValueChanged: {
        sat_max_slider.value = sat_max_spin.value
    }
    sat_max_slider.onValueChanged: {
        sat_max_spin.value = sat_max_slider.value
    }

    val_min_spin.onValueChanged: {
        val_min_slider.value = val_min_spin.value
    }
    val_min_slider.onValueChanged: {
        val_min_spin.value = val_min_slider.value
    }
    val_max_spin.onValueChanged: {
        val_max_slider.value = val_max_spin.value
    }
    val_max_slider.onValueChanged: {
        val_max_spin.value = val_max_slider.value
    }
}
