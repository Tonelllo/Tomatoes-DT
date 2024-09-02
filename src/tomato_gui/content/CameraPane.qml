import QtQuick 2.15
import Tonelllo.ImageProvider 1.0

CameraPaneForm {
    button.onClicked: console.log("Button Pressed")

    saver.onClicked: {
        ImageProvider.saveSettings()
    }
    restorer.onClicked: {
        ImageProvider.restoreSettings()
    }

    Component.onCompleted: {
        ImageProvider.origSink = baseImage.videoSink
        ImageProvider.maskedSink = maskedImage.videoSink
        ImageProvider.radSink = radImage.videoSink
    }

    Binding {
        target: ImageProvider
        property: "hue_min"
        value: hue_min_spin.value
    }
    Binding {
        target: ImageProvider
        property: "hue_min"
        value: hue_min_slider.value
    }
    Binding {
        target: ImageProvider
        property: "hue_max"
        value: hue_max_spin.value
    }
    Binding {
        target: ImageProvider
        property: "hue_max"
        value: hue_max_slider.value
    }

    Binding {
        target: ImageProvider
        property: "sat_min"
        value: sat_min_spin.value
    }
    Binding {
        target: ImageProvider
        property: "sat_min"
        value: sat_min_slider.value
    }
    Binding {
        target: ImageProvider
        property: "sat_max"
        value: sat_max_spin.value
    }
    Binding {
        target: ImageProvider
        property: "sat_max"
        value: sat_max_slider.value
    }

    Binding {
        target: ImageProvider
        property: "val_min"
        value: val_min_spin.value
    }
    Binding {
        target: ImageProvider
        property: "val_min"
        value: val_min_slider.value
    }
    Binding {
        target: ImageProvider
        property: "val_max"
        value: val_max_spin.value
    }
    Binding {
        target: ImageProvider
        property: "val_max"
        value: val_max_slider.value
    }

    Binding {
        target: ImageProvider
        property: "rad_min"
        value: rad_min_spin.value
    }
    Binding {
        target: ImageProvider
        property: "rad_min"
        value: rad_min_slider.value
    }
    Binding {
        target: ImageProvider
        property: "rad_max"
        value: rad_max_spin.value
    }
    Binding {
        target: ImageProvider
        property: "rad_max"
        value: rad_max_slider.value
    }
}
