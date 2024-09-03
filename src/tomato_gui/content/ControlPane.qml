import QtQuick 6.2
import Tonelllo.ProcessCaller 1.0

ControlPaneForm {
    homeButton.onClicked: {
        ProcessCaller.startHomeMovement()
    }
    processOutput.text: ProcessCaller.processOutput
}
