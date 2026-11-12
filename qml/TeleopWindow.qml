import QtQuick
import QtQuick.Controls
import "Tr"

// Standalone teleop window — keyboard-driven robot control.
ApplicationWindow {
    id: root
    visible: true
    width: 400; height: 780
    title: Tr.t("window_teleop")
    color: "#f0f0f0"

    TeleopView {
        anchors.fill: parent
        model: radapter.model.branch("teleop")
    }
}
