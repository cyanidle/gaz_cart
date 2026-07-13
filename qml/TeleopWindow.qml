import QtQuick
import QtQuick.Controls

// Standalone teleop window — keyboard-driven robot control.
ApplicationWindow {
    id: root
    visible: true
    width: 400; height: 550
    title: "gaz_cart — Teleop"
    color: "#f0f0f0"

    TeleopView {
        anchors.fill: parent
        model: radapter.model.branch("teleop")
    }
}
