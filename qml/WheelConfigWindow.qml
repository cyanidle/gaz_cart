import QtQuick
import QtQuick.Controls

// Standalone config/tuning window — PID chart, speed controls, config form.
ApplicationWindow {
    id: root
    visible: true
    width: 700; height: 800
    title: "gaz_cart — Config"
    color: "#f0f0f0"

    WheelConfig {
        anchors.fill: parent
        model: radapter.model.branch("odo")
    }
}
