import QtQuick
import QtQuick.Controls

// Standalone navigation window — costmap, path, click-to-plan.
ApplicationWindow {
    id: root
    visible: true
    width: 900; height: 700
    title: "gaz_cart — Nav"
    color: "#f0f0f0"

    NavView {
        anchors.fill: parent
        model: radapter.model.branch("nav")
    }
}
