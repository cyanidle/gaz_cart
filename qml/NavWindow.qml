import QtQuick
import QtQuick.Controls
import "Tr"

// Standalone navigation window — costmap, manual positioning, and planning.
ApplicationWindow {
    id: root
    visible: true
    width: 900; height: 700
    title: Tr.t("window_nav")
    color: "#f0f0f0"

    NavView {
        anchors.fill: parent
        model: radapter.model.branch("nav")
    }
}
