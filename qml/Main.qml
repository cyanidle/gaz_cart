import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

// Main application window for gaz_cart — merges the config/tuning panel and the
// navigation costmap view into named tabs so everything lives in one window.

ApplicationWindow {
    id: root
    visible: true
    width: 800
    height: 850
    title: "gaz_cart"
    color: "#f0f0f0"

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        TabBar {
            id: tabBar
            objectName: "tabBar"
            Layout.fillWidth: true

            TabButton {
                text: "Config"
            }
            TabButton {
                text: "Nav"
            }
        }

        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: tabBar.currentIndex

            WheelConfig {
                model: radapter.model.node("odo")
                // Tab 0 — PID / geometry tuning panel (WheelConfig.qml)
            }
            NavView {
                model: radapter.model.node("nav")
                // Tab 1 — costmap + path planner panel (NavView.qml)
            }
        }
    }
}
