import QtQuick
import QtQuick.Controls

// Standalone bench-tuning window for one wheel module (debug/tune.lua):
// diagnostics charts, drive sliders and the runtime-config slider form.
ApplicationWindow {
    id: root
    visible: true
    width: 1100; height: 950
    title: "gaz_cart — bench tune"
    color: "#f0f0f0"

    TuneView {
        anchors.fill: parent
        model: radapter.model
    }
}
