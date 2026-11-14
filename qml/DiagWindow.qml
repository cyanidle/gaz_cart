import QtQuick
import QtQuick.Controls
import "Tr"

// The diagnostics window — the module's own output and the recording controls,
// beside the tuner window the same model feeds (mods/tuner.lua opens both).
// A `K` block is five lines and a loop-trace dump a few hundred, while the
// tuner's status bar holds one, so the counters and the dump need a window.
ApplicationWindow {
    id: root
    visible: true
    width: 900; height: 640
    title: Tr.t("window_diag")
    color: "#f0f0f0"

    Diag {
        anchors.fill: parent
        model: radapter.model
    }
}
