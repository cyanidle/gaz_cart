import QtQuick
import QtQuick.Controls
import "Tr"

// The diagnostics window — the module's own output and the recording controls,
// opened by the tuner's Diagnostics button and fed by the same model.
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
        // a second press of the tuner's Diagnostics button brings this window
        // back instead of opening another one — `show()` is what returns a
        // window the operator closed, `requestActivate()` then raises it
        onBringToFront: {
            root.show()
            root.requestActivate()
        }
    }
}
