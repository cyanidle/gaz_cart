import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

// Keyboard teleoperation: arrow keys or WASD drive the robot, Space stops.
// While keys are held, twist commands stream to the Lua teleop node at ~20 Hz.
// Release all keys and the robot stops (dead-man switch).

Item {
    id: root

    property var model: null
    property double linFrac: 1.0    // fraction of max speed to use (0..1)
    property double rotFrac: 1.0    // fraction of max turn rate to use (0..1)

    // Key state: true while held
    property bool keyForward: false
    property bool keyBackward: false
    property bool keyLeft: false
    property bool keyRight: false
    property bool keyStop: false

    // Computed twist (normalised -1..1)
    property double cmdV: 0
    property double cmdOmega: 0

    function updateTwist() {
        var v = 0, omega = 0
        if (keyStop) {
            // Space held — force zero (stop overrides all)
        } else {
            if (keyForward)  v += linFrac
            if (keyBackward) v -= linFrac
            if (keyLeft)     omega += rotFrac
            if (keyRight)    omega -= rotFrac
            v = Math.max(-1, Math.min(1, v))
            omega = Math.max(-1, Math.min(1, omega))
        }
        cmdV = v
        cmdOmega = omega
        if (model) model.send({ action: "twist", v: v, omega: omega })
    }

    Component.onCompleted: {
        model.ensure("v")
        model.ensure("omega")
    }

    // Stream commands continuously while keys are held
    Timer {
        id: cmdTimer
        interval: 50; running: true; repeat: true
        onTriggered: updateTwist()
    }

    // Focus must be on this item for Keys to work. Click anywhere in the
    // teleop panel to grab focus; the focus outline is hidden.
    focus: true
    activeFocusOnTab: true

    Keys.onPressed: function (event) {
        switch (event.key) {
        case Qt.Key_W:     case Qt.Key_Up:      keyForward  = true; break
        case Qt.Key_S:     case Qt.Key_Down:    keyBackward = true; break
        case Qt.Key_A:     case Qt.Key_Left:    keyLeft     = true; break
        case Qt.Key_D:     case Qt.Key_Right:   keyRight    = true; break
        case Qt.Key_Space: keyStop = true; break
        }
        event.accepted = true
    }

    Keys.onReleased: function (event) {
        switch (event.key) {
        case Qt.Key_W:     case Qt.Key_Up:      keyForward  = false; break
        case Qt.Key_S:     case Qt.Key_Down:    keyBackward = false; break
        case Qt.Key_A:     case Qt.Key_Left:    keyLeft     = false; break
        case Qt.Key_D:     case Qt.Key_Right:   keyRight    = false; break
        case Qt.Key_Space:                      keyStop     = false; break
        }
        event.accepted = true
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 12

        // ── Header ────────────────────────────────────────────────────
        Label {
            text: "Keyboard Teleop"
            font.pixelSize: 18
            font.bold: true
            color: "#333"
        }
        Label {
            text: "Click here to grab focus, then use arrow keys or WASD to drive."
            color: "#666"
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
        }

        // ── Speed controls ────────────────────────────────────────────
        GroupBox {
            title: "Speed fraction"
            Layout.fillWidth: true
            ColumnLayout {
                anchors.fill: parent
                spacing: 8
                RowLayout {
                    Layout.fillWidth: true
                    Label { text: "Linear"; Layout.preferredWidth: 120 }
                    Slider {
                        id: linSlider
                        objectName: "linSpdSlider"
                        from: 0.0; to: 1.0; value: root.linFrac
                        Layout.fillWidth: true
                        onValueChanged: root.linFrac = value
                    }
                    Label {
                        text: (root.linFrac * 100).toFixed(0) + "%"
                        Layout.preferredWidth: 40
                        color: "#333"
                    }
                }
                RowLayout {
                    Layout.fillWidth: true
                    Label { text: "Angular"; Layout.preferredWidth: 120 }
                    Slider {
                        id: rotSlider
                        objectName: "rotSpdSlider"
                        from: 0.0; to: 1.0; value: root.rotFrac
                        Layout.fillWidth: true
                        onValueChanged: root.rotFrac = value
                    }
                    Label {
                        text: (root.rotFrac * 100).toFixed(0) + "%"
                        Layout.preferredWidth: 40
                        color: "#333"
                    }
                }
            }
        }

        // ── Twist readout ─────────────────────────────────────────────
        GroupBox {
            title: "Commanded twist (normalised -1..1)"
            Layout.fillWidth: true
            GridLayout {
                anchors.fill: parent
                columns: 2
                rowSpacing: 6; columnSpacing: 16

                Label { text: "Linear v:"; color: "#333" }
                Label { text: root.cmdV.toFixed(3); font.bold: true; color: "#1565c0" }

                Label { text: "Angular ω:"; color: "#333" }
                Label { text: root.cmdOmega.toFixed(3); font.bold: true; color: "#1565c0" }
            }
        }

        // ── Visual key indicators ─────────────────────────────────────
        GroupBox {
            title: "Controls"
            Layout.fillWidth: true
            GridLayout {
                anchors.fill: parent
                columns: 3
                rowSpacing: 4; columnSpacing: 4

                // Row 0: empty, forward, empty
                Item { Layout.preferredWidth: 60; Layout.preferredHeight: 40 }
                Rectangle {
                    Layout.preferredWidth: 60; Layout.preferredHeight: 40
                    radius: 4
                    color: root.keyForward ? "#4caf50" : "#e0e0e0"
                    border.color: root.keyForward ? "#388e3c" : "#bdbdbd"
                    Label {
                        anchors.centerIn: parent
                        text: "▲ W"
                        color: root.keyForward ? "#fff" : "#888"
                        font.bold: true
                    }
                }
                Item { Layout.preferredWidth: 60; Layout.preferredHeight: 40 }

                // Row 1: left, back, right
                Rectangle {
                    Layout.preferredWidth: 60; Layout.preferredHeight: 40
                    radius: 4
                    color: root.keyLeft ? "#4caf50" : "#e0e0e0"
                    border.color: root.keyLeft ? "#388e3c" : "#bdbdbd"
                    Label {
                        anchors.centerIn: parent
                        text: "◀ A"
                        color: root.keyLeft ? "#fff" : "#888"
                        font.bold: true
                    }
                }
                Rectangle {
                    Layout.preferredWidth: 60; Layout.preferredHeight: 40
                    radius: 4
                    color: root.keyBackward ? "#f44336" : "#e0e0e0"
                    border.color: root.keyBackward ? "#c62828" : "#bdbdbd"
                    Label {
                        anchors.centerIn: parent
                        text: "▼ S"
                        color: root.keyBackward ? "#fff" : "#888"
                        font.bold: true
                    }
                }
                Rectangle {
                    Layout.preferredWidth: 60; Layout.preferredHeight: 40
                    radius: 4
                    color: root.keyRight ? "#4caf50" : "#e0e0e0"
                    border.color: root.keyRight ? "#388e3c" : "#bdbdbd"
                    Label {
                        anchors.centerIn: parent
                        text: "▶ D"
                        color: root.keyRight ? "#fff" : "#888"
                        font.bold: true
                    }
                }

                // Row 2: empty, stop, empty
                Item { Layout.preferredWidth: 60; Layout.preferredHeight: 40 }
                Rectangle {
                    Layout.preferredWidth: 60; Layout.preferredHeight: 40
                    radius: 4
                    color: root.keyStop ? "#f44336" : "#eeeeee"
                    border.color: root.keyStop ? "#c62828" : "#bdbdbd"
                    Label {
                        anchors.centerIn: parent
                        text: "⏹ Space"
                        color: root.keyStop ? "#fff" : "#888"
                        font.bold: true
                    }
                }
                Item { Layout.preferredWidth: 60; Layout.preferredHeight: 40 }
            }
        }

        Item { Layout.fillHeight: true }  // spacer
    }
}
