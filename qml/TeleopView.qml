import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "Tr"

// Keyboard teleoperation: arrow keys or WASD drive the robot, Space stops.
// While keys are held, twist commands stream to the Lua teleop node at ~20 Hz.
// Release all keys and the robot stops (dead-man switch).
//
// The window also mirrors the command the cart is actually executing — cart.lua
// arbitrates the drive sources, so that twist may come from the joystick, the
// planner or another GUI, and the source is shown next to it.

Item {
    id: root

    property var model: null
    property double linFrac: 1.0    // fraction of max speed to use (0..1)
    property double rotFrac: 1.0    // fraction of max turn rate to use (0..1)

    // Accepted cart command, published by the drive arbiter on the model.
    property string cartSource: (model && typeof model.source === "string") ? model.source : "none"
    property double cartV: (model && typeof model.v === "number") ? model.v : 0
    property double cartOmega: (model && typeof model.omega === "number") ? model.omega : 0
    property bool cartStopped: !!(model && model.stopped === true)
    // Which manual input owns the cart right now (mods/teleop_switch.lua):
    // "joystick" while the stick reports, "keyboard" otherwise.
    property string operatorInput: (model && typeof model.operator === "string") ? model.operator : "keyboard"

    // Bench mode: drive exactly one wheel, the other three parked.
    property bool singleWheel: false
    property string singleWheelName: "fl"

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
        cmdOmega = singleWheel ? 0 : omega
        if (!model) return
        if (singleWheel) {
            // One wheel on the bench: forward/backward only, everything else parked.
            model.send({ action: "wheel", wheel: singleWheelName, v: v })
        } else {
            model.send({ action: "twist", v: v, omega: omega })
        }
    }

    Component.onCompleted: {
        model.ensure("v")
        model.ensure("omega")
        model.ensure("source")
        model.ensure("stopped")
        model.ensure("operator")
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
        RowLayout {
            Layout.fillWidth: true
            Label {
                text: Tr.t("tele_title")
                font.pixelSize: 18
                font.bold: true
                color: "#333"
            }
            Item { Layout.fillWidth: true }
            LangSwitch { model: root.model }
        }
        Label {
            text: Tr.t("tele_hint")
            color: "#666"
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
        }

        // ── Speed controls ────────────────────────────────────────────
        GroupBox {
            title: Tr.t("tele_speed_fraction")
            Layout.fillWidth: true
            ColumnLayout {
                anchors.fill: parent
                spacing: 8
                RowLayout {
                    Layout.fillWidth: true
                    Label { text: Tr.t("tele_linear"); Layout.preferredWidth: 120 }
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
                    Label { text: Tr.t("tele_angular"); Layout.preferredWidth: 120 }
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

        // ── Cart command (what the wheels are actually doing) ─────────
        GroupBox {
            title: Tr.t("tele_cart_group")
            Layout.fillWidth: true
            ColumnLayout {
                anchors.fill: parent
                spacing: 6

                Rectangle {
                    objectName: "stopBanner"
                    Layout.fillWidth: true
                    Layout.preferredHeight: 30
                    visible: root.cartStopped
                    radius: 4
                    color: "#fdecea"
                    border { color: "#c62828"; width: 1 }
                    Label {
                        anchors.centerIn: parent
                        text: Tr.t("tele_stop_banner")
                        color: "#c62828"
                        font.bold: true
                    }
                }

                GridLayout {
                    columns: 2
                    rowSpacing: 6; columnSpacing: 16

                    Label { text: Tr.t("tele_operator"); color: "#333" }
                    Label {
                        objectName: "operatorLabel"
                        text: Tr.sourceName(root.operatorInput)
                        font.bold: true
                        color: root.operatorInput === "keyboard" ? "#1565c0" : "#6a1b9a"
                    }

                    Label { text: Tr.t("tele_issued_by"); color: "#333" }
                    Label {
                        objectName: "cartSourceLabel"
                        text: Tr.sourceName(root.cartSource)
                        font.bold: true
                        color: "#1565c0"
                    }

                    Label { text: Tr.t("tele_linear_v"); color: "#333" }
                    Label {
                        objectName: "cartVLabel"
                        text: root.cartV.toFixed(3)
                        font.bold: true
                        color: "#1565c0"
                    }

                    Label { text: Tr.t("tele_angular_w"); color: "#333" }
                    Label {
                        objectName: "cartOmegaLabel"
                        text: root.cartOmega.toFixed(3)
                        font.bold: true
                        color: "#1565c0"
                    }
                }
            }
        }

        // ── Twist this window asks for ────────────────────────────────
        GroupBox {
            title: Tr.t("tele_window_group")
            Layout.fillWidth: true
            GridLayout {
                anchors.fill: parent
                columns: 2
                rowSpacing: 6; columnSpacing: 16

                Label { text: Tr.t("tele_linear_v"); color: "#333" }
                Label { text: root.cmdV.toFixed(3); font.bold: true; color: "#777" }

                Label { text: Tr.t("tele_angular_w"); color: "#333" }
                Label { text: root.cmdOmega.toFixed(3); font.bold: true; color: "#777" }
            }
        }

        // ── Bench: one wheel on its own ───────────────────────────────
        GroupBox {
            title: Tr.t("tele_bench_group")
            Layout.fillWidth: true
            ColumnLayout {
                anchors.fill: parent
                spacing: 8

                Label {
                    text: Tr.t("tele_bench_hint")
                    color: "#666"
                    wrapMode: Text.WordWrap
                    Layout.fillWidth: true
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    CheckBox {
                        id: singleWheelCheck
                        objectName: "singleWheelCheck"
                        text: Tr.t("tele_single_wheel")
                        checked: false
                        onCheckedChanged: {
                            root.singleWheel = checked
                            if (model) model.send({ action: "wheel", wheel: checked ? root.singleWheelName : "", v: 0 })
                        }
                    }
                    ComboBox {
                        id: singleWheelBox
                        objectName: "singleWheelBox"
                        model: ["fl", "fr", "rl", "rr"]
                        enabled: root.singleWheel
                        Layout.preferredWidth: 80
                        onCurrentTextChanged: {
                            root.singleWheelName = currentText
                            if (root.singleWheel && model) model.send({ action: "wheel", wheel: currentText, v: 0 })
                        }
                    }
                    Item { Layout.fillWidth: true }
                }
            }
        }

        // ── Visual key indicators ─────────────────────────────────────
        GroupBox {
            title: Tr.t("tele_controls")
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
