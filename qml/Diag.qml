import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "Tr"

// Diagnostics beside the tuner: the module's own output (serial `#` lines — the
// `K` counters, the loop-trace dump, EEPROM and boot messages) in a scrollback,
// the counters/trace buttons that produce it, and the record controls. Recording
// is what the *transport* delivers: the module's lines on a serial bench channel,
// one JSON object per `{state}` snapshot over the cart's service channel.
Item {
    id: root

    property var model
    // `K` and the loop trace exist on the module's own serial channel only; the
    // service channel carries a per-wheel state snapshot and nothing else.
    property bool counters: model.counters === true
    property string transport: model.transport ? model.transport : "serial"
    property bool logActive: model.log_active === true
    property string logPath: model.log_path ? model.log_path : ""
    property string output: model.text ? model.text : ""
    property int rowLabelWidth: 150

    // The chrome must start in the language Lua built the model text in, not
    // always in the QML default (same seed as qml/Tuner.qml).
    property string modelLang: (model && model.lang !== undefined) ? model.lang : "ru"
    onModelLangChanged: Tr.lang = modelLang

    Component.onCompleted: {
        model.ensure("counters")
        model.ensure("log_active")
        model.ensure("log_path")
        model.ensure("text")
        model.ensure("status")
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 6

        RowLayout {
            Layout.fillWidth: true
            spacing: 8

            Button {
                objectName: "countersBtn"
                visible: root.counters
                text: Tr.t("tune_counters")
                onClicked: root.model.send({ line: "K" })
            }
            Button {
                objectName: "traceArmBtn"
                visible: root.counters
                text: Tr.t("tune_trace_arm")
                onClicked: root.model.send({ line: "W1" })
            }
            Button {
                objectName: "traceDumpBtn"
                visible: root.counters
                text: Tr.t("tune_trace_dump")
                onClicked: root.model.send({ line: "W" })
            }
            Label {
                Layout.fillWidth: true
                visible: !root.counters
                text: Tr.t("diag_serial_only")
                wrapMode: Text.WordWrap
                color: "#888"
            }
            Item { Layout.fillWidth: true }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 8

            Label {
                text: Tr.t("diag_log_path")
                Layout.preferredWidth: root.rowLabelWidth
            }
            TextField {
                id: logPathField
                objectName: "logPathField"
                Layout.fillWidth: true
                text: root.logPath
                selectByMouse: true
                enabled: !root.logActive
                onEditingFinished: root.logPath = text
            }
            Button {
                objectName: "logStartBtn"
                enabled: !root.logActive && logPathField.text.length > 0
                text: Tr.t("diag_log_start")
                onClicked: root.model.send({ record: "start", path: logPathField.text })
            }
            Button {
                objectName: "logStopBtn"
                enabled: root.logActive
                text: Tr.t("diag_log_stop")
                onClicked: root.model.send({ record: "stop" })
            }
        }

        Label {
            text: Tr.t("diag_output")
            color: "#555"
        }
        ScrollView {
            Layout.fillWidth: true
            Layout.fillHeight: true

            TextArea {
                id: outputArea
                objectName: "outputArea"
                readOnly: true
                wrapMode: TextEdit.NoWrap
                font.family: "monospace"
                text: root.output
                // the newest line is the interesting one after a `K` or a dump
                onTextChanged: cursorPosition = length
            }
        }

        Label {
            id: statusLabel
            objectName: "diagStatusLabel"
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            color: "#444"
            text: root.model.status ? root.model.status : ""
        }
    }
}
