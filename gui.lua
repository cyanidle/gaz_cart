-- =============================================================================
--  Config + tuning GUI for the cart — runs on a laptop / desktop and connects
--  to main.lua's websocket to tune each wheel module's runtime config live.
--
--      radapter --gui gui.lua [ws://HOST:PORT]
--
--  Defaults to ws://127.0.0.1:6080.  Pick a wheel (or "all"), type a value
--  and press Enter — it goes to the module(s) immediately.  The chart shows
--  target vs actual speed for the selected wheel so you can tune the velocity
--  PID with instant visual feedback.
-- =============================================================================

local config_defs = require "config_defs"

local url    = args[1] or "ws://127.0.0.1:6080"
local client = WebsocketClient { url = url, protocol = "json" }

pipe(client.events, function(ev) log.info("ws {}: {}", url, ev) end)

local view = QML[[
import QtQuick 2.3
import QtQuick.Window 2.3
import QtQuick.Controls 2.3
import QtQuick.Layouts 1.3
import QtCharts 2.3

Window {
    visible: true
    width: 520
    height: 720
    title: "Cart config"

    property var wheels: ["fl", "fr", "rl", "rr"]
    property double lastChartMs: 0
    property string chartWheel: "fl"
    property double chartTime: 0

    Component.onCompleted: {
        radapter.model.ensure("params")
        radapter.model.ensure("chart")
        radapter.model.ensure("odomText")
    }

    // Feed new speed samples into the chart series at ~20 Hz.  Maintain a
    // 10 s rolling window: append the latest point to both series, advance
    // the time counter, then drop points older than the window.
    Timer {
        interval: 50; running: true; repeat: true
        onTriggered: {
            var data = radapter.model.chart
            if (!data) return
            var pt = data[chartWheel]
            if (!pt) return
            var now = Date.now()
            if (now - lastChartMs < 80) return
            lastChartMs = now
            chartTime += 0.1
            tgtSeries.append(chartTime, pt.tgt)
            actSeries.append(chartTime, pt.act)
            // Drop points older than 10 s
            var cutoff = chartTime - 10.0
            while (tgtSeries.count > 0 && tgtSeries.at(0).x < cutoff) {
                tgtSeries.remove(0); actSeries.remove(0)
            }
            xAxis.min = Math.max(0, chartTime - 10.0)
            xAxis.max = Math.max(10.0, chartTime)
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 8

        // ---- Speed tuning chart -------------------------------------------
        GroupBox {
            title: "Speed — target vs actual"
            Layout.fillWidth: true
            Layout.preferredHeight: 260

            ColumnLayout {
                anchors.fill: parent
                spacing: 4

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    Label { text: "Wheel" }
                    ComboBox {
                        id: chartWheelBox
                        objectName: "chartWheelBox"
                        model: wheels
                        currentIndex: 0
                        onCurrentTextChanged: {
                            chartWheel = currentText
                            chartTime = 0
                            tgtSeries.clear()
                            actSeries.clear()
                        }
                    }
                }

                ChartView {
                    id: chartView
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    antialiasing: true
                    theme: ChartView.ChartThemeLight
                    legend.visible: true
                    legend.alignment: Qt.AlignTop

                    LineSeries {
                        id: actSeries
                        name: "actual"
                        color: "#27ae60"
                        width: 2
                    }
                    LineSeries {
                        id: tgtSeries
                        name: "target"
                        color: "#e74c3c"
                        width: 2
                        style: Qt.DashLine
                    }

                    ValueAxis {
                        id: xAxis
                        min: 0; max: 10
                        titleText: "time (s)"
                        labelFormat: "%.0f"
                    }
                    ValueAxis {
                        id: yAxis
                        min: -0.2; max: 0.8
                        titleText: "speed (m/s)"
                        labelFormat: "%.1f"
                    }
                }
            }
        }

        // ---- Target speed row ---------------------------------------------
        RowLayout {
            Layout.fillWidth: true
            spacing: 8
            Label { text: "Target speed" }
            ComboBox {
                id: tgtWheelBox
                objectName: "tgtWheelBox"
                model: ["all", "fl", "fr", "rl", "rr"]
                Layout.preferredWidth: 60
            }
            TextField {
                id: tgtField
                objectName: "tgtField"
                Layout.preferredWidth: 90
                text: "0.0"
                selectByMouse: true
                onAccepted: {
                    var v = parseFloat(text)
                    if (isNaN(v)) return
                    radapter.model.send({ action: "set_speed", wheel: tgtWheelBox.currentText, value: v })
                    statusLabel.text = "Target " + tgtWheelBox.currentText + " = " + v + " m/s"
                }
            }
            Label { text: "m/s"; color: "#888" }
            Item { Layout.fillWidth: true }
        }

        // ---- Config form (data-driven Repeater) ---------------------------
        GroupBox {
            title: "PID & geometry config"
            Layout.fillWidth: true

            ColumnLayout {
                anchors.fill: parent
                spacing: 6

                RowLayout {
                    Layout.fillWidth: true
                    Label { text: "Target wheel" }
                    ComboBox {
                        id: wheelBox
                        objectName: "wheelBox"
                        model: ["all", "fl", "fr", "rl", "rr"]
                        Layout.fillWidth: true
                    }
                }

                Repeater {
                    model: radapter.model.params
                    delegate: RowLayout {
                        Layout.fillWidth: true
                        spacing: 6
                        Label {
                            text: modelData.label
                            Layout.preferredWidth: 150
                        }
                        TextField {
                            objectName: "field_" + modelData.key
                            Layout.preferredWidth: 110
                            selectByMouse: true
                            text: String(modelData.default)
                            onAccepted: {
                                var v = parseFloat(text)
                                if (isNaN(v)) { statusLabel.text = "Not a number: " + text; return }
                                radapter.model.send({ wheel: wheelBox.currentText, id: modelData.id, value: v })
                                statusLabel.text = "Sent " + modelData.label + " = " + v
                                    + " to " + wheelBox.currentText
                            }
                        }
                        Label { text: modelData.unit; color: "#888" }
                    }
                }
            }
        }

        Item { Layout.fillHeight: true }

        // ---- Status bar ---------------------------------------------------
        Label {
            text: radapter.model.odomText ? radapter.model.odomText : "(no telemetry yet)"
            color: "#555"
            Layout.fillWidth: true
        }
        Label {
            id: statusLabel
            objectName: "status"
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            color: "#444"
            text: "Edit a field and press Enter to apply."
        }
    }
}
]]

-- QML "send" events -> websocket client -> main.lua server.
pipe(view, client)

-- Hand the config schema to the model; the Repeater is bound to it.
view { params = config_defs }

-- Route chart + odometry telemetry from the server into the model.
pipe(client, function(msg)
    if type(msg) ~= "table" then return end
    if msg.chart then
        view { chart = msg.chart }
    end
    if msg.odom then
        local o = msg.odom
        view { odomText = string.format("odom  x=%.2f  y=%.2f  theta=%.1f deg  v=%.2f  omega=%.2f",
            o.x or 0, o.y or 0, math.deg(o.theta or 0), o.v or 0, o.omega or 0) }
    end
end)
