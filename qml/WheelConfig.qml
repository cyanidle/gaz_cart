import QtQuick 2.3
import QtQuick.Controls 2.3
import QtQuick.Layouts 1.3
import QtCharts 2.3
import "Tr"

Item {
    id: root

    property var model
    property var wheels: ["fl", "fr", "rl", "rr"]
    property double lastChartMs: 0
    property string chartWheel: "fl"
    property double chartTime: 0

    Component.onCompleted: {
        model.ensure("params")
        model.ensure("chart")
        model.ensure("odomText")
    }

    // "all" is a protocol word, not a label: cart.lua broadcasts only for that
    // literal, and the saved-config keys are "<wheel>:<id>". The combos carry the
    // ids in `id` and show `name`, so the language never reaches the wire.
    ListModel {
        id: wheelChoices
        Component.onCompleted: root.refreshWheelNames()
    }
    property string lang: Tr.lang
    onLangChanged: refreshWheelNames()
    function refreshWheelNames() {
        if (wheelChoices.count !== root.wheels.length + 1) {
            wheelChoices.clear()
            wheelChoices.append({ id: "all", name: Tr.t("wheel_all") })
            for (var i = 0; i < root.wheels.length; i++) {
                wheelChoices.append({ id: root.wheels[i], name: root.wheels[i] })
            }
            return
        }
        wheelChoices.setProperty(0, "name", Tr.t("wheel_all"))
    }

    // Feed new speed samples into the chart series at ~20 Hz.  Maintain a
    // 10 s rolling window: append the latest point to both series, advance
    // the time counter, then drop points older than the window.
    Timer {
        interval: 50; running: true; repeat: true
        onTriggered: {
            var data = model.chart
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
            // Rescale Y to the points still in the window, so the range both
            // grows for spikes and shrinks back once they scroll off.
            var lo = Infinity, hi = -Infinity
            for (var i = 0; i < actSeries.count; i++) {
                var a = actSeries.at(i).y, t = tgtSeries.at(i).y
                if (a < lo) lo = a
                if (t < lo) lo = t
                if (a > hi) hi = a
                if (t > hi) hi = t
            }
            if (lo === Infinity) { lo = 0; hi = 0 }
            var pad = Math.max((hi - lo) * 0.15, 0.05)
            yAxis.min = lo - pad
            yAxis.max = hi + pad
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 8

        // ---- Speed tuning chart -------------------------------------------
        GroupBox {
            title: Tr.t("chart_speed")
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumHeight: 220

            ColumnLayout {
                anchors.fill: parent
                spacing: 4

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    Label { text: Tr.t("wheel") }
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
                    LangSwitch { model: root.model; Layout.leftMargin: 8 }
                    Item { Layout.fillWidth: true }
                }

                ChartView {
                    id: chartView
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    antialiasing: true
                    theme: ChartView.ChartThemeLight
                    legend.visible: true
                    legend.alignment: Qt.AlignTop

                    // Axes must be declared before the series that reference
                    // them, and each series must be bound to axisX/axisY —
                    // otherwise QtCharts silently creates hidden default axes
                    // and our min/max scrolling is ignored.
                    ValueAxis {
                        id: xAxis
                        min: 0; max: 10
                        titleText: Tr.t("axis_time")
                        labelFormat: "%.0f"
                    }
                    ValueAxis {
                        id: yAxis
                        min: -0.2; max: 0.8
                        titleText: Tr.t("cfg_axis_speed")
                        labelFormat: "%.2f"
                    }

                    LineSeries {
                        id: actSeries
                        name: Tr.t("series_actual")
                        color: "#27ae60"
                        width: 2
                        axisX: xAxis
                        axisY: yAxis
                    }
                    LineSeries {
                        id: tgtSeries
                        name: Tr.t("series_target")
                        color: "#e74c3c"
                        width: 2
                        style: Qt.DashLine
                        axisX: xAxis
                        axisY: yAxis
                    }
                }
            }
        }

        // ---- Target speed row ---------------------------------------------
        RowLayout {
            Layout.fillWidth: true
            spacing: 8
            Label { text: Tr.t("tune_target_speed") }
            ComboBox {
                id: tgtWheelBox
                objectName: "tgtWheelBox"
                model: wheelChoices
                textRole: "name"
                valueRole: "id"
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
                    model.send({ action: "set_speed", wheel: tgtWheelBox.currentValue, value: v })
                    statusLabel.text = Tr.tf("cfg_target_status",
                        [tgtWheelBox.currentText, v])
                }
            }
            Label { text: Tr.t("unit_m_s"); color: "#888" }
            Item { Layout.fillWidth: true }
        }

        // ---- Direct voltage row (open-loop, bring-up) ---------------------
        RowLayout {
            Layout.fillWidth: true
            spacing: 8
            Label { text: Tr.t("tune_direct_voltage") }
            ComboBox {
                id: dirWheelBox
                objectName: "dirWheelBox"
                model: wheelChoices
                textRole: "name"
                valueRole: "id"
                Layout.preferredWidth: 60
            }
            TextField {
                id: dirField
                objectName: "dirField"
                Layout.preferredWidth: 90
                text: "0.0"
                selectByMouse: true
                onAccepted: {
                    var v = parseFloat(text)
                    if (isNaN(v)) return
                    model.send({ action: "direct", wheel: dirWheelBox.currentValue, value: v })
                    statusLabel.text = Tr.tf("cfg_direct_status",
                        [dirWheelBox.currentText, v])
                }
            }
            Label { text: Tr.t("unit_v"); color: "#888" }
            Item { Layout.fillWidth: true }
        }

        // ---- Config form (data-driven Repeater) ---------------------------
        GroupBox {
            title: Tr.t("cfg_group_title")
            Layout.fillWidth: true

            ColumnLayout {
                anchors.fill: parent
                spacing: 6

                RowLayout {
                    Layout.fillWidth: true
                    Label { text: Tr.t("cfg_target_wheel") }
                    ComboBox {
                        id: wheelBox
                        objectName: "wheelBox"
                        model: wheelChoices
                        textRole: "name"
                        valueRole: "id"
                        Layout.fillWidth: true
                    }
                }

                Repeater {
                    // qualified via root: bare `model` here is the Repeater's
                    // own model role, not WheelConfig's property
                    model: root.model.params
                    delegate: RowLayout {
                        Layout.fillWidth: true
                        spacing: 8
                        Label {
                            objectName: "cfgparam_" + modelData.key
                            text: modelData.label
                            Layout.preferredWidth: 200
                            elide: Text.ElideRight
                            color: "#333"
                        }
                        // Value + unit as one visual group: a thin bordered
                        // frame around the field and a tight unit suffix.
                        Frame {
                            Layout.preferredWidth: 160
                            padding: 0
                            background: Rectangle {
                                color: "#fcfcfc"
                                border { color: "#ccc"; width: 1 }
                                radius: 4
                            }
                            RowLayout {
                                anchors.fill: parent
                                spacing: 0
                                TextField {
                                    objectName: "field_" + modelData.key
                                    Layout.fillWidth: true
                                    selectByMouse: true
                                    text: String(modelData.default)
                                    placeholderText: modelData.unit
                                    background: Rectangle { color: "transparent" }
                                    onAccepted: {
                                        var v = parseFloat(text)
                                        if (isNaN(v)) {
                                            statusLabel.text = Tr.tf("cfg_not_a_number", text)
                                            return
                                        }
                                        root.model.send({ action: "config", wheel: wheelBox.currentValue, id: modelData.id, value: v })
                                        statusLabel.text = Tr.tf("cfg_sent",
                                            [modelData.label, v, wheelBox.currentText])
                                    }
                                }
                                Label {
                                    text: modelData.unit
                                    color: "#555"
                                    rightPadding: 8
                                    font.pixelSize: 11
                                }
                            }
                        }
                    }
                }

            }
        }

        // ---- Status bar ---------------------------------------------------
        Label {
            text: model.odomText ? model.odomText : Tr.t("no_telemetry")
            color: "#555"
            Layout.fillWidth: true
        }
        Label {
            id: statusLabel
            objectName: "status"
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            color: "#444"
            text: Tr.t("cfg_hint")
        }
    }
}