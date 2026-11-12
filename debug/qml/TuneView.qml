import QtQuick 2.3
import QtQuick.Controls 2.3
import QtQuick.Layouts 1.3
import QtCharts 2.3

Item {
    id: root

    property var model
    // The module's configured full-scale wheel speed (config id 16, read back on
    // connect): a normalized target of 1.0 commands this many m/s.
    property real maxSpeedMps: model.max_speed_mps ? model.max_speed_mps : 0.25
    property real maxVoltage: model.max_voltage ? model.max_voltage : 6.0
    property real minVoltage: model.min_voltage ? model.min_voltage : -6.0
    property double chartTime: 0
    property double lastSample: 0

    // Shared row metric. Every row is label | slider | value field | unit with
    // these three columns fixed, so the slider takes the same remainder in
    // every row and all sliders and fields line up down the panel. A column
    // left to its text width (the units did) makes each row distribute on its
    // own and the rows drift apart.
    property int rowLabelWidth: 170
    property int rowFieldWidth: 90
    property int rowUnitWidth: 80

    Component.onCompleted: {
        model.ensure("params")
        model.ensure("diag")
        model.ensure("status")
        model.ensure("max_speed_mps")
        model.ensure("max_voltage")
        model.ensure("min_voltage")
    }

    function decimalsOf(step) {
        var d = 0
        while (step < 0.999 && d < 5) { step *= 10; d++ }
        return d
    }

    // Feed the latest diagnostics sample into the chart series at ~25 Hz.
    // Maintain a 20 s rolling window: append, drop points older than the
    // window, scroll X on every chart, rescale Y to the points still visible.
    Timer {
        interval: 40; running: true; repeat: true
        onTriggered: {
            var d = model.diag
            if (!d || d.t === undefined) return
            lastSample = d.n
            chartTime += 0.04
            tgtSeries.append(chartTime, d.t)
            actSeries.append(chartTime, d.vm)
            spSeries.append(chartTime, d.sp)
            iqSeries.append(chartTime, d.iq)
            imSeries.append(chartTime, d.im)
            uqSeries.append(chartTime, d.uq)
            udSeries.append(chartTime, d.ud)
            busSeries.append(chartTime, d.b)
            var cutoff = chartTime - 20.0
            while (tgtSeries.count > 0 && tgtSeries.at(0).x < cutoff) {
                tgtSeries.remove(0); actSeries.remove(0)
                spSeries.remove(0); iqSeries.remove(0); imSeries.remove(0)
                uqSeries.remove(0); udSeries.remove(0); busSeries.remove(0)
            }
            scroll(speedX)
            scroll(currX)
            scroll(voltX)
            rescale(speedY, [tgtSeries, actSeries])
            rescale(currY, [spSeries, iqSeries, imSeries])
            rescale(voltY, [uqSeries, udSeries, busSeries])
        }
    }

    // Every chart shares the same rolling window; an axis that is not scrolled
    // leaves its series drawing past `max` and the chart goes blank once the
    // window has advanced past the initial range.
    function scroll(axis) {
        axis.min = Math.max(0, chartTime - 20.0)
        axis.max = Math.max(20.0, chartTime)
    }

    function rescale(axis, series) {
        var lo = Infinity, hi = -Infinity
        for (var s = 0; s < series.length; s++) {
            for (var i = 0; i < series[s].count; i++) {
                var y = series[s].at(i).y
                if (y < lo) lo = y
                if (y > hi) hi = y
            }
        }
        if (lo === Infinity) { lo = 0; hi = 0 }
        var pad = Math.max((hi - lo) * 0.15, 0.05)
        axis.min = lo - pad
        axis.max = hi + pad
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 8

        // ---- Charts -------------------------------------------------------
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 260
            Layout.minimumHeight: 200
            spacing: 8

            ChartView {
                id: speedChart
                objectName: "speedChart"
                Layout.fillWidth: true
                Layout.fillHeight: true
                antialiasing: true
                theme: ChartView.ChartThemeLight
                legend.alignment: Qt.AlignTop
                title: "Speed — target vs actual"

                ValueAxis {
                    id: speedX
                    min: 0; max: 20
                    titleText: "time (s)"
                    labelFormat: "%.0f"
                }
                ValueAxis {
                    id: speedY
                    min: -0.2; max: 0.8
                    titleText: "wheel speed (m/s)"
                    labelFormat: "%.2f"
                }
                LineSeries {
                    id: actSeries
                    objectName: "actSeries"
                    name: "actual"
                    color: "#27ae60"
                    width: 2
                    axisX: speedX; axisY: speedY
                }
                LineSeries {
                    id: tgtSeries
                    name: "target"
                    color: "#e74c3c"
                    width: 2
                    style: Qt.DashLine
                    axisX: speedX; axisY: speedY
                }
            }

            ChartView {
                id: currentChart
                Layout.fillWidth: true
                Layout.fillHeight: true
                antialiasing: true
                theme: ChartView.ChartThemeLight
                legend.alignment: Qt.AlignTop
                title: "Currents"

                ValueAxis {
                    id: currX
                    min: 0; max: 20
                    titleText: "time (s)"
                    labelFormat: "%.0f"
                }
                ValueAxis {
                    id: currY
                    min: -1; max: 6
                    titleText: "current (A)"
                    labelFormat: "%.1f"
                }
                LineSeries {
                    id: spSeries
                    name: "q set-point"
                    color: "#f39c12"
                    width: 1
                    style: Qt.DashLine
                    axisX: currX; axisY: currY
                }
                LineSeries {
                    id: iqSeries
                    name: "q measured"
                    color: "#2980b9"
                    width: 2
                    axisX: currX; axisY: currY
                }
                LineSeries {
                    id: imSeries
                    name: "phase magnitude"
                    color: "#7f8c8d"
                    width: 1
                    axisX: currX; axisY: currY
                }
            }
        }

        ChartView {
            id: voltChart
            Layout.fillWidth: true
            Layout.preferredHeight: 180
            Layout.minimumHeight: 120
            antialiasing: true
            theme: ChartView.ChartThemeLight
            legend.alignment: Qt.AlignTop
            title: "Voltages"

            ValueAxis {
                id: voltX
                min: 0; max: 20
                titleText: "time (s)"
                labelFormat: "%.0f"
            }
            ValueAxis {
                id: voltY
                min: 0; max: 20
                titleText: "voltage (V)"
                labelFormat: "%.1f"
            }
            LineSeries {
                id: uqSeries
                name: "uq"
                color: "#8e44ad"
                width: 2
                axisX: voltX; axisY: voltY
            }
            LineSeries {
                id: udSeries
                name: "ud"
                color: "#16a085"
                width: 1
                axisX: voltX; axisY: voltY
            }
            LineSeries {
                id: busSeries
                name: "bus"
                color: "#2c3e50"
                width: 1
                axisX: voltX; axisY: voltY
            }
        }

        // ---- Drive controls ------------------------------------------------
        RowLayout {
            Layout.fillWidth: true
            spacing: 8

            Label { text: "Target speed"; Layout.preferredWidth: root.rowLabelWidth }
            Slider {
                id: tgtSlider
                objectName: "tgtSlider"
                Layout.fillWidth: true
                // Normalized: -1..1 of the module's full-scale wheel speed. The
                // m/s equivalent is shown next to it; the module owns the
                // mapping, so the same slider drives a module at any top speed.
                from: -1; to: 1
                stepSize: 0.01
                value: 0
                snapMode: Slider.SnapAlways
                property double lastSend: 0
                function send() {
                    root.model.send({ target: value })
                    statusLabel.text = "Target " + value.toFixed(3) + " x full scale = "
                        + (value * root.maxSpeedMps).toFixed(3) + " m/s"
                }
                onMoved: {
                    var now = Date.now()
                    if (now - lastSend > 150) { lastSend = now; send() }
                }
                onPressedChanged: if (!pressed) send()
            }
            TextField {
                id: tgtField
                Layout.preferredWidth: root.rowFieldWidth
                // Bound to the slider so a drag is visible here as well as on
                // the handle; typing a value and pressing Enter moves the
                // slider, which re-fires the binding.
                text: tgtSlider.value.toFixed(3)
                selectByMouse: true
                onAccepted: {
                    var v = parseFloat(text)
                    if (isNaN(v)) return
                    tgtSlider.value = Math.max(-1, Math.min(1, v))
                    tgtSlider.send()
                }
            }
            Label {
                objectName: "tgtMpsLabel"
                Layout.preferredWidth: root.rowUnitWidth
                text: (tgtSlider.value * root.maxSpeedMps).toFixed(3) + " m/s"
                elide: Text.ElideRight
                color: "#888"
            }
        }

        // The full scale once, instead of repeating it in the row above: this is
        // what a normalized 1.0 commands on the connected module.
        Label {
            objectName: "fullScaleLabel"
            Layout.fillWidth: true
            text: "Target speed is normalized (-1..1 of the module's full scale): 1.0 = "
                + root.maxSpeedMps.toFixed(3) + " m/s (config id 16, G16)"
            color: "#888"
            font.pixelSize: 11
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 8

            Label { text: "Direct voltage"; Layout.preferredWidth: root.rowLabelWidth }
            Slider {
                id: voltSlider
                objectName: "voltSlider"
                Layout.fillWidth: true
                from: root.minVoltage; to: root.maxVoltage
                stepSize: 0.05
                value: 0
                snapMode: Slider.SnapAlways
                property double lastSend: 0
                function send() {
                    root.model.send({ voltage: value })
                    statusLabel.text = "Direct voltage " + value.toFixed(3) + " V (open loop)"
                }
                onMoved: {
                    var now = Date.now()
                    if (now - lastSend > 150) { lastSend = now; send() }
                }
                onPressedChanged: if (!pressed) send()
            }
            TextField {
                id: voltField
                Layout.preferredWidth: root.rowFieldWidth
                text: voltSlider.value.toFixed(3)
                selectByMouse: true
                onAccepted: {
                    var v = parseFloat(text)
                    if (isNaN(v)) return
                    voltSlider.value = Math.max(root.minVoltage, Math.min(root.maxVoltage, v))
                    voltSlider.send()
                }
            }
            Label {
                text: "V"
                Layout.preferredWidth: root.rowUnitWidth
                color: "#888"
            }
        }

        // ---- Voltage slider range ------------------------------------------
        // The module's open-loop channel takes any voltage; what it can apply is
        // bounded by the modulation ceiling (half the bus times the bootstrap
        // headroom), so the useful range moves with the rail. Editable here
        // instead of only on the command line (`vmax=`), and remembered in the
        // sent-config state like the rest of the tuning.
        RowLayout {
            Layout.fillWidth: true
            spacing: 8

            Label {
                text: "Voltage range"
                Layout.preferredWidth: root.rowLabelWidth
                color: "#555"
            }
            TextField {
                id: voltMinField
                objectName: "voltMinField"
                Layout.preferredWidth: root.rowFieldWidth
                text: (-root.maxVoltage).toFixed(1)
                selectByMouse: true
                horizontalAlignment: TextInput.AlignHCenter
                validator: DoubleValidator { bottom: -40.0; top: 0.0 }
                onEditingFinished: {
                    var v = parseFloat(text)
                    if (isNaN(v)) { text = (-root.maxVoltage).toFixed(1); return }
                    root.minVoltage = Math.min(0, Math.max(-40, v))
                    text = root.minVoltage.toFixed(1)
                }
            }
            TextField {
                id: voltMaxField
                objectName: "voltMaxField"
                Layout.preferredWidth: root.rowFieldWidth
                text: root.maxVoltage.toFixed(1)
                selectByMouse: true
                horizontalAlignment: TextInput.AlignHCenter
                validator: DoubleValidator { bottom: 0.0; top: 40.0 }
                onEditingFinished: {
                    var v = parseFloat(text)
                    if (isNaN(v)) { text = root.maxVoltage.toFixed(1); return }
                    root.maxVoltage = Math.max(0, Math.min(40, v))
                    text = root.maxVoltage.toFixed(1)
                }
            }
            Label {
                Layout.preferredWidth: root.rowUnitWidth
                text: root.minVoltage.toFixed(1) + ".." + root.maxVoltage.toFixed(1) + " V"
                elide: Text.ElideRight
                color: "#888"
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 8

            Button {
                objectName: "enableBtn"
                text: "Enable"
                onClicked: root.model.send({ enable: true })
            }
            Button {
                text: "Disable"
                onClicked: root.model.send({ enable: false })
            }
            Button {
                text: "Reset"
                // Zeroes the drive sliders and both module set-points; the
                // tuning parameters above are kept as they are.
                onClicked: {
                    tgtSlider.value = 0
                    voltSlider.value = 0
                    // The TextFields are bound to the sliders: setting the
                    // values above updates them, and an imperative text write
                    // here would break that binding for good.
                    root.model.send({ zero: true })
                    statusLabel.text = "Set-points zeroed (parameters kept)"
                }
            }
            Button {
                text: "Calibrate R/L"
                onClicked: {
                    root.model.send({ calibrate: true })
                    statusLabel.text = "Calibration running (wheel must be free, takes seconds)"
                }
            }
            Button {
                text: "Read from module"
                onClicked: root.model.send({ readback: true })
            }
            Button {
                objectName: "traceArmBtn"
                text: "Trace arm"
                // Arms the module's loop-trace dump: the next Hall anomaly,
                // angle snap, guard trip or driver fault dumps the ~51 ms of
                // loop history before it into the log/terminal.
                onClicked: {
                    root.model.send({ line: "W1" })
                    statusLabel.text = "Trace armed: the next anomaly dumps ~51 ms of loop history"
                }
            }
            Button {
                objectName: "traceDumpBtn"
                text: "Trace dump"
                onClicked: {
                    root.model.send({ line: "W" })
                    statusLabel.text = "Trace dump requested (lands in the log)"
                }
            }
            Button {
                objectName: "traceCountsBtn"
                text: "Counters"
                onClicked: {
                    root.model.send({ line: "K" })
                    statusLabel.text = "Event counters requested (hall_bad/rate_bad/angle_snap/...)"
                }
            }
            Item { Layout.fillWidth: true }
        }

        // ---- Parameter sliders ---------------------------------------------
        ScrollView {
            id: paramScroll
            Layout.fillWidth: true
            Layout.fillHeight: true

            ColumnLayout {
                // The scrollbar takes its width out of the view when it appears;
                // binding to the view's own available width keeps the parameter
                // sliders on the same grid as the drive rows either way.
                width: paramScroll.availableWidth
                spacing: 4

                Repeater {
                    model: root.model.params
                    delegate: RowLayout {
                        Layout.fillWidth: true
                        spacing: 8
                        Label {
                            text: modelData.label
                            Layout.preferredWidth: root.rowLabelWidth
                            color: "#333"
                        }
                        Slider {
                            id: paramSlider
                            objectName: "param_" + modelData.key
                            Layout.fillWidth: true
                            from: modelData.min; to: modelData.max
                            stepSize: modelData.step
                            value: modelData.value
                            snapMode: Slider.SnapAlways
                            property double sentValue: modelData.value
                            property double lastSend: 0
                            function send() {
                                if (Math.abs(value - sentValue) < 1e-9) return
                                sentValue = value
                                root.model.send({ set: { id: modelData.id, value: value } })
                                statusLabel.text = "Sent " + modelData.label + " = "
                                    + value.toFixed(root.decimalsOf(modelData.step))
                                    + " " + modelData.unit
                            }
                            onMoved: {
                                var now = Date.now()
                                if (now - lastSend > 150) { lastSend = now; send() }
                            }
                            onPressedChanged: if (!pressed) send()
                        }
                        TextField {
                            Layout.preferredWidth: root.rowFieldWidth
                            text: paramSlider.value.toFixed(root.decimalsOf(modelData.step))
                            selectByMouse: true
                            horizontalAlignment: TextInput.AlignHCenter
                            onAccepted: {
                                var v = parseFloat(text)
                                if (isNaN(v)) return
                                paramSlider.value = Math.max(modelData.min, Math.min(modelData.max, v))
                                paramSlider.send()
                            }
                        }
                        Label {
                            text: modelData.unit
                            Layout.preferredWidth: root.rowUnitWidth
                            elide: Text.ElideRight
                            color: "#555"
                            font.pixelSize: 11
                        }
                    }
                }
            }
        }

        // ---- Status bars ---------------------------------------------------
        Label {
            id: telemetryLabel
            objectName: "telemetryLabel"
            text: model.status ? model.status : "(no telemetry yet)"
            color: "#555"
            Layout.fillWidth: true
        }
        Label {
            id: statusLabel
            objectName: "statusLabel"
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            color: "#444"
            text: "Drag a slider (or type a value and press Enter) to apply."
        }
    }
}
