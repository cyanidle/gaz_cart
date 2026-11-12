import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtCharts 2.3

// One bench tuner view for both transports (mods/tuner.lua decides which):
//   serial  a module over USART2 — per-phase diagnostics, read-back, enable
//   can     the cart's service channel — per-wheel target/actual speed only
//
// The parameter rows carry hover help (qml/ParamHelp.qml): hovering a row shows
// what that knob does and a small sketch of the effect at its current value.
Item {
    id: root

    property var model
    // Full scale a normalized 1.0 means (config id 16, read back on serial).
    property real maxSpeedMps: model.max_speed_mps ? model.max_speed_mps : 0.5
    property real maxVoltage: model.max_voltage ? model.max_voltage : 10.0
    property real minVoltage: model.min_voltage ? model.min_voltage : -10.0
    property bool diagnostics: model.diagnostics === true
    property bool canEnable: model.can_enable === true
    property bool canReadBack: model.readback === true
    property string transport: model.transport ? model.transport : "serial"
    property string wheel: model.wheel ? model.wheel : ""

    property double chartTime: 0
    property double lastSample: 0

    // Shared row metric: label | slider | value field | unit.
    property int rowLabelWidth: 170
    property int rowFieldWidth: 90
    property int rowUnitWidth: 80

    // Wheels the CAN transport can drive (empty on the serial channel, which
    // is attached to exactly one module). Lua pushes a list, which arrives as
    // an array-like object, so copy it into a real JS array here.
    property var wheelList: {
        var source = model.wheels
        var out = []
        if (!source || typeof source.length !== "number") return out
        for (var i = 0; i < source.length; i++) out.push(source[i])
        return out
    }

    Component.onCompleted: {
        model.ensure("params")
        model.ensure("diag")
        model.ensure("status")
        model.ensure("max_speed_mps")
        model.ensure("max_voltage")
        model.ensure("min_voltage")
        model.ensure("wheels")
    }

    function decimalsOf(step) {
        var d = 0
        while (step < 0.999 && d < 5) { step *= 10; d++ }
        return d
    }

    // Narrowing the voltage range must not leave a voltage applied that the
    // slider can no longer show: re-send the clamped value.
    function clampVoltage() {
        var clamped = Math.max(minVoltage, Math.min(maxVoltage, voltSlider.value))
        if (Math.abs(clamped - voltSlider.value) > 1e-9) {
            voltSlider.value = clamped
            voltSlider.send()
        }
    }

    // Feed the latest sample into the chart series at ~25 Hz over a 20 s window.
    Timer {
        interval: 40; running: true; repeat: true
        onTriggered: {
            var d = model.diag
            if (!d || (d.tgt_m_s === undefined && d.act_m_s === undefined)) return
            lastSample = d.t_ms !== undefined ? d.t_ms : lastSample + 1
            chartTime += 0.04
            tgtSeries.append(chartTime, d.tgt_m_s || 0)
            actSeries.append(chartTime, d.act_m_s || 0)
            if (root.diagnostics) {
                iqSeries.append(chartTime, d.iq || 0)
                uqSeries.append(chartTime, d.uq || 0)
                busSeries.append(chartTime, d.bus || 0)
            }
            var cutoff = chartTime - 20.0
            while (tgtSeries.count > 0 && tgtSeries.at(0).x < cutoff) {
                tgtSeries.remove(0); actSeries.remove(0)
                if (root.diagnostics) { iqSeries.remove(0); uqSeries.remove(0); busSeries.remove(0) }
            }
            scroll(speedX)
            rescale(speedY, [tgtSeries, actSeries])
            if (root.diagnostics) {
                scroll(auxX)
                rescale(auxY, [iqSeries, uqSeries])
            }
        }
    }

    // An axis that is not scrolled leaves its series drawing past `max` and the
    // chart goes blank once the window has advanced past the initial range.
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

        // ---- Where the values come from ------------------------------------
        RowLayout {
            Layout.fillWidth: true
            spacing: 8
            Label {
                text: "Wheel"
                visible: wheelBox.visible
            }
            ComboBox {
                id: wheelBox
                objectName: "tuneWheelBox"
                visible: root.wheelList.length > 0
                Layout.preferredWidth: 90
                model: root.wheelList
                currentIndex: Math.max(0, root.wheelList.indexOf(root.wheel))
                onActivated: root.model.send({ wheel: currentText })
            }
            Label {
                objectName: "sourceLabel"
                text: root.transport === "serial"
                    ? "Module over serial — values read from the module"
                    : "Cart over CAN (service channel) — wheel " + root.wheel
                        + ", values are what was sent"
                font.bold: true
                color: "#333"
            }
            Item { Layout.fillWidth: true }
            Label {
                objectName: "fullScaleLabel"
                text: root.maxSpeedMps.toFixed(3) + " m/s at 1.0 (config id 16)"
                color: "#888"
                font.pixelSize: 11
            }
        }

        // ---- Charts --------------------------------------------------------
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 240
            Layout.minimumHeight: 170
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
                    objectName: "tgtSeries"
                    name: "target"
                    color: "#e74c3c"
                    width: 2
                    style: Qt.DashLine
                    axisX: speedX; axisY: speedY
                }
            }

            ChartView {
                id: auxChart
                objectName: "auxChart"
                visible: root.diagnostics
                Layout.fillWidth: true
                Layout.fillHeight: true
                antialiasing: true
                theme: ChartView.ChartThemeLight
                legend.alignment: Qt.AlignTop
                title: "q current (A) and q voltage (V)"

                ValueAxis {
                    id: auxX
                    min: 0; max: 20
                    titleText: "time (s)"
                    labelFormat: "%.0f"
                }
                ValueAxis {
                    id: auxY
                    min: -1; max: 8
                    labelFormat: "%.1f"
                }
                LineSeries {
                    id: iqSeries
                    objectName: "iqSeries"
                    name: "iq (A)"
                    color: "#2980b9"
                    width: 2
                    axisX: auxX; axisY: auxY
                }
                LineSeries {
                    id: uqSeries
                    objectName: "uqSeries"
                    name: "uq (V)"
                    color: "#8e44ad"
                    width: 1
                    axisX: auxX; axisY: auxY
                }
                LineSeries {
                    id: busSeries
                    objectName: "busSeries"
                    name: "bus (V)"
                    color: "#2c3e50"
                    width: 1
                    axisX: auxX; axisY: auxY
                }
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
                // Normalized: -1..1 of the module's full-scale wheel speed.
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
                objectName: "tuneTgtField"
                Layout.preferredWidth: root.rowFieldWidth
                text: tgtSlider.value.toFixed(3)
                selectByMouse: true
                horizontalAlignment: TextInput.AlignHCenter
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
                objectName: "voltField"
                Layout.preferredWidth: root.rowFieldWidth
                text: voltSlider.value.toFixed(3)
                selectByMouse: true
                horizontalAlignment: TextInput.AlignHCenter
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

        // Voltage slider range: the module's ceiling moves with the rail, so the
        // useful range is editable here instead of only on the command line.
        RowLayout {
            Layout.fillWidth: true
            spacing: 8

            Label {
                text: "Voltage range"
                Layout.preferredWidth: root.rowLabelWidth
                color: "#555"
            }
            Item { Layout.fillWidth: true }
            TextField {
                id: voltMinField
                objectName: "voltMinField"
                Layout.preferredWidth: root.rowFieldWidth
                text: root.minVoltage.toFixed(1)
                selectByMouse: true
                horizontalAlignment: TextInput.AlignHCenter
                validator: DoubleValidator { bottom: -40.0; top: 0.0 }
                onEditingFinished: {
                    var v = parseFloat(text)
                    if (isNaN(v)) { text = root.minVoltage.toFixed(1); return }
                    root.minVoltage = Math.min(0, Math.max(-40, v))
                    text = root.minVoltage.toFixed(1)
                    root.clampVoltage()
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
                    root.clampVoltage()
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 8

            Button {
                objectName: "enableBtn"
                visible: root.canEnable
                text: "Enable"
                onClicked: root.model.send({ enable: true })
            }
            Button {
                objectName: "disableBtn"
                visible: root.canEnable
                text: "Disable"
                onClicked: root.model.send({ enable: false })
            }
            Button {
                objectName: "resetBtn"
                text: "Reset"
                // Zeroes the drive sliders and both set-points; parameters stay.
                onClicked: {
                    tgtSlider.value = 0
                    voltSlider.value = 0
                    root.model.send({ zero: true })
                    statusLabel.text = "Set-points zeroed (parameters kept)"
                }
            }
            Button {
                objectName: "calibrateBtn"
                visible: root.diagnostics
                text: "Calibrate R/L"
                onClicked: root.model.send({ line: "A" })
            }
            Button {
                objectName: "readbackBtn"
                visible: root.canReadBack
                text: "Read from module"
                onClicked: root.model.send({ readback: true })
            }
            Button {
                objectName: "traceArmBtn"
                visible: root.diagnostics
                text: "Trace arm"
                onClicked: {
                    root.model.send({ line: "W1" })
                    statusLabel.text = "Trace armed: the next anomaly dumps ~51 ms of loop history"
                }
            }
            Button {
                objectName: "traceDumpBtn"
                visible: root.diagnostics
                text: "Trace dump"
                onClicked: root.model.send({ line: "W" })
            }
            Button {
                objectName: "countersBtn"
                visible: root.diagnostics
                text: "Counters"
                onClicked: root.model.send({ line: "K" })
            }
            Item { Layout.fillWidth: true }
        }

        // ---- Parameter sliders with hover help -----------------------------
        ScrollView {
            id: paramScroll
            Layout.fillWidth: true
            Layout.fillHeight: true

            ColumnLayout {
                width: paramScroll.availableWidth
                spacing: 4

                Repeater {
                    model: root.model.params
                    delegate: RowLayout {
                        id: paramRow
                        objectName: "paramrow_" + modelData.key
                        Layout.fillWidth: true
                        spacing: 8

                        // Hover anywhere on the row (label, slider, field) to see
                        // what the parameter does and how the value shapes it.
                        HoverHandler {
                            id: paramHover
                            onHoveredChanged: {
                                if (hovered) {
                                    help.param = modelData
                                    help.open()
                                } else {
                                    help.close()
                                }
                            }
                        }

                        Label {
                            objectName: "paramlabel_" + modelData.key
                            text: modelData.label
                            Layout.preferredWidth: root.rowLabelWidth
                            color: paramHover.hovered ? "#2c7fb8" : "#333"
                            font.underline: paramHover.hovered
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
                            objectName: "paramfield_" + modelData.key
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

                        ParamHelp {
                            id: help
                            parent: paramRow
                            x: 0
                            // Below the row, or above it when the window has no
                            // room left below — a row near the bottom would
                            // otherwise be cut off by the window edge.
                            y: {
                                var top = paramRow.mapToItem(null, 0, 0).y
                                var fitsBelow = top + paramRow.height + 2 + help.implicitHeight <= root.height
                                return fitsBelow ? paramRow.height + 2 : -help.implicitHeight - 2
                            }
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
            objectName: "tuneStatusLabel"
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            color: "#444"
            text: "Drag a slider (or type a value and press Enter) to apply. Hover a parameter to see what it does."
        }
    }
}
