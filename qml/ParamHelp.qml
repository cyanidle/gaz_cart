import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "Tr"

// Hover help for one tuner parameter: what the knob does, plus a small sketch
// of that effect drawn from the parameter's current value. The sketch kinds
// come from mods/tuner_help.lua (`chart`); every kind has a branch in
// drawSketch().
Popup {
    id: root
    objectName: "paramHelp"

    property var param
    // The parameter's live value, so the sketch moves with the slider instead of
    // being a static picture (the row's own modelData copy is a snapshot).
    property real value: 0

    width: 470
    padding: 10
    modal: false
    focus: false
    closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside

    function normalized() {
        if (!param) return 0
        var span = param.max - param.min
        if (!(span > 0)) return 0
        var frac = (value - param.min) / span
        return Math.max(0, Math.min(1, frac))
    }

    function helpText() {
        return param && param.help ? (param.help.what || "") : ""
    }

    function helpTitle() {
        if (!param) return ""
        return Tr.tf("help_title", [param.label, param.unit])
    }

    contentItem: ColumnLayout {
        spacing: 6

        Label {
            objectName: "helpTitle"
            text: root.helpTitle()
            font.bold: true
            font.pixelSize: 13
        }

        Canvas {
            id: sketch
            objectName: "helpSketch"
            // the tester reads this to prove the sketch follows the value
            property real normalized: root.normalized()
            Layout.fillWidth: true
            Layout.preferredHeight: 104
            antialiasing: true
            onPaint: {
                var ctx = getContext("2d")
                ctx.reset()
                root.drawSketch(ctx, width, height)
            }
            Connections {
                target: root
                function onParamChanged() { sketch.requestPaint() }
                function onValueChanged() { sketch.requestPaint() }
                function onVisibleChanged() { if (root.visible) sketch.requestPaint() }
            }
            Component.onCompleted: requestPaint()
        }

        Label {
            objectName: "helpText"
            text: root.helpText()
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
            font.pixelSize: 11
            color: "#333"
        }
    }

    // ---- Sketch drawing ------------------------------------------------------
    function drawSketch(ctx, w, h) {
        var kind = param && param.help ? param.help.chart : ""
        var t = normalized()
        var pad = 10
        var x0 = pad, y0 = h - pad, x1 = w - pad, y1 = pad
        var fg = "#2c7fb8", grey = "#b0b0b0", hot = "#e74c3c"

        ctx.clearRect(0, 0, w, h)
        ctx.fillStyle = "#fbfbfb"
        ctx.fillRect(0, 0, w, h)
        ctx.strokeStyle = "#e2e2e2"
        ctx.lineWidth = 1
        ctx.beginPath()
        ctx.moveTo(x0, y1); ctx.lineTo(x0, y0); ctx.lineTo(x1, y0)
        ctx.stroke()

        function line(points, color, width, dashed) {
            ctx.strokeStyle = color
            ctx.lineWidth = width || 2
            ctx.setLineDash(dashed ? [4, 3] : [])
            ctx.beginPath()
            for (var i = 0; i < points.length; i++) {
                if (i === 0) ctx.moveTo(points[i][0], points[i][1])
                else ctx.lineTo(points[i][0], points[i][1])
            }
            ctx.stroke()
            ctx.setLineDash([])
        }
        function text(str, x, y, color) {
            ctx.fillStyle = color || "#666"
            ctx.font = "10px sans-serif"
            ctx.fillText(str, x, y)
        }
        function xAt(frac) { return x0 + (x1 - x0) * frac }
        function yAt(frac) { return y0 - (y0 - y1) * frac }
        function curve(fn, steps) {
            var pts = []
            steps = steps || 60
            for (var i = 0; i <= steps; i++) {
                var u = i / steps
                pts.push([xAt(u), yAt(fn(u))])
            }
            return pts
        }

        switch (kind) {
        case "step": {
            // Speed step response: the target is a dashed line, the response
            // curve shows the overshoot/rise the gain buys.
            var role = param.help.role || "kp"
            line([[x0, yAt(0.1)], [x0 + 6, yAt(0.1)], [x0 + 6, yAt(0.75)], [x1, yAt(0.75)]], grey, 1, true)
            var overshoot = 0, rise = 0.35
            if (role === "kp") { overshoot = 0.15 + 0.55 * t; rise = 0.42 - 0.25 * t }
            else if (role === "ki") { overshoot = 0.05 + 0.35 * t; rise = 0.55 - 0.1 * t }
            else { overshoot = 0.5 * (1 - t); rise = 0.3 }
            var f = function(u) {
                if (u < 0.06) return 0.1
                var s = (u - 0.06) / 0.94
                var body = 0.1 + 0.65 * (1 - Math.exp(-s / rise))
                var ring = overshoot * 0.4 * Math.exp(-s * 3.2) * Math.sin(s * 22)
                return Math.min(0.98, body + ring)
            }
            line(curve(f), fg, 2)
            text(role === "kp" ? Tr.t("help_overshoot")
                : role === "ki" ? Tr.t("help_integral")
                : Tr.t("help_derivative"), x0 + 6, y1 + 2)
            break
        }
        case "smoothing": {
            // Raw Hall-rate samples vs the rate the loop closes over.
            var noise = []
            for (var i = 0; i <= 70; i++) {
                var u = i / 70
                noise.push([xAt(u), yAt(0.5 + 0.22 * Math.sin(u * 42) + 0.1 * Math.sin(u * 91))])
            }
            line(noise, grey, 1)
            var tau = 0.02 + 0.5 * t
            line(curve(function(u) { return 0.5 + 0.22 * Math.sin((u - tau) * 42) }), fg, 2)
            text(Tr.tf("help_filtered", (t * 100).toFixed(0)), x0 + 6, y1 + 2)
            break
        }
        case "slope": {
            // Command step (dashed) vs the ramp the parameter imposes.
            line([[x0, y0], [x0 + 8, y0], [x0 + 8, yAt(0.8)], [x1, yAt(0.8)]], grey, 1, true)
            var slope = 0.25 + 0.7 * (1 - t)
            line(curve(function(u) { return Math.min(0.8, Math.max(0, (u * slope) / 0.6)) }), fg, 2)
            text(Tr.t("help_steeper"), x0 + 6, y1 + 2)
            break
        }
        case "level": {
            // A current burst against the trip level the slider sets.
            var level = 0.25 + 0.6 * t
            var burst = curve(function(u) {
                return 0.12 + 0.72 * Math.exp(-Math.pow((u - 0.45) / 0.16, 2))
            })
            line(burst, grey, 1.5)
            line([[x0, yAt(level)], [x1, yAt(level)]], hot, 2, true)
            text(Tr.tf("help_trip_level", level.toFixed(2)), x0 + 6, y1 + 2)
            break
        }
        case "window": {
            // How long the burst must stay above the level.
            var trip = 0.62
            var band = 0.08 + 0.5 * t
            ctx.fillStyle = "rgba(231,76,60,0.15)"
            ctx.fillRect(xAt(0.45 - band / 2), yAt(0.95), (x1 - x0) * band, yAt(0.1) - yAt(0.95))
            line(curve(function(u) {
                return 0.12 + 0.72 * Math.exp(-Math.pow((u - 0.45) / 0.16, 2))
            }), grey, 1.5)
            line([[x0, yAt(trip)], [x1, yAt(trip)]], hot, 2, true)
            text(Tr.tf("help_window", (t * 100).toFixed(0)), x0 + 6, y1 + 2)
            break
        }
        case "clamp": {
            // Demand rising past the ceiling the parameter sets.
            var ceil = 0.2 + 0.7 * t
            line(curve(function(u) { return u < 0.55 ? u * 1.25 : Math.min(ceil, u * 1.25) }), fg, 2)
            line([[x0, yAt(ceil)], [x1, yAt(ceil)]], hot, 2, true)
            text(Tr.tf("help_ceiling", ceil.toFixed(2)), x0 + 6, y1 + 2)
            break
        }
        case "ceiling": {
            // Allowed torque envelope: driving is free, opposing motion is capped.
            var mid = yAt(0.5)
            ctx.strokeStyle = "#e2e2e2"
            ctx.beginPath(); ctx.moveTo(x0, mid); ctx.lineTo(x1, mid); ctx.stroke()
            var cap = 0.08 + 0.4 * t
            ctx.fillStyle = "rgba(44,127,184,0.35)"
            ctx.fillRect(x0 + 12, yAt(0.92), 34, yAt(0.08) - yAt(0.92))
            ctx.fillStyle = "rgba(231,76,60,0.45)"
            ctx.fillRect(x0 + 60, mid, 34, (y0 - mid) * cap / 0.5)
            text(Tr.t("help_driving"), x0 + 12, y0 - 2)
            text(Tr.tf("help_braking", (t * 100).toFixed(0)), x0 + 60, y0 - 2)
            break
        }
        case "speed_threshold": {
            // A wheel coasting down into the stall zone under the threshold.
            var thr = 0.12 + 0.6 * t
            ctx.fillStyle = "rgba(231,76,60,0.12)"
            ctx.fillRect(x0, yAt(thr), x1 - x0, y0 - yAt(thr))
            line(curve(function(u) { return 0.92 * Math.exp(-u * 2.6) }), fg, 2)
            line([[x0, yAt(thr)], [x1, yAt(thr)]], hot, 2, true)
            text(Tr.t("help_stall_zone"), x0 + 6, y1 + 2)
            break
        }
        case "scale": {
            // Normalized -1..1 mapped onto the full-scale speed.
            var mid2 = yAt(0.45)
            ctx.strokeStyle = grey
            ctx.beginPath(); ctx.moveTo(x0, mid2); ctx.lineTo(x1, mid2); ctx.stroke()
            var ticks = [-1, -0.5, 0, 0.5, 1]
            for (var k = 0; k < ticks.length; k++) {
                var xk = xAt((ticks[k] + 1) / 2)
                ctx.beginPath(); ctx.moveTo(xk, mid2 - 5); ctx.lineTo(xk, mid2 + 5); ctx.stroke()
                text(ticks[k].toFixed(1), xk - 8, mid2 + 18)
            }
            var full = param && param.key === "max_speed" ? root.value : 0
            text(Tr.tf("help_full_pos", full.toFixed(3)), x0 + 6, y1 + 2)
            text(Tr.tf("help_full_neg", (-full).toFixed(3)), x1 - 96, y1 + 2)
            break
        }
        case "geometry": {
            // Wheel geometry: the diameter is the m/s <-> rad/s conversion.
            var r = 12 + 26 * t
            var cx = x0 + 46, cy = yAt(0.45)
            ctx.strokeStyle = fg
            ctx.lineWidth = 2
            ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.stroke()
            ctx.beginPath(); ctx.moveTo(cx - r, cy); ctx.lineTo(cx + r, cy); ctx.stroke()
            var d = root.value
            text(Tr.tf("help_diameter", d.toFixed(3)), x0 + 92, cy - 6)
            text(Tr.tf("help_circumference", (Math.PI * d).toFixed(3)), x0 + 92, cy + 10)
            break
        }
        default:
            text(Tr.tf("help_no_sketch", kind), x0 + 6, y1 + 2)
        }
    }
}
