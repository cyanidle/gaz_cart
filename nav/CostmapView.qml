import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

// Costmap visualization + click-to-plan.
// Reinterprets CostmapServer's raw buffer (GridHeader + cells, see
// nav_common.hpp) straight from the message bytes — no repacking anywhere.

ApplicationWindow {
    id: root
    visible: true
    width: 760
    height: 820
    title: "gaz_cart nav — LMB: target, RMB: obstacle"
    color: "#1e2127"

    property int gridW: 0
    property int gridH: 0
    property real gridRes: 0.02
    property var cells: null // Uint8Array over the costmap buffer
    property var path: []
    property var robot: ({ x: 0, y: 0, theta: 0 })
    property var target: null
    property var status: ({})

    function onMsg(msg) {
        if (msg.costmap !== undefined) {
            var dv = new DataView(msg.costmap)
            if (dv.getInt32(0, true) !== 0x47414D50) {
                console.warn("costmap: bad magic")
                return
            }
            root.gridW = dv.getInt32(4, true)
            root.gridH = dv.getInt32(8, true)
            root.gridRes = dv.getFloat32(12, true)
            root.cells = new Uint8Array(msg.costmap, 16)
            canvas.requestPaint()
        }
        if (msg.path !== undefined) {
            root.path = msg.path
            canvas.requestPaint()
        }
        if (msg.position !== undefined) {
            root.robot = msg.position
            canvas.requestPaint()
        }
        if (msg.status !== undefined)
            root.status = msg.status
    }
    Component.onCompleted: radapter.model.received.connect(onMsg)

    Item {
        anchors.fill: parent
        anchors.margins: 8

        Canvas {
            id: canvas
            objectName: "map"
            anchors.left: parent.left
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.right: sidebar.left
            anchors.rightMargin: 8

            property int clicks: 0
            property real cellPx: root.gridW > 0
                ? Math.min(width / root.gridW, height / root.gridH) : 1

            function toScreenX(mx) { return mx / root.gridRes * cellPx }
            function toScreenY(my) { return root.gridH * cellPx - my / root.gridRes * cellPx }
            function toMetersX(sx) { return sx / cellPx * root.gridRes }
            function toMetersY(sy) { return (root.gridH * cellPx - sy) / cellPx * root.gridRes }

            onPaint: {
                var ctx = getContext("2d")
                ctx.reset()
                if (!root.cells || root.gridW <= 0) return

                var w = root.gridW, h = root.gridH, px = cellPx

                ctx.fillStyle = "#282c34"
                ctx.fillRect(0, 0, w * px, h * px)

                // cells: cost 0 transparent, 1..99 yellow->red, 100 solid red
                for (var y = 0; y < h; ++y) {
                    var sy = (h - 1 - y) * px
                    var row = y * w
                    for (var x = 0; x < w; ++x) {
                        var cost = root.cells[row + x]
                        if (!cost) continue
                        ctx.fillStyle = cost >= 100 ? "#e06c75"
                            : Qt.rgba(0.9, 0.75 - 0.5 * cost / 100, 0.2, 0.25 + 0.6 * cost / 100)
                        ctx.fillRect(x * px, sy, px + 0.5, px + 0.5)
                    }
                }

                // global path
                if (root.path.length > 1) {
                    ctx.strokeStyle = "#61afef"
                    ctx.lineWidth = 2
                    ctx.beginPath()
                    ctx.moveTo(toScreenX(root.path[0].x), toScreenY(root.path[0].y))
                    for (var i = 1; i < root.path.length; ++i)
                        ctx.lineTo(toScreenX(root.path[i].x), toScreenY(root.path[i].y))
                    ctx.stroke()
                }

                // target
                if (root.target) {
                    var tx = toScreenX(root.target.x), ty = toScreenY(root.target.y)
                    ctx.strokeStyle = "#98c379"
                    ctx.lineWidth = 2
                    ctx.beginPath()
                    ctx.moveTo(tx - 7, ty); ctx.lineTo(tx + 7, ty)
                    ctx.moveTo(tx, ty - 7); ctx.lineTo(tx, ty + 7)
                    ctx.stroke()
                }

                // robot: circle + heading
                var rx = toScreenX(root.robot.x), ry = toScreenY(root.robot.y)
                ctx.fillStyle = "#c678dd"
                ctx.beginPath()
                ctx.arc(rx, ry, 6, 0, 2 * Math.PI)
                ctx.fill()
                ctx.strokeStyle = "#ffffff"
                ctx.lineWidth = 2
                ctx.beginPath()
                ctx.moveTo(rx, ry)
                ctx.lineTo(rx + 12 * Math.cos(root.robot.theta),
                           ry - 12 * Math.sin(root.robot.theta))
                ctx.stroke()
            }

            MouseArea {
                anchors.fill: parent
                acceptedButtons: Qt.LeftButton | Qt.RightButton
                onClicked: function (mouse) {
                    canvas.clicks++
                    if (root.gridW <= 0) return
                    var mx = canvas.toMetersX(mouse.x)
                    var my = canvas.toMetersY(mouse.y)
                    if (mouse.button === Qt.LeftButton) {
                        root.target = { x: mx, y: my }
                        radapter.model.send({ target: { x: mx, y: my, theta: root.robot.theta } })
                    } else {
                        radapter.model.send({ obstacle: { x: mx, y: my } })
                    }
                    canvas.requestPaint()
                }
            }
        }

        ColumnLayout {
            id: sidebar
            width: 200
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            spacing: 6

            Label { text: "Costmap: " + root.gridW + "×" + root.gridH + " @ " +
                          root.gridRes.toFixed(3) + " m"; color: "#abb2bf" }
            Label { text: "Robot: " + root.robot.x.toFixed(2) + ", " +
                          root.robot.y.toFixed(2) + ", " +
                          root.robot.theta.toFixed(2); color: "#abb2bf" }
            Label { text: "Path points: " + root.path.length; color: "#abb2bf" }
            Label {
                objectName: "statusLabel"
                color: "#abb2bf"
                text: {
                    var s = root.status
                    if (s.reached === undefined) return "Status: —"
                    if (s.reached && s.rotated) return "Status: done"
                    if (s.reached) return "Status: rotating"
                    if (s.is_stuck) return "Status: stuck"
                    return "Status: driving"
                }
            }
            Button {
                objectName: "cancelBtn"
                text: "Cancel target"
                onClicked: {
                    root.target = null
                    radapter.model.send({ cancel: true })
                    canvas.requestPaint()
                }
            }
            Label {
                Layout.fillWidth: true
                wrapMode: Text.WordWrap
                color: "#5c6370"
                text: "Left click — drive there\nRight click — drop an obstacle\n" +
                      "(obstacles expire after keep_points_ms)"
            }
            Item { Layout.fillHeight: true }
        }
    }
}
