import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts

// Costmap visualization + manual positioning + click-to-plan.
// Reinterprets CostmapServer's raw buffer (GridHeader + cells, see
// nav_common.hpp) straight from the message bytes — no repacking anywhere.
//
// A plain left click sets a planner target while retaining the robot heading;
// Shift+left-click repositions the robot. Left press-drag-release sets a target
// at the press point facing along the drag. Right click drops an obstacle.
// Small ticks show path headings; the orange arrow is the next local target.

Item {
    id: root

    property int gridW: 0
    property int gridH: 0
    property real gridRes: 0.02
    property real gridOriginX: 0
    property real gridOriginY: 0
    property var model: null  // radapter.model.branch("nav") — set by Main.qml
    property var cells: null     // Uint8Array over the costmap buffer
    property var path: []
    property var scan: []  // lidar hit points [{x,y}...] in world meters
    property var robot: ({ x: 0, y: 0, theta: 0 })
    property var target: null
    property var localTarget: null
    property var status: ({})
    property var cmdVel: ({ x: 0, y: 0, theta: 0 })
    property bool discoveryEnabled: true
    property string mapIoStatus: ""

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
            var cellCount = root.gridW * root.gridH
            if (dv.byteLength !== 24 + cellCount) {
                console.warn("costmap: invalid buffer size")
                return
            }
            root.gridOriginX = dv.getFloat32(16, true)
            root.gridOriginY = dv.getFloat32(20, true)
            root.cells = new Uint8Array(msg.costmap, 24, cellCount)
            canvas.requestPaint()
        }
        if (msg.path !== undefined) {
            root.path = msg.path
            if (msg.path.length === 0)
                root.localTarget = null
            canvas.requestPaint()
        }
        if (msg.local_target !== undefined) {
            root.localTarget = msg.local_target
            canvas.requestPaint()
        }
        if (msg.scan !== undefined) {
            root.scan = msg.scan.points || []
            canvas.requestPaint()
        }
        if (msg.position !== undefined) {
            root.robot = msg.position
            canvas.requestPaint()
        }
        if (msg.status !== undefined)
            root.status = msg.status
        if (msg.cmd_vel !== undefined) {
            root.cmdVel = msg.cmd_vel
        }
        if (msg.discovery_enabled !== undefined)
            root.discoveryEnabled = msg.discovery_enabled
        if (msg.map_io_status !== undefined)
            root.mapIoStatus = msg.map_io_status
    }
    Component.onCompleted: {
        model.received.connect(onMsg)
    }

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

            property real zoom: 1.0
            property real panX: 0
            property real panY: 0
            property real baseCellPx: root.gridW > 0
                ? Math.min(width / root.gridW, height / root.gridH) : 1
            property real cellPx: baseCellPx * zoom
            property real mapOffsetX: (width - root.gridW * cellPx) / 2 + panX
            property real mapOffsetY: (height - root.gridH * cellPx) / 2 + panY

            function resetView() {
                zoom = 1
                panX = 0
                panY = 0
                requestPaint()
            }

            function toScreenX(mx) {
                return mapOffsetX + (mx - root.gridOriginX) / root.gridRes * cellPx
            }
            function toScreenY(my) {
                return mapOffsetY + root.gridH * cellPx
                    - (my - root.gridOriginY) / root.gridRes * cellPx
            }
            function toMetersX(sx) {
                return root.gridOriginX + (sx - mapOffsetX) / cellPx * root.gridRes
            }
            function toMetersY(sy) {
                return root.gridOriginY
                    + (mapOffsetY + root.gridH * cellPx - sy) / cellPx * root.gridRes
            }

            // heading tick/arrow at screen point (sx, sy); theta is world-frame
            // (y up), screen y grows down, hence the minus on sin
            function drawHeading(ctx, sx, sy, theta, len, width, color, arrowhead) {
                var ex = sx + len * Math.cos(theta)
                var ey = sy - len * Math.sin(theta)
                ctx.strokeStyle = color
                ctx.lineWidth = width
                ctx.beginPath()
                ctx.moveTo(sx, sy)
                ctx.lineTo(ex, ey)
                if (arrowhead) {
                    var hl = Math.max(4, len * 0.35)
                    var a1 = theta + Math.PI * 0.8
                    var a2 = theta - Math.PI * 0.8
                    ctx.moveTo(ex, ey)
                    ctx.lineTo(ex + hl * Math.cos(a1), ey - hl * Math.sin(a1))
                    ctx.moveTo(ex, ey)
                    ctx.lineTo(ex + hl * Math.cos(a2), ey - hl * Math.sin(a2))
                }
                ctx.stroke()
            }

            onPaint: {
                var ctx = getContext("2d")
                ctx.reset()
                if (!root.cells || root.gridW <= 0) return

                var w = root.gridW, h = root.gridH, px = cellPx

                ctx.fillStyle = '#404449'
                ctx.fillRect(mapOffsetX, mapOffsetY, w * px, h * px)

                // cells: cost 0 transparent, 1..99 yellow->red, 100 solid red
                for (var y = 0; y < h; ++y) {
                    var sy = mapOffsetY + (h - 1 - y) * px
                    var row = y * w
                    for (var x = 0; x < w; ++x) {
                        var cost = root.cells[row + x]
                        if (!cost) continue
                        ctx.fillStyle = cost === 255 ? "#25282b"
                            : cost >= 100 ? "#e06c75"
                            : Qt.rgba(0.9, 0.75 - 0.5 * cost / 100, 0.2, 0.25 + 0.6 * cost / 100)
                        ctx.fillRect(mapOffsetX + x * px, sy, px + 0.5, px + 0.5)
                    }
                }

                // lidar scan: raw world-frame hit points
                if (root.scan.length) {
                    ctx.fillStyle = "#2b90d9"
                    for (var si = 0; si < root.scan.length; ++si) {
                        var sp = root.scan[si]
                        ctx.beginPath()
                        ctx.arc(toScreenX(sp.x), toScreenY(sp.y), 1.5, 0, 2 * Math.PI)
                        ctx.fill()
                    }
                }

                // global path + a small heading tick on every point
                if (root.path.length > 1) {
                    ctx.strokeStyle = "#61afef"
                    ctx.lineWidth = 2
                    ctx.beginPath()
                    ctx.moveTo(toScreenX(root.path[0].x), toScreenY(root.path[0].y))
                    for (var i = 1; i < root.path.length; ++i)
                        ctx.lineTo(toScreenX(root.path[i].x), toScreenY(root.path[i].y))
                    ctx.stroke()
                    for (i = 0; i < root.path.length; ++i) {
                        var p = root.path[i]
                        drawHeading(ctx, toScreenX(p.x), toScreenY(p.y),
                                    p.theta || 0, 7, 1, "#1a6daa", false)
                    }
                }

                // local planner's chosen lookahead point: big arrow from the
                // robot base to the point, arrowhead at the point
                if (root.localTarget) {
                    var lx = toScreenX(root.localTarget.x)
                    var ly = toScreenY(root.localTarget.y)
                    var bx = toScreenX(root.robot.x)
                    var by = toScreenY(root.robot.y)
                    var ang = Math.atan2(ly - by, lx - bx)
                    var hl = 9
                    ctx.strokeStyle = "#d19a66"
                    ctx.lineWidth = 3
                    ctx.beginPath()
                    ctx.moveTo(bx, by)
                    ctx.lineTo(lx, ly)
                    ctx.moveTo(lx, ly)
                    ctx.lineTo(lx + hl * Math.cos(ang + Math.PI * 0.8),
                               ly + hl * Math.sin(ang + Math.PI * 0.8))
                    ctx.moveTo(lx, ly)
                    ctx.lineTo(lx + hl * Math.cos(ang - Math.PI * 0.8),
                               ly + hl * Math.sin(ang - Math.PI * 0.8))
                    ctx.stroke()
                    ctx.fillStyle = "#d19a66"
                    ctx.beginPath()
                    ctx.arc(lx, ly, 4, 0, 2 * Math.PI)
                    ctx.fill()
                }

                // target: cross + commanded heading
                if (root.target) {
                    var tx = toScreenX(root.target.x), ty = toScreenY(root.target.y)
                    ctx.strokeStyle = '#3d8025'
                    ctx.lineWidth = 2
                    ctx.beginPath()
                    ctx.moveTo(tx - 7, ty); ctx.lineTo(tx + 7, ty)
                    ctx.moveTo(tx, ty - 7); ctx.lineTo(tx, ty + 7)
                    ctx.stroke()
                    if (root.target.theta !== undefined)
                        drawHeading(ctx, tx, ty, root.target.theta, 16, 2, "#3d8025", true)
                }

                // drag preview: arrow from press point along the drag
                if (mouseArea.pressPos && mouseArea.dragPos
                        && mouseArea.pressPos.button === Qt.LeftButton) {
                    var sx0 = mouseArea.pressPos.x, sy0 = mouseArea.pressPos.y
                    var ang = Math.atan2(-(mouseArea.dragPos.y - sy0),
                                         mouseArea.dragPos.x - sx0)
                    var dlen = Math.hypot(mouseArea.dragPos.x - sx0,
                                          mouseArea.dragPos.y - sy0)
                    drawHeading(ctx, sx0, sy0, ang, Math.max(dlen, 10), 2,
                                dlen >= mouseArea.dragThresholdPx ? "#3d8025" : "#5c6370",
                                true)
                }

                // robot: circle + heading
                var rx = toScreenX(root.robot.x), ry = toScreenY(root.robot.y)
                ctx.fillStyle = "#c678dd"
                ctx.beginPath()
                ctx.arc(rx, ry, 6, 0, 2 * Math.PI)
                ctx.fill()
                drawHeading(ctx, rx, ry, root.robot.theta, 12, 2, "#333333", false)
            }

            MouseArea {
                id: mouseArea
                anchors.fill: parent
                acceptedButtons: Qt.LeftButton | Qt.MiddleButton | Qt.RightButton
                cursorShape: (pressedButtons & Qt.MiddleButton) !== 0
                    ? Qt.ClosedHandCursor : Qt.ArrowCursor

                // Shift+click adjusts the robot pose. A plain click sets a
                // target; drag past the threshold to command its final heading.
                readonly property real dragThresholdPx: 10
                property var pressPos: null // { x, y, button, modifiers }
                property var dragPos: null

                onPressed: function (mouse) {
                    pressPos = {
                        x: mouse.x, y: mouse.y, button: mouse.button,
                        modifiers: mouse.modifiers,
                    }
                    dragPos = mouse.button === Qt.MiddleButton
                        ? { x: mouse.x, y: mouse.y } : null
                }
                onPositionChanged: function (mouse) {
                    if (!pressPos) return
                    if (pressPos.button === Qt.MiddleButton) {
                        canvas.panX += mouse.x - dragPos.x
                        canvas.panY += mouse.y - dragPos.y
                        dragPos = { x: mouse.x, y: mouse.y }
                        canvas.requestPaint()
                        return
                    }
                    dragPos = { x: mouse.x, y: mouse.y }
                    canvas.requestPaint()
                }
                onWheel: function (wheel) {
                    if (root.gridW <= 0) return
                    var worldX = canvas.toMetersX(wheel.x)
                    var worldY = canvas.toMetersY(wheel.y)
                    var factor = Math.pow(1.0015, wheel.angleDelta.y)
                    canvas.zoom = Math.max(0.2, Math.min(20, canvas.zoom * factor))
                    canvas.panX += wheel.x - canvas.toScreenX(worldX)
                    canvas.panY += wheel.y - canvas.toScreenY(worldY)
                    canvas.requestPaint()
                    wheel.accepted = true
                }
                onReleased: function (mouse) {
                    var press = pressPos
                    pressPos = null
                    dragPos = null
                    if (!press || root.gridW <= 0) return
                    if (press.button === Qt.MiddleButton) return
                    var mx = canvas.toMetersX(press.x)
                    var my = canvas.toMetersY(press.y)
                    if (press.button === Qt.RightButton) {
                        model.send({ obstacle: { x: mx, y: my } })
                    } else {
                        var dx = mouse.x - press.x
                        var dy = mouse.y - press.y
                        var dragged = Math.hypot(dx, dy) >= dragThresholdPx
                        var reposition = !dragged
                            && (press.modifiers & Qt.ShiftModifier) !== 0
                        if (reposition) {
                            root.target = null
                            root.robot = { x: mx, y: my, theta: root.robot.theta }
                            model.send({ reposition: root.robot })
                        } else {
                            var theta = dragged
                                ? Math.atan2(-dy, dx) // screen y down -> world y up
                                : root.robot.theta
                            root.target = { x: mx, y: my, theta: theta }
                            model.send({ target: root.target })
                        }
                    }
                    canvas.requestPaint()
                }
                onCanceled: {
                    pressPos = null
                    dragPos = null
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
                          root.gridRes.toFixed(3) + " m"; color: "#333333" }
            Label { text: "Origin: " + root.gridOriginX.toFixed(2) + ", " +
                          root.gridOriginY.toFixed(2); color: "#333333" }
            Label { text: "Robot: " + root.robot.x.toFixed(2) + ", " +
                          root.robot.y.toFixed(2) + ", " +
                          root.robot.theta.toFixed(2); color: "#333333" }
            Label { text: "Path points: " + root.path.length; color: "#333333" }
            Label { text: "Scan hits: " + root.scan.length; color: "#2b90d9" }
            Label {
                objectName: "statusLabel"
                color: "#333333"
                text: {
                    var s = root.status
                    if (s.reached === undefined) return "Status: —"
                    if (s.reached && s.rotated) return "Status: done"
                    if (s.reached) return "Status: rotating"
                    if (s.is_stuck) return "Status: stuck"
                    return "Status: driving"
                }
            }
            Label {
                objectName: "cmdVelLabel"
                color: "#333333"
                text: "cmd_vel  fwd:" + root.cmdVel.x.toFixed(2)
                      + "  lat:" + root.cmdVel.y.toFixed(2)
                      + "  rot:" + root.cmdVel.theta.toFixed(2)
            }
            Button {
                objectName: "cancelBtn"
                text: "Cancel target"
                onClicked: {
                    root.target = null
                    model.send({ cancel: true })
                    canvas.requestPaint()
                }
            }
            CheckBox {
                objectName: "discoveryCheck"
                text: "Map discovery"
                checked: root.discoveryEnabled
                onClicked: model.send({ discovery_enabled: checked })
            }
            RowLayout {
                Layout.fillWidth: true
                Button {
                    objectName: "loadMapBtn"
                    text: "Load map"
                    Layout.fillWidth: true
                    onClicked: loadMapDialog.open()
                }
                Button {
                    objectName: "dumpMapBtn"
                    text: "Dump map"
                    Layout.fillWidth: true
                    onClicked: dumpMapDialog.open()
                }
            }
            Button {
                objectName: "resetViewBtn"
                text: "Reset view"
                Layout.fillWidth: true
                onClicked: canvas.resetView()
            }
            Label {
                visible: root.mapIoStatus.length > 0
                Layout.fillWidth: true
                wrapMode: Text.WordWrap
                color: "#555555"
                text: root.mapIoStatus
            }
            Label {
                Layout.fillWidth: true
                wrapMode: Text.WordWrap
                color: "#555555"
                text: "LMB click — set target\nLMB drag — set target + heading\n" +
                      "Shift+LMB — reposition robot\n" +
                      "Wheel — zoom\nMMB drag — pan\n" +
                      "RMB — drop an obstacle\n(sim: lidar discovers it; else a\ntemporary costmap point)\n\n" +
                      "Blue dots: lidar scan hits\nTicks: path point theta\nOrange arrow: local planner's pick"
            }
            Item { Layout.fillHeight: true }
        }
    }

    FileDialog {
        id: loadMapDialog
        title: "Load occupancy map"
        fileMode: FileDialog.OpenFile
        nameFilters: ["PNG map images (*.png)", "Images (*.png *.jpg *.jpeg *.bmp)"]
        onAccepted: model.send({ load_map_file: selectedFile.toString() })
    }

    FileDialog {
        id: dumpMapDialog
        title: "Dump occupancy map"
        fileMode: FileDialog.SaveFile
        defaultSuffix: "png"
        nameFilters: ["PNG map images (*.png)"]
        onAccepted: model.send({ dump_map_file: selectedFile.toString() })
    }
}
