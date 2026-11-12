import QtQuick
import QtQuick.Controls

// The tuner window — one view over both transports (mods/tuner.lua decides
// which one): the cart's service channel over CAN, or a module on a serial bench
// channel. gui.lua and debug/tune.lua both open this, so there is one tuner.
ApplicationWindow {
    id: root
    visible: true
    width: 1180; height: 980
    title: "gaz_cart — tune"
    color: "#f0f0f0"

    Tuner {
        anchors.fill: parent
        model: radapter.model
    }
}
