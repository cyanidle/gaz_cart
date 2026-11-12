import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "Tr"

// Language tumbler: one per window, all of them driven by the shared Tr
// singleton, so throwing it in any window re-labels every other one.
//
// The chrome text switches right here, in QML. Text the *models* carry
// (parameter names, units, hover help, status lines) is built in Lua, so the
// throw is also sent down the view's model — `{lang = "en"}` — and the owner
// re-labels what it pushes. The branches of each window wrap it
// (`{odo = {lang = …}}`), which is why the Lua side looks in every namespace.
RowLayout {
    id: root
    spacing: 6

    /// The view's model, so the throw reaches the Lua that owns it.
    property var model

    Label {
        objectName: "langLabel"
        text: Tr.t("lang_name")
        color: "#666"
        font.pixelSize: 11
    }

    Switch {
        id: switcher
        objectName: "langSwitch"
        text: Tr.lang === "ru" ? Tr.t("lang_ru") : Tr.t("lang_en")
        checked: Tr.lang === "ru"
        // The click set `checked` itself, which breaks the binding above: put it
        // back so a switch thrown in another window moves this one too.
        onToggled: {
            Tr.lang = checked ? "ru" : "en"
            checked = Qt.binding(function() { return Tr.lang === "ru" })
            if (root.model) root.model.send({ lang: Tr.lang })
        }
    }
}
