pragma Singleton
import QtQuick 2.3

// A typed number, whichever decimal separator the operator's keyboard emits.
// JavaScript's parseFloat is locale-blind and stops at a comma, so "12,5" would
// silently become 12. The Qt Quick numeric validators have the opposite fault:
// they follow LC_NUMERIC, call a period decimal Invalid, and for an Invalid text
// never emit editingFinished — the edit is then lost with no message at all.
QtObject {
    function of(text) {
        return parseFloat(String(text).replace(",", "."))
    }
}
