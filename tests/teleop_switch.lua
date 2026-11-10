-- Self-checking test of the operator-source switch (mods/teleop_switch.lua):
--   build/bin/radapter tests/teleop_switch.lua
-- Exit status 0 on success; a failed assertion aborts with the reason.

package.path = SCRIPT_DIR .. "/../?.lua;" .. package.path
local TeleopSwitch = require "mods.teleop_switch"

do -- auto hands the input to the keyboard until the stick reports
    local changes = {}
    local s = TeleopSwitch.new {
        has_joystick = true,
        on_change = function(source, previous) changes[#changes + 1] = { previous, source } end,
    }
    assert(s:active() == "keyboard", "an idle stick must leave the keyboard in charge")
    assert(s:allows("keyboard") and not s:allows("joystick"))

    assert(s:joystick_reporting(true), "the first report is a change")
    assert(s:active() == "joystick", "a reporting stick owns the operator input")
    assert(s:allows("joystick") and not s:allows("keyboard"))

    assert(not s:joystick_reporting(true), "a repeated report is not a change")
    assert(#changes == 1, "only the actual switch may notify")

    assert(s:joystick_reporting(false), "losing the stick is a change")
    assert(s:active() == "keyboard", "an unplugged stick hands the cart back to the keyboard")
    assert(changes[#changes][1] == "joystick" and changes[#changes][2] == "keyboard")
end

do -- auto without a joystick stays on the keyboard forever
    local s = TeleopSwitch.new { has_joystick = false }
    assert(s:active() == "keyboard")
    assert(not s:joystick_reporting(true), "nothing to switch to")
    assert(s:active() == "keyboard")
end

do -- forced joystick: the keyboard never drives, and silence parks the cart
    local s = TeleopSwitch.new { mode = "joystick", has_joystick = true }
    assert(s:active() == "none", "a silent stick drives nothing in forced mode")
    assert(not s:allows("joystick") and not s:allows("keyboard"))
    s:joystick_reporting(true)
    assert(s:active() == "joystick")
    assert(s:allows("joystick") and not s:allows("keyboard"))
    s:joystick_reporting(false)
    assert(s:active() == "none", "a forced joystick that stops reporting parks the cart")
end

do -- forced keyboard ignores the stick even while it reports
    local s = TeleopSwitch.new { mode = "keyboard", has_joystick = true }
    assert(s:active() == "keyboard")
    assert(not s:joystick_reporting(true), "the stick cannot take over in keyboard mode")
    assert(s:allows("keyboard") and not s:allows("joystick"))
end

do -- an unknown mode is a configuration error, not a silent fallback
    local ok, err = pcall(TeleopSwitch.new, { mode = "wasd" })
    assert(not ok and tostring(err):find("auto/joystick/keyboard"), tostring(err))
end

print("teleop_switch: OK")
shutdown()
