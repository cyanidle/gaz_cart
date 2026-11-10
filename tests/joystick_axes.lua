-- Self-checking test of the joystick axis shaping and stop organ (mods/joystick_axes.lua):
--   build/bin/radapter tests/joystick_axes.lua
-- Exit status 0 on success; a failed assertion aborts with the reason.

package.path = SCRIPT_DIR .. "/../?.lua;" .. package.path
local JoystickAxes = require "mods.joystick_axes"

local function close(a, b, what)
    assert(a ~= nil, what .. ": got nil")
    assert(math.abs(a - b) < 1e-9, ("%s: expected %s, got %s"):format(what, b, a))
end

do -- the deadzone swallows the centre and the rest is rescaled to full deflection
    close(JoystickAxes.value({ y = 0.0 }, "y", false, 0.05), 0.0, "centre")
    close(JoystickAxes.value({ y = 0.05 }, "y", false, 0.05), 0.0, "deadzone edge")
    close(JoystickAxes.value({ y = -0.05 }, "y", false, 0.05), 0.0, "negative deadzone edge")
    close(JoystickAxes.value({ y = 1.0 }, "y", false, 0.05), 1.0, "full deflection")
    close(JoystickAxes.value({ y = -1.0 }, "y", false, 0.05), -1.0, "full reverse")
    close(JoystickAxes.value({ y = 0.5 }, "y", false, 0.05), 0.45 / 0.95, "half deflection")
    close(JoystickAxes.value({ y = -0.5 }, "y", false, 0.05), -0.45 / 0.95, "half reverse")
end

do -- no deadzone default and axis inversion
    close(JoystickAxes.value({ y = 0.25 }, "y"), 0.2 / 0.95, "default deadzone")
    close(JoystickAxes.value({ y = 0.5 }, "y", true, 0.05), -0.45 / 0.95, "inverted")
    close(JoystickAxes.value({ y = -0.5 }, "y", true, 0.05), 0.45 / 0.95, "inverted reverse")
end

do -- a report without the axis is not a zero reading
    assert(JoystickAxes.value({ x = 0.5 }, "y", false, 0.05) == nil, "absent axis must be nil")
end

do -- stop organ: thresholds with hysteresis, an absent axis and a disabled organ are inconclusive
    local axes = { slider = -1.0 }
    assert(JoystickAxes.stop_switch(axes, "slider", -0.8, -0.5) == true, "low extreme engages")
    axes.slider = -0.8
    assert(JoystickAxes.stop_switch(axes, "slider", -0.8, -0.5) == true, "stop_below engages")
    axes.slider = -0.65
    assert(JoystickAxes.stop_switch(axes, "slider", -0.8, -0.5) == nil, "the band is inconclusive")
    axes.slider = -0.5
    assert(JoystickAxes.stop_switch(axes, "slider", -0.8, -0.5) == false, "resume_above releases")
    axes.slider = 1.0
    assert(JoystickAxes.stop_switch(axes, "slider", -0.8, -0.5) == false, "high extreme releases")

    assert(JoystickAxes.stop_switch({ x = -1.0 }, "slider", -0.8, -0.5) == nil, "absent axis")
    assert(JoystickAxes.stop_switch(axes, "", -0.8, -0.5) == nil, "disabled organ")
end

do -- misconfiguration fails loudly instead of flapping
    local ok, err = pcall(JoystickAxes.value, { y = 0.5 }, "y", false, 1.0)
    assert(not ok and tostring(err):find("deadzone"), "a deadzone of 1 must be rejected")
    ok, err = pcall(JoystickAxes.stop_switch, { slider = -1.0 }, "slider", -0.5, -0.8)
    assert(not ok and tostring(err):find("thresholds"), "reversed thresholds must be rejected")
end

print("joystick_axes: OK")
shutdown()
