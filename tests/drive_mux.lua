-- Self-checking test of the drive-source arbiter (mods/drive_mux.lua):
--   build/bin/radapter tests/drive_mux.lua
-- Exit status 0 on success; a failed assertion aborts with the reason.

package.path = SCRIPT_DIR .. "/../?.lua;" .. package.path
local DriveMux = require "mods.drive_mux"

local applied = {}
local states = {}

local function make()
    applied, states = {}, {}
    return DriveMux.new {
        apply = function(v, omega) applied[#applied + 1] = { v, omega } end,
        on_state = function(state) states[#states + 1] = state end,
    }
end

local function last_applied()
    local entry = applied[#applied]
    return entry and entry[1], entry and entry[2]
end

do -- a source asking for standstill never takes the wheels
    local mux = make()
    local ok, reason = mux:command("teleop", 0.0, 0.0)
    assert(not ok and reason == "idle", "an idle command must not be applied")
    assert(#applied == 0, "an idle command must not reach the wheels")
    assert(mux:state().source == nil, "an idle command must not claim the wheels")
end

do -- the last non-zero command owns the wheels until it asks for standstill
    local mux = make()
    assert(mux:command("joystick", 0.4, -0.2))
    assert(mux:state().source == "joystick")
    assert(mux.v == 0.4 and mux.omega == -0.2)

    local ok = mux:command("teleop", 0.0, 0.0)
    assert(not ok, "a standstill stream from another source must not fight the stick")
    assert(mux:state().source == "joystick" and mux.v == 0.4)

    assert(mux:command("joystick", 0.0, 0.0))
    assert(mux:state().source == nil, "standstill must hand the wheels back")
    assert(mux.v == 0.0 and mux.omega == 0.0)
    assert(last_applied() == 0.0, "standstill must reach the wheels")

    assert(mux:command("teleop", -0.3, 0.1))
    assert(mux:state().source == "teleop" and mux.v == -0.3)
end

do -- commands are clamped to the normalized range
    local mux = make()
    mux:command("nav", 5.0, -5.0)
    assert(mux.v == 1.0 and mux.omega == -1.0, "commands must be clamped to -1..1")
end

do -- the stop organ parks the wheels and gates every source
    local mux = make()
    mux:command("joystick", 0.5, 0.0)
    assert(mux:set_stopped(true), "engaging the stop organ is a change")
    assert(last_applied() == 0.0, "engaging the stop organ must park the wheels")

    local ok, reason = mux:command("joystick", 0.5, 0.0)
    assert(not ok and reason == "stopped", "no source may drive while stopped")
    assert(not mux:command("teleop", 0.0, 0.0), "standstill while stopped is not applied either")
    assert(not mux:set_stopped(true), "engaging twice is not a change")
    assert(mux:state().stopped)

    mux:set_stopped(false)
    assert(mux:state().source == nil, "releasing the stop organ drops the owner")
    assert(mux:command("nav", 0.2, 0.0), "a source may drive again after the release")
    assert(mux:state().source == "nav")
end

do -- the state stream fires on changes only
    local mux = make()
    mux:command("nav", 0.2, 0.0)
    local count = #states
    mux:command("nav", 0.2, 0.0)
    assert(#states == count, "an unchanged command must not announce itself")
    mux:command("nav", 0.25, 0.0)
    assert(#states == count + 1, "a changed command must announce itself")
    assert(states[#states].v == 0.25)
end

print("drive_mux: OK")
shutdown()
