-- =============================================================================
--  debug/motor.lua — drive one wheel module from the HID joystick.
--
--  gaz_joystick (/dev/hidraw5) -> USART2 bench channel of a module
--  (/dev/ttyACM0): the chosen axis sets the NORMALIZED wheel speed target
--  (-1..1 of the module's configured full-scale speed, config id 16), rescaled
--  from a deadzone around the centre. The script enables the driver once the
--  board answers and parks it on exit (T0, X, E0).
--
--  The stop switch is an axis at its low extreme (slider = -1 by default):
--  below stop-below the drive gets T0, X, E0 and stays disabled, above
--  resume-above it is enabled again. The gap between the thresholds is
--  hysteresis, so a channel hovering at the edge cannot flap the driver. The
--  drive is also held at zero when joystick reports stop arriving for 0.5 s.
--
--    build/bin/radapter debug/motor.lua [port=/dev/ttyACM0] [hidraw=/dev/hidraw5]
--        [axis=y] [invert] [deadzone=0.05]
--        [stop-axis=slider] [stop-below=-0.8] [resume-above=-0.5]
--
--  Options are `key=value` words, because radapter's own CLI owns the dashed
--  flags. Never point this at a powered cart: it commands the drive directly.
-- =============================================================================

local USAGE = [[
usage: radapter debug/motor.lua [port=/dev/ttyACM0] [hidraw=/dev/hidraw5]
                                [axis=y] [invert] [deadzone=0.05]
                                [stop-axis=slider] [stop-below=-0.8]
                                [resume-above=-0.5]

  port           module's serial bench channel (USART2)
  hidraw         joystick hidraw node
  axis           joystick axis that sets the speed (x, y, z, rx, ry, rz, slider, dial)
  invert         flip the axis direction
  deadzone       normalized deadzone around the axis centre, 0..1
  stop-axis      axis used as the stop switch; empty string disables it
  stop-below     stop-axis value at or below which the drive is parked
  resume-above   stop-axis value at or above which the drive is enabled again

Full deflection commands -1 or +1: the module scales that with its own
full-scale wheel speed (config id 16, readable as `G16`), so the same script
drives a module at any configured top speed.
]]

for _, arg in ipairs(args) do
    if arg == "help" then
        print(USAGE)
        shutdown()
        return
    end
end

local opts = {
    port = "/dev/ttyACM0",
    hidraw = "/dev/hidraw5",
    axis = "y",
    invert = false,
    deadzone = 0.05,
    stop_axis = "slider",
    stop_below = -0.8,
    resume_above = -0.5,
}

local function set_option(key, value)
    local current = opts[key]
    if current == nil then
        error(("unknown option '%s'; run with the word 'help' to list them"):format(key))
    end
    if type(current) == "boolean" then
        opts[key] = value == nil or (value ~= "0" and value ~= "false")
    elseif type(current) == "number" then
        local number = tonumber(value)
        if number == nil then
            error(("%s expects a number, got '%s'"):format(key, tostring(value)))
        end
        opts[key] = number
    else
        if value == nil then
            error(("%s expects a value"):format(key))
        end
        opts[key] = value
    end
end

for _, arg in ipairs(args) do
    local key, value = arg:match("^([%w%-]+)=(.*)$")
    if key then
        set_option((key:gsub("%-", "_")), value)
    else
        set_option((arg:gsub("%-", "_")), nil)
    end
end

if opts.deadzone < 0 or opts.deadzone >= 1 then error("deadzone must be in 0..1") end
if opts.stop_below >= opts.resume_above then
    error("stop-below must be lower than resume-above, otherwise the switch has no hysteresis")
end

-- The bench script reuses the cart's axis shaping and stop organ, so a wheel
-- tuned here behaves exactly like the cart's teleop (nodes/joystick.lua).
package.path = SCRIPT_DIR .. "/../?.lua;" .. package.path

local motor = Serial {
    framing = "line",
    protocol = "text",
    port = opts.port,
    baud = 115200,
}

local socket = require "socket"
local JoystickAxes = require "mods.joystick_axes"

load_plugin("gaz_joystick")

local joystick = Joystick { name = "joystick", path = opts.hidraw }

local answered = false
local armed = false
local target = 0.0
local last_report = socket.gettime()
local stop_state = nil

local function command(line)
    log.info("> {}", line)
    motor(line)
end

-- `speed` is the normalized -1..1 command; the module owns the m/s mapping.
local function drive(speed, force)
    speed = math.max(-1.0, math.min(1.0, speed))
    if not force and math.abs(speed - target) < 0.01 then return end
    target = speed
    motor(("T%.3f"):format(speed))
end

---@param axes table<string, number>
---@return number? normalized -1..1 command, nil when the joystick has no such axis
local function speed_of(axes)
    return JoystickAxes.value(axes, opts.axis, opts.invert, opts.deadzone)
end

---Stop switch read from an axis. The two thresholds give hysteresis, so a
---channel hovering at the edge cannot flap the drive; inside the band the
---previous state is kept.
---@param axes table<string, number>
---@return boolean? true = stop, false = allowed, nil = disabled or inside the band
local function stop_switch(axes)
    return JoystickAxes.stop_switch(axes, opts.stop_axis, opts.stop_below, opts.resume_above)
end

local function park()
    armed = false
    target = 0.0
    command("T0")
    command("X")
    command("E0")
end

local function arm()
    command("M0")
    command("D0")
    command("E0")
    command("E1")
    sleep(1000)
    command("Q0")
    drive(0.0, true)
    armed = true
    log.info("drive enabled: axis '{}' sets the wheel speed, -1..1 of full scale; stop when {} <= {}",
        opts.axis, opts.stop_axis, opts.stop_below)
end

pipe(motor, function(line)
    if line:match("^S ") then
        answered = true
        log.info("< {}", line)
    elseif line:match("^# ") then
        log.info("< {}", line)
    elseif line:match("^ERR") then
        log.warn("< {}", line)
    end
end)

pipe(joystick, function(msg)
    last_report = socket.gettime()

    local stop = stop_switch(msg.axes)
    if stop ~= nil and stop ~= stop_state then
        stop_state = stop
        if stop then
            log.warn("stop switch engaged: drive parked, {} <= {}", opts.stop_axis, opts.stop_below)
            park()
        else
            arm()
        end
    end

    if not armed then return end

    local speed = speed_of(msg.axes)
    if speed == nil then
        local names = {}
        for name in pairs(msg.axes) do names[#names + 1] = name end
        log.error("joystick has no axis '{}' (it has: {})", opts.axis, table.concat(names, ", "))
        park()
        return
    end
    drive(speed)
end)

each(1000, function()
    if armed then motor(("T%.3f"):format(target)) end
    motor("S")
end)

local control_lost = false
each(250, function()
    if not armed then
        control_lost = false
        return
    end
    if socket.gettime() - last_report < 0.5 then
        control_lost = false
        return
    end
    if not control_lost then
        log.warn("no joystick reports for 0.5 s: holding the wheel at zero")
        control_lost = true
    end
    drive(0.0, true)
end)

on_shutdown(function()
    motor("T0")
    motor("X")
    motor("E0")
end)

log.info("probing {} (the board answers ~12 s after flashing)", opts.port)
for _ = 1, 40 do
    motor("S")
    sleep(500)
    if answered then break end
end

if not answered then
    log.error("{} did not answer an S probe; check the port, the baud rate and the board", opts.port)
    shutdown()
    return
end

log.info("waiting for the first joystick report to decide the drive state")
