-- =============================================================================
--  debug/motor.lua — drive one wheel module from the HID joystick.
--
--  gaz_joystick (/dev/hidraw5) -> USART2 bench channel of a module
--  (/dev/ttyACM0): the chosen axis sets the target wheel speed in m/s, clamped
--  to +-max and rescaled from a deadzone around the centre. The script enables
--  the driver after the board answers and parks it on exit (T0, X, E0).
--  One joystick button latches a full stop (T0, X, E0); press it again to
--  re-arm. A button that is already held at start-up is ignored until it has
--  been seen released once. Losing the joystick for 0.5 s holds the wheel at
--  zero (the driver stays enabled).
--
--    build/bin/radapter debug/motor.lua [port=/dev/ttyACM0] [hidraw=/dev/hidraw5]
--        [axis=y] [invert] [max=2] [deadzone=0.05] [stop-button=1]
--
--  Options are `key=value` words, because radapter's own CLI owns the dashed
--  flags. Never point this at a powered cart: it commands the drive directly.
-- =============================================================================

local USAGE = [[
usage: radapter debug/motor.lua [port=/dev/ttyACM0] [hidraw=/dev/hidraw5]
                                [axis=y] [invert] [max=2]
                                [deadzone=0.05] [stop-button=1]

  port         module's serial bench channel (USART2)
  hidraw       joystick hidraw node
  axis         joystick axis that sets the speed (x, y, z, rx, ry, rz, slider, dial)
  invert       flip the axis direction
  max          speed at full deflection, m/s
  deadzone     normalized deadzone around the axis centre, 0..1
  stop-button  joystick button that latches a full stop and re-arms it
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
    max = 2.0,
    deadzone = 0.05,
    stop_button = 1,
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

if opts.max <= 0 then error("max must be positive") end
if opts.deadzone < 0 or opts.deadzone >= 1 then error("deadzone must be in 0..1") end
if opts.stop_button < 1 then error("stop-button must be >= 1") end

local motor = Serial {
    framing = "line",
    protocol = "text",
    port = opts.port,
    baud = 115200,
}

local socket = require "socket"

load_plugin("gaz_joystick")

local joystick = Joystick { name = "joystick", path = opts.hidraw }

local answered = false
local armed = false
local target = 0.0
local stop_was_pressed = true
local last_report = socket.gettime()

local function command(line)
    log.info("> {}", line)
    motor(line)
end

local function drive(speed, force)
    speed = math.max(-opts.max, math.min(opts.max, speed))
    if not force and math.abs(speed - target) < 0.01 then return end
    target = speed
    motor(("T%.3f"):format(speed))
end

---@param axes table<string, number>
---@return number? m/s, nil when the joystick has no such axis
local function speed_of(axes)
    local value = axes[opts.axis]
    if value == nil then return nil end
    if opts.invert then value = -value end
    local magnitude = math.abs(value)
    if magnitude <= opts.deadzone then return 0.0 end
    local scaled = math.min((magnitude - opts.deadzone) / (1 - opts.deadzone), 1.0)
    return (value < 0 and -1 or 1) * scaled * opts.max
end

local function pressed(buttons, wanted)
    for _, button in ipairs(buttons) do
        if button == wanted then return true end
    end
    return false
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
    log.info("drive enabled: axis '{}' sets the wheel speed, +-{} m/s, stop button {}",
        opts.axis, opts.max, opts.stop_button)
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

    local down = pressed(msg.buttons, opts.stop_button)
    local edge = down and not stop_was_pressed
    stop_was_pressed = down

    if edge then
        if armed then
            log.warn("full stop: drive disabled, press button {} again to re-arm", opts.stop_button)
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

arm()
