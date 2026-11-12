-- =============================================================================
--  debug/tune.lua — bench tuning GUI for one wheel module over its serial
--  channel (USART2 / the ST-LINK VCP).
--
--      radapter --gui debug/tune.lua [port=/dev/ttyACM0] [vmax=10] [log=<file>]
--
--  Sliders for every runtime-config parameter (mods/config_defs.lua ids, the
--  same ids the CAN config topic uses), live charts of the `V1` diagnostics
--  line (speed target vs actual, q/phase currents, q voltage and the bus), a
--  NORMALIZED target-speed slider (-1..1 of the module's full-scale wheel
--  speed, config id 16) and an open-loop-voltage slider, with a Reset that
--  zeroes both without touching the parameters, and the enable / zero /
--  calibrate commands. The m/s the slider will command is shown next to it,
--  from the module's own read-back of that full-scale value. The module's
--  EEPROM keeps every accepted value, so a tuning
--  survives GUI restarts and power cycles: on connect the GUI reads each id
--  back (`G<id>`) and the sliders start from what the module runs now.
--
--  Options are `key=value` words, because radapter's own CLI owns the dashed
--  flags. Never point this at a powered cart: it commands the drive directly.
-- =============================================================================

local USAGE = [[
usage: radapter --gui debug/tune.lua [port=/dev/ttyACM0] [vmax=10] [log=<file>]

  port    module's serial bench channel (USART2)
  vmax    direct-voltage slider range, V
  log     append every line the module sends to this file (timestamped)

The target-speed slider is normalized: -1..1 of the module's full-scale wheel
speed (config id 16), which the GUI reads back on connect and shows in m/s.
]]

local opts = {
    port = "/dev/ttyACM0",
    vmax = 10.0,
    log = nil,
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
    if arg == "help" then
        print(USAGE)
        shutdown()
        return
    end
    local key, value = arg:match("^([%w%-]+)=(.*)$")
    if key then
        set_option((key:gsub("%-", "_")), value)
    else
        set_option((arg:gsub("%-", "_")), nil)
    end
end

if opts.vmax <= 0 then error("vmax must be positive") end

package.path = SCRIPT_DIR .. "/../?.lua;" .. package.path

local config_defs = require "mods.config_defs"

local socket = require "socket"

local motor = Serial {
    framing = "line",
    protocol = "text",
    port = opts.port,
    baud = 115200,
}

local log_file = opts.log and io.open(opts.log, "a")

-- Fault names for the numeric `f=` field (MotorFault enum order).
local FAULT_NAMES = {
    "NONE", "INVALID_HALL", "INVALID_SUPPLY", "INVALID_COMMAND",
    "STALLED", "OVERCURRENT", "DRIVER_FAULT",
}

-- The params table drives the QML slider form; `value` starts at the
-- config_defs default and is replaced by the module's read-back on connect.
local params = {}
for key, c in pairs(config_defs) do
    params[#params + 1] = {
        key = key, id = c.id, label = c.label, unit = c.unit,
        min = c.min, max = c.max, step = c.step,
        default = c.default, value = c.default,
    }
end
table.sort(params, function(a, b) return a.id < b.id end)

-- Read-back values by id, filled by the `C<id>=<v>` reply parser.
local values = {}
-- Wheel diameter from id 0 (m/s <-> rad/s for the speed chart).
local wheel_diameter_m = config_defs.wheel_diameter.default
-- Full-scale wheel speed from id 16: the m/s a normalized target of 1.0 means.
local max_speed_m_s = config_defs.max_speed.default
-- Latest diagnostics sample, throttled into the view.
local latest_diag
local last_push_s = 0.0
local answered = false

local view = QML { url = SCRIPT_DIR .. "/qml/TuneWindow.qml" }

local function command(line)
    log.info("> {}", line)
    motor(line)
end

local function push_status(text)
    view { status = text }
end

-- Feed one diagnostics line into the charts. `D` lines arrive at up to ~71 Hz;
-- the QML timer samples the latest at its own pace, so pushes are throttled
-- only enough to keep the worker queue shallow.
local function feed_diag(line)
    local d = {}
    for key, value in line:gmatch("([%a]+)=(-?[%d.]+)") do
        d[key] = tonumber(value) or 0.0
    end
    if d.t == nil and d.v == nil then return end
    latest_diag = d
    local now = socket.gettime()
    if now - last_push_s >= 0.02 then
        last_push_s = now
        local radius = wheel_diameter_m * 0.5
        view {
            diag = {
                t = d.t, vm = d.v * radius,
                sp = d.sp, iq = d.iq, idc = d.id, im = d.im,
                uq = d.uq, ud = d.ud, b = d.b, n = d.n, e = d.e,
            },
        }
        local fault = FAULT_NAMES[math.floor(d.f) + 1] or ("f=" .. tostring(d.f))
        push_status(fmt(
            "v={:.2f} rad/s   i={:.2f} A   bus={:.1f} V   {}   driver {}",
            d.v, d.iq, d.b, fault, d.e == 1 and "on" or "off"
        ))
    end
end

local function read_back_params()
    for _, p in ipairs(params) do
        motor(("G%d"):format(p.id))
    end
end

-- Derived state a parameter change feeds directly: the m/s labels beside the
-- normalized target slider are computed from the full-scale speed, so a C16
-- write has to reach the view without waiting for a re-read of every slider.
local function apply_derived(id, value)
    if id == config_defs.wheel_diameter.id then
        wheel_diameter_m = value
    elseif id == config_defs.max_speed.id then
        max_speed_m_s = value
        view { max_speed_mps = value }
    end
end

local function apply_read_back()
    local changed = false
    for _, p in ipairs(params) do
        local value = values[p.id]
        if value ~= nil then
            p.value = value
            changed = true
            apply_derived(p.id, value)
        end
    end
    if changed then
        view { params = params }
    end
end

local function park()
    command("T0")
    command("X")
    command("E0")
end

on_shutdown(function()
    motor("T0")
    motor("X")
    motor("E0")
    motor("M0")
    motor("V0")
end)

pipe(motor, function(line)
    if log_file then
        log_file:write(fmt("{}\n", line))
        log_file:flush()
    end
    if line:match("^D ") then
        feed_diag(line:sub(3))
    elseif line:match("^S ") then
        answered = true
        log.info("< {}", line)
    elseif line:match("^C%d+=") then
        local id, value = line:match("^C(%d+)=(%-?[%d.]+)")
        if id and value then
            local param_id, number = tonumber(id), tonumber(value)
            values[param_id] = number
            apply_derived(param_id, number)
            -- Keep the model in step with what the module accepted, without
            -- pushing the whole slider list back at a slider under the cursor.
            for _, p in ipairs(params) do
                if p.id == param_id then
                    p.value = number
                    break
                end
            end
        end
    elseif line:match("^# ") then
        log.info("< {}", line)
    elseif line:match("^ERR") then
        log.warn("< {}", line)
    else
        log.info("< {}", line)
    end
end)

-- QML -> serial. The view sends small action tables; anything unknown is
-- logged loudly instead of silently dropped.
pipe(view, function(msg)
    if msg.set then
        command(("C%d:%s"):format(msg.set.id, msg.set.value))
    elseif msg.target ~= nil then
        -- Already normalized -1..1: the module maps it onto its full scale.
        local target = math.max(-1.0, math.min(1.0, msg.target))
        command(("T%.3f"):format(target))
    elseif msg.voltage ~= nil then
        command(("Q%.3f"):format(msg.voltage))
    elseif msg.enable == true then
        command("E1")
    elseif msg.enable == false then
        command("E0")
    elseif msg.zero then
        command("X")
    elseif msg.calibrate then
        command("A")
    elseif msg.readback then
        read_back_params()
    elseif msg.line then
        command(msg.line)
    else
        log.warn("unhandled UI message: {}", msg)
        return nil
    end
    return msg
end)

view {
    params = params,
    max_speed_mps = max_speed_m_s,
    max_voltage = opts.vmax,
    min_voltage = -opts.vmax,
    status = "(connecting)",
}

push_status("(probing " .. opts.port .. " — the board answers ~10 s after a reset)")

log.info("probing {} (the board answers ~12 s after flashing)", opts.port)
for _ = 1, 40 do
    motor("S")
    sleep(500)
    if answered then break end
end

if not answered then
    log.error("{} did not answer an S probe; check the port and the board", opts.port)
    push_status("no answer on " .. opts.port)
    return
end

-- The sliders start from what the module actually runs (EEPROM record or
-- compiled defaults), not from this table's defaults.
command("D0")
read_back_params()
sleep(800)
apply_read_back()

command("M71")
command("V1")
log.info("telemetry on (M71, V1 diagnostics); park on exit")
