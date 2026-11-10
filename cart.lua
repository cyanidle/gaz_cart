-- =============================================================================
--  Cart runtime — drives the four wheel modules over Cyphal and integrates
--  differential-drive odometry. Run on the Raspberry Pi:
--
--      radapter cart.lua
--
--  It also opens a websocket (WS_PORT) that gui.lua — running on another host —
--  connects to in order to tune each module's runtime config live.
--
--  Four sources may drive the cart: the nav planner, the GUI keyboard teleop,
--  the Universal RC Joystick (JOYSTICK) and the optional ROS bridge. mods/
--  drive_mux.lua lets exactly one of them own the wheels, and the joystick's
--  stop organ (the slider at its low extreme) gates all of them.
-- =============================================================================

local config_defs = require "mods.config_defs"
local Odometry   = require "mods.odometry"
local DiffDrive  = require "mods.diff_drive"
local DriveMux   = require "mods.drive_mux"
local TeleopSwitch = require "mods.teleop_switch"
local Rational   = require "mods.rational"
local socket     = require "socket"

-- ---- Tunables --------------------------------------------------------------

local CAN_DEVICE    = args[1] or "can0"   -- socketcan interface the modules sit on
local ROS_PLUGIN_DIR = args[2]  -- optional: dir with radapter_ros; enables ROS cmd_vel
local SIM           = os.getenv("GAZ_SIM") == "1" -- sim robot + sim lidar instead of real hardware
local NODE_ID       = 100       -- this Pi's Cyphal node id
local WS_PORT       = 6080     -- config-GUI websocket (gui.lua connects here)
local SERVICE_PORT  = 6081     -- diagnostic/control websocket (nodes/service.lua)
local TELEOP_MODE   = "auto"   -- "auto": joystick when it reports, else the GUI keyboard
local TRACK_WIDTH   = 0.30     -- distance between left and right wheels, m
local ODO_PERIOD_MS = 20       -- odometry integration period
local TELEM_PERIOD_MS = 100    -- chart telemetry stream period
local MAX_LIN_SPD  = 1       -- m/s at full forward command (v = 1.0)
local MAX_ROT_SPD  = 2       -- rad/s at full turn command (omega = 1.0)
local JOYSTICK     = "/dev/hidraw0"     -- hidraw node of the URCJ, e.g. "/dev/hidraw5"; nil = no joystick
local SIM_START_X  = 0.5     -- initial sim robot x, m
local SIM_START_Y  = 0.5     -- initial sim robot y, m
local REAL_WHEEL_TRAVEL_STDDEV = 0.001 -- per-wheel travel 1-sigma per odometry period, m
local SIM_WHEEL_TRAVEL_STDDEV  = 0.0   -- deterministic mocked encoders
-- Plugin names, not paths: radapter resolves a short name next to its own
-- binary, in <bindir>/plugins, then /usr/lib/radapter/plugins, so the same
-- cart.lua runs both from a build tree (build/bin) and from an install.
local FRAMES_PLUGIN = "gaz_frames"

-- Plugin workers remain opaque implementation details of nodes/nav.lua. Their
-- names and full plugin-native config are declarative here, next to the rest
-- of deployment setup. Nested fields override the node defaults.
---@type NavPluginPaths
local NAV_PLUGINS = {
    frames = FRAMES_PLUGIN,
    nav = "gaz_nav",
    slam = "gaz_slam",
}
---@type NavPluginWorkers
local NAV_WORKERS = {
    -- lidar = { serial = { port = "/dev/ttyUSB0" } }, -- real robot example
    -- slam = { mapper = { do_loop_closing = true } },
}

-- Wheel module node ids (set by each module's DIP switches).  Keys are the
-- wheel names used throughout the script; values are Cyphal node ids.
local WHEELS = { fl = 1, fr = 2, rl = 3, rr = 4 }

-- Cyphal port bases (see CLAUDE.md / app.cpp). Per-module port = base + node id.
local PORT = {
    travel       = 7100,  -- module -> Pi, accumulated wheel travel, m
    linear_speed = 7300,  -- module -> Pi, m/s
    speed_cmd    = 4000,  -- Pi -> module, m/s
    direct_cmd   = 4050,  -- Pi -> module, open-loop voltage (V)
    config       = 4100,  -- Pi -> module, { id, num, den }
}

local INITIAL_CONFIG = {
    wheel_diameter = 0.3,
    kp = 1.2,
    ki = 3,
    kd = 0.001,
    i_limit = 3,
}

-- ---- CAN + Cyphal ----------------------------------------------------------

-- The wheel modules transmit Cyphal over CAN FD (see firmware/Src/fdcan.c:
-- FDCAN_FRAME_FD_BRS, 1 Mbit nominal / 8 Mbit data). A classic socket silently
-- drops FD frames — the modules' telemetry then looks dead while their heartbeats
-- (classic, from this Pi) still arrive — so can_fd is what makes 7100/7300 real.
local can = CAN { plugin = "socketcan", device = CAN_DEVICE, can_fd = true }

local subscribe, publish = {}, {}
for w, nid in pairs(WHEELS) do
    subscribe["trv_" .. w] = { type = "uavcan.si.unit.length.Scalar.1.0",    port = PORT.travel       + nid }
    subscribe["spd_" .. w] = { type = "uavcan.primitive.scalar.Real32.1.0",   port = PORT.linear_speed + nid }
    publish  ["cmd_" .. w] = { type = "uavcan.primitive.scalar.Real32.1.0",   port = PORT.speed_cmd    + nid }
    publish  ["dir_" .. w] = { type = "uavcan.primitive.scalar.Real32.1.0",   port = PORT.direct_cmd   + nid }
    publish  ["cfg_" .. w] = { type = "uavcan.primitive.array.Integer32.1.0", port = PORT.config       + nid }
end

---@type Worker<CartCyphalInput, CartCyphalOutput>
local cyphal = Cyphal {
    can       = can,
    node_id   = NODE_ID,
    subscribe = subscribe,
    publish   = publish,
}

-- latest wheel linear velocity (m/s), refreshed from Cyphal
local wheel_v = { fl = 0.0, fr = 0.0, rl = 0.0, rr = 0.0 }
-- latest absolute wheel travel (m since the module booted), refreshed from Cyphal
local wheel_travel = { fl = 0.0, fr = 0.0, rl = 0.0, rr = 0.0 }
for w in pairs(WHEELS) do
    on(cyphal, "spd_" .. w, function(msg)
        wheel_v[w] = msg.value or 0.0
    end)
    on(cyphal, "trv_" .. w, function(msg)
        wheel_travel[w] = msg.meter or 0.0
    end)
end

-- ---- Driving ---------------------------------------------------------------

-- Per-wheel speed targets (m/s), set by the GUI or programmatically.
local wheel_tgt = { fl = 0.0, fr = 0.0, rl = 0.0, rr = 0.0 }

-- Set by the drive arbiter below: while the joystick's stop organ is engaged
-- the per-wheel debug paths refuse to spin the motors too.
local stop_engaged = false

--- Set target speed for one wheel or all four, and publish to Cyphal.
local function set_speed(wheel, value)
    if stop_engaged and value ~= 0 then
        log.warn("set_speed {} ignored: the joystick stop organ gates the wheels", value)
        return
    end
    local msg = {}
    if not wheel or wheel == "all" then
        for w in pairs(WHEELS) do
            wheel_tgt[w] = value
            msg["cmd_" .. w] = { value = value }
        end
    else
        wheel_tgt[wheel] = value
        msg["cmd_" .. wheel] = { value = value }
    end
    log.info("set_speed {} -> {}", value, wheel or "all")
    cyphal(msg)
end

--- Publish a body twist to the wheels.  v (-1..1) and omega (-1..1) are
--  normalised and clamped, then scaled by MAX_LIN_SPD / MAX_ROT_SPD to physical
--  units.  In sim mode the twist is integrated into a local Odometry instance
--  (mocked motors); in real mode it is published to Cyphal.
local sim_last_t = socket.gettime()
load_plugin(FRAMES_PLUGIN)
local frames = Frames { name = "frames" }
local sim_odo = Odometry.new {
    trackWidth = TRACK_WIDTH,
    wheelTravelStdDev = SIM_WHEEL_TRAVEL_STDDEV,
    frames = frames,
}
sim_odo:reset { x = SIM_START_X, y = SIM_START_Y, theta = 0, timestamp = sim_last_t }

-- Mock encoders: commanded wheel speeds integrate into the same absolute
-- per-wheel travel the modules report, so both paths feed odometry identically.
local sim_travel = { fl = 0.0, fr = 0.0, rl = 0.0, rr = 0.0 }

-- The wheels themselves, reachable only through the source arbiter below.
local hw_drive
if SIM then
    hw_drive = function(v, omega)
        local wheel = DiffDrive.wheels(v, omega, {
            track_width = TRACK_WIDTH, max_linear = MAX_LIN_SPD, max_angular = MAX_ROT_SPD,
            linear_scale = 2, angular_scale = 3,
        })
        local now = socket.gettime()
        local dt = now - sim_last_t
        if dt <= 0 then return end
        sim_last_t = now
        if dt > 0.5 then dt = 0.05 end
        local d = {}
        for w in pairs(WHEELS) do
            d[w] = wheel[w] * dt
            sim_travel[w] = sim_travel[w] + d[w]
            -- Mocked modules: ideal motors, so the tuning chart and the wheel
            -- telemetry show what the cart was asked for instead of nothing.
            wheel_tgt[w] = wheel[w]
            wheel_v[w] = wheel[w]
        end
        sim_odo:update(d.fl, d.fr, d.rl, d.rr, dt, now)
    end
else
    hw_drive = function(v, omega)
        local wheel = DiffDrive.wheels(v, omega, {
            track_width = TRACK_WIDTH, max_linear = MAX_LIN_SPD, max_angular = MAX_ROT_SPD,
        })
        wheel_tgt.fl, wheel_tgt.rl = wheel.fl, wheel.rl
        wheel_tgt.fr, wheel_tgt.rr = wheel.fr, wheel.rr
        cyphal {
            cmd_fl = { value = wheel.fl }, cmd_rl = { value = wheel.rl },
            cmd_fr = { value = wheel.fr }, cmd_rr = { value = wheel.rr },
        }
    end
end

--- Open-loop voltage to one wheel ("fl".."rr") or "all", for bring-up / tuning.
local function direct_voltage(wheel, volts)
    if stop_engaged and volts ~= 0 then
        log.warn("direct_voltage {} V ignored: the joystick stop organ gates the wheels", volts)
        return
    end
    log.info("direct_voltage {}V -> {}", volts, wheel or "all")
    if not wheel or wheel == "all" then
        for w in pairs(WHEELS) do
            cyphal { ["dir_" .. w] = { value = volts } }
        end
    else
        cyphal { ["dir_" .. wheel] = { value = volts } }
    end
end

-- ---- Config over Cyphal ----------------------------------------------------

-- config_defs is keyed by parameter key; index it by id for lookups and stash
-- the key on each entry so we can log/report it.
local config_by_id = {}
for key, c in pairs(config_defs) do
    c.key = key
    config_by_id[c.id] = c
end

--- Push one config value (selected by id) to a wheel ("fl".."rr") or "all".
local function set_config(wheel, id, value)
    local def = config_by_id[id]
    if not def then
        log.warn("set_config: unknown config id {}", id)
        return
    end
    local num, den = Rational.from_number(value)
    if not wheel or wheel == "all" then
        for w in pairs(WHEELS) do
            cyphal { ["cfg_" .. w] = { value = { id, num, den } } }
        end
    else
        cyphal { ["cfg_" .. wheel] = { value = { id, num, den } } }
    end
    log.info("config '{}' = {} -> {}", def.key, value, wheel or "all")
end

-- ---- Config-GUI websocket --------------------------------------------------

local ws = WebsocketServer {
    name = "main_ws",
    port = WS_PORT,
    protocol = "msgpack"
}

local function send_initial_config()
    set_config("all", config_defs.wheel_diameter.id, INITIAL_CONFIG.wheel_diameter)
    set_config("all", config_defs.pid_kp.id, INITIAL_CONFIG.kp)
    set_config("all", config_defs.pid_ki.id, INITIAL_CONFIG.ki)
    set_config("all", config_defs.pid_kd.id, INITIAL_CONFIG.kd)
    set_config("all", config_defs.pid_i_limit.id, INITIAL_CONFIG.i_limit)
    direct_voltage("all", 0.0)
end

send_initial_config()

-- ---- Drive sources ---------------------------------------------------------
-- Every source tags its commands with its own name so the arbiter can keep one
-- of them in control; the accepted command is mirrored to the GUI's teleop tab,
-- which therefore shows the joystick's twist just as it shows its own keys'.

local teleop_model = branch(ws, "teleop")

local blocked = {} -- sources whose command the stop organ dropped, per stop episode
local last_source

local function publish_drive(state)
    stop_engaged = state.stopped
    if not state.stopped and next(blocked) then blocked = {} end
    if state.source ~= last_source then
        if state.source then
            log.info("drive: '{}' has the wheels", state.source)
        else
            log.debug("drive: wheels released")
        end
        last_source = state.source
    end
    teleop_model {
        v = state.v,
        omega = state.omega,
        source = state.source or "none",
        stopped = state.stopped,
    }
end

local mux = DriveMux.new { apply = hw_drive, on_state = publish_drive }

local bench_wheel -- wheel driving alone in bench mode, nil when the cart drives
local single_wheel -- defined with the bench mode below, needed by drive()

---@param v number normalized forward command, -1..1
---@param omega number normalized yaw command, -1..1
---@param source string drive source: "nav", "teleop", "joystick" or "ros"
local function drive(v, omega, source)
    -- A real body twist leaves the single-wheel bench mode; a neutral stream
    -- (the joystick at rest, the keyboard tab's 20 Hz idle tick) does not.
    if bench_wheel and (v ~= 0.0 or omega ~= 0.0) then single_wheel(nil) end
    local applied, reason = mux:command(source, v, omega)
    if applied or reason ~= "stopped" then return end
    if v == 0.0 and omega == 0.0 then return end
    if blocked[source] then return end
    blocked[source] = true
    log.warn("drive: '{}' command ignored, the joystick stop switch gates the wheels", source)
end

-- ---- Operator input ---------------------------------------------------------
-- The joystick and the GUI's WASD window would otherwise alternate ownership of
-- the wheels command by command (the arbiter hands the wheels to whichever of
-- them last asked for a non-zero twist). mods/teleop_switch.lua picks one of
-- them up front: the stick while it reports, the keyboard otherwise, and
-- TELEOP_MODE can force either. The choice is mirrored to the teleop tab.

local joystick_status = {
    receiving = false, axes = {}, stopped = false, calibrated = true, age_ms = 0,
}

local select_source = TeleopSwitch.new {
    mode = TELEOP_MODE,
    has_joystick = JOYSTICK ~= nil and TELEOP_MODE ~= "keyboard",
    on_change = function(source, previous)
        log.info("operator input: {} -> {}", previous, source)
        teleop_model { operator = source }
    end,
}

local keyboard_blocked, joystick_blocked

---Ask the switch whether the operator input a source carries is live, warning
---once per takeover instead of silently dropping every command.
---@param wanted string "keyboard" or "joystick"
---@return boolean
local function operator_active(wanted)
    if select_source:allows(wanted) then
        if wanted == "keyboard" then keyboard_blocked = false else joystick_blocked = false end
        return true
    end
    if wanted == "keyboard" and not keyboard_blocked then
        keyboard_blocked = true
        log.warn("keyboard teleop ignored: the operator input belongs to '{}' (TELEOP_MODE={})",
            select_source:active(), TELEOP_MODE)
    elseif wanted == "joystick" and not joystick_blocked then
        joystick_blocked = true
        log.warn("joystick ignored: the operator input belongs to '{}' (TELEOP_MODE={})",
            select_source:active(), TELEOP_MODE)
    end
    return false
end

-- ---- Single-wheel bench mode ------------------------------------------------
-- One wheel driven on its own, with the other three parked. The GUI's teleop
-- tab asks for it, and so does the service channel; it is how a module is
-- brought up on the bench without the cart having to drive. drive() above
-- leaves the mode as soon as a real body twist arrives.

---Drive exactly one wheel, parking the others the first time that wheel is used.
---@param wheel string? wheel name, nil/"" leaves the mode
---@param v number normalized wheel speed, -1..1 (scaled by MAX_LIN_SPD)
function single_wheel(wheel, v)
    if not wheel or wheel == "" or wheel == "none" then
        if bench_wheel then
            set_speed("all", 0.0)
            log.info("single-wheel bench mode: off")
            bench_wheel = nil
        end
        return
    end
    assert(WHEELS[wheel], ("unknown wheel '%s' (expected fl/fr/rl/rr)"):format(tostring(wheel)))
    if wheel ~= bench_wheel then
        for name in pairs(WHEELS) do
            if name ~= wheel then set_speed(name, 0.0) end
        end
        bench_wheel = wheel
        log.info("single-wheel bench mode: {}", wheel)
    end
    set_speed(wheel, (v or 0.0) * MAX_LIN_SPD)
end

-- ---- Nodes -------------------------------------------------------------------
-- Each node is a module returning a single setup function. It gets everything
-- hardware-tied or externally configurable through its config table, including
-- `model` — a node() worker exchanging unwrapped messages with the GUI
-- websocket under the node's namespace (mirrors Main.qml's model.branch() keys).

local odo = require "nodes.odo" {
    model = branch(ws, "odo"),
    track_width = TRACK_WIDTH,
    wheels = function()
        local w = {}
        for name in pairs(WHEELS) do
            w[name] = { tgt = wheel_tgt[name], act = wheel_v[name] }
        end
        return w
    end,
    travel = function()
        return wheel_travel
    end,
    set_speed = set_speed,
    set_config = set_config,
    direct_voltage = direct_voltage,
    odo_period_ms = ODO_PERIOD_MS,
    telem_period_ms = TELEM_PERIOD_MS,
    wheel_travel_stddev = REAL_WHEEL_TRAVEL_STDDEV,
    frames = frames,
}

-- SIM-aware pose: returns simulated (mocked-motor) pose or real odometry.
local function pose()
    if SIM then
        return sim_odo:pose()
    else
        return odo:pose()
    end
end

-- Full timestamped odometry is the localization/navigation state source.
-- pose() remains available for callers that only need the legacy tuple.
local function odometry()
    if SIM then
        return sim_odo:odometry()
    else
        return odo:odometry()
    end
end

require "nodes.nav" {
    model = branch(ws, "nav"),
    sim   = SIM,
    drive = function(v, omega) drive(v, omega, "nav") end,
    pose  = pose,
    odometry = odometry,
    frames = frames,
    plugins = NAV_PLUGINS,
    workers = NAV_WORKERS,
}

-- Keyboard teleop from the GUI: only while the switch says the operator input
-- is the keyboard (the joystick takes it over while it reports).
require "nodes.teleop" {
    model = teleop_model,
    drive = function(v, omega)
        if not operator_active("keyboard") then return end
        drive(v, omega, "teleop")
    end,
    wheel = single_wheel,
}

-- Joystick teleop: the Universal RC Joystick drives the cart, the same stop
-- organ as the bench script debug/motor.lua latches and releases it. Axis and
-- threshold defaults live in the node; the hidraw node is the JOYSTICK tunable.
-- The node also reports its axes and stop state, which feeds the switch and the
-- service channel.
if JOYSTICK and TELEOP_MODE ~= "keyboard" then
    require "nodes.joystick" {
        path = JOYSTICK,
        drive = function(v, omega)
            if not operator_active("joystick") then return end
            drive(v, omega, "joystick")
        end,
        stop = function(stopped) mux:set_stopped(stopped) end,
        on_status = function(status)
            joystick_status = status
            select_source:joystick_reporting(status.receiving)
        end,
    }
end

-- Optional external drive source: a ROS 2 stack publishing cmd_vel (Twist).
-- Only one of this and the internal nav planner should run at a time.
if ROS_PLUGIN_DIR then
    require "nodes.ros" {
        drive = function(v, omega) drive(v, omega, "ros") end,
        plugin_dir = ROS_PLUGIN_DIR,
    }
end

-- ---- Service channel ---------------------------------------------------------
-- A second websocket with the complete runtime state and every command, for
-- bench bring-up and remote diagnosis. The operator GUI never talks to it.

local service_ws = WebsocketServer {
    name = "service_ws",
    port = SERVICE_PORT,
    protocol = "msgpack",
}

---@return table complete runtime snapshot published on the service channel
local function service_state()
    local wheels = {}
    for name in pairs(WHEELS) do
        wheels[name] = {
            tgt = wheel_tgt[name],
            act = wheel_v[name],
            travel = wheel_travel[name],
        }
    end
    local odom = odometry()
    return {
        sim = SIM,
        wheels = wheels,
        mux = mux:state(),
        operator = select_source:active(),
        teleop_mode = TELEOP_MODE,
        bench_wheel = bench_wheel or "none",
        joystick = joystick_status,
        odom = {
            x = odom.pose.x, y = odom.pose.y, theta = odom.pose.theta,
            v = odom.twist.linear, omega = odom.twist.angular,
        },
    }
end

require "nodes.service" {
    model = service_ws,
    state = service_state,
    drive = function(v, omega)
        drive(v, omega, "service")
    end,
    single_wheel = single_wheel,
    set_speed = set_speed,
    set_config = set_config,
    direct_voltage = direct_voltage,
    stop = function(stopped) mux:set_stopped(stopped) end,
}

log.info("cart up: node {} on {}, ws on :{} (service :{})", NODE_ID, CAN_DEVICE, WS_PORT, SERVICE_PORT)
