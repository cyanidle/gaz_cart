-- =============================================================================
--  Cart runtime — drives the four wheel modules over Cyphal and integrates
--  differential-drive odometry. Run on the Raspberry Pi:
--
--      radapter cart.lua
--
--  It also opens a websocket (WS_PORT) that gui.lua — running on another host —
--  connects to in order to tune each module's runtime config live.
--
--  The returned table (.drive/.stop/.set_config/.set_speed/.odo/...) is handy
--  from an interactive session or another script that `require`s this one.
-- =============================================================================

local config_defs = require "mods.config_defs"
local Odometry   = require "mods.odometry"
local socket     = require "socket"

-- ---- Tunables --------------------------------------------------------------

local CAN_DEVICE    = args[1] or "can0"   -- socketcan interface the modules sit on
local ROS_PLUGIN_DIR = args[2]  -- optional: dir with radapter_ros; enables ROS cmd_vel
local SIM           = true      -- sim robot + sim lidar instead of real hardware
local NODE_ID       = 100       -- this Pi's Cyphal node id
local WS_PORT       = 6080     -- config-GUI websocket (gui.lua connects here)
local TRACK_WIDTH   = 0.30     -- distance between left and right wheels, m
local ODO_PERIOD_MS = 20       -- odometry integration period
local TELEM_PERIOD_MS = 100    -- chart telemetry stream period
local MAX_LIN_SPD  = 1       -- m/s at full forward command (v = 1.0)
local MAX_ROT_SPD  = 2       -- rad/s at full turn command (omega = 1.0)
local SIM_START_X  = 1.0     -- initial sim robot x, m
local SIM_START_Y  = 1.0     -- initial sim robot y, m

-- Wheel module node ids (set by each module's DIP switches).  Keys are the
-- wheel names used throughout the script; values are Cyphal node ids.
local WHEELS = { fl = 1, fr = 2, rl = 3, rr = 4 }

-- Cyphal port bases (see CLAUDE.md / app.cpp). Per-module port = base + node id.
local PORT = {
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

local can = CAN { plugin = "socketcan", device = CAN_DEVICE }

local subscribe, publish = {}, {}
for w, nid in pairs(WHEELS) do
    subscribe["spd_" .. w] = { type = "uavcan.primitive.scalar.Real32.1.0",   port = PORT.linear_speed + nid }
    publish  ["cmd_" .. w] = { type = "uavcan.primitive.scalar.Real32.1.0",   port = PORT.speed_cmd    + nid }
    publish  ["dir_" .. w] = { type = "uavcan.primitive.scalar.Real32.1.0",   port = PORT.direct_cmd   + nid }
    publish  ["cfg_" .. w] = { type = "uavcan.primitive.array.Integer32.1.0", port = PORT.config       + nid }
end

local cyphal = Cyphal {
    can       = can,
    node_id   = NODE_ID,
    subscribe = subscribe,
    publish   = publish,
}

-- latest wheel linear velocity (m/s), refreshed from Cyphal
local wheel_v = { fl = 0.0, fr = 0.0, rl = 0.0, rr = 0.0 }
for w in pairs(WHEELS) do
    on(cyphal, "spd_" .. w, function(msg)
        wheel_v[w] = msg.value or 0.0
    end)
end

-- ---- Driving ---------------------------------------------------------------

-- Per-wheel speed targets (m/s), set by the GUI or programmatically.
local wheel_tgt = { fl = 0.0, fr = 0.0, rl = 0.0, rr = 0.0 }

--- Set target speed for one wheel or all four, and publish to Cyphal.
local function set_speed(wheel, value)
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

--- Command a body twist.  v (-1..1) and omega (-1..1) are normalised and
--  clamped, then scaled by MAX_LIN_SPD / MAX_ROT_SPD to physical units.
--  In sim mode the twist is integrated into a local Odometry instance
--  (mocked motors); in real mode it is published to Cyphal.
local sim_last_t = socket.gettime()
local sim_odo = Odometry.new { trackWidth = TRACK_WIDTH }
sim_odo:reset { x = SIM_START_X, y = SIM_START_Y, theta = 0 }

local drive
if SIM then
    drive = function(v, omega)
        v     = math.max(-1, math.min(1, v or 0))
        omega = math.max(-1, math.min(1, omega or 0))
        local v_real = v * MAX_LIN_SPD * 2
        local omega_real = omega * MAX_ROT_SPD * 3
        local half = TRACK_WIDTH * 0.5
        local vL = v_real - omega_real * half
        local vR = v_real + omega_real * half
        local now = socket.gettime()
        local dt = now - sim_last_t
        sim_last_t = now
        if dt > 0.5 then dt = 0.05 end
        sim_odo:update(vL, vR, vL, vR, dt)
    end
else
    drive = function(v, omega)
        v     = math.max(-1, math.min(1, v or 0))
        omega = math.max(-1, math.min(1, omega or 0))
        local v_real = v * MAX_LIN_SPD
        local omega_real = omega * MAX_ROT_SPD
        local half = TRACK_WIDTH * 0.5
        local vL = v_real - omega_real * half
        local vR = v_real + omega_real * half
        wheel_tgt.fl, wheel_tgt.rl = vL, vL
        wheel_tgt.fr, wheel_tgt.rr = vR, vR
        cyphal {
            cmd_fl = { value = vL }, cmd_rl = { value = vL },
            cmd_fr = { value = vR }, cmd_rr = { value = vR },
        }
    end
end

--- Open-loop voltage to one wheel ("fl".."rr") or "all", for bring-up / tuning.
local function direct_voltage(wheel, volts)
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

local function gcd(a, b)
    a, b = math.abs(a), math.abs(b)
    while b > 0.5 do a, b = b, a % b end
    return a
end

-- Express a float as an int32 numerator/denominator pair (value = num/den).
local function to_rational(x)
    local den = 1000000
    local num = math.floor(x * den + (x >= 0 and 0.5 or -0.5))
    local g   = gcd(num, den)
    if g < 1 then g = 1 end
    return math.floor(num / g), math.floor(den / g)
end

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
    local num, den = to_rational(value)
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
    set_speed = set_speed,
    set_config = set_config,
    direct_voltage = direct_voltage,
    odo_period_ms = ODO_PERIOD_MS,
    telem_period_ms = TELEM_PERIOD_MS,
}

-- SIM-aware pose: returns simulated (mocked-motor) pose or real odometry.
local function pose()
    if SIM then
        return sim_odo:pose()
    else
        return odo:pose()
    end
end

require "nodes.nav" {
    model = branch(ws, "nav"),
    sim   = SIM,
    drive = drive,
    pose  = pose,
}

-- Keyboard teleop from the GUI. Shares drive() with the nav planner — only
-- one should command the robot at a time, or they will fight.
require "nodes.teleop" {
    model = branch(ws, "teleop"),
    drive = drive,
}

-- Optional external drive source: a ROS 2 stack publishing cmd_vel (Twist).
-- Only one of this and the internal nav planner should run at a time.
if ROS_PLUGIN_DIR then
    require "nodes.ros" {
        drive = drive,
        plugin_dir = ROS_PLUGIN_DIR,
    }
end

log.info("cart up: node {} on {}, ws on :{}", NODE_ID, CAN_DEVICE, WS_PORT)