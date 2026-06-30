-- =============================================================================
--  Cart runtime — drives the four wheel modules over Cyphal and integrates
--  differential-drive odometry. Run on the Raspberry Pi:
--
--      radapter main.lua
--
--  It also opens a websocket (WS_PORT) that gui.lua — running on another host —
--  connects to in order to tune each module's runtime config live.
--
--  The returned table (.drive/.stop/.set_config/.set_speed/.odo/...) is handy
--  from an interactive session or another script that `require`s this one.
-- =============================================================================

local Odometry    = require "odometry"
local config_defs = require "config_defs"
local socket      = require "socket"

-- ---- Tunables --------------------------------------------------------------

local CAN_DEVICE    = "can0"   -- socketcan interface the modules sit on
local NODE_ID       = 100      -- this Pi's Cyphal node id
local WS_PORT       = 6080     -- config-GUI websocket (gui.lua connects here)
local TRACK_WIDTH   = 0.30     -- distance between left and right wheels, m
local ODO_PERIOD_MS = 20       -- odometry integration period
local TELEM_PERIOD_MS = 100    -- chart telemetry stream period

-- Wheel module node ids (set by each module's DIP switches).
local NODES = { fl = 1, fr = 2, rl = 3, rr = 4 }

-- Cyphal port bases (see CLAUDE.md / app.cpp). Per-module port = base + node id.
local PORT = {
    linear_speed = 7300,  -- module -> Pi, m/s
    speed_cmd    = 4000,  -- Pi -> module, m/s
    direct_cmd   = 4050,  -- Pi -> module, open-loop voltage (V)
    config       = 4100,  -- Pi -> module, { id, num, den }
}

local WHEELS = { "fl", "fr", "rl", "rr" }

-- ---- CAN + Cyphal ----------------------------------------------------------

local can = CAN { plugin = "socketcan", device = CAN_DEVICE }

local subscribe, publish = {}, {}
for _, w in ipairs(WHEELS) do
    local node = NODES[w]
    subscribe["spd_" .. w] = { type = "uavcan.primitive.scalar.Real32.1.0",   port = PORT.linear_speed + node }
    publish  ["cmd_" .. w] = { type = "uavcan.primitive.scalar.Real32.1.0",   port = PORT.speed_cmd    + node }
    publish  ["dir_" .. w] = { type = "uavcan.primitive.scalar.Real32.1.0",   port = PORT.direct_cmd   + node }
    publish  ["cfg_" .. w] = { type = "uavcan.primitive.array.Integer32.1.0", port = PORT.config       + node }
end

local cyphal = Cyphal {
    can       = can,
    node_id   = NODE_ID,
    subscribe = subscribe,
    publish   = publish,
}

-- ---- Odometry --------------------------------------------------------------

local odo = Odometry.new { trackWidth = TRACK_WIDTH }

-- latest wheel linear velocity (m/s), refreshed from Cyphal
local wheel_v = { fl = 0.0, fr = 0.0, rl = 0.0, rr = 0.0 }
for _, w in ipairs(WHEELS) do
    on(cyphal, "spd_" .. w, function(msg)
        wheel_v[w] = msg.value or 0.0
    end)
end

local last_t
each(ODO_PERIOD_MS, function()
    local now = socket.gettime()
    local dt  = last_t and (now - last_t) or 0.0
    last_t = now
    odo:update(wheel_v.fl, wheel_v.fr, wheel_v.rl, wheel_v.rr, dt)
end)

-- ---- Driving ---------------------------------------------------------------

-- Per-wheel speed targets (m/s), set by the GUI or programmatically.
local wheel_tgt = { fl = 0.0, fr = 0.0, rl = 0.0, rr = 0.0 }

--- Set target speed for one wheel or all four, and publish to Cyphal.
local function set_speed(wheel, value)
    local targets = (not wheel or wheel == "all") and WHEELS or { wheel }
    local msg = {}
    for _, w in ipairs(targets) do
        wheel_tgt[w] = value
        msg["cmd_" .. w] = { value = value }
    end
    cyphal(msg)
end

--- Command a body twist: forward speed v (m/s) and yaw rate omega (rad/s).
local function drive(v, omega)
    local half = TRACK_WIDTH * 0.5
    local vL, vR = v - omega * half, v + omega * half
    wheel_tgt.fl, wheel_tgt.rl = vL, vL
    wheel_tgt.fr, wheel_tgt.rr = vR, vR
    cyphal {
        cmd_fl = { value = vL }, cmd_rl = { value = vL },
        cmd_fr = { value = vR }, cmd_rr = { value = vR },
    }
end

local function stop() drive(0.0, 0.0) end

--- Open-loop voltage to one wheel ("fl".."rr"), for bring-up / tuning.
local function direct_voltage(wheel, volts)
    cyphal { ["dir_" .. wheel] = { value = volts } }
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

local def_by_id = {}
for _, c in ipairs(config_defs) do def_by_id[c.id] = c end

--- Push one config value (selected by id) to a wheel ("fl".."rr") or "all".
local function set_config(wheel, id, value)
    local def = def_by_id[id]
    if not def then
        log.warn("set_config: unknown config id {}", id)
        return
    end
    local num, den = to_rational(value)
    local targets = (not wheel or wheel == "all") and WHEELS or { wheel }
    for _, w in ipairs(targets) do
        cyphal { ["cfg_" .. w] = { value = { id, num, den } } }
    end
    log.info("config '{}' = {} ({}/{}) -> {}", def.key, value, num, den, wheel or "all")
end

-- ---- Config-GUI websocket --------------------------------------------------

local ws = WebsocketServer { port = WS_PORT, protocol = "json" }

pipe(ws, function(msg)
    if type(msg) ~= "table" then return end
    -- Config edit from the form fields.
    if msg.id ~= nil and msg.value ~= nil then
        set_config(msg.wheel or "all", math.floor(msg.id), tonumber(msg.value))
    end
    -- Per-wheel speed target from the tuning panel.
    if msg.action == "set_speed" and msg.wheel then
        set_speed(msg.wheel, tonumber(msg.value) or 0.0)
    end
end)

-- Stream chart data (per-wheel target + actual speed) to the GUI.
each(TELEM_PERIOD_MS, function()
    local ch = {}
    for _, w in ipairs(WHEELS) do
        ch[w] = { tgt = wheel_tgt[w], act = wheel_v[w] }
    end
    local x, y, th = odo:pose()
    local v, omega = odo:velocity()
    ws {
        chart = ch,
        odom = { x = x, y = y, theta = th, v = v, omega = omega },
    }
end)

log.info("cart up: node {} on {}, ws on :{}", NODE_ID, CAN_DEVICE, WS_PORT)

-- ---- Public API ------------------------------------------------------------

return {
    cyphal = cyphal, can = can, odo = odo, wheel_v = wheel_v,
    drive = drive, stop = stop, direct_voltage = direct_voltage,
    set_speed = set_speed, set_config = set_config,
}
