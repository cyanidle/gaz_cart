-- =============================================================================
--  Cyphal proof on Qt's virtualcan backend — no CAN hardware, no root.
--
--      radapter debug/cyphal_vcan.lua
--
--  Four simulated wheel modules (node ids 1..4, the cart's DIP ids) publish the
--  travel (7100+n) and linear-speed (7300+n) topics; the cart node (id 100)
--  subscribes and publishes the speed (4000+n), direct-voltage (4050+n) and
--  config (4100+n) commands exactly as cart.lua does. The real nodes/odo.lua is
--  wired on top of it, so the odometry integration runs for real. A raw CAN
--  worker on the same virtual bus records what is actually on the wire, so the
--  CAN FD requirement is measured, not assumed.
-- =============================================================================

package.path = SCRIPT_DIR .. "/../?.lua;" .. package.path

local socket      = require "socket"
local config_defs = require "mods.config_defs"
local Rational    = require "mods.rational"
local odo_node    = require "nodes.odo"

local DEVICE = "gaz_cyphal_proof"
local WHEELS = { fl = 1, fr = 2, rl = 3, rr = 4 }
local PORT = { travel = 7100, linear_speed = 7300, speed_cmd = 4000, direct_cmd = 4050, config = 4100 }
local CART_NODE = 100
local TRACK_WIDTH = 0.30
local WHEEL_SPEED = 0.3 -- m/s the simulated modules report, forward

local failures = {}
local function check(ok, what)
    if ok then
        print(("  ok   %s"):format(what))
    else
        print(("  FAIL %s"):format(what))
        failures[#failures + 1] = what
    end
end

-- The bus rate the firmware really runs: 1 Mbit nominal, 8 Mbit data (see
-- firmware/Src/fdcan.c). virtualcan ignores the numbers, but the FD flag has to
-- be on everywhere or the frames are classic and nothing multi-frame works.
local function new_can()
    return CAN {
        plugin = "virtualcan", device = DEVICE,
        bitrate = 1000000, data_bitrate = 8000000, can_fd = true,
    }
end

-- ---- the wire ----------------------------------------------------------------
local wire = new_can()
local frames = {}
pipe(wire, function(frame) frames[#frames + 1] = frame end)

-- ---- the cart side: cart.lua's port wiring, unchanged ------------------------
local subscribe, publish = {}, {}
for w, nid in pairs(WHEELS) do
    subscribe["trv_" .. w] = { type = "uavcan.si.unit.length.Scalar.1.0",  port = PORT.travel       + nid }
    subscribe["spd_" .. w] = { type = "uavcan.primitive.scalar.Real32.1.0", port = PORT.linear_speed + nid }
    publish  ["cmd_" .. w] = { type = "uavcan.primitive.scalar.Real32.1.0", port = PORT.speed_cmd    + nid }
    publish  ["dir_" .. w] = { type = "uavcan.primitive.scalar.Real32.1.0", port = PORT.direct_cmd   + nid }
    publish  ["cfg_" .. w] = { type = "uavcan.primitive.array.Integer32.1.0", port = PORT.config     + nid }
end

local cart = Cyphal {
    can = new_can(), node_id = CART_NODE,
    node_info = { name = "gaz-cart-vcan-proof", software_version = { major = 7, minor = 3 } },
    subscribe = subscribe, publish = publish,
}

local wheel_v = { fl = 0.0, fr = 0.0, rl = 0.0, rr = 0.0 }
local wheel_travel = { fl = 0.0, fr = 0.0, rl = 0.0, rr = 0.0 }
local seen = { trv = 0, spd = 0 }
for w in pairs(WHEELS) do
    on(cart, "spd_" .. w, function(msg)
        wheel_v[w] = msg.value or 0.0
        seen.spd = seen.spd + 1
    end)
    on(cart, "trv_" .. w, function(msg)
        wheel_travel[w] = msg.meter or 0.0
        seen.trv = seen.trv + 1
    end)
end

-- ---- the modules: what a wheel firmware publishes and accepts ----------------
local modules = {}
for w, nid in pairs(WHEELS) do
    local m = { wheel = w, node_id = nid, travel = 0.0, cmd = {}, dir = {}, cfg = {} }
    m.node = Cyphal {
        can = new_can(),
        node_id = nid,
        publish = {
            travel = { type = "uavcan.si.unit.length.Scalar.1.0",  port = PORT.travel       + nid },
            linear = { type = "uavcan.primitive.scalar.Real32.1.0", port = PORT.linear_speed + nid },
        },
        subscribe = {
            cmd = { type = "uavcan.primitive.scalar.Real32.1.0",   port = PORT.speed_cmd  + nid },
            dir = { type = "uavcan.primitive.scalar.Real32.1.0",   port = PORT.direct_cmd + nid },
            cfg = { type = "uavcan.primitive.array.Integer32.1.0", port = PORT.config     + nid },
        },
    }
    on(m.node, "cmd", function(msg) m.cmd[#m.cmd + 1] = msg.value end)
    on(m.node, "dir", function(msg) m.dir[#m.dir + 1] = msg.value end)
    on(m.node, "cfg", function(msg) m.cfg[#m.cfg + 1] = msg.value end)
    modules[nid] = m
end

-- every module reports the same forward speed, so the cart should drive straight
-- (20 ms, the period nodes/odo.lua integrates over: a slower stream would leave
-- integration periods with no new travel, and the instantaneous twist would
-- alternate between the real speed and zero)
each(20, function()
    for _, m in pairs(modules) do
        m.travel = m.travel + WHEEL_SPEED * 0.02
        m.node { travel = { meter = m.travel }, linear = { value = WHEEL_SPEED } }
    end
end)

-- ---- cart.lua's publish helpers, unchanged ----------------------------------
local function wheel_command(wheel_m_s)
    local full_scale = config_defs.max_speed.default
    return math.max(-1.0, math.min(1.0, full_scale > 0 and (wheel_m_s / full_scale) or 0.0))
end

local function set_speed(wheel, value)
    local msg = {}
    if not wheel or wheel == "all" then
        for w in pairs(WHEELS) do msg["cmd_" .. w] = { value = wheel_command(value) } end
    else
        msg["cmd_" .. wheel] = { value = wheel_command(value) }
    end
    cart(msg)
end

local function direct_voltage(wheel, volts)
    local msg = {}
    if not wheel or wheel == "all" then
        for w in pairs(WHEELS) do msg["dir_" .. w] = { value = volts } end
    else
        msg["dir_" .. wheel] = { value = volts }
    end
    cart(msg)
end

local function set_config(wheel, id, value)
    local num, den = Rational.from_number(value)
    local msg = {}
    if not wheel or wheel == "all" then
        for w in pairs(WHEELS) do msg["cfg_" .. w] = { value = { id, num, den } } end
    else
        msg["cfg_" .. wheel] = { value = { id, num, den } }
    end
    cart(msg)
end

-- ---- the real odometry node on top of it ------------------------------------
local telemetry = {}
local function model(msg) telemetry[#telemetry + 1] = msg end

local odo = odo_node {
    model = model,
    track_width = TRACK_WIDTH,
    wheels = function()
        local out = {}
        for w in pairs(WHEELS) do out[w] = { tgt = 0.0, act = wheel_v[w] } end
        return out
    end,
    travel = function() return wheel_travel end,
    set_speed = set_speed,
    set_config = set_config,
    direct_voltage = direct_voltage,
    odo_period_ms = 20,
    telem_period_ms = 50,
}

-- ---- telemetry up ------------------------------------------------------------
local t_start = socket.gettime()
sleep(2000)
print(("wheels reporting: %d travel / %d speed messages"):format(seen.trv, seen.spd))
check(seen.trv > 100 and seen.spd > 100, "all four modules' travel + linear speed reach the cart")
check(math.abs(wheel_v.fl - WHEEL_SPEED) < 1e-3, "wheel speed value decodes (fl = what the module sent)")
check(wheel_travel.fl > 0.4, "wheel travel accumulates (fl)")

local x, y, theta = odo:pose()
local v, omega = odo:velocity()
local elapsed = socket.gettime() - t_start
print(("odometry: x=%.3f y=%.3f theta=%.3f v=%.3f omega=%.3f over %.2f s")
    :format(x, y, theta, v, omega, elapsed))
check(x > 0.4 and math.abs(y) < 0.02 and math.abs(theta) < 0.05, "four equal wheels integrate to a straight line")
-- the per-period twist aliases against the 50 Hz stream (a period sees 0 or 2
-- steps), so the integrated pose is what has to match the wheel speed
check(math.abs(x / elapsed - WHEEL_SPEED) < 0.03,
    ("the integrated pose advances at the wheel speed (%.3f m/s)"):format(x / elapsed))
check(#telemetry > 5, "the odometry node streams telemetry")

-- ---- telemetry down: the cart's command topics ------------------------------
local target = 0.25
set_speed("fl", target)
direct_voltage("rl", 1.5)
set_config("fr", config_defs.pid_kp.id, 27.57)
sleep(500)

check(#modules[1].cmd > 0 and math.abs(modules[1].cmd[#modules[1].cmd] - wheel_command(target)) < 1e-3,
    "speed command 4000+1 arrives normalized")
check(#modules[1].cmd == 1 and #modules[2].cmd == 0,
    "a per-wheel command reaches that wheel only")
check(#modules[3].dir > 0 and math.abs(modules[3].dir[#modules[3].dir] - 1.5) < 1e-3,
    "direct voltage 4050+3 arrives in volts")
local cfg = modules[2].cfg[#modules[2].cfg]
check(cfg and math.floor(cfg[1]) == config_defs.pid_kp.id
    and math.abs(cfg[2] / cfg[3] - 27.57) < 1e-3,
    ("config 4100+2 arrives as {id, num, den} = %s"):format(cfg and table.concat(cfg, ", ") or "nil"))

set_speed("all", 0.0)
sleep(300)
local all_reached = true
for _, m in pairs(modules) do
    if m.cmd[#m.cmd] ~= 0.0 then all_reached = false end
end
check(all_reached, "an all-wheel command reaches every module")

-- ---- Cyphal itself: multi-frame transfers in both directions ---------------
-- the answer has to carry the server's node info, not just arrive: GetInfo's
-- request extent is zero, and a response subscription sized from it truncates
-- the payload to defaults
local res, err = modules[1].node:Request(
    { type = "uavcan.node.GetInfo.1.0", server = CART_NODE, port = 430 }, {}):await()
check(res ~= nil and err == nil, ("module->cart GetInfo answered (%s)"):format(tostring(err)))
check(res and res.name == "gaz-cart-vcan-proof",
    ("GetInfo carries the node name (%s)"):format(res and tostring(res.name) or "no answer"))
check(res and res.software_version and res.software_version.major == 7
    and res.software_version.minor == 3, "GetInfo carries the software version")
local res2, err2 = cart:Request(
    { type = "uavcan.node.GetInfo.1.0", server = 2, port = 430 }, {}):await()
check(res2 ~= nil and err2 == nil, ("cart->module GetInfo answered (%s)"):format(tostring(err2)))

-- ---- what is really on the wire --------------------------------------------
local fd, fd_brs, big, classic = 0, 0, 0, 0
for _, f in ipairs(frames) do
    if f.can_fd then
        fd = fd + 1
        if f.bitrate_switch then fd_brs = fd_brs + 1 end
        if #f.payload > 8 then big = big + 1 end
    else
        classic = classic + 1
    end
end
print(("%d frames on the wire: %d FD (%d with BRS, %d longer than 8 bytes), %d classic")
    :format(#frames, fd, fd_brs, big, classic))
check(#frames > 200, "the bus carries traffic")
check(fd > 0, "Cyphal frames carry the FD flag")
check(fd_brs > 0, "Cyphal frames use the FD data phase (BRS)")
check(big > 0, "multi-frame transfers happen (GetInfo responses exceed one classic frame)")
check(classic == 0, "no classic frame is emitted by the Cyphal workers")

print()
if #failures == 0 then
    print("cyphal_vcan: ok")
    os.exit(0)
end
print(("cyphal_vcan: %d FAILED"):format(#failures))
for _, what in ipairs(failures) do print("  - " .. what) end
os.exit(1)
