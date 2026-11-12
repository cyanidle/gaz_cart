-- Offscreen proof of the shared tuner over its CAN transport: the real
-- mods/tuner.lua drives the real qml/TunerWindow.qml against a fake cart service
-- channel served in this process, so nothing needs hardware or a display.
--
--     QT_QPA_PLATFORM=offscreen radapter --gui tests/gui_tuner.lua
--
-- Covers what the tuner must do over CAN: the service-channel handshake, the
-- per-wheel state snapshot reaching the view, the message shapes for a wheel
-- speed, an open-loop voltage and a config write, the wheel selector (switching
-- parks the wheel it leaves), and the hover help on every parameter row
-- (mods/tuner_help.lua text + a sketch). Screenshots go to $TUNER_SHOT_DIR.

local REPO = SCRIPT_DIR .. "/.."
local PORT = 16081
local SHOTS = os.getenv("TUNER_SHOT_DIR")

package.path = REPO .. "/?.lua;" .. package.path
local config_defs = require "mods.config_defs"
local tuner_help = require "mods.tuner_help"

local received = {}
local server = WebsocketServer { name = "fake_cart", port = PORT, protocol = "msgpack" }
pipe(server, function(msg)
    -- a listener message is a lazy view (userdata), not a table: snapshot it
    received[#received + 1] = json_decode(tostring(msg))
    log.info("cart service <- {}", msg)
    server {
        state = {
            wheels = {
                fl = { tgt = 0.2, act = 0.18, travel = 1.0 },
                fr = { tgt = 0.0, act = 0.0, travel = 0.0 },
                rl = { tgt = 0.0, act = 0.0, travel = 0.0 },
                rr = { tgt = 0.0, act = 0.0, travel = 0.0 },
            },
            joystick = { stop = false },
        },
    }
end)

local qt = QML_Tester()
require("mods.tuner").new {
    transport = "can",
    url = "ws://127.0.0.1:" .. PORT,
    wheel = "fl",
    vmax = 10.0,
}

local function find_action(action, field, value)
    for _, msg in ipairs(received) do
        if type(msg) == "table" and msg.action == action then
            if field == nil then return msg end
            if value == nil and msg[field] ~= nil then return msg end
            if msg[field] == value then return msg end
        end
    end
    return nil
end

local function last_action_since(mark, action)
    local found
    for i = mark + 1, #received do
        if received[i].action == action then found = received[i] end
    end
    return found
end

qt:wait(2500)
assert(qt:find("paramrow_pid_kp"), "the tuner window has no parameter rows")
assert(qt:find("paramrow_wheel_diameter"), "the tuner window has no parameter rows")

-- ---- service channel ---------------------------------------------------------
assert(find_action("state"), "the tuner never asked the service channel for state")
local telemetry = tostring(qt:prop("telemetryLabel", "text"))
assert(telemetry:find("0.200") or telemetry:find("0.18"),
    "the per-wheel state snapshot did not reach the view: " .. telemetry)

-- ---- wheel selector ----------------------------------------------------------
assert(qt:prop("tuneWheelBox", "count") == 4, "the wheel selector does not list four wheels")
assert(qt:prop("tuneWheelBox", "currentIndex") == 0, "the wheel selector does not start on fl")
assert(tostring(qt:prop("tuneWheelBox", "currentText")) == "fl", "the wheel selector does not read fl")

-- ---- hover help on the parameter rows ---------------------------------------
local function hover(name)
    local c = qt:center(name)
    assert(c and #c == 2, "no such row: " .. name)
    qt:move(c[1], c[2])
    qt:wait(120)
    qt:move(c[1] + 1, c[2] + 1)
    qt:wait(250)
    return tostring(qt:prop("helpText", "text"))
end

local seen = {}
for _, key in ipairs({ "pid_kp", "pid_ki", "max_speed", "wheel_diameter", "oc_peak_a", "vel_lpf" }) do
    assert(qt:find("paramlabel_" .. key), "no row for parameter " .. key)
    local text = hover("paramlabel_" .. key)
    assert(text == tuner_help.params[key].what, "hover help drifted for " .. key)
    assert(qt:find("helpSketch"), "the help sketch is missing for " .. key)
    seen[text] = true
    qt:move(5, 5)
    qt:wait(120)
end
local distinct = 0
for _ in pairs(seen) do distinct = distinct + 1 end
assert(distinct == 6, "the parameter rows share one help text")
if SHOTS then qt:screenshot(SHOTS .. "/gui_tuner_help.png") end

-- ---- drive controls ----------------------------------------------------------
local mark = #received
qt:click_item("resetBtn")
qt:wait(300)
assert(find_action("wheel", "wheel", "fl"), "Reset did not zero the wheel")
assert(find_action("direct"), "Reset did not zero the direct voltage")

mark = #received
local tc = qt:center("tgtSlider")
qt:press(tc[1], tc[2])
qt:move(tc[1] + 60, tc[2])
qt:release(tc[1] + 60, tc[2])
qt:wait(400)
local target = last_action_since(mark, "wheel")
assert(target, "the target slider sent nothing")
assert(target.wheel == "fl", "the target went to the wrong wheel")
assert(target.v > 0.1 and target.v <= 1.0, "target out of range: " .. tostring(target.v))

-- ---- config write ------------------------------------------------------------
mark = #received
local pc = qt:center("param_pid_kp")
qt:press(pc[1], pc[2])
qt:move(pc[1] + 40, pc[2])
qt:release(pc[1] + 40, pc[2])
qt:wait(400)
local config = last_action_since(mark, "config")
assert(config, "the parameter slider sent no config write")
assert(config.id == config_defs.pid_kp.id, "the config write has the wrong id")
assert(config.wheel == "fl", "the config write went to the wrong wheel")

-- ---- switching the wheel parks the one it leaves -----------------------------
mark = #received
qt:click_item("tuneWheelBox")
qt:wait(300)
qt:key_click("Down")
qt:wait(150)
qt:key_click("Return")
qt:wait(400)
local switched = last_action_since(mark, "wheel")
assert(switched, "switching the wheel sent nothing")
assert(switched.v == 0, "the wheel being left was not zeroed: " .. json_encode(switched))
assert(tostring(qt:prop("tuneWheelBox", "currentText")) == "fr", "the wheel selector did not move to fr")
assert(tostring(qt:prop("sourceLabel", "text")):find("fr"), "the source line still names the old wheel")

if SHOTS then qt:screenshot(SHOTS .. "/gui_tuner_can.png") end
print("gui_tuner: ok")
shutdown()
