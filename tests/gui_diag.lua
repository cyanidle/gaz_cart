-- Offscreen proof of the diagnostics window (qml/DiagWindow.qml → qml/Diag.qml):
-- the tuner opens it beside itself, the counters/trace buttons live there instead
-- of in the tuner window, and recording can be started and stopped from it.
--
--     QT_QPA_PLATFORM=offscreen radapter --gui tests/gui_diag.lua
--
-- Over the cart's service channel a session is a stream of `{state}` snapshots,
-- so that is what the window records: one JSON object per line. The module's own
-- `#` lines (counters, the loop trace) exist on the serial bench channel only, so
-- over CAN the counters buttons must stay out of reach.

local REPO = SCRIPT_DIR .. "/.."
local PORT = 16084
local LOG = os.tmpname()

package.path = REPO .. "/?.lua;" .. package.path

local server = WebsocketServer { name = "fake_cart", port = PORT, protocol = "msgpack" }
pipe(server, function(msg)
    log.info("cart service <- {}", msg)
end)

-- the service channel is a snapshot stream: push one every 50 ms so a recording
-- has something to write whatever the tuner asks for
each(50, function()
    server {
        state = {
            wheels = {
                fl = { tgt = 0.2, act = 0.18, travel = 1.0 },
                fr = { tgt = 0.0, act = 0.0, travel = 0.0 },
                rl = { tgt = 0.0, act = 0.0, travel = 0.0 },
                rr = { tgt = 0.0, act = 0.0, travel = 0.0 },
            },
            joystick = { stopped = false },
        },
    }
end)

local qt = QML_Tester()
require("mods.tuner").new {
    transport = "can", url = "ws://127.0.0.1:" .. PORT, wheel = "fl", vmax = 10.0,
}

qt:wait(2500)

-- ---- the window is there, beside the tuner -----------------------------------
local titles = qt:windows()
local function list(windows)
    local parts = {}
    for _, title in ipairs(windows) do parts[#parts + 1] = tostring(title) end
    return table.concat(parts, " | ")
end
print("windows: " .. list(titles))
assert(#titles == 2, "the tuner must open two windows, got: " .. list(titles))

local diag_index
for i, title in ipairs(titles) do
    if tostring(title) == "Диагностика" or tostring(title) == "Diagnostics" then diag_index = i - 1 end
end
assert(diag_index, "no diagnostics window among: " .. list(titles))
qt:set_window(diag_index)

assert(qt:find("outputArea"), "the diagnostics window has no output scrollback")
assert(qt:find("logStartBtn") and qt:find("logStopBtn"), "no recording controls")
assert(qt:find("diagStatusLabel"), "no status line")

-- ---- counters belong to the serial channel only -------------------------------
assert(qt:prop("countersBtn", "visible") == false,
    "the counters button is reachable over the service channel")
assert(qt:prop("logStartBtn", "enabled") == true, "recording cannot be started")

-- ---- recording ---------------------------------------------------------------
qt:set_prop("logPathField", "text", LOG)
qt:click_item("logStartBtn")
qt:wait(600)

assert(qt:prop("logStopBtn", "enabled") == true, "the recording never started")
local status = tostring(qt:prop("diagStatusLabel", "text"))
assert(status:find("пишу", 1, true) or status:find("recording to", 1, true),
    "the status line does not say where it records: " .. status)

local function lines(path)
    local handle = io.open(path, "r")
    if not handle then return {} end
    local out = {}
    for line in handle:lines() do out[#out + 1] = line end
    handle:close()
    return out
end

local recorded = lines(LOG)
assert(#recorded > 0, "nothing was recorded")
assert(json_decode(recorded[#recorded]).state ~= nil,
    "a recorded line is not a service-channel snapshot: " .. tostring(recorded[#recorded]))

qt:click_item("logStopBtn")
qt:wait(300)
local frozen = #lines(LOG)
qt:wait(400)
assert(#lines(LOG) == frozen,
    "the recording kept growing after Stop: " .. #lines(LOG) .. " vs " .. frozen)
status = tostring(qt:prop("diagStatusLabel", "text"))
assert(status:find("выключена", 1, true) or status:find("closed", 1, true),
    "the status line does not report the stop: " .. status)

print(("gui_diag: ok (recorded %d snapshots to %s)"):format(frozen, LOG))
os.remove(LOG)
shutdown()
