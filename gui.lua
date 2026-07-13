-- =============================================================================
--  Config + tuning GUI for the cart — runs on a laptop / desktop and connects
--  to main.lua's websocket to tune each wheel module's runtime config live.
--
--      radapter --gui gui.lua [ws://HOST:PORT]
--
--  Defaults to ws://127.0.0.1:6080.  Pick a wheel (or "all"), type a value
--  and press Enter — it goes to the module(s) immediately.  The chart shows
--  target vs actual speed for the selected wheel so you can tune the velocity
--  PID with instant visual feedback.
-- =============================================================================

local config_defs = require "mods.config_defs"

local url    = args[1] or "ws://127.0.0.1:6080"
local client = WebsocketClient { url = url, protocol = "msgpack" }

-- ---- Sent-config persistence -------------------------------------------------
-- The wheel modules keep runtime config in RAM, so a robot power-cycle reverts
-- them to cart.lua's defaults. Remember every config value sent from the UI
-- (keyed by wheel + id), persist the set as JSON next to this script, and
-- replay it on every (re)connection to the robot.

local STATE_FILE = SCRIPT_DIR .. "/config.json"

local saved = {} -- ["<wheel>:<id>"] = { wheel = ..., id = ..., value = ... }
do
    local f = io.open(STATE_FILE, "r")
    if f then
        saved = json_decode(f:read("a"))
        f:close()
        log.info("loaded saved config from {}", STATE_FILE)
    end
end

local function remember(cmd)
    local id = math.floor(cmd.id)
    if cmd.wheel == "all" then
        -- an "all" write supersedes any per-wheel override of the same id
        for k, e in pairs(saved) do
            if e.id == id then saved[k] = nil end
        end
    end
    saved[cmd.wheel .. ":" .. id] = { wheel = cmd.wheel, id = id, value = cmd.value }
    local f = assert(io.open(STATE_FILE, "w"))
    f:write(json_encode(saved, { pretty = true }))
    f:close()
end

local function resend_config()
    local entries = {}
    for _, e in pairs(saved) do entries[#entries + 1] = e end
    -- "all" first so surviving per-wheel overrides re-apply on top of it
    table.sort(entries, function(a, b)
        if (a.wheel == "all") ~= (b.wheel == "all") then return a.wheel == "all" end
        if a.wheel ~= b.wheel then return a.wheel < b.wheel end
        return a.id < b.id
    end)
    for _, e in ipairs(entries) do
        log.info("resend config id {} = {} -> {}", e.id, e.value, e.wheel)
        client { odo = { action = "config", wheel = e.wheel, id = e.id, value = e.value } }
    end
end

pipe(client.events, function(ev)
    log.info("ws {}: {}", url, ev)
    if ev.state == "ConnectedState" then
        resend_config()
    end
end)

local cfgView    = QML { url = "./qml/WheelConfigWindow.qml" }
local navView    = QML { url = "./qml/NavWindow.qml" }
local teleopView = QML { url = "./qml/TeleopWindow.qml" }

-- QML "send" events -> websocket client -> main.lua server.
-- Each view sends through child model branch() nodes, so every outgoing
-- message is already wrapped as {odo = ...} or {nav = ...} or {teleop = ...}.
-- Config sends are additionally remembered for replay / persistence.
local function outgoing(msg)
    log("From UI: {}", msg)
    if msg.odo and msg.odo.action == "config" then
        remember(msg.odo)
    end
    return msg
end
pipe(cfgView,    outgoing, client)
pipe(teleopView, client)

-- Map files live on the GUI machine, while mapping runs on the cart. Keep file
-- I/O local and transfer the PNG payload as MsgPack binary over the websocket.
local pending_map_dump
local function local_file_path(url)
    local path = tostring(url):gsub("^file://", "")
    return (path:gsub("%%(%x%x)", function(hex)
        return string.char(tonumber(hex, 16))
    end))
end
local function nav_status(text)
    navView { nav = { map_io_status = text } }
end
local function nav_outgoing(msg)
    local nav = msg.nav
    if not nav then return msg end
    if nav.load_map_file then
        local path = local_file_path(nav.load_map_file)
        local file, err = io.open(path, "rb")
        if not file then
            nav_status("Load failed: " .. tostring(err))
            return nil
        end
        local payload = bytes(file:read("a"))
        file:close()
        nav_status("Uploading " .. path .. " …")
        client { nav = { load_map = payload } }
        return nil
    end
    if nav.dump_map_file then
        pending_map_dump = local_file_path(nav.dump_map_file)
        nav_status("Requesting map …")
        client { nav = { dump_map = true } }
        return nil
    end
    return msg
end
pipe(navView, nav_outgoing, client)

-- Flatten the keyed config map into an array ordered by id (the Repeater needs
-- a list) and hand it to the model. A saved "all" value replaces the default,
-- so the form shows what the modules will actually be running after replay.
local params = {}
for key, c in pairs(config_defs) do
    local all = saved["all:" .. c.id]
    params[#params + 1] = {
        key = key, id = c.id, label = c.label, unit = c.unit,
        default = all and all.value or c.default,
    }
end
table.sort(params, function(a, b) return a.id < b.id end)
cfgView { odo = { params = params } }

-- Route telemetry from the server into the model. Everything arrives already
-- namespaced by node key ({odo = {chart, odom}}, {nav = {costmap, path, ...}});
-- applyIncoming routes each inner map to the matching child GuiModel node,
-- where WheelConfig / NavView listen. Only odomText needs local formatting.
pipe(client, function(msg)
    if msg.odo and msg.odo.odom then
        local o = msg.odo.odom
        msg.odo.odomText = fmt("odom  x={:.2f}  y={:.2f}  theta={:.1f} deg  v={:.2f}  omega={:.2f}",
            o.x or 0, o.y or 0, math.deg(o.theta or 0), o.v or 0, o.omega or 0)
    end
    if msg.nav and msg.nav.map_image then
        if pending_map_dump then
            local file, err = io.open(pending_map_dump, "wb")
            if file then
                file:write(msg.nav.map_image:str())
                file:close()
                nav_status("Map saved to " .. pending_map_dump)
            else
                nav_status("Save failed: " .. tostring(err))
            end
            pending_map_dump = nil
        end
        msg.nav.map_image = nil
    end
    cfgView(msg)
    navView(msg)
    teleopView(msg)
end)
