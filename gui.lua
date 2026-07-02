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
local client = WebsocketClient { url = url, protocol = "json" }

pipe(client.events, function(ev) log.info("ws {}: {}", url, ev) end)

local view = QML { url = "./qml/WheelConfig.qml" }
local nav_view = QML { url = "./qml/NavView.qml" }

-- QML "send" events -> websocket client -> main.lua server.
pipe(view, function(msg)
    log("From UI: {}", msg)
    return msg
end,
client)

-- Flatten the keyed config map into an array ordered by id (the Repeater needs
-- a list) and hand it to the model.
local params = {}
for key, c in pairs(config_defs) do
    params[#params + 1] = {
        key = key, id = c.id, label = c.label, unit = c.unit, default = c.default,
    }
end
table.sort(params, function(a, b) return a.id < b.id end)
view { params = params }

-- Route chart + odometry telemetry from the server into the model.
pipe(client, function(msg)
    if msg.chart then
        view { chart = msg.chart }
    end
    if msg.odom then
        local o = msg.odom
        view { odomText = string.format("odom  x=%.2f  y=%.2f  theta=%.1f deg  v=%.2f  omega=%.2f",
            o.x or 0, o.y or 0, math.deg(o.theta or 0), o.v or 0, o.omega or 0) }
    end
end)

pipe(client, unwrap("nav"), nav_view)
pipe(nav_view, wrap("nav"), client)