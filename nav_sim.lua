-- Standalone sim for the navigation stack: run this, then connect gui.lua
-- (pointed at ws://localhost:6080) to see the NAV tab with a simulated robot
-- and a raycasting lidar.
--
-- Usage:
--   build/bin/radapter nav_sim.lua [ws_port]

local WS_PORT = tonumber(args[1] or "6080")

load_plugin(SCRIPT_DIR .. "/../build/nav/libgaz_nav")

local ws = WebsocketServer {
    name = "main_ws",
    port = WS_PORT,
    protocol = "msgpack",
}

local model = node(ws, "nav")
require "nodes.nav" { model = model, sim = true }

log.info("nav sim up on ws://localhost:{}", WS_PORT)
