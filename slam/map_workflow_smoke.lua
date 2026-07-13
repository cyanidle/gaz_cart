-- PNG round-trip + unknown-aware global/local planner regression.
--   build/bin/radapter slam/map_workflow_smoke.lua

local os = require "os"

load_plugin(SCRIPT_DIR .. "/../build/nav/libgaz_nav")

local width, height, resolution = 7, 5, 0.5
local origin_x, origin_y = -1.5, -1.0
local cells = {}
for y = 0, height - 1 do
    for x = 0, width - 1 do
        -- A short known corridor ends at x=3; x=4..6 is exploration space.
        local value = (y == 2 and x >= 1 and x <= 3) and 0 or 255
        cells[#cells + 1] = string.char(value)
    end
end
local cell_bytes = table.concat(cells)
local grid = bytes(string.pack("<i4i4i4fff", 0x47414D50, width, height,
    resolution, origin_x, origin_y) .. cell_bytes)

local source = CostmapServer {
    update_rate_ms = 20,
    inflate = { robot_safe_radius = 0 },
    inflate_static = { robot_safe_radius = 0 },
}
source { static_map = grid }
local png = source:DumpMap()
assert(png:str():sub(1, 8) == "\137PNG\r\n\26\n", "DumpMap did not return PNG")

local restored = CostmapServer {
    update_rate_ms = 20,
    inflate = { robot_safe_radius = 0 },
    inflate_static = { robot_safe_radius = 0 },
}
local global = GlobalPlanner {
    update_rate_ms = 20,
    max_points = 500,
    a_star = { unknown_cost = 5 },
}
local local_planner = LocalPlanner {
    tick_rate = 20,
    path = {
        approximation_max_cost = 30,
        fallback_min_points_count = 1,
        unknown_inflation_radius = 0.75,
    },
}
pipe(restored, global)
pipe(restored, local_planner)

local roundtrip_ok, global_unknown_ok, local_frontier_ok = false, false, false
pipe(restored, function(msg)
    if not msg.costmap then return end
    local raw = msg.costmap:str()
    local _, w, h, res, ox, oy = string.unpack("<i4i4i4fff", raw)
    roundtrip_ok = w == width and h == height
        and math.abs(res - resolution) < 1e-6
        and math.abs(ox - origin_x) < 1e-6
        and math.abs(oy - origin_y) < 1e-6
        and raw:sub(25) == cell_bytes
end)

pipe(global, function(msg)
    if not msg.path or #msg.path == 0 then return end
    local last = msg.path[#msg.path]
    global_unknown_ok = math.abs(last.x - 1.5) < 1e-6 and math.abs(last.y) < 1e-6
end)

pipe(local_planner, function(msg)
    if not msg.local_target then return end
    -- It must not shortcut to the unknown end of the global path. The nearest
    -- known waypoint is selected while lidar gets a chance to reveal space.
    local_frontier_ok = msg.local_target.x <= -0.5 + 1e-6
end)

restored:LoadMap(png)
after(60, function()
    local position = { x = -1.0, y = 0.0, theta = 0 }
    global {
        position = position,
        target = { x = 1.5, y = 0.0, theta = 0 },
    }
    local_planner {
        position = position,
        path = {
            { x = -1.0, y = 0.0, theta = 0 },
            { x = -0.5, y = 0.0, theta = 0 },
            { x =  0.0, y = 0.0, theta = 0 },
            { x =  0.5, y = 0.0, theta = 0 },
            { x =  1.0, y = 0.0, theta = 0 },
        },
    }
end)

each(20, function()
    if roundtrip_ok and global_unknown_ok and local_frontier_ok then
        log.info("map workflow smoke OK (PNG round-trip, global exploration, local frontier)")
        shutdown()
    end
end)

after(3000, function()
    log.error("map workflow smoke FAILED (roundtrip={}, global={}, local={})",
        roundtrip_ok, global_unknown_ok, local_frontier_ok)
    os.exit(1)
end)
