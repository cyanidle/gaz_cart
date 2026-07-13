-- Variable-origin GAMP -> CostmapServer -> GlobalPlanner regression.
--   build/bin/radapter slam/variable_grid_smoke.lua

local os = require "os"

load_plugin(SCRIPT_DIR .. "/../build/nav/libgaz_nav")

local width, height, resolution = 5, 5, 0.5
local origin_x, origin_y = -1.0, -1.0
local cells = {}
for y = 0, height - 1 do
    for x = 0, width - 1 do
        local value = (x == 0 or y == 0 or x == width - 1 or y == height - 1) and 255 or 0
        if x == 2 and y == 2 then value = 100 end
        cells[#cells + 1] = string.char(value)
    end
end
local grid = bytes(string.pack("<i4i4i4fff", 0x47414D50, width, height,
    resolution, origin_x, origin_y) .. table.concat(cells))

local costmap = CostmapServer {
    update_rate_ms = 20,
    inflate = { robot_safe_radius = 0 },
    inflate_static = { robot_safe_radius = 0 },
}
local planner = GlobalPlanner { update_rate_ms = 20, max_points = 100 }
pipe(costmap, planner)

local geometry_ok, unknown_ok = false, false
pipe(costmap, function(msg)
    if not msg.costmap then return end
    local raw = msg.costmap:str()
    local _, w, h, res, ox, oy = string.unpack("<i4i4i4fff", raw)
    geometry_ok = w == width and h == height and math.abs(res - resolution) < 1e-6
        and math.abs(ox - origin_x) < 1e-6 and math.abs(oy - origin_y) < 1e-6
    unknown_ok = raw:byte(25) == 255
end)

pipe(planner, function(msg)
    if not msg.path or #msg.path == 0 then return end
    local last = msg.path[#msg.path]
    local reaches_outside = false
    for _, point in ipairs(msg.path) do
        reaches_outside = reaches_outside or point.x > 1.5
    end
    if geometry_ok and unknown_ok
            and reaches_outside
            and math.abs(last.x - 2.0) < 1e-6 and math.abs(last.y - 0.5) < 1e-6 then
        log.info("variable-grid outside-target smoke OK ({} path points)", #msg.path)
        shutdown()
    end
end)

costmap { static_map = grid }
after(50, function()
    planner {
        position = { x = -0.5, y = -0.5, theta = 0 },
        -- The rendered grid ends at x=1.5. Space beyond it must be planned as
        -- unexplored instead of being rejected as outside the A* domain.
        target = { x = 2.0, y = 0.5, theta = 0 },
    }
end)

after(3000, function()
    log.error("variable-grid planner smoke FAILED (geometry={}, unknown={})",
        geometry_ok, unknown_ok)
    os.exit(1)
end)
