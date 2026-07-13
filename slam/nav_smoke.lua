-- End-to-end simulated lidar -> SLAM -> costmap smoke test.
--   build/bin/radapter slam/nav_smoke.lua

local os = require "os"

load_plugin(SCRIPT_DIR .. "/../build/nav/libgaz_nav")
load_plugin(SCRIPT_DIR .. "/../build/slam/libgaz_slam")

local lidar = Lidar {
    sim = {
        beams = 180,
        noise = 0.0,
        obstacles = {
            { x = 1.6, y = 1.0, radius = 0.15 },
            { x = 1.2, y = 1.8, radius = 0.12 },
            { x = 2.0, y = 1.6, radius = 0.10 },
        },
    },
    scan_frequency = 10,
}
local slam = Slam {
    map = {
        resolution = 0.03,
        update_interval_ms = 100, min_pass_through = 0,
    },
    mapper = {
        minimum_time_interval = 0.05,
        minimum_travel_distance = 0.02,
        do_loop_closing = false,
    },
}
local costmap = CostmapServer {
    width = 100, height = 100, resolution = 0.03,
    update_rate_ms = 50,
}

local dynamic_map, map_has_free, map_has_unknown, costmap_has_unknown = false, false, false, false
local function inspect_grid(bytes)
    local raw = bytes:str()
    local magic, width, height, resolution, origin_x, origin_y =
        string.unpack("<i4i4i4fff", raw)
    assert(magic == 0x47414D50 and #raw == 24 + width * height)
    local has_free, has_unknown = false, false
    for i = 25, #raw do
        local cell = raw:byte(i)
        if cell == 0 then has_free = true end
        if cell == 255 then has_unknown = true end
        if has_free and has_unknown then break end
    end
    return width, height, resolution, origin_x, origin_y, has_free, has_unknown
end

pipe(lidar, slam)
pipe(slam, function(msg)
    if msg.map then
        local width, height, _, origin_x, origin_y, has_free, has_unknown = inspect_grid(msg.map)
        dynamic_map = width ~= 100 or height ~= 100 or origin_x ~= 0 or origin_y ~= 0
        map_has_free, map_has_unknown = has_free, has_unknown
        costmap { static_map = msg.map }
    end
end)

local processed, corrected_x = 0, 0
pipe(slam, function(msg)
    if msg.slam then processed = msg.slam.processed_scans or processed end
    if msg.position then corrected_x = msg.position.x end
end)
local pose_x, ticks = 1.0, 0
local pose_timer
pipe(costmap, function(msg)
    if msg.costmap then
        local _, _, _, _, _, _, has_unknown = inspect_grid(msg.costmap)
        costmap_has_unknown = has_unknown
    end
    if msg.costmap and processed >= 4 and pose_x >= 1.25 and corrected_x > 1.15
            and dynamic_map and map_has_free and map_has_unknown and costmap_has_unknown then
        log.info("SLAM/nav motion smoke OK ({} scans, raw x={}, corrected x={})",
            processed, pose_x, corrected_x)
        pose_timer:Stop()
        shutdown()
    end
end)

pose_timer = each(30, function()
    ticks = ticks + 1
    if ticks > 5 then pose_x = math.min(1.3, pose_x + 0.015) end
    local pose = { x = pose_x, y = 1.0, theta = 0.0 }
    lidar { position = pose }
    slam { odometry = pose }
end)

after(5000, function()
    log.error("SLAM/nav motion smoke FAILED ({} scans, raw x={}, corrected x={})",
        processed, pose_x, corrected_x)
    os.exit(1)
end)
