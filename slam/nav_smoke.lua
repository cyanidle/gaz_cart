-- End-to-end simulated lidar -> SLAM -> costmap smoke test.
--   build/bin/radapter slam/nav_smoke.lua

local os = require "os"

load_plugin(SCRIPT_DIR .. "/../build/nav/libgaz_nav")
load_plugin(SCRIPT_DIR .. "/../build/slam/libgaz_slam")

local lidar = Lidar {
    sim = {
        beams = 180,
        noise = 0.0,
        obstacles = { { x = 1.6, y = 1.0, radius = 0.15 } },
    },
    scan_frequency = 10,
}
local slam = Slam {
    map = {
        width = 100, height = 100, resolution = 0.03,
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

pipe(lidar, slam)
pipe(slam, function(msg)
    if msg.map then costmap { static_map = msg.map } end
end)

local processed = 0
pipe(slam, function(msg)
    if msg.slam then processed = msg.slam.processed_scans or processed end
end)
pipe(costmap, function(msg)
    if msg.costmap and processed >= 2 then
        log.info("SLAM/nav smoke test OK ({} scans)", processed)
        shutdown()
    end
end)

each(30, function()
    local pose = { x = 1.0, y = 1.0, theta = 0.0 }
    lidar { position = pose }
    slam { odometry = pose }
end)

after(5000, function()
    log.error("SLAM/nav smoke test FAILED ({} scans)", processed)
    os.exit(1)
end)
