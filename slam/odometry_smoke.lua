-- Full timestamped odometry -> SLAM smoke test. Verifies that twist and
-- covariance survive correction and that pose is projected to scan time.
--   build/bin/radapter slam/odometry_smoke.lua

local os = require "os"
local socket = require "socket"

load_plugin(SCRIPT_DIR .. "/../build/slam/libgaz_slam")

local slam = Slam {
    map = { update_interval_ms = 100 },
    mapper = { do_loop_closing = false },
    max_odometry_extrapolation = 0.2,
}

local received = false
pipe(slam, function(msg)
    local o = msg.odometry
    if not o or not msg.scan then return end
    assert(math.abs(o.pose.x - 1.1) < 0.02,
        "odometry was not projected to the scan timestamp: " .. tostring(o.pose.x))
    assert(o.twist.linear == 1.0 and o.twist.angular == 0.0)
    assert(o.pose_covariance.xx > 0.01,
        "projected pose covariance was not preserved/propagated")
    assert(o.twist_covariance.linear == 0.04)
    received = true
    log.info("SLAM full-odometry smoke OK (projected x={}, covariance xx={})",
        o.pose.x, o.pose_covariance.xx)
    shutdown()
end)

local t = socket.gettime()
slam {
    odometry = {
        timestamp = t,
        pose = { x = 1.0, y = 0.0, theta = 0.0 },
        twist = { linear = 1.0, angular = 0.0 },
        pose_covariance = {
            xx = 0.01, xy = 0, xtheta = 0,
            yy = 0.01, ytheta = 0, thetatheta = 0.01,
        },
        twist_covariance = { linear = 0.04, linear_angular = 0, angular = 0.02 },
    },
    scan = {
        ranges = { 1.0, 1.0, 1.0, 1.0 },
        angle_min = 0.0,
        angle_increment = math.pi / 2,
        range_min = 0.05,
        range_max = 4.0,
        timestamp = t + 0.1,
    },
}

after(2000, function()
    if not received then
        log.error("SLAM full-odometry smoke FAILED")
        os.exit(1)
    end
end)
