-- Native Frames plugin smoke test.
--   build/bin/radapter frames/smoke.lua

load_plugin(SCRIPT_DIR .. "/../build/frames/libgaz_frames")

local frames = Frames { name = "frames_smoke" }
frames:set("map", "odom", { x = 2, y = 1, theta = math.pi / 2 })
frames:set("odom", "base_link", { x = 3, y = 0, theta = 0 })

local pose = assert(frames:lookup("map", "base_link"))
assert(math.abs(pose.x - 2) < 1e-9 and math.abs(pose.y - 4) < 1e-9,
    "frame composition is incorrect")

local odometry = frames:transform_odometry({
    frame_id = "odom",
    child_frame_id = "base_link",
    pose = { x = 3, y = 0, theta = 0 },
    pose_covariance = { xx = 1, xy = 0, xtheta = 0, yy = 1, ytheta = 0, thetatheta = 1 },
}, "map")
assert(math.abs(odometry.pose.x - 2) < 1e-9 and math.abs(odometry.pose.y - 4) < 1e-9,
    "odometry pose was not transformed")

frames:reanchor("map", "odom", { x = 3, y = 0, theta = 0 }, { x = 5, y = 6, theta = 0 })
local reanchored = assert(frames:lookup("map", "odom"))
assert(math.abs(reanchored.x - 2) < 1e-9 and math.abs(reanchored.y - 6) < 1e-9,
    "re-anchor is incorrect")

log.info("native Frames smoke OK")
shutdown()
