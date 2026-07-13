-- End-to-end NavView command path: manual map-frame reposition must survive
-- subsequent simulated lidar scans instead of snapping back to raw odometry.
--   build/bin/radapter slam/reposition_smoke.lua

local os = require "os"
package.path = SCRIPT_DIR .. "/../?.lua;" .. package.path

local model = create_worker(function(self, msg, sender)
    notify_all(self, msg, sender)
end)

local repositioned = false
local passed = false
local latest = { x = 0, y = 0, theta = 0 }
local received = 0
local raw_x = 0.5
local desired_theta = 0.25
local movement_started = false
local worst_error = 0
local discovery_disabled = false
pipe(model, function(msg)
    if msg.position then latest = msg.position end
    if msg.slam then received = msg.slam.received_scans or received end
    if msg.discovery_enabled ~= nil then discovery_disabled = not msg.discovery_enabled end
    local dx = raw_x - 0.5
    local expected_x = 2.0 + math.cos(desired_theta) * dx
    local expected_y = 2.0 + math.sin(desired_theta) * dx
    if movement_started then
        local error = math.sqrt((latest.x - expected_x)^2 + (latest.y - expected_y)^2)
        worst_error = math.max(worst_error, error)
    end
    if not passed and repositioned and discovery_disabled and raw_x >= 0.8 and received >= 8
            and math.abs(latest.x - expected_x) < 0.08
            and math.abs(latest.y - expected_y) < 0.08 then
        passed = true
        log.info("SLAM moving-reposition smoke OK (x={}, y={}, scans={}, worst error={})",
            latest.x, latest.y, received, worst_error)
        shutdown()
    end
end)

require "nodes.nav" {
    model = model,
    sim = true,
    drive = function() end,
    pose = function() return raw_x, 0.5, 0.0 end,
}

after(400, function()
    repositioned = true
    received = 0
    model { reposition = { x = 2.0, y = 2.0, theta = desired_theta } }
    after(250, function()
        model { discovery_enabled = false }
    end)
    after(350, function()
        movement_started = true
        each(50, function()
            raw_x = math.min(0.8, raw_x + 0.01)
        end)
    end)
end)

after(5000, function()
    log.error("SLAM moving-reposition smoke FAILED (raw x={}, corrected x={}, y={}, scans={}, worst error={})",
        raw_x, latest.x, latest.y, received, worst_error)
    os.exit(1)
end)
