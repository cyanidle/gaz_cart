-- =============================================================================
--  Navigation demo + costmap visualization.
--
--      radapter --gui nav/nav.lua
--
--  Wires CostmapServer -> Global/LocalPlanner and opens CostmapView.qml:
--    left click  — set the planner target (build & follow a path)
--    right click — drop a temporary obstacle into the costmap
--
--  A simulated robot integrates LocalPlanner's cmd_vel so the whole loop runs
--  without hardware. On the real cart, replace the "Simulated robot" block:
--  feed odometry into `position` and pass cmd_vel to main.lua's drive().
-- =============================================================================

load_plugin(SCRIPT_DIR .. "/build/libgaz_nav")

local MAX_LIN_SPD = 0.5 -- m/s at cmd_vel = 1
local MAX_ROT_SPD = 1.5 -- rad/s at cmd_vel = 1
local SIM_MS      = 50
local ROBOT_RADIUS = 0.1

local costmap = CostmapServer {
    update_rate_ms = 100,
    keep_points_ms = 30000,
    width = 101, height = 151, resolution = 0.02,
    inflate = { robot_safe_radius = ROBOT_RADIUS },
    inflate_static = { robot_safe_radius = ROBOT_RADIUS },
    -- image = SCRIPT_DIR .. "/costmap.png",  -- optional static map
}

local gp = GlobalPlanner {
    update_rate_ms = 100,
    a_star = { max_cost = 35 },
}

local lp = LocalPlanner {
    tick_rate = 20,
    margins = { position = 0.03, theta = 0.05 },
}

pipe(costmap, gp)
pipe(costmap, lp)
pipe(gp, lp) -- path
pipe(lp, gp) -- status feedback (finishes the target once idle)

-- ---- Simulated robot ---------------------------------------------------------

local pos = { x = 0.3, y = 0.3, theta = 0 }
local cmd = { x = 0, y = 0, theta = 0 }

on(lp, "cmd_vel", function(c) cmd = c end)

-- ---- GUI ---------------------------------------------------------------------

local view = QML { url = SCRIPT_DIR .. "/CostmapView.qml" }

pipe(costmap, view) -- costmap bytes
pipe(gp, view)      -- path
pipe(lp, view)      -- status

pipe(view, function(msg)
    if msg.target then gp { target = msg.target } end
    if msg.obstacle then costmap { point = msg.obstacle } end
    if msg.cancel then
        lp { cancel = true }
        gp { cancel = true }
    end
end)

each(SIM_MS, function()
    local dt = SIM_MS / 1000
    local c, s = math.cos(pos.theta), math.sin(pos.theta)
    pos.x = pos.x + (cmd.x * c - cmd.y * s) * MAX_LIN_SPD * dt
    pos.y = pos.y + (cmd.x * s + cmd.y * c) * MAX_LIN_SPD * dt
    pos.theta = pos.theta + cmd.theta * MAX_ROT_SPD * dt

    local msg = { position = pos }
    gp(msg)
    lp(msg)
    view(msg)
end)

log.info("nav demo up: left click = target, right click = obstacle")

return { costmap = costmap, gp = gp, lp = lp, view = view }
