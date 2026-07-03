-- =============================================================================
--  Navigation node: costmap -> global/local planner pipeline + the NavView tab.
--
--  Talks to the GUI through cfg.model — a node() worker namespaced under
--  "nav" (the NavView model node key in Main.qml). In NavView:
--    left click  — set the planner target (build & follow a path)
--    right click — drop a temporary obstacle into the costmap
--
--  Robot loop, selected by whether cfg.drive is given:
--    real  — cfg.pose() feeds odometry into `position`; LocalPlanner's cmd_vel
--            is scaled to a body twist and handed to cfg.drive(v, omega).
--    sim   — a simulated robot integrates cmd_vel in place, so the whole loop
--            runs without hardware (used when nav is wired standalone).
-- =============================================================================

load_plugin(SCRIPT_DIR .. "/../build/nav/libgaz_nav")

local MAX_LIN_SPD  = 0.5 -- m/s at cmd_vel = 1
local MAX_ROT_SPD  = 1.5 -- rad/s at cmd_vel = 1
local UPDATE_MS    = 50
local ROBOT_RADIUS = 0.1

---@class NavConfig
---@field model Pipable GUI model node from node(); sends wrapped, receives unwrapped
---@field drive fun(v: number, omega: number)? body twist sink (m/s, rad/s); its presence selects real mode
---@field pose fun(): number, number, number? robot pose source x, y, theta (required with drive)

---Wire the navigation stack.
---@param cfg NavConfig
return function(cfg)
    local costmap = CostmapServer {
        name = "costmap_server",
        update_rate_ms = 100,
        keep_points_ms = 30000,
        width = 101, height = 151, resolution = 0.02,
        inflate = { robot_safe_radius = ROBOT_RADIUS },
        inflate_static = { robot_safe_radius = ROBOT_RADIUS },
        -- image = SCRIPT_DIR .. "/costmap.png",  -- optional static map
    }

    local gp = GlobalPlanner {
        name = "global_planer",
        update_rate_ms = 100,
        a_star = { max_cost = 35 },
    }

    local lp = LocalPlanner {
        name = "local_planer",
        tick_rate = 20,
        margins = { position = 0.03, theta = 0.05 },
    }

    pipe(costmap, gp)
    pipe(costmap, lp)
    pipe(gp, lp) -- path
    pipe(lp, gp) -- status feedback (finishes the target once idle)

    -- ---- GUI -------------------------------------------------------------------
    pipe(costmap, cfg.model) -- costmap bytes
    pipe(gp, cfg.model)      -- path
    pipe(lp, cfg.model)      -- status

    pipe(cfg.model, function(msg)
        if msg.target then
            gp { target = msg.target }
        end
        if msg.obstacle then
            costmap { point = msg.obstacle }
        end
        if msg.cancel then
            lp { cancel = true }
            gp { cancel = true }
        end
    end)

    -- ---- Robot loop ------------------------------------------------------------
    -- Publish the current pose to both planners and the GUI, at UPDATE_MS.
    local function publish_pose(pos)
        local msg = { position = pos }
        gp(msg)
        lp(msg)
        cfg.model(msg)
    end

    if cfg.drive then
        -- Real robot: odometry pose in, cmd_vel out to the drivetrain. cmd.y is
        -- ignored — a diff cart can't strafe (LocalPlanner defaults to diff mode).
        on(lp, "cmd_vel", function(c)
            cfg.drive(c.x * MAX_LIN_SPD, c.theta * MAX_ROT_SPD)
        end)
        each(UPDATE_MS, function()
            local x, y, theta = cfg.pose()
            publish_pose { x = x, y = y, theta = theta }
        end)
    else
        -- Simulated robot: integrate cmd_vel in place (no hardware).
        local pos = { x = 0.3, y = 0.3, theta = 0 }
        local cmd = { x = 0, y = 0, theta = 0 }
        on(lp, "cmd_vel", function(c) cmd = c end)
        each(UPDATE_MS, function()
            local dt = UPDATE_MS / 1000
            local c, s = math.cos(pos.theta), math.sin(pos.theta)
            pos.x = pos.x + (cmd.x * c - cmd.y * s) * MAX_LIN_SPD * dt
            pos.y = pos.y + (cmd.x * s + cmd.y * c) * MAX_LIN_SPD * dt
            pos.theta = pos.theta + cmd.theta * MAX_ROT_SPD * dt
            publish_pose(pos)
        end)
    end
end
