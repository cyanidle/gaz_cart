-- =============================================================================
--  Navigation node: costmap -> global/local planner pipeline + the NavView tab.
--
--  Talks to the GUI through cfg.model — a branch() worker namespaced under
--  "nav" (the NavView model branch key in Main.qml). In NavView:
--    left click  — set the planner target
--    left drag   — set the planner target and heading
--    shift+click — manually reposition the robot/map pose
--    right click — drop a temporary obstacle into the costmap
--
--  Robot loop: cfg.pose() feeds position to the planners; LocalPlanner's
--  cmd_vel (-1..1) is handed to cfg.drive(v, omega).  cfg.sim only controls
--  whether lidar is simulated or real — drive/pose are SIM-aware already.
-- =============================================================================

load_plugin(SCRIPT_DIR .. "/../build/nav/libgaz_nav")
load_plugin(SCRIPT_DIR .. "/../build/slam/libgaz_slam")

local UPDATE_MS    = 50
local ROBOT_RADIUS = 0.1

-- Ground-truth obstacles the simulated lidar raycasts against (world meters).
-- Seeded so the NAV tab shows the lidar discovering obstacles on start-up;
-- right-clicking the map adds more (see the model handler below).
local SIM_OBSTACLES = {
    { x = 1.0, y = 1.0, radius = 0.12 },
    { x = 1.4, y = 2.0, radius = 0.10 },
    { x = 0.6, y = 1.6, radius = 0.15 },
}
local SIM_OBSTACLE_RADIUS = 0.1 -- radius for obstacles dropped via right-click

---@class NavConfig
---@field model Pipable GUI model node from branch(); sends wrapped, receives unwrapped
---@field sim boolean? -- default false; sim robot + sim lidar instead of real hardware
---@field drive (fun(v: number, omega: number))? body twist sink (m/s, rad/s); required when sim=false
---@field pose (fun(): number, number, number)? robot pose source x, y, theta; required when sim=false
---@field lidar_port string? serial device of a real RPLidar (ignored when sim=true)

---Wire the navigation stack.
---@param cfg NavConfig
return function(cfg)
    local sim = cfg.sim

    assert(cfg.drive, "cfg.drive required")
    assert(cfg.pose, "cfg.pose required")

    -- Manual positioning changes the map-frame origin without modifying the
    -- wheel odometry source. This works for both simulated and real odometry:
    -- subsequent motion is composed onto the pose selected in NavView.
    local poseOffset = { x = 0, y = 0, theta = 0 }
    local lastRawPose
    local function normalizeTheta(theta)
        return (theta + math.pi) % (2 * math.pi) - math.pi
    end
    local function composePose(a, b)
        local c, s = math.cos(a.theta), math.sin(a.theta)
        return {
            x = a.x + c * b.x - s * b.y,
            y = a.y + s * b.x + c * b.y,
            theta = normalizeTheta(a.theta + b.theta),
        }
    end
    local function inversePose(p)
        local c, s = math.cos(p.theta), math.sin(p.theta)
        return {
            x = -c * p.x - s * p.y,
            y =  s * p.x - c * p.y,
            theta = normalizeTheta(-p.theta),
        }
    end
    local function adjustedPose(raw)
        return composePose(poseOffset, raw)
    end

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

    -- Lidar: sim mode gets a raycasting emulator; real mode uses a hardware
    -- RPLidar when lidar_port is given. Detected obstacles feed the costmap;
    -- the raw scan goes to the GUI for visualization.
    local lidar
    if sim then
        lidar = Lidar {
            name = "lidar",
            scan_frequency = 12,
            sim = { beams = 360, obstacles = SIM_OBSTACLES },
        }
    elseif cfg.lidar_port then
        lidar = Lidar { name = "lidar", serial = { port = cfg.lidar_port } }
    end
    if lidar then
        -- SLAM consumes the LaserScan-shaped payload (ranges + geometry), not
        -- the lidar's odometry-projected obstacle list. Keep the latter as a
        -- short-lived costmap layer so newly seen/dynamic objects are avoided
        -- before Karto has enough passes to classify them as map occupancy.
        pipe(lidar, filter("objects"), costmap)
    end

    -- Karto scan matching + Ceres pose-graph optimization. CostmapServer adopts
    -- the map's changing dimensions and world origin, so Lua only renames `map`
    -- to `static_map` at the boundary.
    local slam
    if lidar then
        slam = Slam {
            name = "slam",
            map = {
                resolution = 0.02,
                update_interval_ms = 500,
                min_pass_through = 2,
                occupancy_threshold = 0.1,
            },
            mapper = {
                use_scan_matching = true,
                minimum_travel_distance = 0.04,
                minimum_travel_heading = 0.04,
                scan_buffer_size = 40,
                do_loop_closing = true,
            },
            solver = { threads = 2 },
        }
        pipe(lidar, slam)
        pipe(slam, function(m)
            if m.map then costmap { static_map = m.map } end
            if m.position then
                local p = { position = m.position }
                gp(p)
                lp(p)
                cfg.model(p)
            end
            if m.scan then cfg.model { scan = m.scan } end
            if m.slam then cfg.model { slam = m.slam } end
        end)
    end

    pipe(costmap, gp)
    pipe(costmap, lp)
    pipe(gp, lp) -- path
    pipe(lp, gp) -- status feedback (finishes the target once idle)

    -- ---- GUI -------------------------------------------------------------------
    pipe(costmap, cfg.model) -- costmap bytes
    pipe(gp, cfg.model)      -- path
    pipe(lp, cfg.model)      -- status

    -- Raw wheel odometry is transformed into the manually selected map frame.
    -- The simulated lidar must use this ground-truth pose only; feeding SLAM's
    -- corrected estimate back into it creates a localization feedback loop.
    local function publish_odometry(pos)
        if lidar then
            lidar { position = pos }
        end
        if slam then
            slam { odometry = pos }
        else
            local msg = { position = pos }
            gp(msg)
            lp(msg)
            cfg.model(msg)
        end
    end

    pipe(cfg.model, function(msg)
        if msg.target then
            gp { target = msg.target }
        end
        if msg.obstacle then
            -- With a simulated lidar, dropped obstacles are ground truth the
            -- sensor discovers; otherwise they go straight into the costmap.
            if sim then
                lidar { sim_obstacle = {
                    x = msg.obstacle.x, y = msg.obstacle.y, radius = SIM_OBSTACLE_RADIUS,
                } }
            else
                costmap { point = msg.obstacle }
            end
        end
        if msg.cancel then
            lp { cancel = true }
            gp { cancel = true }
        end
        if msg.reposition then
            local desired = msg.reposition
            local raw = lastRawPose
            if not raw then
                local x, y, theta = cfg.pose()
                raw = { x = x, y = y, theta = theta }
                lastRawPose = raw
            end
            desired = {
                x = desired.x,
                y = desired.y,
                theta = desired.theta or adjustedPose(raw).theta,
            }
            poseOffset = composePose(desired, inversePose(raw))
            lp { cancel = true }
            gp { cancel = true }
            if slam then slam:Reset() end
            publish_odometry(adjustedPose(raw))
        end
    end)

    -- ---- Robot loop ------------------------------------------------------------
    -- Always use cfg.drive / cfg.pose — they are SIM-aware (cart.lua wires
    -- either real-Cyphal or mocked-motor versions depending on the SIM flag).
    on(lp, "cmd_vel", function(c)
        cfg.drive(c.x, c.theta)
    end)
    each(UPDATE_MS, function()
        local x, y, theta = cfg.pose()
        lastRawPose = { x = x, y = y, theta = theta }
        publish_odometry(adjustedPose(lastRawPose))
    end)
end
