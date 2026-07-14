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
--  Robot loop: cfg.odometry() publishes `odom -> base_link`; this node queries
--  a shared tf-style `map -> odom` transform before feeding SLAM/planners.
--  LocalPlanner's
--  cmd_vel (-1..1) is handed to cfg.drive(v, omega).  cfg.sim only controls
--  whether lidar is simulated or real — drive/pose are SIM-aware already.
-- =============================================================================

local socket = require "socket"

local NODE_DIR = SCRIPT_DIR or "."
if NODE_DIR:sub(1, 1) ~= "/" and lfs and lfs.currentdir then
    NODE_DIR = assert(lfs.currentdir()) .. "/" .. NODE_DIR
end

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
---@field model Pipable<NavGuiTelemetry, NavGuiCommand> GUI node: emits navigation state, receives commands
---@field sim boolean? -- default false; sim robot + sim lidar instead of real hardware
---@field drive (fun(v: number, omega: number))? body twist sink (m/s, rad/s); required when sim=false
---@field pose (fun(): number, number, number)? robot pose source x, y, theta; required when sim=false
---@field odometry (fun(): CartOdometry)? full pose/twist/covariance source; preferred over pose
---@field frames Frames? shared native tf-style worker; defaults to a private map -> odom tree
---@field plugins NavPluginPaths? plugin library locations; set plugins.slam=false to run without SLAM
---@field workers NavPluginWorkers? config passed directly to nav/SLAM plugin workers

---Wire the navigation stack.
---@param cfg NavConfig
return function(cfg)
    local sim = cfg.sim
    local mapDiscovery = true
    local plugins = cfg.plugins or {}
    local workers = cfg.workers or {}

    -- Plugin locations and all individual worker settings are supplied through
    -- cfg.  The defaults keep this checkout runnable while allowing a deployed
    -- cart to point at packaged/out-of-tree plugins without editing this node.
    load_plugin(plugins.nav or (NODE_DIR .. "/../build/nav/libgaz_nav"))
    local useSlam = plugins.slam ~= false
    if useSlam then
        load_plugin(plugins.slam or (NODE_DIR .. "/../build/slam/libgaz_slam"))
    end

    assert(cfg.drive, "cfg.drive required")
    assert(cfg.odometry or cfg.pose, "cfg.odometry or cfg.pose required")

    -- Keep pose-only sources working for standalone smoke tests and external
    -- users, but normalize the rest of this node onto one complete state.
    local function rawOdometry()
        if cfg.odometry then
            local odometry = cfg.odometry()
            -- `frame_id`/`child_frame_id` became explicit with the frame tree.
            -- Accept older producers during migration as conventional odometry.
            odometry.frame_id = odometry.frame_id or "odom"
            odometry.child_frame_id = odometry.child_frame_id or "base_link"
            return odometry
        end
        local x, y, theta = cfg.pose()
        return {
            timestamp = socket.gettime(),
            frame_id = "odom",
            child_frame_id = "base_link",
            pose = { x = x, y = y, theta = theta },
            twist = { linear = 0, angular = 0 },
            pose_covariance = {
                xx = 0, xy = 0, xtheta = 0,
                yy = 0, ytheta = 0, thetatheta = 0,
            },
            twist_covariance = { linear = 0, linear_angular = 0, angular = 0 },
        }
    end

    local frames = cfg.frames
    if not frames then
        load_plugin(plugins.frames or (NODE_DIR .. "/../build/frames/libgaz_frames"))
        frames = Frames { name = "frames" }
    end
    -- The map frame starts coincident with odometry. Localization and the UI
    -- may subsequently move only this transform; wheel integration is never
    -- patched or re-based.
    if not frames:lookup("map", "odom") then
        frames:set("map", "odom", { x = 0, y = 0, theta = 0 })
    end

    local lastRawOdometry
    local function mapOdometry(raw)
        return frames:transform_odometry(raw, "map")
    end

    local costmap = CostmapServer(merge({
        name = "costmap_server",
        update_rate_ms = 100,
        keep_points_ms = 30000,
        width = 101, height = 151, resolution = 0.02,
        inflate = { robot_safe_radius = ROBOT_RADIUS },
        inflate_static = { robot_safe_radius = ROBOT_RADIUS },
        -- image = SCRIPT_DIR .. "/costmap.png",  -- optional static map
    }, workers.costmap))

    local gp = GlobalPlanner(merge({
        name = "global_planer",
        update_rate_ms = 100,
        a_star = { max_cost = 35 },
    }, workers.global_planner))

    local lp = LocalPlanner(merge({
        name = "local_planer",
        tick_rate = 20,
        margins = { position = 0.03, theta = 0.05 },
    }, workers.local_planner))

    -- Lidar: sim mode gets a raycasting emulator; real mode is enabled by a
    -- `workers.lidar` plugin config. Detected obstacles feed the costmap;
    -- the raw scan goes to the GUI for visualization.
    local lidar
    if sim then
        lidar = Lidar(merge({
            name = "lidar",
            scan_frequency = 12,
            sim = { beams = 360, obstacles = SIM_OBSTACLES },
        }, workers.lidar))
    elseif workers.lidar then
        lidar = Lidar(merge({ name = "lidar" }, workers.lidar))
    end
    if lidar then
        -- SLAM consumes the LaserScan-shaped payload (ranges + geometry), not
        -- the lidar's odometry-projected obstacle list. Keep the latter as a
        -- short-lived costmap layer so newly seen/dynamic objects are avoided
        -- before Karto has enough passes to classify them as map occupancy.
        pipe(lidar, filter("objects"), costmap)
    end

    -- Karto scan matching + Ceres pose-graph optimization. Every scan also
    -- carries the current wheel pose as Karto's odometric prior. In simulation
    -- both odometry and raycasts are exact; Karto explicitly recommends turning
    -- scan matching off in that case because its small corrections only add
    -- pose jitter. Real hardware keeps scan matching and loop closure enabled.
    -- CostmapServer adopts the map's changing dimensions and world origin, so
    -- Lua only renames `map` to `static_map` at the boundary.
    local slam
    if lidar and useSlam then
        slam = Slam(merge({
            name = "slam",
            map = {
                resolution = 0.02,
                update_interval_ms = 500,
                min_pass_through = 2,
                occupancy_threshold = 0.1,
            },
            mapper = {
                -- Mock odometry and raycasts are exact. Letting scan matching
                -- perturb that ground truth is the SIM version of SLAM and
                -- odometry "fighting" each other.
                use_scan_matching = not sim,
                minimum_travel_distance = 0.04,
                minimum_travel_heading = 0.04,
                scan_buffer_size = 40,
                do_loop_closing = not sim,
            },
            solver = { threads = 2 },
        }, workers.slam))
        pipe(lidar, slam)
        pipe(slam, function(m)
            if m.map and mapDiscovery then costmap { static_map = m.map } end
            if m.odometry or m.position then
                -- SLAM's map-frame correction becomes a first-class tf edge.
                -- Every downstream user therefore sees the same map -> odom
                -- relation instead of each node carrying an offset.
                if m.odometry and lastRawOdometry then
                    frames:reanchor("map", "odom", lastRawOdometry.pose, m.odometry.pose)
                end
                local position = m.odometry and m.odometry.pose or m.position
                local p = { position = position, odometry = m.odometry }
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
    pipe(costmap, function(m)
        m.discovery_enabled = mapDiscovery
        return m
    end, cfg.model) -- costmap bytes + current discovery state
    pipe(gp, cfg.model)      -- path
    pipe(lp, cfg.model)      -- status

    -- The wheel producer publishes odom -> base_link. This node merely queries
    -- map -> odom from the shared frame tree before wiring the result onward.
    local function publish_odometry(raw)
        lastRawOdometry = raw
        local odom = mapOdometry(raw)
        if lidar then
            lidar { position = odom.pose }
        end
        if slam then
            slam { odometry = odom }
        else
            local msg = { position = odom.pose, odometry = odom }
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
        if msg.discovery_enabled ~= nil then
            mapDiscovery = not not msg.discovery_enabled
        end
        if msg.dump_map then
            local ok, image = pcall(function() return costmap:DumpMap() end)
            if ok then
                cfg.model { map_image = image }
            else
                cfg.model { map_io_status = "Dump failed: " .. tostring(image) }
            end
        end
        if msg.load_map then
            local ok, err = pcall(function() costmap:LoadMap(msg.load_map) end)
            if ok then
                mapDiscovery = false
                cfg.model {
                    discovery_enabled = false,
                    map_io_status = "Map loaded; discovery disabled",
                }
            else
                cfg.model { map_io_status = "Load failed: " .. tostring(err) }
            end
        end
        if msg.reposition then
            local raw = lastRawOdometry or rawOdometry()
            local current = mapOdometry(raw).pose
            local desired = {
                x = msg.reposition.x,
                y = msg.reposition.y,
                theta = msg.reposition.theta or current.theta,
            }
            frames:reanchor("map", "odom", raw.pose, desired)
            lp { cancel = true }
            gp { cancel = true }
            if slam then slam:Reset() end
            publish_odometry(raw)
        end
    end)

    -- ---- Robot loop ------------------------------------------------------------
    -- Always use cfg.drive / rawOdometry — they are SIM-aware (cart.lua wires
    -- either real-Cyphal or mocked-motor versions depending on the SIM flag).
    on(lp, "cmd_vel", function(c)
        cfg.drive(c.x, c.theta)
    end)
    each(UPDATE_MS, function()
        local raw = rawOdometry()
        publish_odometry(raw)
    end)
end
