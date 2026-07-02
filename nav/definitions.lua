---@meta gaz_nav
--  Lua LSP definitions for the gaz_nav radapter plugin (this repo's nav/*.cpp,
--  loaded with load_plugin(".../build/nav/libgaz_nav")). Project-local
--  counterpart of radapter/definitions.lua — that one covers only the engine
--  API and must not be edited for plugin workers. Keep in sync with the config
--  structs and OnMsg/SendMsg fields in the corresponding .cpp.

---A { x, y, theta } pose/point in meters / radians.
---@class NavPose
---@field x number
---@field y number
---@field theta number? -- defaults to 0 where optional

-- ---- CostmapServer -----------------------------------------------------------

---@class InflateConfig
---@field robot_safe_radius number? -- inflation radius, m (default 0.20)

---@class CostmapServerConfig : WorkerConfig
---@field update_rate_ms integer? -- costmap publish period (default 80)
---@field keep_points_ms integer? -- manual `point` obstacles expire this long after the last one (default 15000)
---@field ignore_all_outside boolean? -- drop objects outside the grid (default true)
---@field image string? -- optional static map image, grayscale, must match width x height
---@field width integer? -- grid width, cells (default 101)
---@field height integer? -- grid height, cells (default 151)
---@field resolution number? -- meters per cell side (default 0.02)
---@field inflate InflateConfig? -- inflation of dynamic obstacles
---@field inflate_static InflateConfig? -- inflation baked into the static map

---An obstacle tracked by id: { x, y, size (diameter, m), ttl (ms), id, source_id }.
---@class MapObject
---@field x number?
---@field y number?
---@field size number?
---@field ttl number?
---@field id integer?
---@field source_id integer?

---@class CostmapServer : Worker
---@field Reload fun(self: CostmapServer, cfg: CostmapServerConfig): boolean -- re-apply a full config table (defaults re-applied)

---Costmap aggregator/publisher.
---Input fields:  `objects` (MapObject or MapObject[]), `point` (NavPose: manual
---obstacle, cleared keep_points_ms after the last one).
---Output (data channel), every update_rate_ms: `costmap` — one immutable bytes
---buffer: GridHeader (magic "GAMP", width, height, resolution) + one cost byte
---(0..100) per cell, row-major (see nav/nav_common.hpp). Consumers (planners,
---QML DataView) reinterpret the buffer in place — no repacking.
---@param cfg CostmapServerConfig
---@return CostmapServer
function CostmapServer(cfg) end

-- ---- GlobalPlanner -----------------------------------------------------------

---@class AStarConfig
---@field costmap_to_node_cost_coeff number? -- weight of cell cost in node cost (default 5.0)
---@field cell_cost number? -- base cost per step (default 10.0)
---@field diagonal_coeff number? -- diagonal step multiplier (default 1.25)
---@field max_cost integer? -- cells above this are impassable (default 35)

---@class GlobalPlannerConfig : WorkerConfig
---@field a_star AStarConfig?
---@field nodes_batch_size integer? -- graph/open-set reserve (default 10000)
---@field reserve_in_path_size integer? -- path list reserve (default 60)
---@field update_rate_ms integer? -- replan period (default 50)
---@field max_points integer? -- A* node budget before a plan fails (default 2000)
---@field consider_reached_after number? -- s of local-planner idle to finish a target (default 1.5)
---@field min_time_for_target number? -- s before idle can finish a fresh target (default 0.5)

---@class GlobalPlanner : Worker
---@field Reload fun(self: GlobalPlanner, cfg: GlobalPlannerConfig): boolean -- re-apply a full config table (defaults re-applied)

---A* planner on the costmap, replanning on a timer while a target is active.
---Input fields:  `costmap` (CostmapServer bytes), `position` (NavPose),
---`target` (NavPose command), `cancel` (any non-nil), `status` (LocalPlanner's,
---finishes the target once reached && rotated and idle long enough).
---Output (data channel): `path` — NavPose[]; empty on failure / cancel.
---Events: `planning` — "success" | "failed".
---@param cfg GlobalPlannerConfig
---@return GlobalPlanner
function GlobalPlanner(cfg) end

-- ---- LocalPlanner ------------------------------------------------------------

---@class MarginsConfig
---@field position number? -- goal position tolerance, m (default 0.03)
---@field theta number? -- goal heading tolerance, rad (default 0.04)

---@class PathConfig
---@field half_slow_per_cost_of number? -- halve speed per this much path cost (default 20.0)
---@field approximation_step_points integer? -- lookahead scan stride, points (default 2)
---@field approximation_max_cost number? -- max cell cost on the shortcut to a candidate (default 30.0)
---@field fallback_min_points_count integer? -- min points to keep when no candidate passes (default 3)

---@class DriveConfig
---@field min_speed_coeff number? -- floor of the speed scale (default 0.4)
---@field min_rotation_spd number? -- rad/s floor while rotating (default 0.3)
---@field full_rot_spd_per_radians number? -- full rotation speed at this heading error (default 2.0)
---@field enable_mid_path_rotation boolean? -- rotate toward path theta while driving (default true)
---@field max_radians_per_meter number? -- rotation budget per meter driven (default 1.0)
---@field max_speed_for_meters number? -- full speed at this remaining distance (default 0.5)

---@class LocalPlannerConfig : WorkerConfig
---@field tick_rate number? -- control loop rate, Hz (default 12.0)
---@field margins MarginsConfig?
---@field path PathConfig?
---@field drive DriveConfig?

---@class LocalPlannerStatus
---@field reached boolean
---@field rotated boolean
---@field is_stuck boolean
---@field driving_for number -- s
---@field idle_for number -- s

---@class LocalPlanner : Worker
---@field Reload fun(self: LocalPlanner, cfg: LocalPlannerConfig): boolean -- re-apply a full config table (defaults re-applied)

---Path follower: drives toward the furthest cheaply-reachable path point,
---rotates into the goal heading, reports status.
---Input fields:  `path` (NavPose[]; empty stops the robot), `costmap`
---(CostmapServer bytes), `position` (NavPose), `pause` (bool: freeze while true).
---Output (data channel), every tick: `cmd_vel` (NavPose: body-frame linear m/s
---+ angular rad/s), `status` (LocalPlannerStatus — feed back to GlobalPlanner),
---`local_target` (NavPose; only while a path is active).
---@param cfg LocalPlannerConfig
---@return LocalPlanner
function LocalPlanner(cfg) end
