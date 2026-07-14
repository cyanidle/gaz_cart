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
---@field image string? -- optional fixed static map image, grayscale, must match width x height
---@field width integer? -- initial/fallback grid width before a SLAM map arrives (default 101)
---@field height integer? -- initial/fallback grid height before a SLAM map arrives (default 151)
---@field resolution number? -- initial/fallback meters per cell (default 0.02)
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

---@class CostmapServer : Worker<CostmapInput, CostmapOutput>
---@field Reload fun(self: CostmapServer, cfg: CostmapServerConfig): boolean -- re-apply a full config table (defaults re-applied)
---@field DumpMap fun(self: CostmapServer): bytes -- PNG of current SLAM/loaded map, including geometry metadata
---@field LoadMap fun(self: CostmapServer, image: bytes): boolean -- transparent pixels are unknown

---@class CostmapInput
---@field objects MapObject|MapObject[]?
---@field point NavPose? manual obstacle
---@field static_map Bytes? GAMP grid from Slam

---@class CostmapOutput
---@field costmap Bytes GAMP grid: header followed by cost bytes

---Costmap aggregator/publisher.
---Input (`CostmapInput`) fields: `objects` (MapObject or MapObject[]), `point` (NavPose: manual
---obstacle, cleared keep_points_ms after the last one), `static_map` (a GAMP
---grid from Slam; its dimensions, resolution and origin are adopted dynamically,
---then it is merged and inflated with the other layers).
---Output (`CostmapOutput`, data channel), every update_rate_ms: `costmap` — one immutable bytes
---buffer: GridHeader (magic "GAMP", width, height, resolution, origin_x,
---origin_y) + one row-major byte per cell: 0..100 cost or 255 unknown (see
---nav/nav_common.hpp). Consumers (planners, QML DataView) reinterpret the
---buffer in place — no repacking. DumpMap/LoadMap use PNG: transparent pixels
---are unknown, black is occupied, white is free, and PNG text fields preserve
---resolution and origin.
---@param cfg CostmapServerConfig
---@return CostmapServer
function CostmapServer(cfg) end

-- ---- GlobalPlanner -----------------------------------------------------------

---@class AStarConfig
---@field costmap_to_node_cost_coeff number? -- weight of cell cost in node cost (default 5.0)
---@field cell_cost number? -- base cost per step (default 10.0)
---@field diagonal_coeff number? -- diagonal step multiplier (default 1.25)
---@field max_cost integer? -- known cells above this are impassable (default 35)
---@field unknown_cost integer? -- A* cost for traversable unknown cells, 0..100 (default 30)

---@class GlobalPlannerConfig : WorkerConfig
---@field a_star AStarConfig?
---@field nodes_batch_size integer? -- graph/open-set reserve (default 10000)
---@field reserve_in_path_size integer? -- path list reserve (default 60)
---@field update_rate_ms integer? -- replan period (default 50)
---@field max_points integer? -- A* node budget before a plan fails (default 2000)
---@field outside_map_margin number? -- temporary unknown padding for an out-of-grid target, meters (default 0.25)
---@field consider_reached_after number? -- s of local-planner idle to finish a target (default 1.5)
---@field min_time_for_target number? -- s before idle can finish a fresh target (default 0.5)

---@class GlobalPlanner : Worker<GlobalPlannerInput, GlobalPlannerOutput>
---@field Reload fun(self: GlobalPlanner, cfg: GlobalPlannerConfig): boolean -- re-apply a full config table (defaults re-applied)

---@class GlobalPlannerInput
---@field costmap Bytes?
---@field position NavPose?
---@field target NavPose?
---@field cancel any?
---@field status LocalPlannerStatus?

---@class GlobalPlannerOutput
---@field path NavPose[] empty on failure/cancel

---A* planner on the costmap, replanning on a timer while a target is active.
---Unknown cells are traversable using `a_star.unknown_cost`; known obstacle
---costs above `max_cost` remain impassable.
---Input (`GlobalPlannerInput`) fields: `costmap` (CostmapServer bytes), `position` (NavPose),
---`target` (NavPose command), `cancel` (any non-nil), `status` (LocalPlanner's,
---finishes the target once reached && rotated and idle long enough).
---Output (`GlobalPlannerOutput`, data channel): `path` — NavPose[]; empty on failure / cancel.
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
---@field unknown_inflation_radius number? -- local danger radius around unknown space, m (default 0.20)

---Holonomic (omni-wheel) mode — set under `drive.omni_drive` to select it.
---@class OmniDriveConfig
---@field enable_mid_path_rotation boolean? -- rotate toward path theta while strafing (default true)
---@field mid_path_rotation_gain number? -- speed fraction per rad·m of mid-path rotation (default 1.0)

---Differential (tank) mode — set under `drive.diff_drive` to select it (the default).
---@class DiffDriveConfig
---@field heading_kp number? -- speed fraction per radian of heading error (default 1.5)
---@field turn_in_place_angle number? -- rad; above this heading error, stop and rotate in place (default 0.8)
---@field allow_reverse boolean? -- drive backward toward targets behind the robot (default false)

---Motion params common to both drive modes plus the two optional mode blocks.
---Exactly one of `diff_drive` / `omni_drive` selects the kinematics; if neither
---is given, `diff_drive` is the default. Setting both raises.
---@class DriveConfig
---@field min_speed number? -- forward-speed floor, fraction of full (default 0.4)
---@field min_rotation_speed number? -- in-place rotation floor, fraction of full (default 0.3)
---@field rotation_gain number? -- in-place rotation speed fraction per radian (default 2.0)
---@field full_speed_distance number? -- distance at which forward speed saturates at 1.0, m (default 0.5)
---@field diff_drive DiffDriveConfig? -- present => differential (tank) mode
---@field omni_drive OmniDriveConfig? -- present => holonomic (omni) mode

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

---@class LocalPlanner : Worker<LocalPlannerInput, LocalPlannerOutput>
---@field Reload fun(self: LocalPlanner, cfg: LocalPlannerConfig): boolean -- re-apply a full config table (defaults re-applied)

---@class LocalPlannerInput
---@field path NavPose[]?
---@field costmap Bytes?
---@field position NavPose?
---@field pause boolean?
---@field cancel boolean?

---@class LocalPlannerOutput
---@field cmd_vel NavPose body-frame normalized command: x forward, theta yaw
---@field status LocalPlannerStatus
---@field local_target NavPose?

---Path follower: drives toward the furthest cheaply-reachable path point,
---rotates into the goal heading, reports status. Unknown cells count as maximum
---danger and are locally inflated by `path.unknown_inflation_radius`.
---Input (`LocalPlannerInput`) fields: `path` (NavPose[]; empty stops the robot), `costmap`
---(CostmapServer bytes), `position` (NavPose), `pause` (bool: freeze while true).
---Output (`LocalPlannerOutput`, data channel), every tick: `cmd_vel` (NavPose: body-frame speed
---+ fractions -1..1), `status` (LocalPlannerStatus — feed back to GlobalPlanner),
---`local_target` (NavPose; only while a path is active).
---@param cfg LocalPlannerConfig
---@return LocalPlanner
function LocalPlanner(cfg) end

-- ---- Lidar -------------------------------------------------------------------

---Segment detector params (bigbang's ObjectDetection): a run of consecutive,
---close-together beam hits becomes one obstacle.
---@class ObjectDetectionConfig
---@field min_points integer? -- min beams in a run to accept it (default 3)
---@field split_each integer? -- force a break after this many beams (default 60)
---@field max_dist number? -- ignore hits farther than this from the robot, m (default 3.5)
---@field max_dist_between_dots number? -- consecutive-hit gap that breaks a run, m (default 0.05)
---@field start_ttl number? -- ms an object lives after it was last seen (default 800)
---@field map_ttl_coeff number? -- ttl scaling applied when handed to the costmap (default 0.8)
---@field max_deviation number? -- cross-scan dedup radius, m (default 0.02)
---@field min_x number? -- object accept bounds, m (defaults -3/5/-3/6)
---@field max_x number?
---@field min_y number?
---@field max_y number?

---A ground-truth circle the built-in simulator raycasts against.
---@class SimObstacle
---@field x number?
---@field y number?
---@field radius number? -- m (default 0.1)

---Simulation backend. Its presence selects sim mode (no hardware); absence
---means the real RPLidar device is opened.
---@class LidarSimConfig
---@field beams integer? -- rays per revolution (default 360)
---@field noise number? -- range noise amplitude, m (default 0.005)
---@field obstacles SimObstacle[]? -- initial ground-truth obstacles

---@class LidarSerialConfig
---@field port string? -- serial device (default "/dev/ttyUSB0")
---@field baud integer? -- baud rate (default 256000)

---@class LidarNetworkConfig
---@field host string? -- device host (default "192.168.1.25")
---@field port integer? -- device port (default 20108)
---@field use_tcp boolean? -- TCP vs UDP channel (default true)

---@class LidarConfig : WorkerConfig
---@field range_min number? -- discard hits nearer than this, m (default 0.15)
---@field range_max number? -- discard hits farther than this, m (default 12.0)
---@field reversed boolean? -- scan arrives clockwise (default true)
---@field lidar_offset number? -- sensor yaw mount offset, rad (default 0)
---@field lidar_x_offset number? -- sensor x mount offset, m (default 0)
---@field lidar_y_offset number? -- sensor y mount offset, m (default 0)
---@field range_correction number? -- added to every range, m (default 0)
---@field source_id integer? -- source_id stamped on emitted MapObjects (default 0)
---@field scan_frequency number? -- Hz; sim tick rate / motor RPM (default 10)
---@field scan_mode string? -- hardware scan mode ("" = driver's first)
---@field use_serial boolean? -- serial vs network hardware channel (default true)
---@field grab_with_interval boolean? -- use getScanDataWithIntervalHq (default false)
---@field objects ObjectDetectionConfig?
---@field serial LidarSerialConfig?
---@field network LidarNetworkConfig?
---@field sim LidarSimConfig? -- present => simulate instead of using hardware

---@class Lidar : Worker<LidarInput, LidarOutput>
---@field Reload fun(self: Lidar, cfg: LidarConfig): boolean -- re-apply a full config table (defaults re-applied)

---@class LaserScanPoint
---@field x number world x, m
---@field y number world y, m

---@class LaserScan
---@field pose NavPose scan-time robot pose in the map frame
---@field points LaserScanPoint[] world-frame hits
---@field ranges number[] range per beam, m
---@field angle_min number radians
---@field angle_max number radians
---@field angle_increment number radians
---@field range_min number meters
---@field range_max number meters
---@field timestamp number Unix time, seconds

---@class LidarInput
---@field position NavPose? map-frame robot pose
---@field scan number[]|{angle: number, range: number}[]?
---@field sim_obstacle SimObstacle?
---@field clear_sim any?

---@class LidarOutput
---@field objects MapObject[]
---@field scan LaserScan

---RPLidar-driven obstacle detector (port of bigbang's rplidarnode). Parses each
---beam to a world point using the latest pose, segments them into objects, and
---emits them as the costmap's `objects` list.
---Input (`LidarInput`) fields: `position` (NavPose), `scan` (inject a raw scan: number[] of
---ranges, or { angle, range }[]), `sim_obstacle` (SimObstacle, sim mode),
---`clear_sim` (any non-nil: drop sim obstacles).
---Output (`LidarOutput`, data channel), per scan: `objects` (MapObject[] for the costmap),
---`scan` ({ pose, points, ranges, angle_min, angle_max, angle_increment,
---range_min, range_max, timestamp }); the range fields intentionally mirror
---ROS LaserScan and are consumed directly by the Slam worker.
---@param cfg LidarConfig
---@return Lidar
function Lidar(cfg) end
