---@meta gaz_slam
-- Lua LSP definitions for build/slam/libgaz_slam.

---@class SlamMapConfig
---@field width integer? deprecated compatibility field; dynamic Karto bounds are used
---@field height integer? deprecated compatibility field; dynamic Karto bounds are used
---@field resolution number? meters per cell (default 0.02)
---@field update_interval_ms integer? occupancy-map publish period (default 1000)
---@field min_pass_through integer? beams required before classifying a cell (default 2)
---@field occupancy_threshold number? hit/pass ratio for occupancy (default 0.1)

---@class SlamLaserConfig
---@field x number? lidar x offset in the robot frame, meters
---@field y number? lidar y offset in the robot frame, meters
---@field theta number? lidar yaw offset in the robot frame, radians
---@field min_range number? optional lower range override
---@field max_range number? optional mapping range cap (default 20)

---@class SlamMapperConfig
---@field use_scan_matching boolean?
---@field use_scan_barycenter boolean?
---@field minimum_time_interval number?
---@field minimum_travel_distance number?
---@field minimum_travel_heading number?
---@field scan_buffer_size integer?
---@field scan_buffer_maximum_scan_distance number?
---@field link_match_minimum_response_fine number?
---@field link_scan_maximum_distance number?
---@field loop_search_maximum_distance number?
---@field do_loop_closing boolean?
---@field loop_match_minimum_chain_size integer?
---@field loop_match_maximum_variance_coarse number?
---@field loop_match_minimum_response_coarse number?
---@field loop_match_minimum_response_fine number?
---@field correlation_search_space_dimension number?
---@field correlation_search_space_resolution number?
---@field correlation_search_space_smear_deviation number?
---@field loop_search_space_dimension number?
---@field loop_search_space_resolution number?
---@field loop_search_space_smear_deviation number?
---@field distance_variance_penalty number?
---@field angle_variance_penalty number?
---@field fine_search_angle_offset number?
---@field coarse_search_angle_offset number?
---@field coarse_angle_resolution number?
---@field minimum_angle_penalty number?
---@field minimum_distance_penalty number?
---@field use_response_expansion boolean?

---@alias SlamLinearSolver
---| `"SPARSE_NORMAL_CHOLESKY"`
---| `"SPARSE_SCHUR"`
---| `"ITERATIVE_SCHUR"`
---| `"CGNR"`

---@alias SlamPreconditioner
---| `"JACOBI"`
---| `"IDENTITY"`
---| `"SCHUR_JACOBI"`

---@alias SlamTrustStrategy
---| `"LEVENBERG_MARQUARDT"`
---| `"DOGLEG"`

---@alias SlamDoglegType
---| `"TRADITIONAL_DOGLEG"`
---| `"SUBSPACE_DOGLEG"`

---@alias SlamLossFunction
---| `"None"`
---| `"HuberLoss"`
---| `"CauchyLoss"`

---@class SlamSolverConfig
---@field linear_solver SlamLinearSolver?
---@field preconditioner SlamPreconditioner?
---@field trust_strategy SlamTrustStrategy?
---@field dogleg_type SlamDoglegType?
---@field loss_function SlamLossFunction?
---@field threads integer? Ceres worker threads (default 1)
---@field debug_logging boolean?

---@class SlamConfig : WorkerConfig
---@field map SlamMapConfig?
---@field laser SlamLaserConfig?
---@field mapper SlamMapperConfig?
---@field solver SlamSolverConfig?
---@field throttle_scans integer? process every Nth lidar scan (default 1)

---@class SlamStats
---@field received_scans integer
---@field processed_scans integer
---@field localized boolean
---@field paused boolean

---@class Slam : Worker
---@field Reload fun(self: Slam, cfg: SlamConfig): boolean
---@field Reset fun(self: Slam): boolean
---@field Save fun(self: Slam, base_path: string): boolean writes `.posegraph` and `.data`

---Online Karto/Ceres SLAM worker.
---Input: `odometry` (NavPose), `scan` (the Lidar worker's LaserScan-shaped
---payload), `pause` (bool), and `reset` (any non-nil).
---Output: `position` (map-corrected pose), `map` (origin-aware, dynamically
---bounded GAMP occupancy bytes; 0 free, 100 occupied, 255 unknown),
---`scan` (corrected world-frame hit points), `covariance`, and `slam` stats.
---@param cfg SlamConfig
---@return Slam
function Slam(cfg) end
