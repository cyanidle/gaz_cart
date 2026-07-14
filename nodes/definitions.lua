---@meta gaz_cart_nodes
-- Shared node-port contracts. Keep this file beside the nodes so LuaLS exposes
-- the wire shape at every `require "nodes.*"` call without loading hardware.

---@class WheelState
---@field tgt number closed-loop target linear speed, m/s
---@field act number measured linear speed, m/s

---@class CartPose
---@field x number meters
---@field y number meters
---@field theta number radians, counter-clockwise

---@class CartTwist
---@field linear number body-forward velocity, m/s
---@field angular number body yaw rate, rad/s

---@class CartPoseCovariance
---@field xx number
---@field xy number
---@field xtheta number
---@field yy number
---@field ytheta number
---@field thetatheta number

---@class CartTwistCovariance
---@field linear number
---@field linear_angular number
---@field angular number

---Pose is expressed in `frame_id`; twist is expressed in `child_frame_id`.
---@class CartOdometry
---@field timestamp number Unix time, seconds
---@field frame_id string pose reference frame, normally "odom" or "map"
---@field child_frame_id string robot body frame, normally "base_link"
---@field pose CartPose
---@field twist CartTwist
---@field pose_covariance CartPoseCovariance
---@field twist_covariance CartTwistCovariance

---@class OdoGuiCommand
---@field action "config"|"set_speed"|"direct"
---@field wheel string? "fl"|"fr"|"rl"|"rr"|"all"
---@field id integer? runtime-config id; required for action="config"
---@field value number|string command value

---@class OdoGuiTelemetry
---@field chart table<string, WheelState>
---@field odom {x: number, y: number, theta: number, v: number, omega: number}

---@class TeleopGuiCommand
---@field action "twist"
---@field v number normalized forward command, -1..1
---@field omega number normalized yaw command, -1..1

---@class NavGuiCommand
---@field target CartPose?
---@field obstacle {x: number, y: number}?
---@field cancel boolean?
---@field discovery_enabled boolean?
---@field dump_map boolean?
---@field load_map Bytes?
---@field reposition CartPose? requested map-frame pose; re-anchors map -> odom

---@class NavGuiTelemetry
---@field position CartPose?
---@field odometry CartOdometry?
---@field costmap Bytes?
---@field path CartPose[]?
---@field cmd_vel CartPose?
---@field status LocalPlannerStatus?
---@field scan LaserScan?
---@field slam SlamStats?
---@field map_image Bytes?
---@field map_io_status string?
---@field discovery_enabled boolean?

---@class RosTwist
---@field linear {x: number, y: number, z: number}
---@field angular {x: number, y: number, z: number}

---@class NavPluginPaths
---@field nav string? path to libgaz_nav; default is this checkout's build output
---@field slam string|false? path to libgaz_slam; false disables SLAM loading/use
---@field frames string? path to libgaz_frames when Nav creates its private frame worker

---@class NavPluginWorkers
---@field costmap CostmapServerConfig?
---@field global_planner GlobalPlannerConfig?
---@field local_planner LocalPlannerConfig?
---@field lidar LidarConfig?
---@field slam SlamConfig?

---@class CartCyphalInput
---@field spd_fl {value: number} module front-left linear speed, m/s
---@field spd_fr {value: number} module front-right linear speed, m/s
---@field spd_rl {value: number} module rear-left linear speed, m/s
---@field spd_rr {value: number} module rear-right linear speed, m/s

---@class CartCyphalOutput
---@field cmd_fl {value: number}? front-left target speed, m/s
---@field cmd_fr {value: number}? front-right target speed, m/s
---@field cmd_rl {value: number}? rear-left target speed, m/s
---@field cmd_rr {value: number}? rear-right target speed, m/s
---@field dir_fl {value: number}? front-left open-loop voltage, V
---@field dir_fr {value: number}? front-right open-loop voltage, V
---@field dir_rl {value: number}? rear-left open-loop voltage, V
---@field dir_rr {value: number}? rear-right open-loop voltage, V
---@field cfg_fl {value: integer[]}? front-left { id, numerator, denominator }
---@field cfg_fr {value: integer[]}? front-right { id, numerator, denominator }
---@field cfg_rl {value: integer[]}? rear-left { id, numerator, denominator }
---@field cfg_rr {value: integer[]}? rear-right { id, numerator, denominator }
