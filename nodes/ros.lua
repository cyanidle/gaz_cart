-- =============================================================================
--  ROS 2 bridge node: subscribe to an external cmd_vel and drive the cart.
--
--  Lets a ROS 2 stack (nav2, teleop, joystick, ...) command the cart directly:
--  it subscribes to a geometry_msgs/Twist topic and forwards linear.x / angular.z
--  to cfg.drive(v, omega). ROS Twist is already in SI (m/s, rad/s), so — unlike
--  LocalPlanner's normalized cmd_vel (nodes/nav.lua) — no scaling is applied.
--
--  This is an INDEPENDENT drive source from the internal planner; run only one
--  of them at a time or they will fight over the wheels.
--
--  Requires the radapter ROS plugin (radapter built with RADAPTER_ROS2). It is
--  loaded on demand — cart.lua only wires this node when a plugin dir is given.
-- =============================================================================

---@class RosConfig
---@field drive fun(v: number, omega: number) body twist sink (normalised -1..1)
---@field plugin_dir string directory holding radapter_ros(.so)
---@field cmd_vel_topic string? Twist topic to subscribe to (default "/cmd_vel")
---@field domain_id integer? ROS_DOMAIN_ID (default: environment / 0)
---@field max_lin_spd number? m/s at full forward (default 0.5)
---@field max_rot_spd number? rad/s at full turn (default 1.5)

---Wire the ROS cmd_vel bridge.
---@param cfg RosConfig
---@return Worker node the ROS2 worker
return function(cfg)
    load_plugin(cfg.plugin_dir .. "/radapter_ros")

    local topic = cfg.cmd_vel_topic or "/cmd_vel"
    local max_lin = cfg.max_lin_spd or 0.5
    local max_rot = cfg.max_rot_spd or 1.5

    local node = ROS2 {
        name = "ros_bridge",
        domain_id = cfg.domain_id,
        subs = {
            [topic] = {
                type = "geometry_msgs/msg/Twist",
                handler = function(twist)
                    -- Normalise Twist (SI → -1..1) so drive() can clamp + scale
                    local v     = twist.linear.x  / max_lin
                    local omega = twist.angular.z / max_rot
                    cfg.drive(v, omega)
                end,
            },
        },
    }

    log.info("ros bridge: driving from {} (Twist, max v={} ω={})", topic, max_lin, max_rot)
    return node
end
