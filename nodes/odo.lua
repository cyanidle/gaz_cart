-- =============================================================================
--  Odometry + wheel-tuning node.
--
--  Integrates diff-drive odometry from the per-wheel speeds, streams chart +
--  odom telemetry to the WheelConfig tab, and applies its config / speed /
--  direct-voltage commands. Talks to the GUI through cfg.model — a branch()
--  worker namespaced under "odo" (the WheelConfig model branch key in Main.qml).
-- =============================================================================

local Odometry = require "mods.odometry"
local socket   = require "socket"

---@class OdoConfig
---@field model Pipable<OdoGuiTelemetry, OdoGuiCommand> GUI node: emits telemetry, receives commands
---@field track_width number distance between left and right wheels, m
---@field wheels fun(): table<string, {tgt: number, act: number}> per-wheel target + actual linear speed, m/s
---@field set_speed fun(wheel: string?, value: number) closed-loop speed target, m/s
---@field set_config fun(wheel: string?, id: integer, value: number) push one runtime config value
---@field direct_voltage fun(wheel: string?, volts: number) open-loop voltage, V
---@field odo_period_ms integer? odometry integration period (default 20)
---@field telem_period_ms integer? telemetry stream period (default 100)
---@field wheel_speed_stddev number? per-wheel speed 1-sigma uncertainty, m/s
---@field frames Frames? native tf-style worker receiving odom -> base_link
---@field odom_frame string? pose frame id (default "odom")
---@field base_frame string? body frame id (default "base_link")

---Wire the odometry/tuning node.
---@param cfg OdoConfig
---@return table odo the live Odometry integrator (:pose(), :velocity())
return function(cfg)
    local odo = Odometry.new {
        trackWidth = cfg.track_width,
        wheelSpeedStdDev = cfg.wheel_speed_stddev,
        frameId = cfg.odom_frame,
        childFrameId = cfg.base_frame,
        frames = cfg.frames,
    }

    local last_t
    each(cfg.odo_period_ms or 20, function()
        local now = socket.gettime()
        local dt  = last_t and (now - last_t) or 0.0
        last_t = now
        local w = cfg.wheels()
        odo:update(w.fl.act, w.fr.act, w.rl.act, w.rr.act, dt, now)
    end)

    pipe(cfg.model, function(msg)
        log("Received command: {}", msg)
        if msg.action == "config" then
            cfg.set_config(msg.wheel or "all", math.floor(msg.id),
                assert(tonumber(msg.value), "config value is not a number"))
        end
        if msg.action == "set_speed" then
            cfg.set_speed(msg.wheel, tonumber(msg.value) or 0.0)
        end
        if msg.action == "direct" then
            cfg.direct_voltage(msg.wheel, tonumber(msg.value) or 0.0)
        end
    end)

    each(cfg.telem_period_ms or 100, function()
        local x, y, th = odo:pose()
        local v, omega = odo:velocity()
        cfg.model {
            chart = cfg.wheels(),
            odom = { x = x, y = y, theta = th, v = v, omega = omega },
        }
    end)

    return odo
end
