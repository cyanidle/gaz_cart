-- =============================================================================
--  Odometry + wheel-tuning node.
--
--  Integrates diff-drive odometry from the per-wheel absolute travel, streams
--  chart + odom telemetry to the WheelConfig tab, and applies its config /
--  speed / direct-voltage commands. Talks to the GUI through cfg.model — a
--  branch() worker namespaced under "odo" (the WheelConfig model branch key in
--  Main.qml).
-- =============================================================================

local Odometry = require "mods.odometry"
local socket   = require "socket"

-- No wheel of this cart can cover more than this in one odometry period, so a
-- larger step is a discontinuity (a module rebooted and restarted its travel).
local MAX_WHEEL_SPEED = 5.0  -- m/s

---@class OdoConfig
---@field model Pipable<OdoGuiTelemetry, OdoGuiCommand> GUI node: emits telemetry, receives commands
---@field track_width number distance between left and right wheels, m
---@field wheels fun(): table<string, {tgt: number, act: number}> per-wheel target + actual linear speed, m/s
---@field travel fun(): table<string, number> per-wheel absolute travel since the module booted, m
---@field set_speed fun(wheel: string?, value: number) closed-loop speed target, m/s
---@field set_config fun(wheel: string?, id: integer, value: number) push one runtime config value
---@field direct_voltage fun(wheel: string?, volts: number) open-loop voltage, V
---@field odo_period_ms integer? odometry integration period (default 20)
---@field telem_period_ms integer? telemetry stream period (default 100)
---@field wheel_travel_stddev number? per-wheel travel 1-sigma uncertainty, m per period
---@field frames Frames? native tf-style worker receiving odom -> base_link
---@field odom_frame string? pose frame id (default "odom")
---@field base_frame string? body frame id (default "base_link")

---Wire the odometry/tuning node.
---@param cfg OdoConfig
---@return table odo the live Odometry integrator (:pose(), :velocity())
return function(cfg)
    local odo = Odometry.new {
        trackWidth = cfg.track_width,
        wheelTravelStdDev = cfg.wheel_travel_stddev,
        frameId = cfg.odom_frame,
        childFrameId = cfg.base_frame,
        frames = cfg.frames,
    }

    local last_t = socket.gettime()
    local last_travel = {}
    each(cfg.odo_period_ms or 20, function()
        local now = socket.gettime()
        local dt  = now - last_t
        if dt <= 0 then return end
        last_t = now

        local d = {}
        for wheel, travel in pairs(cfg.travel()) do
            local previous = last_travel[wheel]
            last_travel[wheel] = travel
            local step = previous and (travel - previous) or 0.0
            if math.abs(step) > MAX_WHEEL_SPEED * dt then
                log.warn("wheel {} travel jumped {} m in {} s: ignoring it", wheel, step, dt)
                step = 0.0
            end
            d[wheel] = step
        end
        odo:update(d.fl or 0.0, d.fr or 0.0, d.rl or 0.0, d.rr or 0.0, dt, now)
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
