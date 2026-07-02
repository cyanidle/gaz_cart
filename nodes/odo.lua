-- =============================================================================
--  Odometry + wheel-tuning node.
--
--  Integrates diff-drive odometry from the per-wheel speeds, streams chart +
--  odom telemetry to the WheelConfig tab, and applies its config / speed /
--  direct-voltage commands. Talks to the GUI through cfg.model — a node()
--  worker namespaced under "odo" (the WheelConfig model node key in Main.qml).
-- =============================================================================

local Odometry = require "mods.odometry"
local socket   = require "socket"

---@class OdoConfig
---@field model Pipable GUI model node from node(); sends wrapped, receives unwrapped
---@field track_width number distance between left and right wheels, m
---@field wheels fun(): table<string, {tgt: number, act: number}> per-wheel target + actual linear speed, m/s
---@field set_speed fun(wheel: string?, value: number) closed-loop speed target, m/s
---@field set_config fun(wheel: string?, id: integer, value: number) push one runtime config value
---@field direct_voltage fun(wheel: string?, volts: number) open-loop voltage, V
---@field odo_period_ms integer? odometry integration period (default 20)
---@field telem_period_ms integer? telemetry stream period (default 100)

---Wire the odometry/tuning node.
---@param cfg OdoConfig
---@return table odo the live Odometry integrator (:pose(), :velocity())
return function(cfg)
    local odo = Odometry.new { trackWidth = cfg.track_width }

    local last_t
    each(cfg.odo_period_ms or 20, function()
        local now = socket.gettime()
        local dt  = last_t and (now - last_t) or 0.0
        last_t = now
        local w = cfg.wheels()
        odo:update(w.fl.act, w.fr.act, w.rl.act, w.rr.act, dt)
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
