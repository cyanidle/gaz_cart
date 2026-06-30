-- =============================================================================
--  Cart main module — loads odometry and drives 4 motors from Cyphal topics.
--
--  Usage from Lua:
--    require "cart.main" {
--        can_device = "vcan0",
--        node_id = 42,
--        drivetrain = "diffdrive",
--        wheel_radius = 0.05,
--        ticks_per_rev = 12,
--        trackWidth = 0.30,
--        -- or: drivetrain = "mecanum", halfTrack = 0.15, halfBase = 0.10,
--        -- Cyphal topic config for each motor's hall sensor:
--        motor_topic_type = "custom.MotorHall.1.0",
--        motor_topic_ports = { fl = 1000, fr = 1001, rl = 1002, rr = 1003 },
--        pos_field = "position",   -- field name in the incoming message
--        update_rate_ms = 10,      -- how often to call odo:update()
--    }
--
--  The returned table has .odo and .motors so you can query pose/velocities.
-- =============================================================================

local odometry = require "odometry"

---@class CartOpts
---@field can_device string          e.g. "vcan0"
---@field node_id number             Cyphal node id
---@field drivetrain '"mecanum"' | '"diffdrive"'
---@field wheel_radius number        meters
---@field ticks_per_rev number?      default 12
---@field halfTrack number?          mecanum only
---@field halfBase number?           mecanum only
---@field trackWidth number?         diffdrive only
---@field motor_topic_type string    Cyphal message type for motor hall sensor
---@field motor_topic_ports table<string, number>  map: "fl","fr","rl","rr" -> port
---@field pos_field string?          field name in the incoming message (default "position")
---@field update_rate_ms number?     periodic update interval in ms (default 10)

---@param opts CartOpts
local function init(opts)
    assert(opts.can_device, "can_device is required")
    assert(opts.node_id, "node_id is required")
    assert(opts.motor_topic_type, "motor_topic_type is required")
    assert(opts.motor_topic_ports, "motor_topic_ports is required")
    local ports = opts.motor_topic_ports
    assert(ports.fl and ports.fr and ports.rl and ports.rr,
        "motor_topic_ports needs fl, fr, rl, rr")

    local ticksPerRev = opts.ticks_per_rev or 12
    local wheelRadius = opts.wheel_radius or 0.05
    local posField     = opts.pos_field or "position"
    local updateMs     = opts.update_rate_ms or 10

    -- ---- CAN + Cyphal ------------------------------------------------

    local can = CAN {
        plugin = "socketcan",
        device = opts.can_device,
    }

    ---@type table<string, CyphalTopic>
    local subscribe = {}

    for _, wheel in ipairs({"fl", "fr", "rl", "rr"}) do
        subscribe[wheel] = {
            type = opts.motor_topic_type,
            port = ports[wheel],
        }
    end

    local cyphal = Cyphal {
        can = can,
        node_id = opts.node_id,
        subscribe = subscribe,
    }

    -- ---- Motors + Odometry -------------------------------------------

    local motors = {
        fl = odometry.Motor(wheelRadius, ticksPerRev),
        fr = odometry.Motor(wheelRadius, ticksPerRev),
        rl = odometry.Motor(wheelRadius, ticksPerRev),
        rr = odometry.Motor(wheelRadius, ticksPerRev),
    }

    local odo = odometry.Odometry {
        drivetrain = opts.drivetrain,
        fl = motors.fl,
        fr = motors.fr,
        rl = motors.rl,
        rr = motors.rr,
        halfTrack  = opts.halfTrack,
        halfBase   = opts.halfBase,
        trackWidth = opts.trackWidth,
    }

    -- ---- Buffer latest positions from Cyphal -------------------------

    local latest = { fl = 0, fr = 0, rl = 0, rr = 0 }

    for _, wheel in ipairs({"fl", "fr", "rl", "rr"}) do
        on(cyphal, wheel, function(msg)
            local v = msg[posField]
            if v then
                latest[wheel] = v
            end
        end)
    end

    -- ---- Periodic odometry update ------------------------------------

    local socket = require "socket"
    local lastTs = nil

    each(updateMs, function()
        local now = socket.gettime()    -- seconds with sub-ms precision
        local dt
        if lastTs then
            dt = now - lastTs
        else
            dt = 0.0
        end
        lastTs = now

        odo:update(latest.fl, latest.fr, latest.rl, latest.rr, dt)
    end)

    return { odo = odo, motors = motors, cyphal = cyphal, can = can }
end

