-- =============================================================================
--  Differential-drive odometry.
--
--  Fed the linear velocity (m/s) of each of the four wheels — exactly what each
--  wheel module publishes on its Cyphal linear-speed topic — plus the elapsed
--  time, it integrates a robot-frame twist into a global pose.
--
--  Frame convention:  +x forward, +y left, +theta counter-clockwise.
--  Sides:             left  = front-left + rear-left
--                     right = front-right + rear-right
--  If a wheel reports the wrong sign for your wiring, negate it before update().
-- =============================================================================

local Odometry = {}
Odometry.__index = Odometry

---@param opts { trackWidth: number }  distance between the left and right wheels, m
function Odometry.new(opts)
    assert(opts and opts.trackWidth and opts.trackWidth > 0,
        "Odometry needs a positive trackWidth (left<->right wheel spacing, m)")
    return setmetatable({
        trackWidth = opts.trackWidth,
        -- global pose
        x = 0.0, y = 0.0, theta = 0.0,
        -- last robot-frame velocities
        v = 0.0, omega = 0.0,
    }, Odometry)
end

--- Feed the four wheel linear velocities (m/s) and the time step (s).
function Odometry:update(vFL, vFR, vRL, vRR, dt)
    local vLeft  = (vFL + vRL) * 0.5
    local vRight = (vFR + vRR) * 0.5

    local v     = (vLeft + vRight) * 0.5              -- forward speed, m/s
    local omega = (vRight - vLeft) / self.trackWidth  -- yaw rate, rad/s

    -- Integrate over dt with the midpoint heading (2nd order, accurate in turns).
    local dTheta = omega * dt
    local dFwd   = v * dt
    local mid    = self.theta + dTheta * 0.5

    self.x     = self.x + dFwd * math.cos(mid)
    self.y     = self.y + dFwd * math.sin(mid)
    self.theta = (self.theta + dTheta) % (math.pi * 2)

    self.v, self.omega = v, omega
    return self
end

function Odometry:pose()      return self.x, self.y, self.theta end
function Odometry:velocity()  return self.v, self.omega end
function Odometry:thetaDeg()  return self.theta * 180.0 / math.pi end

function Odometry:reset(start)
    start = start or {}
    self.x, self.y, self.theta = start.x or 0.0, start.y or 0.0, start.theta or 0.0
    self.v, self.omega = 0.0, 0.0
    return self
end

return Odometry
