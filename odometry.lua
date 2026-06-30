-- =============================================================================
--  4-wheel odometry — reusable Lua module.
--
--  Frame convention:  +x forward, +y left, +theta counter-clockwise.
--  Wheel order:       front-left, front-right, rear-left, rear-right.
--
--  Pick the drivetrain via the Odometry constructor's `drivetrain` option:
--    - "mecanum"   : mecanum / X-drive wheels (can strafe)
--                    requires halfTrack, halfBase
--    - "diffdrive" : standard cart, tank / skid-steer (cannot strafe)
--                    requires trackWidth
--
--  If a wheel spins "the wrong way" on your robot, flip the sign of its
--  reading (e.g. setPosition(-pos)) or swap two wheels in the constructor.
-- =============================================================================

local odometry = {}

-- -----------------------------------------------------------------------------
--  Motor: one wheel + one hall sensor.
--
--  The hall sensor gives an ABSOLUTE position 0 .. ticksPerRev-1, where each
--  tick is 1/ticksPerRev of a full wheel turn. (For your case ticksPerRev = 12,
--  so a reading of 12 simply wraps back to 0.)
-- -----------------------------------------------------------------------------

local Motor_mt = {}
Motor_mt.__index = Motor_mt

---@class Motor
---@field ticksPerRev number
---@field metersPerTick number
---@field currentPos number
---@field lastPos number
---@field initialized boolean
---@field deltaTicks number
---@field speed number
---@field totalDistance number
---@field setPosition fun(self: Motor, absolutePosition: number)
---@field process fun(self: Motor, dt: number)
---@field deltaDistance fun(self: Motor): number
---@field reset fun(self: Motor)
function odometry.Motor(wheelRadius, ticksPerRev)
    ticksPerRev = ticksPerRev or 12
    local self = setmetatable({
        ticksPerRev = ticksPerRev,
        metersPerTick = (2.0 * math.pi * wheelRadius) / ticksPerRev,
        currentPos = 0,
        lastPos = 0,
        initialized = false,
        deltaTicks = 0,
        speed = 0.0,
        totalDistance = 0.0,
    }, Motor_mt)
    return self
end

--- Feed the latest hall reading. Any int is accepted and normalised into
--- [0, ticksPerRev); a reading equal to ticksPerRev wraps to 0.
function Motor_mt:setPosition(absolutePosition)
    local p = absolutePosition % self.ticksPerRev
    if p < 0 then p = p + self.ticksPerRev end
    self.currentPos = p
end

--- Compute delta + speed since the previous call. dt is seconds.
function Motor_mt:process(dt)
    if not self.initialized then
        self.lastPos = self.currentPos
        self.initialized = true
        self.deltaTicks = 0
        self.speed = 0.0
        return
    end

    local delta = self.currentPos - self.lastPos
    -- Take the shortest signed arc so wrap-around (e.g. 11 -> 0) is handled.
    -- Assumes the wheel moves LESS than half a turn between two process()
    -- calls — so sample often enough relative to wheel speed.
    local half = math.floor(self.ticksPerRev / 2)
    if delta >  half then delta = delta - self.ticksPerRev end
    if delta < -half then delta = delta + self.ticksPerRev end

    self.deltaTicks = delta
    local dDist = delta * self.metersPerTick
    self.totalDistance = self.totalDistance + dDist
    self.speed = (dt > 0.0 and dDist / dt) or 0.0
    self.lastPos = self.currentPos
end

function Motor_mt:deltaDistance()
    return self.deltaTicks * self.metersPerTick
end

function Motor_mt:reset()
    self.initialized = false
    self.deltaTicks = 0
    self.speed = 0.0
    self.totalDistance = 0.0
end

-- -----------------------------------------------------------------------------
--  Odometry: everything that is the same for every drivetrain.
-- -----------------------------------------------------------------------------

local Odometry_mt = {}
Odometry_mt.__index = Odometry_mt

---@class OdometryOpts
---@field drivetrain '"mecanum"' | '"diffdrive"'
---@field fl Motor      front-left motor
---@field fr Motor      front-right motor
---@field rl Motor      rear-left motor
---@field rr Motor      rear-right motor
---@field halfTrack number?   mecanum: center -> wheel along Y (half the left/right spacing), meters
---@field halfBase number?    mecanum: center -> wheel along X (half the front/back spacing), meters
---@field trackWidth number?  diffdrive: distance between left and right wheels, meters.
---  For 4-wheel SKID-STEER this is an *effective* value — tune it until reported
---  theta matches reality after a few full spins.

---@param opts OdometryOpts
function odometry.Odometry(opts)
    if opts.drivetrain == "mecanum" then
        assert(opts.halfTrack, "mecanum requires halfTrack")
        assert(opts.halfBase, "mecanum requires halfBase")
    elseif opts.drivetrain == "diffdrive" then
        assert(opts.trackWidth, "diffdrive requires trackWidth")
    else
        error("drivetrain must be 'mecanum' or 'diffdrive'")
    end

    local self = setmetatable({
        drivetrain = opts.drivetrain,
        fl = opts.fl,
        fr = opts.fr,
        rl = opts.rl,
        rr = opts.rr,
        -- mecanum geometry
        L = (opts.halfTrack or 0) + (opts.halfBase or 0),
        -- diffdrive geometry
        trackWidth = opts.trackWidth or 0,
        -- pose
        x = 0.0,
        y = 0.0,
        theta = 0.0,
        -- robot-frame velocities
        vx = 0.0,
        vy = 0.0,
        omega = 0.0,
    }, Odometry_mt)
    return self
end

--- The one call you make each loop: feed the 4 raw hall positions + dt.
---@param posFL number
---@param posFR number
---@param posRL number
---@param posRR number
---@param dt number  seconds since last update
function Odometry_mt:update(posFL, posFR, posRL, posRR, dt)
    self.fl:setPosition(posFL)
    self.fr:setPosition(posFR)
    self.rl:setPosition(posRL)
    self.rr:setPosition(posRR)
    self.fl:process(dt)
    self.fr:process(dt)
    self.rl:process(dt)
    self.rr:process(dt)
    self:_integrate(dt)
end

function Odometry_mt:thetaDeg()
    return self.theta * 180.0 / math.pi
end

function Odometry_mt:reset(start)
    start = start or {}
    self.x = start.x or 0.0
    self.y = start.y or 0.0
    self.theta = start.theta or 0.0
    self.vx = 0.0
    self.vy = 0.0
    self.omega = 0.0
    self.fl:reset()
    self.fr:reset()
    self.rl:reset()
    self.rr:reset()
end

-- ---------------------------------------------------------------------------
--  Drivetrain-specific kinematics (private).
-- ---------------------------------------------------------------------------

function Odometry_mt:_integrate(dt)
    if self.drivetrain == "mecanum" then
        self:_integrateMecanum(dt)
    else
        self:_integrateDiffDrive(dt)
    end
end

function Odometry_mt:_integrateMecanum(dt)
    local dFL = self.fl:deltaDistance()
    local dFR = self.fr:deltaDistance()
    local dRL = self.rl:deltaDistance()
    local dRR = self.rr:deltaDistance()

    local dFwd  = ( dFL + dFR + dRL + dRR) * 0.25
    local dLeft = (-dFL + dFR + dRL - dRR) * 0.25
    local dTh   = (-dFL + dFR - dRL + dRR) / (4.0 * self.L)

    self:_integrateTwist(dFwd, dLeft, dTh, dt)
end

function Odometry_mt:_integrateDiffDrive(dt)
    local dLeftSide  = (self.fl:deltaDistance() + self.rl:deltaDistance()) * 0.5
    local dRightSide = (self.fr:deltaDistance() + self.rr:deltaDistance()) * 0.5

    local dFwd = (dLeftSide + dRightSide) * 0.5
    local dTh  = (dRightSide - dLeftSide) / self.trackWidth

    self:_integrateTwist(dFwd, 0.0, dTh, dt)
end

--- Integrate a robot-frame twist (forward, left, rotation, all for this
--- cycle) into the global pose, and update the velocities. Uses the midpoint
--- heading (2nd order — accurate while turning).
function Odometry_mt:_integrateTwist(dFwd, dLeft, dTheta, dt)
    local mid = self.theta + dTheta * 0.5
    local c   = math.cos(mid)
    local s   = math.sin(mid)

    self.x     = self.x + dFwd * c - dLeft * s
    self.y     = self.y + dFwd * s + dLeft * c
    self.theta = self.theta + dTheta

    if dt > 0.0 then
        self.vx    = dFwd   / dt
        self.vy    = dLeft  / dt
        self.omega = dTheta / dt
    end
end

return odometry
