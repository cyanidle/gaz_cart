-- =============================================================================
--  Differential-drive odometry.
--
--  Fed the travel increment (m) of each of the four wheels — the difference of
--  what each wheel module publishes on its Cyphal travel topic — plus the
--  elapsed time, it integrates them into a global pose and a robot-frame twist.
--
--  Frame convention:  +x forward, +y left, +theta counter-clockwise.
--  Sides:             left  = front-left + rear-left
--                     right = front-right + rear-right
--  If a wheel reports the wrong sign for your wiring, negate it before update().
-- =============================================================================

local Odometry = {}
Odometry.__index = Odometry

---@param opts { trackWidth: number, wheelTravelStdDev: number?, initialPoseStdDev: table?, frameId: string?, childFrameId: string?, frames: Frames? }
---`wheelTravelStdDev` is the 1-sigma uncertainty of one wheel's travel
---increment in m per update — encoder quantisation and slip, not a speed.
---`initialPoseStdDev` may contain x/y/theta standard deviations. Both default
---to zero (deterministic odometry).
function Odometry.new(opts)
    assert(opts and opts.trackWidth and opts.trackWidth > 0,
        "Odometry needs a positive trackWidth (left<->right wheel spacing, m)")
    local initial = opts.initialPoseStdDev or {}
    local wheelVariance = (opts.wheelTravelStdDev or 0.0)^2
    local initialPxx = (initial.x or 0.0)^2
    local initialPyy = (initial.y or 0.0)^2
    local initialPtt = (initial.theta or 0.0)^2
    return setmetatable({
        trackWidth = opts.trackWidth,
        frameId = opts.frameId or "odom",
        childFrameId = opts.childFrameId or "base_link",
        frames = opts.frames,
        wheelVariance = wheelVariance,
        -- global pose
        x = 0.0, y = 0.0, theta = 0.0,
        -- last robot-frame velocities
        v = 0.0, omega = 0.0,
        -- symmetric covariance of [x, y, theta]
        pxx = initialPxx,
        pxy = 0.0,
        pxt = 0.0,
        pyy = initialPyy,
        pyt = 0.0,
        ptt = initialPtt,
        initialPxx = initialPxx,
        initialPyy = initialPyy,
        initialPtt = initialPtt,
        -- covariance of the measured body motion [forward travel, heading]
        cvv = wheelVariance * 0.25,
        cvw = 0.0,
        cww = wheelVariance / (opts.trackWidth * opts.trackWidth),
        timestamp = nil,
    }, Odometry)
end

function Odometry:publishFrame()
    if self.frames then
        self.frames:set(self.frameId, self.childFrameId,
            { x = self.x, y = self.y, theta = self.theta })
    end
end

--- Feed the four per-wheel travel increments (m) since the previous call and
--- the time step (s).
---@param timestamp number? seconds on the Unix clock for this measurement
function Odometry:update(dFL, dFR, dRL, dRR, dt, timestamp)
    if not (dt and dt > 0) then
        return self
    end

    local dLeft  = (dFL + dRL) * 0.5
    local dRight = (dFR + dRR) * 0.5

    local dFwd   = (dLeft + dRight) * 0.5             -- forward travel, m
    local dTheta = (dRight - dLeft) / self.trackWidth -- heading increment, rad

    -- Integrate along the midpoint heading (2nd order, accurate in turns).
    local mid = self.theta + dTheta * 0.5

    -- EKF prediction for x' = x + dFwd*cos(mid), y' = y + dFwd*sin(mid),
    -- theta' = theta + dTheta, with the per-wheel travel noise in cvv/cww.
    local c, s = math.cos(mid), math.sin(mid)
    local fxTheta, fyTheta = -dFwd * s, dFwd * c   -- d/dtheta
    local gxF, gxT = c, -dFwd * s * 0.5            -- d/d[dFwd, dTheta]
    local gyF, gyT = s, dFwd * c * 0.5

    local pxx = self.pxx + 2 * fxTheta * self.pxt + fxTheta^2 * self.ptt
    local pxy = self.pxy + fxTheta * self.pyt + fyTheta * self.pxt
        + fxTheta * fyTheta * self.ptt
    local pxt = self.pxt + fxTheta * self.ptt
    local pyy = self.pyy + 2 * fyTheta * self.pyt + fyTheta^2 * self.ptt
    local pyt = self.pyt + fyTheta * self.ptt
    local ptt = self.ptt

    pxx = pxx + gxF^2 * self.cvv + 2 * gxF * gxT * self.cvw + gxT^2 * self.cww
    pxy = pxy + gxF * gyF * self.cvv
        + (gxF * gyT + gxT * gyF) * self.cvw + gxT * gyT * self.cww
    pxt = pxt + gxF * self.cvw + gxT * self.cww
    pyy = pyy + gyF^2 * self.cvv + 2 * gyF * gyT * self.cvw + gyT^2 * self.cww
    pyt = pyt + gyF * self.cvw + gyT * self.cww
    ptt = ptt + self.cww

    self.x     = self.x + dFwd * c
    self.y     = self.y + dFwd * s
    self.theta = (self.theta + dTheta) % (math.pi * 2)

    self.v, self.omega = dFwd / dt, dTheta / dt
    self.pxx, self.pxy, self.pxt = pxx, pxy, pxt
    self.pyy, self.pyt, self.ptt = pyy, pyt, ptt
    self.timestamp = timestamp or self.timestamp
    self:publishFrame()
    return self
end

function Odometry:pose()      return self.x, self.y, self.theta end
function Odometry:velocity()  return self.v, self.omega end
function Odometry:thetaDeg()  return self.theta * 180.0 / math.pi end

---Return one self-contained, ROS-Odometry-like 2D state. Twist is expressed
---in the robot frame; pose and its covariance are expressed in the odom frame.
function Odometry:odometry(timestamp)
    return {
        timestamp = timestamp or self.timestamp,
        frame_id = self.frameId,
        child_frame_id = self.childFrameId,
        pose = { x = self.x, y = self.y, theta = self.theta },
        twist = { linear = self.v, angular = self.omega },
        pose_covariance = {
            xx = self.pxx, xy = self.pxy, xtheta = self.pxt,
            yy = self.pyy, ytheta = self.pyt, thetatheta = self.ptt,
        },
        twist_covariance = {
            linear = self.cvv,
            linear_angular = self.cvw,
            angular = self.cww,
        },
    }
end

function Odometry:reset(start)
    start = start or {}
    self.x, self.y, self.theta = start.x or 0.0, start.y or 0.0, start.theta or 0.0
    self.v, self.omega = 0.0, 0.0
    self.pxx, self.pxy, self.pxt = self.initialPxx, 0.0, 0.0
    self.pyy, self.pyt, self.ptt = self.initialPyy, 0.0, self.initialPtt
    self.timestamp = start.timestamp
    self:publishFrame()
    return self
end

return Odometry
