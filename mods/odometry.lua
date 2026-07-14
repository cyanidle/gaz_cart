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

---@param opts { trackWidth: number, wheelSpeedStdDev: number?, initialPoseStdDev: table?, frameId: string?, childFrameId: string?, frames: Frames? }
---`wheelSpeedStdDev` is the 1-sigma uncertainty of one wheel's linear-speed
---measurement in m/s. `initialPoseStdDev` may contain x/y/theta standard
---deviations. Both default to zero (deterministic odometry).
function Odometry.new(opts)
    assert(opts and opts.trackWidth and opts.trackWidth > 0,
        "Odometry needs a positive trackWidth (left<->right wheel spacing, m)")
    local initial = opts.initialPoseStdDev or {}
    local wheelVariance = (opts.wheelSpeedStdDev or 0.0)^2
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
        -- covariance of the measured body twist [v, omega]
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

--- Feed the four wheel linear velocities (m/s) and the time step (s).
---@param timestamp number? seconds on the Unix clock for this measurement
function Odometry:update(vFL, vFR, vRL, vRR, dt, timestamp)
    local vLeft  = (vFL + vRL) * 0.5
    local vRight = (vFR + vRR) * 0.5

    local v     = (vLeft + vRight) * 0.5              -- forward speed, m/s
    local omega = (vRight - vLeft) / self.trackWidth  -- yaw rate, rad/s

    -- Integrate over dt with the midpoint heading (2nd order, accurate in turns).
    local dTheta = omega * dt
    local dFwd   = v * dt
    local mid    = self.theta + dTheta * 0.5

    -- EKF prediction for f([x,y,theta], [v,omega]). Wheel-speed noise is
    -- converted to body-twist noise above; the midpoint integration Jacobian
    -- keeps the uncertainty consistent with the pose integration below.
    local c, s = math.cos(mid), math.sin(mid)
    local fxTheta, fyTheta = -dFwd * s, dFwd * c
    local gxV, gyV = dt * c, dt * s
    local gxW, gyW = -dFwd * s * dt * 0.5, dFwd * c * dt * 0.5

    local pxx = self.pxx + 2 * fxTheta * self.pxt + fxTheta^2 * self.ptt
    local pxy = self.pxy + fxTheta * self.pyt + fyTheta * self.pxt
        + fxTheta * fyTheta * self.ptt
    local pxt = self.pxt + fxTheta * self.ptt
    local pyy = self.pyy + 2 * fyTheta * self.pyt + fyTheta^2 * self.ptt
    local pyt = self.pyt + fyTheta * self.ptt
    local ptt = self.ptt

    pxx = pxx + gxV^2 * self.cvv + 2 * gxV * gxW * self.cvw + gxW^2 * self.cww
    pxy = pxy + gxV * gyV * self.cvv
        + (gxV * gyW + gxW * gyV) * self.cvw + gxW * gyW * self.cww
    pxt = pxt + gxV * dt * self.cvw + gxW * dt * self.cww
    pyy = pyy + gyV^2 * self.cvv + 2 * gyV * gyW * self.cvw + gyW^2 * self.cww
    pyt = pyt + gyV * dt * self.cvw + gyW * dt * self.cww
    ptt = ptt + dt^2 * self.cww

    self.x     = self.x + dFwd * math.cos(mid)
    self.y     = self.y + dFwd * math.sin(mid)
    self.theta = (self.theta + dTheta) % (math.pi * 2)

    self.v, self.omega = v, omega
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
