-- =============================================================================
--  Drive-source arbiter.
--
--  The planner, the GUI keyboard teleop, the joystick and the ROS bridge all
--  command the same wheels, so exactly one of them owns the cart at a time.
--  The owner is the last source that commanded a non-zero twist: a source that
--  streams standstill (the keyboard tab does, at 20 Hz) can neither stop nor
--  steer a stick that is actively driving, and a source hands the wheels back
--  by commanding zero — releasing the keys or centring the stick passes control
--  straight to the next source.
--
--  The stop organ gates every source at once: while it is engaged nothing is
--  applied and the wheels are parked.
-- =============================================================================

local DriveMux = {}
DriveMux.__index = DriveMux

local function clamp(value, low, high)
    return math.max(low, math.min(high, value or 0.0))
end

local function is_standstill(v, omega)
    return v == 0.0 and omega == 0.0
end

---@class DriveMuxState
---@field v number accepted normalized forward command, -1..1
---@field omega number accepted normalized yaw command, -1..1
---@field source string? source holding the wheels, nil when nobody does
---@field stopped boolean stop organ engaged: no source may command the wheels

---@class DriveMuxOpts
---@field apply fun(v: number, omega: number) sink that commands the wheels
---@field on_state fun(state: DriveMuxState)? receives the new state on every change

---@param opts DriveMuxOpts
---@return DriveMux
function DriveMux.new(opts)
    assert(opts and opts.apply, "DriveMux needs an apply(v, omega) sink")
    return setmetatable({
        apply = opts.apply,
        on_state = opts.on_state,
        v = 0.0,
        omega = 0.0,
        source = nil,
        stopped = false,
    }, DriveMux)
end

---@return DriveMuxState
function DriveMux:state()
    return { v = self.v, omega = self.omega, source = self.source, stopped = self.stopped }
end

function DriveMux:announce()
    if self.on_state then self.on_state(self:state()) end
end

---@param source string
---@param v number normalized forward command, clamped to -1..1
---@param omega number normalized yaw command, clamped to -1..1
---@return boolean applied
---@return string? reason "stopped" when the stop organ gated it, "idle" when a
---non-owner asked for standstill
function DriveMux:command(source, v, omega)
    assert(type(source) == "string" and source ~= "", "a command source is required")
    v = clamp(v, -1.0, 1.0)
    omega = clamp(omega, -1.0, 1.0)

    if self.stopped then return false, "stopped" end

    local standstill = is_standstill(v, omega)
    if self.source ~= source and standstill then return false, "idle" end

    local was_source, was_v, was_omega = self.source, self.v, self.omega
    self.source, self.v, self.omega = source, v, omega
    self.apply(v, omega)
    if standstill then self.source = nil end
    if was_source ~= self.source or was_v ~= v or was_omega ~= omega then
        self:announce()
    end
    return true
end

---@param stopped boolean
---@return boolean changed
function DriveMux:set_stopped(stopped)
    if self.stopped == stopped then return false end
    self.stopped = stopped
    self.source = nil
    self.v, self.omega = 0.0, 0.0
    self.apply(0.0, 0.0)
    self:announce()
    return true
end

return DriveMux
