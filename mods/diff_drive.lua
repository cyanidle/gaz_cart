-- Differential-drive command math kept independent from cart wiring.

local Drive = {}

local function clamp(value, low, high)
    return math.max(low, math.min(high, value or 0.0))
end

---@class DiffDriveLimits
---@field track_width number left-to-right wheel spacing, m
---@field max_linear number linear speed at a normalized command of 1, m/s
---@field max_angular number yaw speed at a normalized command of 1, rad/s
---@field linear_scale number? simulation/drive gain, default 1
---@field angular_scale number? simulation/drive gain, default 1

---Convert normalized body commands into physical wheel targets.
---@param v number normalized forward command, -1..1
---@param omega number normalized yaw command, -1..1
---@param limits DiffDriveLimits
---@return table<string, number> wheels m/s, keyed fl/fr/rl/rr
---@return number linear m/s
---@return number angular rad/s
function Drive.wheels(v, omega, limits)
    local linear = clamp(v, -1, 1) * limits.max_linear * (limits.linear_scale or 1)
    local angular = clamp(omega, -1, 1) * limits.max_angular * (limits.angular_scale or 1)
    local half = limits.track_width * 0.5
    local left = linear - angular * half
    local right = linear + angular * half
    return { fl = left, fr = right, rl = left, rr = right }, linear, angular
end

return Drive
