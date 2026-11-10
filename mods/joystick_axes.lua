-- Axis shaping and the stop organ shared by the joystick inputs: nodes/joystick.lua
-- drives the whole cart, debug/motor.lua a single wheel module on the bench.

local JoystickAxes = {}

---Rescale one raw axis from the deadzone to 0..1, keeping its sign.
---@param axes table<string, number>
---@param name string
---@param invert boolean?
---@param deadzone number? normalized deadzone around the axis centre, 0..1 (default 0.05)
---@return number? normalized -1..1, nil when the report has no such axis
function JoystickAxes.value(axes, name, invert, deadzone)
    deadzone = deadzone or 0.05
    assert(deadzone >= 0 and deadzone < 1, "deadzone must be in 0..1")
    local value = axes[name]
    if value == nil then return nil end
    if invert then value = -value end
    local magnitude = math.abs(value)
    if magnitude <= deadzone then return 0.0 end
    magnitude = math.min((magnitude - deadzone) / (1 - deadzone), 1.0)
    return value < 0 and -magnitude or magnitude
end

---Read the stop organ off one axis. The two thresholds give hysteresis: inside
---the band the reading is inconclusive, so the caller keeps its previous state.
---@param axes table<string, number>
---@param name string axis used as the stop organ, "" disables it
---@param below number value at or below which the organ engages
---@param above number value at or above which it releases
---@return boolean? true = engage, false = release, nil = disabled, absent axis or inside the band
function JoystickAxes.stop_switch(axes, name, below, above)
    if name == "" then return nil end
    assert(below < above, "stop thresholds must be ordered below < above")
    local value = axes[name]
    if value == nil then return nil end
    if value <= below then return true end
    if value >= above then return false end
    return nil
end

return JoystickAxes
