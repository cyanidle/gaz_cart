---@meta gaz_joystick
-- Lua LSP definitions for the native gaz_joystick plugin (libgaz_joystick).

---@class JoystickConfig : WorkerConfig
---@field path string? hidraw device node to read; defaults to "/dev/hidraw5"

---@class JoystickAxes normalised axes, -1..1; -1 is the low RC endpoint, 0 the centre
---@field x number
---@field y number
---@field z number
---@field rx number
---@field ry number
---@field rz number
---@field slider number
---@field dial number

---@class JoystickRaw
---@field axes integer[] raw 16-bit values as they arrive, one per axis
---@field buttons integer pressed buttons as a bitmask

---@class JoystickState emitted once per HID report
---@field axes JoystickAxes
---@field buttons integer[] pressed buttons, 1-based
---@field raw JoystickRaw

---@class Joystick : Worker<any, JoystickState>

---@param cfg JoystickConfig
---@return Joystick
function Joystick(cfg) end
