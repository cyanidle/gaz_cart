-- =============================================================================
--  Operator-source switch: exactly one manual input drives the cart.
--
--  The joystick and the GUI's WASD window both want the same wheels, and their
--  commands arrive at different rates over different transports. The mux in
--  mods/drive_mux.lua decides per command, so an actively streaming source
--  keeps taking the wheels back — two live sources would alternate 20 times a
--  second. This switch picks ONE of them up front:
--
--    auto      the joystick while it is reporting, the keyboard otherwise
--    joystick  the joystick only (nothing drives when it is not reporting)
--    keyboard  the keyboard only (the joystick node is not even created)
--
--  "Reporting" means the joystick node has seen a report within its own
--  timeout — an unplugged stick hands the cart back to the keyboard.
-- =============================================================================

local TeleopSwitch = {}
TeleopSwitch.__index = TeleopSwitch

local MODES = { auto = true, joystick = true, keyboard = true }

---@class TeleopSwitchOpts
---@field mode string? "auto" (default), "joystick" or "keyboard"
---@field has_joystick boolean? the cart has a joystick configured
---@field on_change fun(source: string)? notified when the active source changes

---@class TeleopSwitch
---@field mode string
---@field has_joystick boolean
---@field reporting boolean
---@field source string "joystick", "keyboard" or "none"

---@param opts TeleopSwitchOpts
---@return TeleopSwitch
function TeleopSwitch.new(opts)
    opts = opts or {}
    local mode = opts.mode or "auto"
    assert(MODES[mode], ("teleop mode must be auto/joystick/keyboard, got %s"):format(tostring(mode)))
    local self = setmetatable({
        mode = mode,
        has_joystick = opts.has_joystick or false,
        reporting = false,
        on_change = opts.on_change,
        source = "none",
    }, TeleopSwitch)
    self.source = self:decide()
    return self
end

---@return string "joystick", "keyboard" or "none" (nobody may drive)
function TeleopSwitch:decide()
    if self.mode == "keyboard" then return "keyboard" end
    if self.has_joystick and self.reporting then return "joystick" end
    if self.mode == "auto" then return "keyboard" end
    return "none"
end

function TeleopSwitch:update()
    local source = self:decide()
    if source == self.source then return false end
    local previous = self.source
    self.source = source
    if self.on_change then self.on_change(source, previous) end
    return true
end

---Tell the switch whether the joystick node is currently receiving reports.
---@param reporting boolean
---@return boolean changed
function TeleopSwitch:joystick_reporting(reporting)
    reporting = reporting and true or false
    if reporting == self.reporting then return false end
    self.reporting = reporting
    return self:update()
end

---@return string the source that currently owns the manual input
function TeleopSwitch:active()
    return self.source
end

---@param source string "joystick" or "keyboard"
---@return boolean true when that source may drive right now
function TeleopSwitch:allows(source)
    return self.source == source
end

return TeleopSwitch
