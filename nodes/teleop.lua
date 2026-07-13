-- =============================================================================
--  Teleoperation node: receives twist commands from the TeleopView tab and
--  forwards them to the cart's drive function (body twist: v m/s, omega rad/s).
--
--  Talks to the GUI through cfg.model — a branch() worker namespaced under
--  "teleop" (the TeleopView model branch key in Main.qml).
-- =============================================================================

---@class TeleopConfig
---@field model Pipable GUI model node from branch(); sends wrapped, receives unwrapped
---@field drive fun(v: number, omega: number) body twist sink (m/s, rad/s)

---Wire the teleop node.
---@param cfg TeleopConfig
return function(cfg)
    pipe(cfg.model, function(msg)
        if msg.action == "twist" then
            cfg.drive(msg.v or 0, msg.omega or 0)
        end
    end)
end
