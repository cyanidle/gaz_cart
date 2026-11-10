-- =============================================================================
--  Joystick teleop node: the Universal RC Joystick (the gaz_joystick plugin,
--  see joystick/README.md) drives the cart — the throttle axis sets the forward
--  speed, the other axis of the same stick the turn rate.
--
--  The stop organ is the bench script's one (debug/motor.lua): the stop axis at
--  its low extreme parks the cart and gates every other drive source, and the
--  band up to resume_above is hysteresis, so a channel hovering at the edge
--  cannot flap the cart. Reports that stop arriving hold the cart at zero.
-- =============================================================================

local socket = require "socket"
local JoystickAxes = require "mods.joystick_axes"

---@class JoystickTeleopConfig
---@field drive fun(v: number, omega: number) normalized twist sink, -1..1
---@field stop fun(stopped: boolean) safety gate: true parks the cart and blocks every source
---@field path string hidraw node of the joystick
---@field throttle_axis string? axis setting the forward speed (default "y")
---@field steer_axis string? axis setting the turn rate (default "x")
---@field deadzone number? normalized deadzone around an axis centre, 0..1 (default 0.05)
---@field linear_limit number? fraction of the cart's max speed at full deflection (default 1)
---@field angular_limit number? fraction of the cart's max turn rate (default 1)
---@field invert_throttle boolean? flip the throttle direction (default false)
---@field invert_steer boolean? flip the steering direction (default false)
---@field stop_axis string? axis used as the stop organ, "" disables it (default "slider")
---@field stop_below number? stop-axis value at or below which the cart is parked (default -0.8)
---@field resume_above number? stop-axis value at or above which control returns (default -0.5)
---@field report_timeout_ms integer? joystick silence that holds the cart at zero (default 500)

---Wire the joystick teleop node.
---@param cfg JoystickTeleopConfig
return function(cfg)
    assert(cfg.drive, "cfg.drive required")
    assert(cfg.stop, "cfg.stop required")
    assert(cfg.path, "cfg.path required")

    local throttle_axis = cfg.throttle_axis or "y"
    local steer_axis = cfg.steer_axis or "x"
    local deadzone = cfg.deadzone or 0.05
    local linear_limit = cfg.linear_limit or 1.0
    local angular_limit = cfg.angular_limit or 1.0
    local stop_axis = cfg.stop_axis
    if stop_axis == nil then stop_axis = "slider" end
    local stop_below = cfg.stop_below or -0.8
    local resume_above = cfg.resume_above or -0.5
    local report_timeout_ms = cfg.report_timeout_ms or 500

    if deadzone < 0 or deadzone >= 1 then
        error(("deadzone must be in 0..1, got %s"):format(deadzone))
    end
    if stop_axis ~= "" and stop_below >= resume_above then
        error("stop_below must be lower than resume_above, otherwise the stop organ has no hysteresis")
    end

    local function axis_names(axes)
        local names = {}
        for name in pairs(axes) do names[#names + 1] = name end
        table.sort(names)
        return table.concat(names, ", ")
    end

    load_plugin("gaz_joystick")
    local joystick = Joystick { name = "joystick", path = cfg.path }

    -- Fail safe: the organ's position is unknown until a report arrives, so the
    -- cart stays parked until the stop axis confirms it is released.
    local stop_state -- true while the organ gates the cart, nil when it is disabled
    if stop_axis ~= "" then
        stop_state = true
        cfg.stop(true)
    end
    local missing = {} -- axes already reported as absent
    local received = false
    local lost = false
    local last_report = socket.gettime()

    ---Engage or release the stop organ. Inside the hysteresis band the previous
    ---state is kept.
    ---@param axes table<string, number>
    local function read_stop(axes)
        local stopped = JoystickAxes.stop_switch(axes, stop_axis, stop_below, resume_above)
        if stopped == nil or stopped == stop_state then return end
        stop_state = stopped
        cfg.stop(stopped)
        if stopped then
            log.warn("stop organ engaged: cart parked until '{}' >= {}", stop_axis, resume_above)
        else
            log.info("stop organ released: the cart can be driven again")
        end
    end

    pipe(joystick, function(msg)
        last_report = socket.gettime()
        received = true
        lost = false
        read_stop(msg.axes)
        if stop_state then return end

        local throttle = JoystickAxes.value(msg.axes, throttle_axis, cfg.invert_throttle, deadzone)
        local steer = JoystickAxes.value(msg.axes, steer_axis, cfg.invert_steer, deadzone)
        local absent = nil
        if throttle == nil then absent = throttle_axis end
        if steer == nil then absent = steer_axis end
        if absent then
            if not missing[absent] then
                missing[absent] = true
                log.error("joystick report has no axis '{}' (it has: {})", absent, axis_names(msg.axes))
            end
            cfg.drive(0.0, 0.0)
            return
        end
        cfg.drive(throttle * linear_limit, steer * angular_limit)
    end)

    each(math.max(50, math.floor(report_timeout_ms / 4)), function()
        if not received then return end
        if socket.gettime() - last_report < report_timeout_ms / 1000 then return end
        if not lost then
            lost = true
            log.warn("no joystick reports for {} ms: holding the cart at zero", report_timeout_ms)
        end
        cfg.drive(0.0, 0.0)
    end)

    log.info("joystick {}: throttle '{}', steer '{}' (limits {} / {}), stop when '{}' <= {}",
        cfg.path, throttle_axis, steer_axis, linear_limit, angular_limit,
        stop_axis == "" and "-" or stop_axis, stop_below)
    if stop_state then
        log.info("cart parked until '{}' confirms the stop organ is released", stop_axis)
    end
end
