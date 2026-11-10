-- =============================================================================
--  Service channel: a second websocket that exposes the cart's full runtime
--  state and accepts every command the GUI can send. It exists for bring-up,
--  bench debugging and remote diagnosis — the control GUI (port 6080) stays
--  what the operator uses.
--
--      radapter cart.lua can0            # cart
--      radapter clients wrapped around ws://CART:6081
--
--  Every period the node publishes a complete snapshot; a client that only
--  wants history sends { action = "state" } to get one immediately.
-- =============================================================================

---@class ServiceConfig
---@field model Pipable<any, ServiceCommand> the service websocket
---@field state fun(): table complete runtime snapshot
---@field drive fun(v: number, omega: number) normalized body twist, -1..1
---@field single_wheel fun(wheel: string?, v: number)? single-wheel bench mode, normalized
---@field set_speed fun(wheel: string?, value: number) closed-loop wheel speed, m/s
---@field set_config fun(wheel: string?, id: integer, value: number) runtime config value
---@field direct_voltage fun(wheel: string?, volts: number) open-loop voltage, V
---@field stop fun(stopped: boolean) joystick stop-organ gate
---@field period_ms integer? snapshot period (default 200)

---Wire the service channel.
---@param cfg ServiceConfig
return function(cfg)
    assert(cfg.model, "cfg.model required")
    assert(cfg.state, "cfg.state required")

    local function publish()
        cfg.model { state = cfg.state() }
    end

    pipe(cfg.model, function(msg)
        local action = msg.action
        if action == nil then return end
        log.info("service: {}", msg)
        if action == "state" then
            publish()
        elseif action == "drive" then
            cfg.drive(tonumber(msg.v) or 0.0, tonumber(msg.omega) or 0.0)
        elseif action == "wheel" then
            assert(cfg.single_wheel, "this cart has no single-wheel bench mode")
            cfg.single_wheel(msg.wheel, tonumber(msg.v) or 0.0)
        elseif action == "speed" then
            cfg.set_speed(msg.wheel, tonumber(msg.value) or 0.0)
        elseif action == "direct" then
            cfg.direct_voltage(msg.wheel, tonumber(msg.value) or 0.0)
        elseif action == "config" then
            local id = tonumber(msg.id)
            local value = tonumber(msg.value)
            assert(id, "config command needs an id")
            assert(value, "config command needs a numeric value")
            cfg.set_config(msg.wheel or "all", math.floor(id), value)
        elseif action == "stop" then
            cfg.stop(msg.value == true)
        else
            log.warn("service: unknown action '{}'", tostring(action))
        end
    end)

    each(cfg.period_ms or 200, publish)
    log.info("service channel: snapshot every {} ms, actions drive/wheel/speed/direct/config/stop/state",
        cfg.period_ms or 200)
end
