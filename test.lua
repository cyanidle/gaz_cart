local PORT = {
    linear_speed = 7300,  -- module -> Pi, m/s
    speed_cmd    = 4000,  -- Pi -> module, m/s
    direct_cmd   = 4050,  -- Pi -> module, open-loop voltage (V)
    config       = 4100,  -- Pi -> module, { id, num, den }
}

local motor = Cyphal {
    can = CAN {
        plugin = "socketcan",
        device = "can0"
    },
    node_id = 103,
    publish = {
        cmd_1 = { type = "uavcan.primitive.scalar.Real32.1.0", port = PORT.direct_cmd  + 1 },
        cmd_2 = { type = "uavcan.primitive.scalar.Real32.1.0", port = PORT.direct_cmd  + 2 },
        cmd_3 = { type = "uavcan.primitive.scalar.Real32.1.0", port = PORT.direct_cmd  + 3 },
        cmd_4 = { type = "uavcan.primitive.scalar.Real32.1.0", port = PORT.direct_cmd  + 4 },
    },
}


local function send(val)
    motor {
        cmd_1 = {value = val},
        cmd_2 = {value = val},
        cmd_3 = {value = val},
        cmd_4 = {value = val},
    }
end

send(0)

on_shutdown(function ()
    send(0)
end)

local dir = 0
local scale = 4
-- Target for the current phase. The streamer below re-sends it at 10 Hz, so a
-- single lost CAN frame costs 100 ms instead of a whole phase — the original
-- version's one-shot send made any dropped frame look like a dead motor.
local target = 0
each(1000, function ()
    -- +scale -> 0 -> -scale -> 0 -> ... : never reverse instantaneously
    -- under load, an abrupt sign flip can trip the gate driver.
    if dir == 0 then
        target = scale
    elseif dir == 2 then
        target = -scale
    else
        target = 0
    end
    dir = (dir + 1) % 4
end)

each(100, function ()
    send(target)
end)


pipe(motor, function (msg)
    log(msg)
end)
