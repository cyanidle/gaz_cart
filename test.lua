local PORT = {
    linear_speed = 7300,  -- module -> Pi, m/s
    speed_cmd    = 4000,  -- Pi -> module, m/s
    direct_cmd   = 4050,  -- Pi -> module, open-loop voltage (V)
    config       = 4100,  -- Pi -> module, { id, num, den }
}

local NODE = 103

local motor = Cyphal {
    can = CAN {
        plugin = "socketcan",
        device = "can0"
    },
    node_id = NODE,
    publish = {
        cmd_1 = { type = "uavcan.primitive.scalar.Real32.1.0", port = PORT.direct_cmd  + 1 },
        cmd_2 = { type = "uavcan.primitive.scalar.Real32.1.0", port = PORT.direct_cmd  + 2 },
        cmd_3 = { type = "uavcan.primitive.scalar.Real32.1.0", port = PORT.direct_cmd  + 3 },
        cmd_4 = { type = "uavcan.primitive.scalar.Real32.1.0", port = PORT.direct_cmd  + 4 },
    },
}

local dir = 0

motor {
    cmd_1 = {value = 0},
    cmd_2 = {value = 0},
    cmd_3 = {value = 0},
    cmd_4 = {value = 0},
}

local scale = 3

local function send(val)
    motor {
        ["cmd_"..1] = {
            value = val
        },
        ["cmd_"..2] = {
            value = val
        },
        ["cmd_"..3] = {
            value = val
        },
        ["cmd_"..4] = {
            value = val
        }
    }
end

each(1000, function ()
    local val
    if dir == 0 then
        val = scale
        dir = 1
    elseif dir == 1 then
        val = -scale
        dir = -1
    elseif dir == -1 then
        val = 0
        dir = 0
    end
    send(val)
end)

on_shutdown(function ()
    send(0)
end)

pipe(motor, function (msg)
    log(msg)
end)