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
        cmd = { type = "uavcan.primitive.scalar.Real32.1.0",   port = PORT.direct_cmd  + 1 }
    },
    subscribe = {
        speed = { type = "uavcan.primitive.scalar.Real32.1.0",   port = PORT.linear_speed + 1 }
    }
}

local dir = 0

motor {
    cmd = {
        value = 0
    }
}

each(1000, function ()
    if dir == 0 then
        motor {
            cmd = {
                value = 2
            }
        }
        dir = 1
    elseif dir == 1 then
        motor {
            cmd = {
                value = -2
            }
        }
        dir = -1
    elseif dir == -1 then
        motor {
            cmd = {
                value = 0
            }
        }
        dir = 0
    end
end)


pipe(motor, function (msg, source)
    log(msg)
end)