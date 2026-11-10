-- Joystick prototype: print the live controls of the Universal RC Joystick.
--   build/bin/radapter joystick/joystick.lua [/dev/hidraw5]
--
-- One line per change, e.g.
--   x=+0.00 y=-0.42 z=-1.00 rx=+0.05 ry=+0.00 rz=-1.00 slider=-1.00 dial=-1.00 buttons=-
--   x=+0.00 y=-0.42 z=-1.00 rx=+0.05 ry=+0.00 rz=-1.00 slider=-1.00 dial=-1.00 buttons=1

local path = args[1] or "/dev/hidraw5"

load_plugin("gaz_joystick")

local joystick = Joystick { name = "joystick", path = path }

local AXES = { "x", "y", "z", "rx", "ry", "rz", "slider", "dial" }

local function describe(msg)
    local parts = {}
    for _, name in ipairs(AXES) do
        local value = msg.axes[name]
        if value ~= nil then
            if math.abs(value) < 0.005 then value = 0 end
            parts[#parts + 1] = string.format("%s=%+.2f", name, value)
        end
    end
    local buttons = {}
    for _, button in ipairs(msg.buttons) do
        buttons[#buttons + 1] = tostring(button)
    end
    return string.format("%s buttons=%s", table.concat(parts, " "),
        #buttons > 0 and table.concat(buttons, ",") or "-")
end

local last
pipe(joystick, function(msg)
    local line = describe(msg)
    if line == last then return end
    last = line
    print(line)
end)

log.info("watching {}: move the sticks and press the buttons", path)
