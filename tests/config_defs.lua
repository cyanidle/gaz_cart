-- Self-checking test that mods/config_defs.lua still matches the firmware's
-- ConfigId enum (firmware/App/config_ids.h), including CFG_ID_COUNT:
--   build/bin/radapter tests/config_defs.lua
-- Exit status 0 on success; a failed assertion aborts with the reason.

package.path = SCRIPT_DIR .. "/../?.lua;" .. package.path
local defs = require "mods.config_defs"

local firmware_name = {
    wheel_diameter = "WHEEL_DIAMETER",
    pid_kp         = "PID_KP",
    pid_ki         = "PID_KI",
    pid_kd         = "PID_KD",
    pid_i_limit    = "PID_I_LIMIT",
    oc_peak_a      = "OC_PEAK_A",
    oc_peak_ms     = "OC_PEAK_MS",
    oc_sustain_a   = "OC_SUSTAIN_A",
    oc_sustain_ms  = "OC_SUSTAIN_MS",
    stall_current  = "STALL_CURRENT",
    stall_speed    = "STALL_SPEED",
    stall_ms       = "STALL_MS",
    vel_lpf        = "VEL_LPF",
    speed_ramp     = "SPEED_RAMP",
    torque_ramp    = "TORQUE_RAMP",
    brake_current  = "BRAKE_CURRENT",
    max_speed      = "MAX_SPEED",
}

local enum, id_count = {}, nil
local header = assert(
    io.open(SCRIPT_DIR .. "/../firmware/App/config_ids.h", "r"),
    "cannot read firmware/App/config_ids.h"
)
for line in header:lines() do
    local name, value = line:match("^%s*CFG_([%u%d_]+)%s*=%s*(%d+)")
    if name == "ID_COUNT" then
        id_count = tonumber(value)
    elseif name then
        enum[name] = tonumber(value)
    end
end
header:close()

assert(id_count, "config_ids.h has no CFG_ID_COUNT")
assert(next(enum), "config_ids.h has no CFG_ ids")

local count = 0
for key, param in pairs(defs) do
    local name = assert(firmware_name[key],
        "config_defs entry '" .. key .. "' has no firmware name in this test")
    local firmware_id = assert(enum[name], "config_ids.h has no CFG_" .. name)
    assert(param.id == firmware_id,
        ("id mismatch for %s: config_defs %d, CFG_%s %d")
            :format(key, param.id, name, firmware_id))
    enum[name] = nil
    count = count + 1
end

local leftovers = {}
for name in pairs(enum) do
    leftovers[#leftovers + 1] = "CFG_" .. name
end
table.sort(leftovers)
assert(#leftovers == 0,
    "config_ids.h ids missing from config_defs: " .. table.concat(leftovers, " "))
assert(count == id_count,
    ("config_defs has %d ids, CFG_ID_COUNT is %d"):format(count, id_count))

-- The compiled-default backend is SimpleFOC, so the gain defaults and the
-- derived integral clamp have to be the numbers a module reports on G4, not
-- just plausible ones: id 4 is ki * integral_error_lim in the firmware.
local pid = {}
local in_simplefoc = false
local tuning = assert(
    io.open(SCRIPT_DIR .. "/../firmware/App/motor_tuning.h", "r"),
    "cannot read firmware/App/motor_tuning.h"
)
for line in tuning:lines() do
    if line:match("^#elif%s+defined%(VB_MOTOR_IMPL_SIMPLEFOC%)") then
        in_simplefoc = true
    elseif in_simplefoc and line:match("^#else") then
        break
    elseif in_simplefoc then
        local field, value = line:match("^%s*%.([%w_]+)%s*=%s*([%d%.]+)f")
        if field then
            pid[field] = tonumber(value)
        end
    end
end
tuning:close()

assert(pid.kp and pid.ki and pid.integral_error_lim,
    "cannot parse the SimpleFOC velocity-PID defaults from motor_tuning.h")
for _, case in ipairs {
    { "pid_kp", pid.kp }, { "pid_ki", pid.ki }, { "pid_kd", pid.kd or 0.0 },
} do
    local key, firmware = case[1], case[2]
    assert(math.abs(defs[key].default - firmware) < 1e-9,
        ("default mismatch for %s: config_defs %s, firmware %s")
            :format(key, defs[key].default, firmware))
end
local i_limit = pid.ki * pid.integral_error_lim
assert(math.abs(defs.pid_i_limit.default - i_limit) < 1e-9,
    ("default mismatch for pid_i_limit: config_defs %s, firmware ki * integral_error_lim = %s")
        :format(defs.pid_i_limit.default, i_limit))

print("config_defs: ok (" .. count .. " ids, gains match the SimpleFOC defaults)")
os.exit(0)
