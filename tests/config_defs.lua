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

print("config_defs: ok (" .. count .. " ids)")
os.exit(0)
