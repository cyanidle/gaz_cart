-- Self-checking test that every runtime-config parameter has tuner hover help,
-- and that the help names a sketch the view can draw:
--   build/bin/radapter tests/tuner_help.lua
-- Exit status 0 on success; a failed assertion aborts with the reason.

package.path = SCRIPT_DIR .. "/../?.lua;" .. package.path

local defs = require "mods.config_defs"
local help = require "mods.tuner_help"

assert(type(help.kinds) == "table", "tuner_help.kinds missing")
assert(type(help.params) == "table", "tuner_help.params missing")

local count = 0
local kinds_used = {}
for key, def in pairs(defs) do
    local entry = help.params[key]
    assert(entry, "no tuner help for config parameter '" .. key .. "'")
    assert(type(entry.what) == "string", key .. ": 'what' must be a string")
    assert(#entry.what > 40, key .. ": 'what' is too short to explain the parameter")
    assert(type(entry.chart) == "string", key .. ": 'chart' must be a string")
    assert(help.kinds[entry.chart],
        ("%s: chart '%s' is not one of the kinds the view draws"):format(key, tostring(entry.chart)))
    if entry.role then
        assert(entry.role == "kp" or entry.role == "ki" or entry.role == "kd",
            key .. ": role must be kp, ki or kd for a step sketch")
    end
    if entry.chart == "step" then
        assert(entry.role, key .. ": a step sketch needs a role (kp/ki/kd)")
    end
    kinds_used[entry.chart] = true
    count = count + 1
end

for key in pairs(help.params) do
    assert(defs[key], "tuner help for unknown parameter '" .. key .. "'")
end

local unused = {}
for kind in pairs(help.kinds) do
    if not kinds_used[kind] then unused[#unused + 1] = kind end
end

local defs_count = 0
for _ in pairs(defs) do defs_count = defs_count + 1 end
assert(count == defs_count,
    ("expected %d help entries, got %d"):format(defs_count, count))

local detail = ""
if #unused > 0 then
    table.sort(unused)
    detail = ", kinds declared but not used: " .. table.concat(unused, " ")
end

local kinds_count = 0
for _ in pairs(help.kinds) do kinds_count = kinds_count + 1 end

print(("tuner_help: ok (%d parameters, %d sketch kinds%s)")
    :format(count, kinds_count, detail))
os.exit(0)
