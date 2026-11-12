-- Self-checking test that every runtime-config parameter has tuner hover help in
-- both languages, that the help names a sketch the view can draw, and that the
-- parameter's own name and unit are localized too (the GUI tumbler switches
-- between them):
--   build/bin/radapter tests/tuner_help.lua
-- Exit status 0 on success; a failed assertion aborts with the reason.

package.path = SCRIPT_DIR .. "/../?.lua;" .. package.path

local defs = require "mods.config_defs"
local help = require "mods.tuner_help"
local i18n = require "mods.i18n"

assert(type(help.kinds) == "table", "tuner_help.kinds missing")
assert(type(help.params) == "table", "tuner_help.params missing")
assert(#i18n.langs == 2, "mods/i18n.lua must know exactly ru and en")

local count = 0
local kinds_used = {}
for key, def in pairs(defs) do
    local entry = help.params[key]
    assert(entry, "no tuner help for config parameter '" .. key .. "'")
    assert(type(entry.what) == "table", key .. ": 'what' must map language -> text")
    for _, lang in ipairs(i18n.langs) do
        local text = entry.what[lang]
        assert(type(text) == "string" and #text > 40,
            ("%s: 'what.%s' is missing or too short to explain the parameter"):format(key, lang))
    end
    assert(entry.what.ru ~= entry.what.en, key .. ": the Russian help is still the English text")
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

    -- the row's own label and unit, per language
    assert(type(def.label) == "string" and type(def.unit) == "string",
        key .. ": config_defs entry needs label and unit")
    assert(type(def.label_ru) == "string" and def.label_ru ~= "",
        key .. ": config_defs entry needs label_ru")
    assert(type(def.unit_ru) == "string" and def.unit_ru ~= "",
        key .. ": config_defs entry needs unit_ru")

    kinds_used[entry.chart] = true
    count = count + 1
end

for key in pairs(help.params) do
    assert(defs[key], "tuner help for unknown parameter '" .. key .. "'")
end

-- every status line the Lua side hands a view has to exist in both languages
local texts = 0
for _, key in ipairs({ "status_target", "status_stopped", "diag_v1", "status_wheel" }) do
    for _, lang in ipairs(i18n.langs) do
        i18n.set(lang)
        assert(#i18n.text(key) > 2, ("%s has no %s text"):format(key, lang))
    end
    texts = texts + 1
end
i18n.set("ru")

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

print(("tuner_help: ok (%d parameters in %d languages, %d sketch kinds, %d status lines%s)")
    :format(count, #i18n.langs, kinds_count, texts, detail))
os.exit(0)
