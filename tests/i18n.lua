-- Self-checking test of the GUI localization:
--   1. qml/Tr/Tr.qml carries the same keys in Russian and English, all non-empty,
--      and no Russian entry is a leftover copy of the English one;
--   2. every Tr.t("key") / Tr.tf("key", …) the QML asks for exists;
--   3. every key in the table is actually used by a window (no dead text);
--   4. every key mods/i18n.lua carries for the Lua side has both languages.
--   build/bin/radapter tests/i18n.lua
-- Exit status 0 on success; a failed assertion aborts with the reason.

package.path = SCRIPT_DIR .. "/../?.lua;" .. package.path

local REPO = SCRIPT_DIR .. "/.."
local i18n = require "mods.i18n"

-- texts that are identical in both languages on purpose (placeholders and the
-- language codes themselves)
local SAME_IN_BOTH = { help_title = true, lang_en = true }

local function read(path)
    local file, err = io.open(path, "r")
    assert(file, ("cannot read %s: %s"):format(path, tostring(err)))
    local text = file:read("a")
    file:close()
    return text
end

---@param text string contents of Tr.qml
---@param lang string "ru" or "en"
---@return table<string, string>
local function table_of(text, lang)
    local entries = {}
    local collecting = false
    for line in text:gmatch("[^\n]+") do
        if not collecting and line:match("^%s*" .. lang .. ": {%s*$") then
            collecting = true
        elseif collecting then
            local key, value = line:match('^%s*([%w_]+):%s*"(.-)",%s*$')
            if key then
                entries[key] = value
            elseif line:match("^%s*},%s*$") then
                break
            end
        end
    end
    return entries
end

local tr_text = read(REPO .. "/qml/Tr/Tr.qml")
local ru = table_of(tr_text, "ru")
local en = table_of(tr_text, "en")

local ru_count, en_count = 0, 0
for _ in pairs(ru) do ru_count = ru_count + 1 end
for _ in pairs(en) do en_count = en_count + 1 end
assert(ru_count > 80, ("qml/Tr/Tr.qml parsed only %d Russian entries"):format(ru_count))
assert(ru_count == en_count,
    ("qml/Tr/Tr.qml has %d Russian and %d English entries"):format(ru_count, en_count))

for key, value in pairs(ru) do
    assert(value ~= "", ("qml/Tr/Tr.qml: '%s' has an empty Russian text"):format(key))
    local english = en[key]
    assert(english, ("qml/Tr/Tr.qml: '%s' exists in Russian but not in English"):format(key))
    assert(english ~= "", ("qml/Tr/Tr.qml: '%s' has an empty English text"):format(key))
    if value == english and not SAME_IN_BOTH[key] then
        error(("qml/Tr/Tr.qml: '%s' is identical in both languages (%s)"):format(key, value))
    end
    if key:match("^source_") == nil and not SAME_IN_BOTH[key] then
        assert(value:find("[%a\128-\255]"), ("'%s' has no text"):format(key))
    end
end
for key in pairs(en) do
    assert(ru[key], ("qml/Tr/Tr.qml: '%s' exists in English but not in Russian"):format(key))
end

-- every window's texts are in the table, and every used key exists
local qml_files = {
    "qml/LangSwitch.qml", "qml/ParamHelp.qml", "qml/Tuner.qml", "qml/TunerWindow.qml",
    "qml/WheelConfig.qml", "qml/WheelConfigWindow.qml", "qml/NavView.qml",
    "qml/NavWindow.qml", "qml/TeleopView.qml", "qml/TeleopWindow.qml",
}

local usage = {}
local requested = 0
for _, name in ipairs(qml_files) do
    local text = read(REPO .. "/" .. name)
    for key in text:gmatch('Tr%.t%("([%w_]+)"') do
        requested = requested + 1
        assert(ru[key], ("%s asks for Tr.t(\"%s\") but qml/Tr/Tr.qml has no such key")
            :format(name, key))
        usage[key] = true
    end
    for key in text:gmatch('Tr%.tf%("([%w_]+)"') do
        requested = requested + 1
        assert(ru[key], ("%s asks for Tr.tf(\"%s\") but qml/Tr/Tr.qml has no such key")
            :format(name, key))
        usage[key] = true
    end
    -- keys that reach t()/tf() through an expression still appear as literals
    for key in text:gmatch('"([%w_]+)"') do
        if ru[key] then usage[key] = true end
    end
end
assert(requested > 60, ("only %d Tr.t()/Tr.tf() call sites found"):format(requested))

for key in pairs(ru) do
    -- source_* names are looked up dynamically (Tr.sourceName("nav")), so they
    -- have no literal call site to find
    if not usage[key] and not SAME_IN_BOTH[key] and not key:match("^source_") then
        error(("qml/Tr/Tr.qml: '%s' is used by no window"):format(key))
    end
end

-- the Lua side: every text it can hand a view has both languages
local lua_texts = 0
for _, lang in ipairs(i18n.langs) do
    i18n.set(lang)
    for _, key in ipairs({
        "status_probing", "status_no_answer", "status_nothing_attached",
        "status_serial", "status_connected", "status_wheel", "status_service",
        "status_readback_serial_only", "status_enable_cart", "status_console_serial",
        "status_one_module", "status_target", "status_stopped", "status_rejected",
        "diag_v1", "diag_status", "driver_on", "driver_off", "enabled", "disabled",
        "fault_none", "fault_invalid_hall", "fault_invalid_supply",
        "fault_invalid_command", "fault_stalled", "fault_overcurrent", "fault_driver",
        "map_load_failed", "map_uploading", "map_saved", "map_save_failed",
        "map_requesting",
    }) do
        assert(#i18n.text(key) > 1, ("mods/i18n.lua: '%s' has no %s text"):format(key, lang))
        lua_texts = lua_texts + 1
    end
end
i18n.set("ru")

print(("i18n: ok (%d QML keys in 2 languages, %d call sites checked, %d Lua texts)")
    :format(ru_count, requested, lua_texts))
os.exit(0)
