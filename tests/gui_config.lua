-- Offscreen proof of the config window's wheel selection: the "all" entry is a
-- protocol word, not a label (cart.lua broadcasts only for that literal, and the
-- saved-config keys are "<wheel>:<id>"), so it must reach the wire untranslated
-- while the combo shows it in the window's language.
--
--     QT_QPA_PLATFORM=offscreen radapter --gui tests/gui_config.lua

local REPO = SCRIPT_DIR .. "/.."

local view = QML { url = REPO .. "/qml/WheelConfigWindow.qml" }

local sent = {}
pipe(view, function(msg) sent[#sent + 1] = json_decode(tostring(msg)) end)

view {
    odo = {
        params = {
            { key = "pid_kp", id = 1, label = "Velocity Kp", unit = "A/(rad/s)", default = 1.0 },
        },
        odomText = "odom  x=0.00  y=0.00",
    },
}

local qt = QML_Tester()
qt:wait(1200)
assert(qt:find("tgtWheelBox"), "the config window has no target-wheel selector")

---The last message of `action` the view sent, whatever namespace wraps it.
local function last_action(action)
    for i = #sent, 1, -1 do
        local msg = sent[i]
        if type(msg) == "table" then
            local odo = msg.odo or msg
            if odo.action == action then return odo end
        end
    end
    return nil
end

local function submit(field, text)
    qt:click_item(field)
    qt:wait(100)
    qt:set_prop(field, "text", text)
    qt:key_click("Return")
    qt:wait(250)
end

local function current_text(name)
    return tostring(qt:prop(name, "currentText"))
end

-- ---- Russian by default, and the id stays "all" ------------------------------
assert(current_text("tgtWheelBox") == "все",
    "the wheel selector is not Russian by default: " .. current_text("tgtWheelBox"))

submit("tgtField", "0.25")
local speed = last_action("set_speed")
assert(speed, "the target-speed field sent nothing")
assert(speed.wheel == "all",
    "the localized 'all' label reached the wire as a wheel id: " .. tostring(speed.wheel))
assert(math.abs(speed.value - 0.25) < 1e-6,
    "the target speed was not sent: " .. tostring(speed.value))

submit("dirField", "0.5")
local direct = last_action("direct")
assert(direct, "the direct-voltage field sent nothing")
assert(direct.wheel == "all",
    "the localized 'all' label reached the wire as a wheel id: " .. tostring(direct.wheel))

submit("field_pid_kp", "2.5")
local config = last_action("config")
assert(config, "the config form sent nothing")
assert(config.wheel == "all",
    "the localized 'all' label reached the wire as a wheel id: " .. tostring(config.wheel))
assert(math.floor(config.id) == 1, "the config form sent the wrong id: " .. tostring(config.id))

-- ---- one wheel selected ------------------------------------------------------
qt:set_prop("tgtWheelBox", "currentIndex", 2)
qt:wait(150)
assert(current_text("tgtWheelBox") == "fr", "the selector did not move to fr")
submit("tgtField", "0.1")
speed = last_action("set_speed")
assert(speed.wheel == "fr", "a single wheel was not addressed by its own id: "
    .. tostring(speed.wheel))

-- ---- the switch relabels the entry, not the id -------------------------------
qt:set_prop("tgtWheelBox", "currentIndex", 0)
qt:click_item("langSwitch")
qt:wait(600)
assert(current_text("tgtWheelBox") == "all",
    "the 'all' entry did not switch to English: " .. current_text("tgtWheelBox"))
submit("tgtField", "0.2")
speed = last_action("set_speed")
assert(speed.wheel == "all",
    "the English 'all' label reached the wire as a wheel id: " .. tostring(speed.wheel))

qt:click_item("langSwitch")
qt:wait(600)
assert(current_text("tgtWheelBox") == "все",
    "the 'all' entry did not switch back to Russian: " .. current_text("tgtWheelBox"))

print("gui_config: ok")
os.exit(0)
