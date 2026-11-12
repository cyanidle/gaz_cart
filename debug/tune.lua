-- =============================================================================
--  debug/tune.lua — bench tuning GUI for one wheel module over its serial
--  channel (USART2 / the ST-LINK VCP).
--
--      radapter --gui debug/tune.lua [port=/dev/ttyACM0] [vmax=10] [log=<file>]
--                                     [lang=ru|en]
--
--  A thin launcher: the tuner itself is mods/tuner.lua and its window is
--  qml/TunerWindow.qml, the same pair gui.lua hosts (there it defaults to the
--  cart's CAN service channel; `serial=<port>` there opens this very window on
--  a bench port). Nothing that talks to a module lives here.
--
--  Sliders for every runtime-config parameter (mods/config_defs.lua ids, the
--  same ids the CAN config topic uses), hover help that sketches what each one
--  does (mods/tuner_help.lua), live charts of the `V1` diagnostics line (speed
--  target vs actual, the q and d voltages, the q and phase currents and the
--  bus), a NORMALIZED
--  target-speed slider (-1..1 of the module's full-scale wheel speed, config
--  id 16) and an open-loop-voltage slider, with a Reset that zeroes both
--  without touching the parameters, and the enable / zero / calibrate commands.
--  The m/s the slider commands is shown next to it, from the module's own
--  read-back of that full-scale value. The module's EEPROM keeps every accepted
--  value, so the sliders start from what the module runs now.
--
--  Options are `key=value` words, because radapter's own CLI owns the dashed
--  flags. Never point this at a powered cart: it commands the drive directly.
-- =============================================================================

local USAGE = [[
usage: radapter --gui debug/tune.lua [port=/dev/ttyACM0] [vmax=10] [log=<file>]
                                         [lang=ru|en]

  port    module's serial bench channel (USART2)
  vmax    direct-voltage slider range, V
  log     append every line the module sends to this file
  lang    UI language; Russian is the default, and every window carries a
          tumbler that switches all of them

The target-speed slider is normalized: -1..1 of the module's full-scale wheel
speed (config id 16), which the GUI reads back on connect and shows in m/s.
This is the same window gui.lua opens; gui.lua defaults to the cart's CAN
service channel and takes serial=<port> to come here instead.
]]

local opts = {
    port = "/dev/ttyACM0",
    vmax = 10.0,
    log = nil,
    lang = nil,   -- nil keeps mods/i18n.lua's default (Russian)
}

local function set_option(key, value)
    if key == "vmax" then
        local number = tonumber(value)
        if number == nil then
            error(("%s expects a number, got '%s'"):format(key, tostring(value)))
        end
        opts.vmax = number
    elseif key == "port" then
        opts.port = assert(value, "port expects a device node")
    elseif key == "log" then
        opts.log = assert(value, "log expects a file name")
    elseif key == "lang" then
        opts.lang = assert(value, "lang expects ru or en")
    else
        error(("unknown option '%s'; run with the word 'help' to list them"):format(key))
    end
end

for _, arg in ipairs(args) do
    if arg == "help" then
        print(USAGE)
        shutdown()
        return
    end
    local key, value = arg:match("^([%w%-]+)=(.*)$")
    if key then
        set_option((key:gsub("%-", "_")), value)
    else
        error(("unexpected argument '%s'; run with the word 'help' to list the options"):format(arg))
    end
end

if opts.vmax <= 0 then error("vmax must be positive") end

package.path = SCRIPT_DIR .. "/../?.lua;" .. package.path

if opts.lang then require("mods.i18n").set(opts.lang) end

local tuner = require("mods.tuner").new {
    transport = "serial",
    port = opts.port,
    vmax = opts.vmax,
    log_file = opts.log,
}

on_shutdown(function() tuner:park() end)
