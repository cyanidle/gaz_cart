-- =============================================================================
--  mods/tuner.lua — one bench tuner for two transports.
--
--  The same parameter form, drive controls and charts drive
--    serial  a single module over its USART2 bench channel (Serial worker), and
--    can     the cart over its service websocket (cart.lua, port 6081), which
--            forwards config writes to the modules on the Cyphal config topic.
--
--  Entry points: gui.lua (the cart GUI, `can` by default and `serial=<port>`
--  for a module on the bench) and debug/tune.lua (serial bench launcher). Both
--  use this module and qml/TunerWindow.qml — there is no second tuner.
--
--      local tuner = require "mods.tuner".new{ transport = "can", wheel = "fl" }
--
--  View contract (qml/Tuner.qml reads these model keys):
--    params          array ordered by config id, each with its hover help
--    max_speed_mps   full scale a normalized 1.0 means: read back from the
--                    module (config id 16) on serial, the cart's own constant
--                    on CAN, where nothing is read back
--    wheel_diameter_m, max_voltage, min_voltage, wheel, wheels, transport
--    diagnostics     true when per-phase telemetry exists (serial only)
--    can_enable      true when the bridge enable can be commanded (serial)
--    readback        true when the values shown were read from the module
--    status          one line of text for the bottom bar
--    diag            { tgt_m_s, act_m_s, iq, uq, ud, im, bus, t_ms } — latest
--                    sample; ud/im/t_ms only exist on the `V1` serial line
--
--  A second window (qml/DiagWindow.qml → qml/Diag.qml) rides on the same model,
--  opened by the tuner's Diagnostics button — created on the first press, raised
--  afterwards; its keys are:
--    counters        true when `K`/`W` exist at all (the serial bench channel;
--                    the service channel carries a state snapshot only)
--    text            the module's own output, newest last, MAX_OUTPUT_LINES kept
--    log_active      a recording is open
--    log_path        file the next (or the running) recording writes to
--    status          one line for the window's bottom bar
--    raise           bumped on every press: the window is shown and raised, so
--                    one the operator closed comes back
--  Messages it sends are the tuner's own: { line = "K"|"W"|"W1" } (serial),
--  { record = "start", path = … } / { record = "stop" } on either transport, and
--  the tuner window's { open = "diag" } — the button that opens it.
-- =============================================================================

local config_defs = require "mods.config_defs"
local tuner_help = require "mods.tuner_help"
local i18n = require "mods.i18n"

local MODULE_DIR = (debug.getinfo(1, "S").source:gsub("^@", "")):match("^(.*)[/\\][^/\\]*$")
local REPO_DIR = MODULE_DIR .. "/.."

local Tuner = {}
Tuner.__index = Tuner

local FAULT_NAMES = {
    "fault_none", "fault_invalid_hall", "fault_invalid_supply",
    "fault_invalid_command", "fault_stalled", "fault_overcurrent", "fault_driver",
}

-- The service channel addresses one wheel of a four-module cart.
local CAN_WHEELS = { "fl", "fr", "rl", "rr" }

-- How many of the module's own output lines the diagnostics window keeps. A
-- loop-trace dump alone is a few hundred lines, so the scrollback holds more
-- than one `K` block and drops the oldest.
local MAX_OUTPUT_LINES = 400

-- How often the diagnostics scrollback is handed to the view, ms: a trace dump
-- is delivered a few lines per poll pass, so one push per dump would re-send the
-- whole ring dozens of times.
local OUTPUT_PUSH_MS = 100

---Directory a session's recording lands in: the same per-user data directory the
---GUI keeps its sent-config state in, never the repository checkout.
local function log_dir()
    local base = os.getenv("XDG_DATA_HOME")
    if base == nil or base == "" then
        base = (os.getenv("HOME") or ".") .. "/.local/share"
    end
    local dir = base .. "/gaz-gui"
    local ok, lfs = pcall(require, "lfs")
    if ok and lfs then
        pcall(lfs.mkdir, base)
        pcall(lfs.mkdir, dir)
    end
    return dir
end

---Default recording file, one per start; the window shows it and the operator
---can edit it before pressing Start.
local function default_log_path()
    return ("%s/tune-%s.log"):format(log_dir(), os.date("%Y%m%d-%H%M%S"))
end

---Parameters the tuner shows: config_defs plus the hover help, in the active
---language (mods/i18n.lua).
---@return table[] params ordered by id
local function build_params()
    local params = {}
    for key, def in pairs(config_defs) do
        local help = assert(tuner_help.params[key], "no tuner help for config id " .. def.id)
        params[#params + 1] = {
            key = key, id = def.id, label = i18n.label(def), unit = i18n.unit(def),
            min = def.min, max = def.max, step = def.step,
            value = def.default,
            help = { what = i18n.help(help), chart = help.chart, role = help.role },
        }
    end
    table.sort(params, function(a, b) return a.id < b.id end)
    return params
end

---@param opts table transport, port/url, wheel, vmax, log_file, on_config_sent
function Tuner.new(opts)
    opts = opts or {}
    local self = setmetatable({}, Tuner)

    self.transport = opts.transport or "serial"
    assert(self.transport == "serial" or self.transport == "can",
        "transport must be 'serial' or 'can'")
    self.wheels = self.transport == "can" and CAN_WHEELS or {}
    self.wheel = opts.wheel or "fl"
    if self.transport == "can" then
        assert(self:_is_wheel(self.wheel),
            ("unknown wheel '%s' (expected fl/fr/rl/rr)"):format(tostring(self.wheel)))
    end
    self.vmax = opts.vmax or 10.0
    assert(self.vmax > 0, "vmax must be positive")

    self.params = build_params()
    self.by_id = {}
    for _, param in ipairs(self.params) do
        self.by_id[param.id] = param
    end
    self.max_speed_m_s = config_defs.max_speed.default
    self.wheel_diameter_m = config_defs.wheel_diameter.default
    self.on_config_sent = opts.on_config_sent
    self.on_lang = opts.on_lang

    -- The diagnostics window is not opened until the operator asks for it: it
    -- holds the module's own output (`K` counters, the loop-trace dump, EEPROM
    -- and boot lines) and the record controls, which a tuning session does not
    -- need on screen the whole time.
    self.view = QML { url = opts.view or (REPO_DIR .. "/qml/TunerWindow.qml") }

    self.output = {}
    self.log_file = nil
    self.log_path = opts.log_file
    self.diag_url = opts.diag_view or (REPO_DIR .. "/qml/DiagWindow.qml")
    self.diag_status_text = i18n.text("diag_idle")
    self.diag_raise = 0

    self.view {
        transport = self.transport,
        wheel = self.wheel,
        wheels = self.wheels,
        params = self.params,
        max_speed_mps = self.max_speed_m_s,
        wheel_diameter_m = self.wheel_diameter_m,
        max_voltage = self.vmax,
        min_voltage = -self.vmax,
        diagnostics = self.transport == "serial",
        can_enable = self.transport == "serial",
        readback = self.transport == "serial",
        status = i18n.text("status_connecting"),
        lang = i18n.lang,
    }

    -- The serial probe can take 20 s: a language throw must not fall on the
    -- floor meanwhile, while anything that would touch the transport has to
    -- wait until it exists. Both windows speak to the same model, so the
    -- routing is shared.
    self.opening = true
    pipe(self.view, function(msg) return self:_dispatch(msg) end)

    if self.log_path then
        -- `log=` on the command line starts recording before the probe's first
        -- line arrives; the window can start and stop it afterwards.
        self:_set_log("start", self.log_path)
    end

    if self.transport == "serial" then
        self:_open_serial(opts)
    else
        self:_open_can(opts)
    end

    self.opening = false
    return self
end

-- ---- Shared state ------------------------------------------------------------

---@param name string
---@return boolean
function Tuner:_is_wheel(name)
    for _, wheel in ipairs(self.wheels) do
        if wheel == name then return true end
    end
    return false
end

function Tuner:_status(text)
    self.view { status = text }
end

---One message from either window. During the serial probe only the local
---choices go through — a language throw must not fall on the floor, a recording
---is not tied to the transport, and the diagnostics window is a window: the
---operator can open it on a link that is still coming up.
---@param msg table
function Tuner:_dispatch(msg)
    if self.opening then
        if msg.lang ~= nil then self:set_lang(msg.lang) end
        if msg.record ~= nil then
            self:_set_log(tostring(msg.record), msg.path and tostring(msg.path) or nil)
        end
        if msg.open == "diag" then self:open_diagnostics() end
        return nil
    end
    return self:_on_view(msg)
end

---Create the diagnostics window the first time the operator asks for it, and
---bring it to the front on every later press.
function Tuner:open_diagnostics()
    if not self.diag_view then
        self.diag_view = QML { url = self.diag_url }
        pipe(self.diag_view, function(msg) return self:_dispatch(msg) end)
    end
    self.diag_raise = self.diag_raise + 1
    self:_push_diagnostics()
end

---Hand the diagnostics window its whole state: it may have been opened long
---after the lines it shows were collected.
function Tuner:_push_diagnostics()
    if not self.diag_view then return end
    self.diag_view {
        transport = self.transport,
        counters = self.transport == "serial",
        log_active = self.log_file ~= nil,
        log_path = self.log_path or default_log_path(),
        text = table.concat(self.output, "\n"),
        status = self.diag_status_text,
        lang = i18n.lang,
        raise = self.diag_raise,
    }
end

---Status line of the diagnostics window (counters, recording).
---@param text string
function Tuner:_diag_status(text)
    self.diag_status_text = text
    if self.diag_view then self.diag_view { status = text } end
end

---Append one line of the module's own output to the diagnostics scrollback.
---@param line string
function Tuner:_output_line(line)
    local lines = self.output
    lines[#lines + 1] = line
    if #lines > MAX_OUTPUT_LINES then table.remove(lines, 1) end
    if not self.diag_view then return end
    -- A trace dump arrives a few lines per poll pass, and re-sending the whole
    -- scrollback for each of them is quadratic: refresh the view at most once
    -- per OUTPUT_PUSH_MS and always with the newest ring.
    self.output_pending = true
    if self.output_scheduled then return end
    self.output_scheduled = true
    after(OUTPUT_PUSH_MS, function()
        self.output_scheduled = false
        if not self.output_pending or not self.diag_view then return end
        self.output_pending = false
        self.diag_view { text = table.concat(self.output, "\n") }
    end)
end

---Record one delivered message: the module's line on a serial bench channel, one
---JSON object per service-channel snapshot over CAN.
---@param text string
function Tuner:_log_record(text)
    if not self.log_file then return end
    self.log_file:write(text, "\n")
    self.log_file:flush()
end

---Start or stop recording what the transport delivers.
---@param command string "start" or "stop"
---@param path string? file to append to; the window's own default when absent
function Tuner:_set_log(command, path)
    if command == "stop" then
        if not self.log_file then
            self:_diag_status(i18n.text("diag_log_off"))
            return
        end
        self.log_file:close()
        self.log_file = nil
        if self.diag_view then self.diag_view { log_active = false } end
        self:_diag_status(i18n.text("diag_log_off"))
        log.info("tuner: recording stopped")
        return
    end
    if self.log_file then
        self:_diag_status(i18n.fmt("diag_log_on", tostring(self.log_path)))
        return
    end
    local target = (path ~= nil and path ~= "") and path or self.log_path or default_log_path()
    local file, err = io.open(target, "a")
    if not file then
        log.error("tuner: cannot record to {}: {}", target, tostring(err))
        self:_diag_status(i18n.fmt("diag_log_failed", tostring(target)))
        return
    end
    self.log_file = file
    self.log_path = target
    if self.diag_view then self.diag_view { log_active = true, log_path = target } end
    self:_diag_status(i18n.fmt("diag_log_on", target))
    log.info("tuner: recording to {}", target)
end

---Pick which wheel the CAN transport drives. The previous wheel is zeroed
---first, so switching never leaves a wheel spinning.
---@param name string
function Tuner:_select_wheel(name)
    if self.transport ~= "can" then
        self:_status(i18n.text("status_one_module"))
        return
    end
    if not self:_is_wheel(name) then
        log.warn("tuner: unknown wheel '{}'", tostring(name))
        return
    end
    if name == self.wheel then return end
    self:_send_target(0.0)
    self:_send_voltage(0.0)
    self.wheel = name
    self.view { wheel = name }
    self:_status(i18n.fmt("status_wheel", name))
    self.client { action = "state" }
end

---Switch the language of everything the model carries (parameter names, units,
---hover help, status lines) and hand the view the re-labelled parameter list.
---The window chrome switches inside QML, on the shared Tr singleton.
---@param lang string "ru" or "en"
function Tuner:set_lang(lang)
    if lang == nil then return end
    i18n.set(lang)
    local values = {}
    for _, param in ipairs(self.params) do values[param.id] = param.value end
    self.params = build_params()
    self.by_id = {}
    for _, param in ipairs(self.params) do
        if values[param.id] ~= nil then param.value = values[param.id] end
        self.by_id[param.id] = param
    end
    self.view { params = self.params, lang = i18n.lang }
    if self.diag_view then self.diag_view { lang = i18n.lang } end
    log.info("tuner: language {}", i18n.lang)
    if self.on_lang then self.on_lang(i18n.lang) end
end

---Track a value the operator set, without echoing the parameter list back: the
---slider is under the operator's hand, and a model reset would drop the drag.
function Tuner:_set_value(id, value)
    local param = self.by_id[id]
    if not param then return end
    param.value = value
    if id == config_defs.wheel_diameter.id then
        self.wheel_diameter_m = value
        self.view { wheel_diameter_m = value }
    elseif id == config_defs.max_speed.id then
        self.max_speed_m_s = value
        self.view { max_speed_mps = value }
    end
end

---Commands the view sends; the transport decides how each one travels.
function Tuner:_on_view(msg)
    -- a language throw is a local UI choice: it must work even when the link
    -- never came up, otherwise the window would be half translated
    if msg.lang ~= nil then
        self:set_lang(msg.lang)
        return nil
    end
    -- opening the diagnostics window is a local UI choice as well: it must work
    -- on a link that never came up, it just shows what the module never sent
    if msg.open == "diag" then
        self:open_diagnostics()
        return nil
    end
    if self.inert and not msg.park then
        self:_status(i18n.fmt("status_nothing_attached", tostring(self.port)))
        return nil
    end
    if msg.set then
        self:_set_value(msg.set.id, msg.set.value)
        self:_send_config(msg.set.id, msg.set.value)
    elseif msg.wheel ~= nil then
        self:_select_wheel(msg.wheel)
    elseif msg.target ~= nil then
        self:_send_target(math.max(-1.0, math.min(1.0, msg.target)))
    elseif msg.voltage ~= nil then
        self:_send_voltage(msg.voltage)
    elseif msg.enable ~= nil then
        self:_send_enable(msg.enable == true)
    elseif msg.zero then
        self:_send_target(0.0)
        self:_send_voltage(0.0)
    elseif msg.readback then
        self:_read_back()
    elseif msg.park then
        self:park()
    elseif msg.line then
        self:_terminal(msg.line)
    elseif msg.record ~= nil then
        self:_set_log(tostring(msg.record), msg.path and tostring(msg.path) or nil)
    else
        log.warn("tuner: unhandled UI message: {}", msg)
        return nil
    end
    return msg
end

---Zero the set-points. Called on shutdown by both entry points.
function Tuner:park()
    -- lines already in flight still arrive after this point; they must not be
    -- pushed into a view that is being torn down
    self.closing = true
    self:_send_target(0.0)
    self:_send_voltage(0.0)
    if self.transport == "serial" then
        -- the bench channel owns the driver, so it can also park it and stop
        -- the telemetry stream; over CAN both belong to the cart
        self.motor("X")
        self.motor("E0")
        self.motor("M0")
        self.motor("V0")
    end
    if self.log_file then
        self.log_file:close()
        self.log_file = nil
        log.info("tuner: recording closed")
    end
end

-- ---- Serial transport -------------------------------------------------------

function Tuner:_open_serial(opts)
    local motor = Serial {
        framing = "line",
        protocol = "text",
        port = assert(opts.port, "serial transport needs a port"),
        baud = 115200,
    }
    self.motor = motor
    self.port = opts.port

    pipe(motor, function(line)
        self:_log_record(line)
        self:_on_serial_line(line)
    end)

    self:_status(i18n.fmt("status_probing", tostring(opts.port)))
    local answered = false
    self.answered = function() answered = true end
    for _ = 1, 40 do
        motor("S")
        sleep(500)
        if answered then break end
    end
    if not answered then
        log.error("{} did not answer an S probe; check the port and the board", opts.port)
        -- the channel never came up: the window stays, but nothing is sent into
        -- a link that has shown it cannot carry a command
        self.inert = true
        self:_status(i18n.fmt("status_no_answer", tostring(opts.port)))
        return
    end
    motor("D0")
    self:_read_back()
    sleep(800)
    self:_apply_read_back()
    motor("M71")
    motor("V1")
    self:_status(i18n.fmt("status_serial", tostring(opts.port)))
end

function Tuner:_on_serial_line(line)
    if self.closing then
        log.info("< {}", line)
        return
    end
    if line:match("^D ") then
        local fields = {}
        for key, value in line:gsub("^D ", ""):gmatch("([%a]+)=(-?[%d.]+)") do
            fields[key] = tonumber(value) or 0.0
        end
        local radius = self.wheel_diameter_m * 0.5
        self.view {
            diag = {
                tgt_m_s = fields.t or 0.0,
                act_m_s = (fields.v or 0.0) * radius,
                iq = fields.iq, uq = fields.uq, ud = fields.ud, im = fields.im,
                bus = fields.b, t_ms = fields.n,
            },
        }
        local fault_index = math.floor((fields.f or 0) + 0.5) + 1
        local fault_key = FAULT_NAMES[fault_index]
        local fault = fault_key and i18n.text(fault_key)
            or i18n.fmt("fault_unknown", tostring(fields.f))
        self:_status(i18n.fmt("diag_v1",
            fields.v or 0.0, fields.iq or 0.0, fields.b or 0.0, fault,
            i18n.text(fields.e == 1 and "driver_on" or "driver_off")))
    elseif line:match("^S ") then
        if self.answered then self.answered() end
        local fields = {}
        for key, value in line:gsub("^S ", ""):gmatch("([%a]+)=(-?[%d.]+)") do
            fields[key] = tonumber(value) or 0.0
        end
        local radius = self.wheel_diameter_m * 0.5
        self.view {
            diag = {
                tgt_m_s = (fields.t or 0.0) * self.max_speed_m_s,
                act_m_s = (fields.v or 0.0) * radius,
                iq = fields.i,
            },
        }
        self:_status(i18n.fmt("diag_status",
            fields.t or 0.0, fields.v or 0.0,
            i18n.text(fields.e == 1 and "enabled" or "disabled")))
    elseif line:match("^C%d+=") then
        local id, value = line:match("^C(%d+)=(%-?[%d.]+)")
        if id and value then
            self.values = self.values or {}
            self.values[tonumber(id)] = tonumber(value)
        end
    elseif line:match("^I ") then
        log.info("< {}", line)
        self:_output_line(line)
        self:_status(line)
    elseif line:match("^ERR") then
        log.warn("< {}", line)
        self:_output_line(line)
        self:_status(i18n.fmt("status_rejected", line))
    elseif line:match("^# ") then
        log.info("< {}", line)
        self:_output_line(line)
    else
        log.info("< {}", line)
        self:_output_line(line)
    end
end

function Tuner:_apply_read_back()
    local changed = false
    for _, param in ipairs(self.params) do
        local value = self.values and self.values[param.id]
        if value ~= nil then
            self:_set_value(param.id, value)
            changed = true
        end
    end
    self.view {
        params = self.params,
        wheel_diameter_m = self.wheel_diameter_m,
        max_speed_mps = self.max_speed_m_s,
    }
    return changed
end

function Tuner:_read_back()
    if self.transport ~= "serial" then
        self:_status(i18n.text("status_readback_serial_only"))
        return
    end
    self.values = {}
    for _, param in ipairs(self.params) do
        self.motor(("G%d"):format(param.id))
    end
    sleep(500)
    self:_apply_read_back()
end

function Tuner:_send_config(id, value)
    if self.transport == "serial" then
        self.motor(("C%d:%s"):format(id, value))
    else
        self.client { action = "config", wheel = self.wheel, id = id, value = value }
        if self.on_config_sent then self.on_config_sent(self.wheel, id, value) end
        log.info("config id {} = {} -> {}", id, value, self.wheel)
    end
end

function Tuner:_send_target(value)
    if self.transport == "serial" then
        self.motor(("T%.3f"):format(value))
    else
        self.client { action = "wheel", wheel = self.wheel, v = value }
    end
end

function Tuner:_send_voltage(volts)
    if self.transport == "serial" then
        self.motor(("Q%.3f"):format(volts))
    else
        self.client { action = "direct", wheel = self.wheel, value = volts }
    end
end

function Tuner:_send_enable(on)
    if self.transport ~= "serial" then
        self:_status(i18n.text("status_enable_cart"))
        return
    end
    self.motor(on and "E1" or "E0")
end

function Tuner:_terminal(line)
    if self.transport ~= "serial" then
        self:_status(i18n.text("status_console_serial"))
        return
    end
    self.motor(line)
    -- the counters and the trace land in the diagnostics window, so their own
    -- feedback belongs there too
    if line == "W1" then
        self:_diag_status(i18n.text("status_trace_armed"))
    end
end

-- ---- CAN transport ----------------------------------------------------------

function Tuner:_open_can(opts)
    -- The service channel (cart.lua, port 6081) is the bench/diagnosis socket:
    -- it accepts normalized single-wheel speed, direct voltage and config
    -- writes, and publishes the per-wheel state snapshot.
    local url = assert(opts.url, "can transport needs the service websocket url")
    self.client = WebsocketClient { url = url, protocol = "msgpack" }

    pipe(self.client.events, function(ev)
        log.info("service ws {}: {}", url, ev)
        if ev.state == "ConnectedState" then
            self:_status(i18n.fmt("status_connected", url))
            self.client { action = "state" }
        else
            self:_status(i18n.fmt("status_link", tostring(ev.state or ev)))
        end
    end)

    pipe(self.client, function(msg)
        if self.closing then return end
        -- the recording of a CAN session is this snapshot stream, one JSON object
        -- per line, because the service channel carries nothing else
        self:_log_record(tostring(msg))
        local state = msg.state
        if not state then return end
        local wheels = state.wheels or {}
        local wheel = wheels[self.wheel] or {}
        local tgt = wheel.tgt or 0.0
        local act = wheel.act or 0.0
        self.view {
            diag = { tgt_m_s = tgt, act_m_s = act },
            status = i18n.fmt("status_target", self.wheel, tgt, act),
        }
        if state.joystick and state.joystick.stopped then
            self.view { status = i18n.fmt("status_stopped", self.wheel) }
        end
    end)

    self.client { action = "state" }
    self:_status(i18n.fmt("status_service", url, self.wheel))
end

return Tuner
