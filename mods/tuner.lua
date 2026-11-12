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

    self.view = QML { url = opts.view or (REPO_DIR .. "/qml/TunerWindow.qml") }

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

    if self.transport == "serial" then
        self:_open_serial(opts)
    else
        self:_open_can(opts)
    end

    pipe(self.view, function(msg) return self:_on_view(msg) end)
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

    local log_file = opts.log_file and io.open(opts.log_file, "a")

    pipe(motor, function(line)
        if log_file then
            log_file:write(fmt("{}\n", line))
            log_file:flush()
        end
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
        self:_status(line)
    elseif line:match("^ERR") then
        log.warn("< {}", line)
        self:_status(i18n.fmt("status_rejected", line))
    elseif line:match("^# ") then
        log.info("< {}", line)
    else
        log.info("< {}", line)
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
