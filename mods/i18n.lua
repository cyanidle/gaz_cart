-- =============================================================================
--  mods/i18n.lua — the language of every text the Lua side hands to a view.
--
--  QML chrome text lives in qml/Tr/Tr.qml (a singleton the views bind to). What
--  the *models* carry is chosen here: parameter names and units
--  (mods/config_defs.lua `label`/`label_ru`, `unit`/`unit_ru`), the hover help
--  (mods/tuner_help.lua `what.en`/`what.ru`) and the status lines below.
--
--  Russian is the default — the operators are Russian-speaking, English is one
--  throw of the window's language tumbler away. The choice is per session; the
--  GUI does not persist it.
--
--      local i18n = require "mods.i18n"
--      i18n.set("en")                       -- or "ru"
--      i18n.label(config_defs.pid_kp)       -- localized name
--      i18n.fmt("status_probing", port)     -- localized status line
-- =============================================================================

local i18n = {
    lang = "ru",
    langs = { "ru", "en" },
}

---Status lines the tuner and the cart GUI hand to a view.
---@type table<string, { en: string, ru: string }>
local TEXT = {
    status_probing = {
        en = "(probing {} — the board answers ~10 s after a reset)",
        ru = "(опрашиваю {} — плата отвечает через ~10 с после сброса)",
    },
    status_no_answer = {
        en = "no answer on {} — commands are disabled",
        ru = "нет ответа на {} — команды заблокированы",
    },
    status_nothing_attached = {
        en = "nothing is attached — {} never answered",
        ru = "ничего не подключено — {} не ответил",
    },
    status_serial = {
        en = "serial {}",
        ru = "последовательный порт {}",
    },
    status_connected = {
        en = "connected to {}",
        ru = "подключено к {}",
    },
    status_connecting = {
        en = "(connecting)",
        ru = "(подключение)",
    },
    status_link = {
        en = "link: {}",
        ru = "канал: {}",
    },
    status_wheel = {
        en = "wheel {}",
        ru = "колесо {}",
    },
    status_service = {
        en = "service {} ({})",
        ru = "сервис {} ({})",
    },
    status_readback_serial_only = {
        en = "the cart keeps its own record; read-back is a serial command",
        ru = "пульт ведёт свою запись; чтение — команда serial",
    },
    status_enable_cart = {
        en = "enable is owned by the cart; a wheel command energizes the module",
        ru = "включением управляет пульт; команда колеса подаёт питание на модуль",
    },
    status_console_serial = {
        en = "the console is a serial feature",
        ru = "консоль — функция последовательного порта",
    },
    status_one_module = {
        en = "the serial channel drives the one module it is attached to",
        ru = "serial-канал управляет тем модулем, к которому подключён",
    },
    status_target = {
        en = "{}: target {:.3f} m/s   actual {:.3f} m/s",
        ru = "{}: задание {:.3f} м/с   факт {:.3f} м/с",
    },
    status_stopped = {
        en = "{}: stop organ engaged (cart parked)",
        ru = "{}: сработал орган останова (тележка стоит)",
    },
    status_trace_armed = {
        en = "trace armed",
        ru = "трасса взведена",
    },
    status_rejected = {
        en = "rejected: {}",
        ru = "отклонено: {}",
    },
    diag_v1 = {
        en = "v={:.2f} rad/s   i={:.2f} A   bus={:.1f} V   {}   driver {}",
        ru = "v={:.2f} рад/с   i={:.2f} А   шина={:.1f} В   {}   привод {}",
    },
    diag_status = {
        en = "t={:.3f}  v={:.2f} rad/s  {}",
        ru = "t={:.3f}  v={:.2f} рад/с  {}",
    },
    driver_on = { en = "on", ru = "вкл" },
    driver_off = { en = "off", ru = "выкл" },
    enabled = { en = "enabled", ru = "включён" },
    disabled = { en = "disabled", ru = "отключён" },
    fault_none = { en = "NONE", ru = "норма" },
    fault_invalid_hall = { en = "INVALID_HALL", ru = "сбой Холла" },
    fault_invalid_supply = { en = "INVALID_SUPPLY", ru = "просадка питания" },
    fault_invalid_command = { en = "INVALID_COMMAND", ru = "неверная команда" },
    fault_stalled = { en = "STALLED", ru = "застой" },
    fault_overcurrent = { en = "OVERCURRENT", ru = "переток" },
    fault_driver = { en = "DRIVER_FAULT", ru = "отказ драйвера" },
    fault_unknown = { en = "UNKNOWN (code {})", ru = "неизвестный код {}" },

    map_load_failed = { en = "Load failed: {}", ru = "Не удалось загрузить: {}" },
    map_uploading = { en = "Uploading {} …", ru = "Отправка {} …" },
    map_saved = { en = "Map saved to {}", ru = "Карта сохранена в {}" },
    map_save_failed = { en = "Save failed: {}", ru = "Не удалось сохранить: {}" },
    map_requesting = { en = "Requesting map …", ru = "Запрос карты …" },
}

---@param lang string|nil
---@return string the active language after the call
function i18n.set(lang)
    if lang == nil then return i18n.lang end
    local wanted = tostring(lang)
    for _, known in ipairs(i18n.langs) do
        if known == wanted then
            i18n.lang = wanted
            return i18n.lang
        end
    end
    error(("unknown language '%s' (expected ru or en)"):format(wanted))
end

---The text of `key` in the active language.
---@param key string
---@return string
function i18n.text(key)
    local entry = TEXT[key]
    if not entry then error(("no localized text for '%s'"):format(key)) end
    return entry[i18n.lang] or entry.en
end

---`i18n.text(key)` through radapter's `fmt`, for the lines that carry values.
function i18n.fmt(key, ...)
    return fmt(i18n.text(key), ...)
end

---Localized parameter name.
---@param def table config_defs entry
---@return string
function i18n.label(def)
    if i18n.lang == "ru" then return def.label_ru or def.label end
    return def.label
end

---Localized parameter unit (SI symbols differ: rad/s vs рад/с).
---@param def table config_defs entry
---@return string
function i18n.unit(def)
    if i18n.lang == "ru" then return def.unit_ru or def.unit end
    return def.unit
end

---Localized hover help for a tuner parameter.
---@param entry table tuner_help entry
---@return string
function i18n.help(entry)
    local what = entry.what
    return what[i18n.lang] or what.en
end

return i18n
