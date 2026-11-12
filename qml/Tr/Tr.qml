pragma Singleton
import QtQuick

// UI language for every window of the cart GUI.
//
// One instance per radapter engine, shared by all QML windows of the process, so
// a switch in any window re-labels every other one. Russian is the default: the
// operators are Russian-speaking, and English is one click away.
//
// Texts live here in two tables, one entry per line and `key: "value",` shaped,
// because tests/i18n.lua parses them to prove both languages carry the same keys
// (a missing Russian string would otherwise show up as an English one by
// accident, or as the bare key).
//
// Text that the Lua side owns — parameter names and units (mods/config_defs.lua),
// the hover help (mods/tuner_help.lua) and the status lines (mods/i18n.lua) —
// arrives through the model already in the active language; the switch also tells
// Lua which language that is.
QtObject {
    id: root

    property string lang: "ru"
    readonly property var languages: ["ru", "en"]

    function toggle() {
        lang = (lang === "ru" ? "en" : "ru")
    }

    function table() {
        return tables[lang] !== undefined ? tables[lang] : tables.en
    }

    function t(key) {
        var text = table()[key]
        if (text === undefined) text = tables.en[key]
        return text !== undefined ? text : key
    }

    /// `t()` with %1..%9 replaced by the arguments (a list or a single value).
    function tf(key, args) {
        var text = t(key)
        if (args === undefined) return text
        var list = Array.isArray(args) ? args : [args]
        for (var i = 0; i < list.length; i++) {
            text = text.replace("%" + (i + 1), String(list[i]))
        }
        return text
    }

    /// Cart-side identifiers (drive source, operator input) arrive as words like
    /// "teleop" or "nav"; translate the ones we know, show the rest as they came.
    function sourceName(value) {
        var word = String(value)
        var key = "source_" + word
        var text = table()[key]
        if (text === undefined) text = tables.en[key]
        return text !== undefined ? text : word
    }

    readonly property var tables: ({
        ru: {
            lang_name: "Язык",
            lang_ru: "РУ",
            lang_en: "EN",

            window_config: "gaz_cart — конфигурация",
            window_nav: "gaz_cart — навигация",
            window_teleop: "gaz_cart — телеуправление",
            window_tune: "gaz_cart — настройка",

            wheel: "Колесо",
            wheel_all: "все",
            unit_m_s: "м/с",
            unit_v: "В",
            axis_time: "время, с",
            chart_speed: "Скорость — задание и факт",
            series_actual: "факт",
            series_target: "задание",
            no_telemetry: "(телеметрии пока нет)",

            tune_source_serial: "Модуль по последовательному порту — значения прочитаны из модуля",
            tune_source_can: "Пульт по CAN (сервисный канал) — колесо %1, значения показаны отправленные",
            tune_fullscale_serial: "%1 м/с при 1.0 (config id 16)",
            tune_fullscale_can: "%1 м/с при 1.0 (полная шкала пульта)",
            tune_axis_speed: "скорость колеса, м/с",
            tune_chart_aux: "токи (А) и напряжения d/q (В)",
            tune_series_iq: "iq (А)",
            tune_series_uq: "uq (В)",
            tune_series_ud: "ud (В)",
            tune_series_im: "|i| фазный (А)",
            tune_series_bus: "шина (В)",
            tune_target_speed: "Задание скорости",
            tune_target_status: "Задание %1 × полная шкала = %2 м/с",
            tune_direct_voltage: "Прямое напряжение",
            tune_direct_status: "Прямое напряжение %1 В (разомкнутый контур)",
            tune_voltage_range: "Диапазон напряжения",
            tune_enable: "Включить",
            tune_disable: "Отключить",
            tune_reset: "Сброс",
            tune_calibrate: "Калибровка R/L",
            tune_readback: "Прочитать из модуля",
            tune_trace_arm: "Взвести трассу",
            tune_trace_dump: "Снять трассу",
            tune_counters: "Счётчики",
            tune_reset_status: "Задания обнулены (параметры сохранены)",
            tune_trace_armed: "Трасса взведена: первая аномалия сбросит ~51 мс истории цикла",
            tune_sent: "Отправлено %1 = %2 %3",
            tune_no_telemetry: "(телеметрии пока нет)",
            tune_hint: "Двигайте ползунок (или введите значение и нажмите Enter), чтобы применить. Наведите на имя параметра — появится описание и эскиз.",

            help_title: "%1  [%2]",
            help_overshoot: "перерегулирование растёт с Kp",
            help_integral: "интеграл: медленнее, накапливается",
            help_derivative: "Kd гасит подход",
            help_filtered: "отфильтровано (плавнее, %1 % диапазона)",
            help_steeper: "круче — быстрее подход",
            help_trip_level: "уровень срабатывания %1 шкалы",
            help_window: "окно %1 % — надо продержаться выше линии",
            help_ceiling: "потолок %1",
            help_driving: "разгон",
            help_braking: "торможение ≤ %1 %",
            help_stall_zone: "зона застоя ниже линии",
            help_full_pos: "1.0 = %1 м/с",
            help_full_neg: "-1.0 = %1 м/с",
            help_diameter: "d = %1 м",
            help_circumference: "длина окружности %1 м за оборот",
            help_no_sketch: "(для «%1» нет эскиза)",

            cfg_group_title: "PID и геометрия",
            cfg_target_wheel: "Целевое колесо",
            cfg_axis_speed: "скорость, м/с",
            cfg_target_status: "Задание %1 = %2 м/с",
            cfg_direct_status: "Прямое %1 = %2 В (разомкнутый контур)",
            cfg_sent: "Отправлено %1 = %2 → %3",
            cfg_not_a_number: "Не число: %1",
            cfg_hint: "Измените поле и нажмите Enter, чтобы применить.",

            nav_costmap: "Карта: %1×%2 @ %3 м",
            nav_origin: "Начало: %1, %2",
            nav_robot: "Робот: %1, %2, %3",
            nav_path_points: "Точек пути: %1",
            nav_scan_hits: "Точек скана: %1",
            nav_status_none: "Статус: —",
            nav_status_done: "Статус: готово",
            nav_status_rotating: "Статус: доворот",
            nav_status_stuck: "Статус: застрял",
            nav_status_driving: "Статус: едет",
            nav_cmd_vel: "cmd_vel  вперёд:%1  бок:%2  поворот:%3",
            nav_cancel: "Отменить цель",
            nav_discovery: "Разведка карты",
            nav_load_map: "Загрузить карту",
            nav_dump_map: "Выгрузить карту",
            nav_reset_view: "Сбросить вид",
            nav_help: "ЛКМ — задать цель\nЛКМ с протяжкой — цель + курс\nShift+ЛКМ — переставить робота\nКолесо — масштаб\nСКМ с протяжкой — сдвиг\nПКМ — поставить препятствие\n(в симуляции его находит лидар,\nиначе временная точка карты)\n\nСиние точки: попадания лидара\nЧёрточки: курс точки пути\nОранжевая стрелка: выбор локального планировщика",
            nav_load_title: "Загрузить карту занятости",
            nav_dump_title: "Выгрузить карту занятости",
            nav_filter_png: "Изображения карты PNG (*.png)",
            nav_filter_images: "Изображения (*.png *.jpg *.jpeg *.bmp)",

            tele_title: "Телеуправление с клавиатуры",
            tele_hint: "Щёлкните здесь, чтобы забрать фокус, затем ведите стрелками или WASD. Пробел останавливает команду этого окна; орган останова джойстика останавливает тележку для всех источников.",
            tele_speed_fraction: "Доля скорости",
            tele_linear: "Линейная",
            tele_angular: "Угловая",
            tele_cart_group: "Команда пульта (нормированная −1..1)",
            tele_stop_banner: "СТОП — сработал орган останова джойстика",
            tele_operator: "Ввод оператора:",
            tele_issued_by: "Источник:",
            tele_linear_v: "Линейная v:",
            tele_angular_w: "Угловая ω:",
            tele_window_group: "Это окно (нормированное −1..1)",
            tele_bench_group: "Стенд — одно колесо",
            tele_bench_hint: "Крутите одно колесо через W/S (или ▲/▼), остальные три стоят. Так поднимают модуль, не двигая тележку.",
            tele_single_wheel: "одно колесо",
            tele_controls: "Клавиши",

            source_keyboard: "клавиатура",
            source_joystick: "джойстик",
            source_teleop: "телеуправление",
            source_nav: "планировщик",
            source_service: "сервис",
            source_none: "нет",
        },
        en: {
            lang_name: "Language",
            lang_ru: "RU",
            lang_en: "EN",

            window_config: "gaz_cart — Config",
            window_nav: "gaz_cart — Nav",
            window_teleop: "gaz_cart — Teleop",
            window_tune: "gaz_cart — tune",

            wheel: "Wheel",
            wheel_all: "all",
            unit_m_s: "m/s",
            unit_v: "V",
            axis_time: "time (s)",
            chart_speed: "Speed — target vs actual",
            series_actual: "actual",
            series_target: "target",
            no_telemetry: "(no telemetry yet)",

            tune_source_serial: "Module over serial — values read from the module",
            tune_source_can: "Cart over CAN (service channel) — wheel %1, values are what was sent",
            tune_fullscale_serial: "%1 m/s at 1.0 (config id 16)",
            tune_fullscale_can: "%1 m/s at 1.0 (cart full scale)",
            tune_axis_speed: "wheel speed (m/s)",
            tune_chart_aux: "currents (A) and d/q voltages (V)",
            tune_series_iq: "iq (A)",
            tune_series_uq: "uq (V)",
            tune_series_ud: "ud (V)",
            tune_series_im: "|i| phase (A)",
            tune_series_bus: "bus (V)",
            tune_target_speed: "Target speed",
            tune_target_status: "Target %1 x full scale = %2 m/s",
            tune_direct_voltage: "Direct voltage",
            tune_direct_status: "Direct voltage %1 V (open loop)",
            tune_voltage_range: "Voltage range",
            tune_enable: "Enable",
            tune_disable: "Disable",
            tune_reset: "Reset",
            tune_calibrate: "Calibrate R/L",
            tune_readback: "Read from module",
            tune_trace_arm: "Trace arm",
            tune_trace_dump: "Trace dump",
            tune_counters: "Counters",
            tune_reset_status: "Set-points zeroed (parameters kept)",
            tune_trace_armed: "Trace armed: the next anomaly dumps ~51 ms of loop history",
            tune_sent: "Sent %1 = %2 %3",
            tune_no_telemetry: "(no telemetry yet)",
            tune_hint: "Drag a slider (or type a value and press Enter) to apply. Hover a parameter's name to see what it does.",

            help_title: "%1  [%2]",
            help_overshoot: "overshoot grows with Kp",
            help_integral: "integral: slower, winds up",
            help_derivative: "Kd damps the approach",
            help_filtered: "filtered (smoother, %1% of range)",
            help_steeper: "steeper = faster approach",
            help_trip_level: "trip level %1 of scale",
            help_window: "window %1% — must stay above the line",
            help_ceiling: "ceiling %1",
            help_driving: "driving",
            help_braking: "braking ≤ %1%",
            help_stall_zone: "stall zone below the line",
            help_full_pos: "1.0 = %1 m/s",
            help_full_neg: "-1.0 = %1 m/s",
            help_diameter: "d = %1 m",
            help_circumference: "circumference %1 m per turn",
            help_no_sketch: "(no sketch for '%1')",

            cfg_group_title: "PID & geometry config",
            cfg_target_wheel: "Target wheel",
            cfg_axis_speed: "speed (m/s)",
            cfg_target_status: "Target %1 = %2 m/s",
            cfg_direct_status: "Direct %1 = %2 V (open loop)",
            cfg_sent: "Sent %1 = %2 to %3",
            cfg_not_a_number: "Not a number: %1",
            cfg_hint: "Edit a field and press Enter to apply.",

            nav_costmap: "Costmap: %1×%2 @ %3 m",
            nav_origin: "Origin: %1, %2",
            nav_robot: "Robot: %1, %2, %3",
            nav_path_points: "Path points: %1",
            nav_scan_hits: "Scan hits: %1",
            nav_status_none: "Status: —",
            nav_status_done: "Status: done",
            nav_status_rotating: "Status: rotating",
            nav_status_stuck: "Status: stuck",
            nav_status_driving: "Status: driving",
            nav_cmd_vel: "cmd_vel  fwd:%1  lat:%2  rot:%3",
            nav_cancel: "Cancel target",
            nav_discovery: "Map discovery",
            nav_load_map: "Load map",
            nav_dump_map: "Dump map",
            nav_reset_view: "Reset view",
            nav_help: "LMB click — set target\nLMB drag — set target + heading\nShift+LMB — reposition robot\nWheel — zoom\nMMB drag — pan\nRMB — drop an obstacle\n(sim: lidar discovers it; else a\ntemporary costmap point)\n\nBlue dots: lidar scan hits\nTicks: path point theta\nOrange arrow: local planner's pick",
            nav_load_title: "Load occupancy map",
            nav_dump_title: "Dump occupancy map",
            nav_filter_png: "PNG map images (*.png)",
            nav_filter_images: "Images (*.png *.jpg *.jpeg *.bmp)",

            tele_title: "Keyboard Teleop",
            tele_hint: "Click here to grab focus, then use arrow keys or WASD to drive. Space stops this window's command; the joystick's stop organ stops the cart for every source.",
            tele_speed_fraction: "Speed fraction",
            tele_linear: "Linear",
            tele_angular: "Angular",
            tele_cart_group: "Cart command (normalised -1..1)",
            tele_stop_banner: "STOP — joystick stop organ engaged",
            tele_operator: "Operator input:",
            tele_issued_by: "Issued by:",
            tele_linear_v: "Linear v:",
            tele_angular_w: "Angular ω:",
            tele_window_group: "This window (normalised -1..1)",
            tele_bench_group: "Bench — single wheel",
            tele_bench_hint: "Drive one wheel with W/S (or ▲/▼); the other three stay parked. Use it to bring up a module without driving the cart.",
            tele_single_wheel: "single wheel",
            tele_controls: "Controls",

            source_keyboard: "keyboard",
            source_joystick: "joystick",
            source_teleop: "teleop",
            source_nav: "planner",
            source_service: "service",
            source_none: "none",
        },
    })
}
