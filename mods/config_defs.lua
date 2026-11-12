-- =============================================================================
--  Runtime-tunable wheel-module config — the single source of truth.
--
--  Shared by main.lua (which publishes values over Cyphal) and gui.lua (which
--  renders one editable control per entry).  Keyed by the parameter key; the
--  GUI orders its rows by `id`.
--
--  Each module receives a config update as an Integer32 triple
--      { id, numerator, denominator }
--  on its CONFIG_PORT (4100 + node id), and applies  value = numerator/denominator
--  to the parameter selected by `id`.
--
--  **The `id`s MUST stay in sync with the `ConfigId` enum in
--  firmware/App/config_ids.h** (tests/config_defs.lua checks that they are).
--  `default`/`min`/`max`/`step` only drive the GUI.
--
--  **The gain units follow the backend's regulator, not the wire.**  The module
--  façade converts m/s <-> rad/s with the wheel radius at its edge, so the
--  velocity loop itself runs on shaft angular velocity and its gains are
--  A/(rad/s), A/(rad/s·s) and A·s/(rad/s) on the FOC builds — SimpleFOC is the
--  compiled default and regulates a q-current; the six-step build interprets
--  the same numbers as V-based gains instead.  A gain expressed per m/s would
--  need a per-wheel-radius conversion the module does not do — see "CAN config
--  coverage" in firmware/README.md.  The numbers are therefore gain-per-rad/s,
--  and this table tracks the SimpleFOC bench tuning in motor_tuning.h (a
--  six-step build compiles its own V-based defaults, so a factory-fresh
--  six-step module reports different numbers until this table is sent to it).
--  A wheel's EEPROM record overrides these defaults until edited here.
--
--  The wheel-diameter default mirrors the compiled-in `BOOT_WHEEL_RADIUS_M * 2`
--  in firmware/App/app.cpp.
-- =============================================================================

return {
    -- tests/tuner_help.lua requires label_ru/unit_ru beside every label/unit.
    wheel_diameter = {
        id = 0, label = "Wheel diameter", label_ru = "Диаметр колеса",
        unit = "m", unit_ru = "м",
        default = 0.32, min = 0.01, max = 1.0, step = 0.005,
    },
    pid_kp         = {
        id = 1, label = "Velocity Kp", label_ru = "Kp скорости",
        unit = "A/(rad/s)", unit_ru = "А/(рад/с)",
        default = 2.10, min = 0.0,  max = 50,  step = 0.01,
    },
    pid_ki         = {
        id = 2, label = "Velocity Ki", label_ru = "Ki скорости",
        unit = "A/(rad/s·s)", unit_ru = "А/(рад/с·с)",
        default = 1.11, min = 0.0,  max = 50,  step = 0.01,
    },
    pid_kd         = {
        id = 3, label = "Velocity Kd", label_ru = "Kd скорости",
        unit = "A·s/(rad/s)", unit_ru = "А·с/(рад/с)",
        default = 0.00, min = 0.0,  max = 50,  step = 0.01,
    },
    -- The output-domain integral clamp: the firmware derives it as
    -- ki * integral_error_lim = 1.11 * 10.0, which is what a fresh module
    -- reports on G4. SimpleFOC's PID ignores it (it clamps the integral with
    -- the whole output), so on the default build it only has to stay in step
    -- with that derived value.
    pid_i_limit    = {
        id = 4, label = "Integral limit", label_ru = "Предел интеграла",
        unit = "A", unit_ru = "А",
        default = 11.1, min = 0.0,  max = 24,  step = 0.1,
    },

    -- Current guard (overcurrent / stall protection). The overcurrent ids
    -- (5..8) apply to every backend that measures phase current (six-step and
    -- SimpleFOC both read the module's 3-shunt front end); the stall ids
    -- (9..11) only take effect on the SimpleFOC build — the six-step keeps its
    -- hall-progress watchdog, which the guard's stall timer would pre-empt.
    -- A stall window of 0 disables the stall check. These values are persisted
    -- in the module EEPROM (config record version 5).
    oc_peak_a      = {
        id = 5, label = "OC peak level", label_ru = "Пиковый переток",
        unit = "A", unit_ru = "А",
        default = 14,   min = 1,    max = 40,   step = 0.5,
    },
    oc_peak_ms     = {
        id = 6, label = "OC peak window", label_ru = "Окно пика",
        unit = "ms", unit_ru = "мс",
        default = 25,   min = 0,    max = 1000, step = 5,
    },
    oc_sustain_a   = {
        id = 7, label = "OC sustain level", label_ru = "Длительный переток",
        unit = "A", unit_ru = "А",
        default = 10,   min = 1,    max = 40,   step = 0.5,
    },
    oc_sustain_ms  = {
        id = 8, label = "OC sustain window", label_ru = "Окно длительного перетока",
        unit = "ms", unit_ru = "мс",
        default = 250,  min = 0,    max = 5000, step = 10,
    },
    stall_current  = {
        id = 9, label = "Stall current", label_ru = "Ток застоя",
        unit = "A", unit_ru = "А",
        default = 2,    min = 0.1,  max = 20,   step = 0.1,
    },
    stall_speed    = {
        id = 10, label = "Stall speed", label_ru = "Скорость застоя",
        unit = "rad/s", unit_ru = "рад/с",
        default = 0.11, min = 0.01, max = 5,    step = 0.01,
    },
    stall_ms       = {
        id = 11, label = "Stall window", label_ru = "Окно застоя",
        unit = "ms", unit_ru = "мс",
        default = 600,  min = 0,    max = 5000, step = 50,
    },

    -- Loop-shaping knobs of the SimpleFOC backend (ids 12..15; a backend
    -- without the knob rejects the write, so the six-step build ignores them).
    -- The velocity LPF (12) is the measured-velocity low-pass the velocity PID
    -- closes over; the speed ramp (13) limits the set-point slew on every
    -- backend; the torque ramp (14) slews the PID's q-current output; the
    -- brake current (15) is the ceiling for a torque that opposes the measured
    -- rotation — the regen limit on a supply that cannot sink it, where 0
    -- leaves a moving wheel to coast. These are persisted in the module EEPROM
    -- (config record version 5) like the ids above.
    vel_lpf        = {
        id = 12, label = "Velocity LPF", label_ru = "ФНЧ скорости",
        unit = "s", unit_ru = "с",
        default = 0.05, min = 0.0,  max = 1,    step = 0.005,
    },
    speed_ramp     = {
        id = 13, label = "Speed ramp", label_ru = "Разгон скорости",
        unit = "m/s²", unit_ru = "м/с²",
        default = 0.75, min = 0.05, max = 100,  step = 0.05,
    },
    torque_ramp    = {
        id = 14, label = "Torque ramp", label_ru = "Разгон момента",
        unit = "A/s", unit_ru = "А/с",
        default = 50,   min = 1,    max = 1000, step = 1,
    },
    brake_current  = {
        id = 15, label = "Brake current", label_ru = "Ток торможения",
        unit = "A", unit_ru = "А",
        default = 2.2,  min = 0,    max = 8,    step = 0.1,
    },

    -- Full-scale wheel speed (id 16): every speed command is normalized, -1..1,
    -- and the module scales it with this value. The unit is m/s at the wheel.
    -- The firmware derives its own boot value from the drive's EMF constant and
    -- the live modulation ceiling, and clamps every command to that ceiling, so
    -- neither this table nor a stored EEPROM value can ask for a speed the
    -- drive has no voltage to regulate.
    max_speed      = {
        id = 16, label = "Max wheel speed", label_ru = "Макс. скорость колеса",
        unit = "m/s", unit_ru = "м/с",
        default = 0.50, min = 0.01, max = 2.0, step = 0.005,
    },
}
