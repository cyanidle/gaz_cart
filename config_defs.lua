-- =============================================================================
--  Runtime-tunable wheel-module config — the single source of truth.
--
--  Shared by main.lua (which publishes values over Cyphal) and gui.lua (which
--  renders one editable control per entry).
--
--  Each module receives a config update as an Integer32 triple
--      { id, numerator, denominator }
--  on its CONFIG_PORT (4100 + node id), and applies  value = numerator/denominator
--  to the parameter selected by `id`.
--
--  **The `id`s MUST stay in sync with the `ConfigId` enum in
--  SixStep/App/app.cpp.**  `default`/`min`/`max`/`step` only drive the GUI.
-- =============================================================================

return {
    { id = 0, key = "wheel_diameter", label = "Wheel diameter", unit = "m",          default = 0.10, min = 0.01, max = 1.0, step = 0.005 },
    { id = 1, key = "pid_kp",         label = "Velocity Kp",     unit = "V/(m/s)",    default = 0.50, min = 0.0,  max = 50,  step = 0.01 },
    { id = 2, key = "pid_ki",         label = "Velocity Ki",     unit = "V/(m·s)",    default = 0.10, min = 0.0,  max = 50,  step = 0.01 },
    { id = 3, key = "pid_kd",         label = "Velocity Kd",     unit = "V·s/m",      default = 0.00, min = 0.0,  max = 50,  step = 0.01 },
    { id = 4, key = "pid_i_limit",    label = "Integral limit",  unit = "V",          default = 10.0, min = 0.0,  max = 24,  step = 0.1  },
    { id = 5, key = "pid_tolerance",  label = "PID tolerance",   unit = "m/s",        default = 0.05, min = 0.0,  max = 5,   step = 0.01 },
}
