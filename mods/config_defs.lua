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
--  firmware/App/app.cpp.**  `default`/`min`/`max`/`step` only drive the GUI.
--
--  **The gain units follow the backend's regulator, not the wire.**  The module
--  façade converts m/s <-> rad/s with the wheel radius at its edge, so the
--  velocity loop itself runs on shaft angular velocity and its gains are
--  V/(rad/s), V/(rad/s·s) and V·s/(rad/s) on the six-step and SimpleFOC builds
--  (A/(rad/s) on the libvoltbro FOC one).  A gain expressed per m/s would need
--  a per-wheel-radius conversion the module does not do — see "CAN config
--  coverage" in firmware/README.md.  The numbers are therefore gain-per-rad/s
--  and the cart's wheel (six-step) is what they are tuned against.
--
--  The defaults mirror the compiled-in ones so a factory-fresh module and the
--  GUI agree: `motor_tuning::default_velocity_pid_config()` and
--  `BOOT_WHEEL_RADIUS_M * 2` in firmware/App/app.cpp.
-- =============================================================================

return {
    wheel_diameter = { id = 0, label = "Wheel diameter", unit = "m",            default = 0.32, min = 0.01, max = 1.0, step = 0.005 },
    pid_kp         = { id = 1, label = "Velocity Kp",     unit = "V/(rad/s)",   default = 0.50, min = 0.0,  max = 50,  step = 0.01 },
    pid_ki         = { id = 2, label = "Velocity Ki",     unit = "V/(rad/s·s)", default = 0.10, min = 0.0,  max = 50,  step = 0.01 },
    pid_kd         = { id = 3, label = "Velocity Kd",     unit = "V·s/(rad/s)", default = 0.00, min = 0.0,  max = 50,  step = 0.01 },
    pid_i_limit    = { id = 4, label = "Integral limit",  unit = "V",           default = 10.0, min = 0.0,  max = 24,  step = 0.1  },
}
