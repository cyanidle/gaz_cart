-- =============================================================================
--  mods/tuner_help.lua — what each runtime-config parameter does, for the
--  tuner's hover help: one sentence plus the kind of sketch that shows it.
--
--  The sketch kinds are drawn by qml/ParamHelp.qml. Keep `kinds` and that
--  switch in step: tests/tuner_help.lua fails if a parameter names a kind the
--  view does not know, so a new config id cannot ship without its explanation.
--
--  Keyed by the config_defs key (one entry per ConfigId).
-- =============================================================================

local KINDS = {
    step = "response of the speed loop to a step (Kp/Ki/Kd family)",
    clamp = "a rising demand clipped at a ceiling",
    level = "a current burst against a trip level",
    window = "how long a condition must last before it trips",
    speed_threshold = "a wheel slowing into a trip zone",
    smoothing = "raw noisy rate vs the filtered one the loop sees",
    slope = "set-point slope: the command ramp",
    ceiling = "allowed torque envelope (braking vs driving)",
    scale = "normalized -1..1 command mapped onto m/s",
    geometry = "wheel geometry: the m/s <-> rad/s conversion",
}

local PARAMS = {
    wheel_diameter = {
        chart = "geometry",
        what = "Physical wheel diameter. The module converts every m/s value (and the "
            .. "cart its odometry) with it, so a wrong diameter scales all speed and "
            .. "distance readings by the same error.",
    },
    pid_kp = {
        chart = "step",
        role = "kp",
        what = "Velocity-loop proportional gain: how hard the drive answers a speed "
            .. "error. Too low lags and sags under load; too high overshoots and "
            .. "oscillates at the loop's sample rate.",
    },
    pid_ki = {
        chart = "step",
        role = "ki",
        what = "Velocity-loop integral gain: removes the steady-state error the P term "
            .. "leaves. Too high winds up on reversals and hunts; the integral clamp "
            .. "(next row) bounds what it may add.",
    },
    pid_kd = {
        chart = "step",
        role = "kd",
        what = "Velocity-loop derivative gain: damps the approach. The rate comes from "
            .. "Hall edges a few tens of ms apart, so D mostly amplifies quantisation "
            .. "noise here — 0 is the bench default.",
    },
    pid_i_limit = {
        chart = "clamp",
        what = "Ceiling on what the integral term may add, in output units (A on the "
            .. "FOC builds, V on six-step). The firmware derives it as Ki x the error "
            .. "limit, which is what a fresh module reports on G4.",
    },
    oc_peak_a = {
        chart = "level",
        what = "Instantaneous overcurrent trip: a phase-current magnitude above this "
            .. "for the peak window parks the bridge. It sits above the drive's "
            .. "current ceiling so a current-limited launch does not trip it.",
    },
    oc_peak_ms = {
        chart = "window",
        what = "How long the current may sit above the peak level before the guard "
            .. "trips. 0 disables the level check; shorter windows catch a short but "
            .. "still trips on switching noise.",
    },
    oc_sustain_a = {
        chart = "level",
        what = "Sustained overcurrent level: a lower threshold than the peak trip, for "
            .. "a load that heats the bridge over hundreds of ms rather than instantly.",
    },
    oc_sustain_ms = {
        chart = "window",
        what = "How long the current may stay above the sustained level before the "
            .. "guard parks the bridge (the stall/thermal window).",
    },
    stall_current = {
        chart = "level",
        what = "Stall detector current floor: the wheel counts as stalled only while "
            .. "it draws more than this — a free-spinning wheel below the speed "
            .. "threshold is not a stall.",
    },
    stall_speed = {
        chart = "speed_threshold",
        what = "Stall speed threshold, wheel rad/s: with current above the floor, a "
            .. "shaft slower than this for the stall window trips the guard. Set it "
            .. "above the rate noise at the speeds the cart actually holds.",
    },
    stall_ms = {
        chart = "window",
        what = "Stall window: how long the slow-and-loaded condition must persist "
            .. "before the guard trips. 0 disables the stall check (the six-step "
            .. "backend keeps its own Hall-progress watchdog instead).",
    },
    vel_lpf = {
        chart = "smoothing",
        what = "Velocity-feedback low-pass, s. The Hall rate estimate is coarse (one "
            .. "edge every few tens of ms), so the loop closes over a filtered rate; "
            .. "longer is smoother but adds phase lag and invites hunting.",
    },
    speed_ramp = {
        chart = "slope",
        what = "Set-point ramp, m/s^2: how fast the applied wheel-speed target may "
            .. "change. It shapes every command into a slope, so a step cannot ask "
            .. "for an acceleration the drive cannot deliver.",
    },
    torque_ramp = {
        chart = "slope",
        what = "q-current set-point slew, A/s: the soft start of the torque loop. It "
            .. "turns a step demand into a current ramp; too slow feels mushy, too "
            .. "fast kicks the gearbox.",
    },
    brake_current = {
        chart = "ceiling",
        what = "Ceiling on torque that opposes the measured motion (the regen limit). "
            .. "It is what stops a moving wheel from pushing energy back into a supply "
            .. "that cannot sink it; 0 leaves the wheel to coast.",
    },
    max_speed = {
        chart = "scale",
        what = "Full-scale wheel speed: a normalized 1.0 command means this many m/s, "
            .. "so it sets what every -1..1 source (cart, joystick, serial T) can ask "
            .. "for. The module still clamps to what its rail can regulate.",
    },
}

return { kinds = KINDS, params = PARAMS }
