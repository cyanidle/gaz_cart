-- =============================================================================
--  mods/tuner_help.lua — what each runtime-config parameter does, for the
--  tuner's hover help: one sentence per language plus the kind of sketch that
--  shows it.
--
--  The sketch kinds are drawn by qml/ParamHelp.qml. Keep `kinds` and that
--  switch in step: tests/tuner_help.lua fails if a parameter names a kind the
--  view does not know, or if a language is missing, so a new config id cannot
--  ship without its explanation.
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
        what = {
            en = "Physical wheel diameter. The module converts every m/s value (and the "
                .. "cart its odometry) with it, so a wrong diameter scales all speed and "
                .. "distance readings by the same error.",
            ru = "Физический диаметр колеса. Модуль пересчитывает через него все значения "
                .. "м/с (а тележка — свою одометрию), поэтому неверный диаметр одинаково "
                .. "масштабирует все скорости и расстояния.",
        },
    },
    pid_kp = {
        chart = "step",
        role = "kp",
        what = {
            en = "Velocity-loop proportional gain: how hard the drive answers a speed "
                .. "error. Too low lags and sags under load; too high overshoots and "
                .. "oscillates at the loop's sample rate.",
            ru = "Пропорциональный коэффициент контура скорости: насколько резко привод "
                .. "отвечает на ошибку скорости. Слишком мал — отставание и просадка под "
                .. "нагрузкой; слишком велик — перерегулирование и колебания на частоте "
                .. "опроса контура.",
        },
    },
    pid_ki = {
        chart = "step",
        role = "ki",
        what = {
            en = "Velocity-loop integral gain: removes the steady-state error the P term "
                .. "leaves. Too high winds up on reversals and hunts; the integral clamp "
                .. "(next row) bounds what it may add.",
            ru = "Интегральный коэффициент контура скорости: убирает статическую ошибку, "
                .. "которую оставляет P-составляющая. Слишком велик — накопление при "
                .. "реверсах и рыскание; ограничение интеграла (строка ниже) задаёт, "
                .. "сколько он может добавить.",
        },
    },
    pid_kd = {
        chart = "step",
        role = "kd",
        what = {
            en = "Velocity-loop derivative gain: damps the approach. The rate comes from "
                .. "Hall edges a few tens of ms apart, so D mostly amplifies quantisation "
                .. "noise here — 0 is the bench default.",
            ru = "Дифференциальный коэффициент контура скорости: гасит подход. Скорость "
                .. "считается по фронтам Холла, отстоящим на десятки мс, поэтому D здесь "
                .. "в основном усиливает шум квантования — на стенде по умолчанию 0.",
        },
    },
    pid_i_limit = {
        chart = "clamp",
        what = {
            en = "Ceiling on what the integral term may add, in output units (A on the "
                .. "FOC builds, V on six-step). The firmware derives it as Ki x the error "
                .. "limit, which is what a fresh module reports on G4.",
            ru = "Потолок того, что может добавить интегральная составляющая, в единицах "
                .. "выхода (А на сборках с FOC, В на шестишаговой). Прошивка выводит его "
                .. "как Ki × предел ошибки — именно это показывает новый модуль по G4.",
        },
    },
    oc_peak_a = {
        chart = "level",
        what = {
            en = "Instantaneous overcurrent trip: a phase-current magnitude above this "
                .. "for the peak window parks the bridge. It sits above the drive's "
                .. "current ceiling so a current-limited launch does not trip it.",
            ru = "Мгновенное срабатывание по перетоку: если модуль фазного тока выше этого "
                .. "значения держится всё окно пика, мост запирается. Порог стоит выше "
                .. "токового потолка привода, чтобы пуск с ограничением тока его не "
                .. "сработал.",
        },
    },
    oc_peak_ms = {
        chart = "window",
        what = {
            en = "How long the current may sit above the peak level before the guard "
                .. "trips. 0 disables the level check; shorter windows catch a short but "
                .. "still trips on switching noise.",
            ru = "Сколько ток может держаться выше пикового уровня до срабатывания защиты. "
                .. "0 отключает проверку уровня; короткое окно ловит замыкание, но "
                .. "срабатывает и от помех переключения.",
        },
    },
    oc_sustain_a = {
        chart = "level",
        what = {
            en = "Sustained overcurrent level: a lower threshold than the peak trip, for "
                .. "a load that heats the bridge over hundreds of ms rather than instantly.",
            ru = "Уровень длительного перетока: более низкий порог, чем пиковый, для "
                .. "нагрузки, которая греет мост сотни мс, а не мгновенно.",
        },
    },
    oc_sustain_ms = {
        chart = "window",
        what = {
            en = "How long the current may stay above the sustained level before the "
                .. "guard parks the bridge (the stall/thermal window).",
            ru = "Сколько ток может держаться выше длительного уровня, прежде чем защита "
                .. "закроет мост (окно застоя и нагрева).",
        },
    },
    stall_current = {
        chart = "level",
        what = {
            en = "Stall detector current floor: the wheel counts as stalled only while "
                .. "it draws more than this — a free-spinning wheel below the speed "
                .. "threshold is not a stall.",
            ru = "Токовый порог детектора застоя: колесо считается застрявшим, только пока "
                .. "потребляет больше этого — свободно вращающееся колесо ниже порога "
                .. "скорости застоем не считается.",
        },
    },
    stall_speed = {
        chart = "speed_threshold",
        what = {
            en = "Stall speed threshold, wheel rad/s: with current above the floor, a "
                .. "shaft slower than this for the stall window trips the guard. Set it "
                .. "above the rate noise at the speeds the cart actually holds.",
            ru = "Порог скорости застоя, рад/с на колесе: при токе выше порога вал "
                .. "медленнее этого значения в течение окна застоя срабатывает защита. "
                .. "Ставьте выше шума оценки скорости на тех скоростях, которые тележка "
                .. "реально держит.",
        },
    },
    stall_ms = {
        chart = "window",
        what = {
            en = "Stall window: how long the slow-and-loaded condition must persist "
                .. "before the guard trips. 0 disables the stall check (the six-step "
                .. "backend keeps its own Hall-progress watchdog instead).",
            ru = "Окно застоя: сколько должно длиться состояние «медленно и под нагрузкой» "
                .. "до срабатывания защиты. 0 отключает проверку застоя (шестишаговая "
                .. "сборка остаётся со своим сторожевым таймером по Холлам).",
        },
    },
    vel_lpf = {
        chart = "smoothing",
        what = {
            en = "Velocity-feedback low-pass, s. The Hall rate estimate is coarse (one "
                .. "edge every few tens of ms), so the loop closes over a filtered rate; "
                .. "longer is smoother but adds phase lag and invites hunting.",
            ru = "Фильтр низких частот по скорости, с. Оценка скорости по Холлам грубая "
                .. "(один фронт в десятки мс), поэтому контур замыкается по "
                .. "отфильтрованной скорости; больше — плавнее, но добавляет "
                .. "запаздывание и провоцирует рыскание.",
        },
    },
    speed_ramp = {
        chart = "slope",
        what = {
            en = "Set-point ramp, m/s^2: how fast the applied wheel-speed target may "
                .. "change. It shapes every command into a slope, so a step cannot ask "
                .. "for an acceleration the drive cannot deliver.",
            ru = "Разгон задания, м/с²: как быстро может меняться поданное задание "
                .. "скорости колеса. Он превращает любую команду в наклон, поэтому "
                .. "ступенька не потребует ускорения, которого привод не даст.",
        },
    },
    torque_ramp = {
        chart = "slope",
        what = {
            en = "q-current set-point slew, A/s: the soft start of the torque loop. It "
                .. "turns a step demand into a current ramp; too slow feels mushy, too "
                .. "fast kicks the gearbox.",
            ru = "Скорость изменения задания q-тока, А/с: мягкий пуск контура момента. "
                .. "Ступенька превращается в нарастание тока; слишком медленно — вяло, "
                .. "слишком быстро — удар по редуктору.",
        },
    },
    brake_current = {
        chart = "ceiling",
        what = {
            en = "Ceiling on torque that opposes the measured motion (the regen limit). "
                .. "It is what stops a moving wheel from pushing energy back into a supply "
                .. "that cannot sink it; 0 leaves the wheel to coast.",
            ru = "Потолок момента, противодействующего измеренному движению (предел "
                .. "рекуперации). Именно он не даёт движущемуся колесу гнать энергию "
                .. "обратно в питание, которое её не принимает; 0 — колесо катится "
                .. "накатом.",
        },
    },
    max_speed = {
        chart = "scale",
        what = {
            en = "Full-scale wheel speed: a normalized 1.0 command means this many m/s, "
                .. "so it sets what every -1..1 source (cart, joystick, serial T) can ask "
                .. "for. The module still clamps to what its rail can regulate.",
            ru = "Полная шкала скорости колеса: нормированная команда 1.0 означает столько "
                .. "м/с, то есть задаёт, что может запросить любой источник −1..1 "
                .. "(тележка, джойстик, T по serial). Модуль всё равно ограничивает "
                .. "команду тем, что его питание способно регулировать.",
        },
    },
}

return { kinds = KINDS, params = PARAMS }
