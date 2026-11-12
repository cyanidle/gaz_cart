# HANDOFF — wheel module bench bring-up (2026-11-12)

State at hand-off: firmware flashed and correct, tree clean. The periodic
stutter is **identified as the bench supply**, not the drive. The drive's top
speed at 18.5 V went from 0.1943 to **0.2795 m/s** by measuring the EMF
constant instead of trusting the nameplate.

## Where things stand

| # | finding | status |
|---|---------|--------|
| 1 | **The regular stutter is the bench supply.** At a steady `Q6` the rail runs a hiccup cycle with a **4.21–4.22 s period**, every time, both directions: 18.5 V → **22.0 V** → 20.5 V → **16.5 V** → 18.5 V, `uq` collapsing and the wheel dropping 1.5 → 0.3 rad/s at each event. 41 out-of-window samples in 60 s, spaced 4.21/4.22 s apart | **external** — needs a supply that can hold the load (see below) |
| 2 | `DriveInfo::torque_const` was the nameplate 0.069 V/(rotor rad/s), **44 % high**. Measured 0.0478 (stable 0.0472–0.0482 over 0.27–0.34 m/s, both directions) | fixed, committed |
| 2b | Two compiled caps bound the modulation limit well above today's rail: `simplefoc_driver_voltage_limit_v` 19 V and `simplefoc_motor_voltage_limit_v` 17 V of phase (0.64 m/s on their own). They must be raised before any bigger supply helps | **open**, see the table below |
| 3 | The speed/voltage budgets were derived from a **single ADC bus frame**, so the ceiling moved under a held command (applied target wandered 0.1900–0.1970 m/s) | fixed (0.05 s bus filter), committed |
| 4 | The supply derate **entered on a single frame** and zeroed the whole torque demand, then took 0.25 s to rebuild: 51 entries in a 30 s steady hold at 18.5 V | fixed (1.5 ms confirmation), 51 → 19, committed |
| 5 | `micros_64()` can step **~1 ms backwards**: `millis_k` is incremented by the TIM7 callback some time after the counter wraps | **open**, documented in `App/common.cpp`; needs a hardware timebase (`DWT->CYCCNT`) |
| 6 | The loop is **allocation-free** — proven, not assumed (`--wrap=malloc,...`, `heap=` in `K`) | done |
| 7 | Every mechanism that can stop the drive or take authority from a regulator has its own counter (`K1..K5`) | done |

## What the counters say (typical run, wheel free)

```
# K1 hall bad=0 edges=16792 calls=279018 rate_bad=0 snap=16778 wrap=699
# K2 stop guard_oc=0 stall=0 derate_hi=20 derate_lo=16 driver=0 coast=0
# K3 limit clamp=0 cur=0 volt=0 brake=0 reset=1
# K4 loop slow=9 max_us=3036 vel_to=63 heap=0/7 heap_us=0/4
# K5 ph pid=12 update=163 poll=206 persist=11 cyphal=252      (idle maxima)
```

Readings that are **not** faults: `snap ≈ edges` (a new sector centre is half a
sector ahead of the previous one, so the half-sector clamp fires by construction)
and `wrap` = one per rotor revolution (699 wraps ≈ 16792 edges / 24). `heap=` must
stay `0/7`; anything in the first number means a hot-path allocation.

`slow=9..15` with `max_us≈3036` is the 3 ms stall — its rate (~1 per 4 s) matches
the supply hiccup above.

## The top speed question

Ceiling = `(modulation limit − 1.6 V floor) / EMF`, EMF = 3.06 V per **wheel**
rad/s, wheel Ø 0.255 m in config (r = 0.1275). The modulation limit is
`min(min(bus, simplefoc_driver_voltage_limit_v) · 0.5 · 0.9,
simplefoc_motor_voltage_limit_v)` — and **two compiled caps bound it**, which is
why the rail is not the only lever:

| bus | modulation limit | ceiling, as built | ceiling, both caps lifted |
|-----|------------------|-------------------|---------------------------|
| 18.5 V (today) | 8.33 V | **0.2803 m/s** (measured 0.2795) | same |
| 24 V | 8.55 V (driver cap 19 V) | 0.2897 m/s | 0.383 m/s |
| 36 V | 8.55 V (driver cap) | 0.2897 m/s | 0.609 m/s |
| 48 V | 8.55 V (driver cap) | 0.2897 m/s | 0.834 m/s |
| 60 V | 8.55 V (driver cap) | 0.2897 m/s | 1.059 m/s |

So **a bigger supply alone buys nothing**: `simplefoc_driver_voltage_limit_v`
(19 V) and `simplefoc_motor_voltage_limit_v` (17 V, a phase-voltage cap) must be
raised first, and the motor cap is the tighter one — 17 V of phase bounds the
drive at 0.64 m/s whatever the rail.

What each target needs (with the caps raised to match):

| target | wheel rate | phase voltage | bus (·2/0.9) |
|--------|-----------|---------------|--------------|
| 0.4 m/s | 3.14 rad/s | 11.2 V | 24.9 V |
| 0.64 m/s | 5.02 rad/s | 17.0 V | 37.7 V |
| 1.0 m/s | 7.84 rad/s | 25.6 V | 56.9 V |

`C16` (`max_speed`) is currently **0.675 m/s**: any command above ~0.41 of full
scale is clamped to the ceiling, which only shortens the useful slider range —
set it near the ceiling if the clamp message is noise.

Also open: the wheel diameter in config is **0.255 m**, while the build docs say
0.32 m. Every m/s number and all odometry scale with it — worth settling (at
0.32 m the same wheel rate is 1.25× the speed).

## Reproducing

```sh
# build + flash (OpenOCD telnet 4444, stlink-v2 + stm32l4x)
cmake --build firmware/build -j 8
python3 firmware/scripts/upload.py firmware/build/vbcore.elf

# host tests
ctest --test-dir firmware/tests/build --output-on-failure      # 6/6

# bench — check nobody holds the port first
fuser /dev/ttyACM0
python3 debug/bench.py script debug/bench_speed_sweep.txt      # EMF fit
python3 debug/bench.py script debug/bench_maxspeed_hold.txt    # stutter hunt
python3 debug/bench.py script debug/bench_voltage_hold.txt     # supply hiccup
```

The tuner GUI (`debug/tune.lua`, needs a radapter GUI build): the direct-voltage
min/max are now editable in the panel, the target slider is normalized -1..1 and
shows the m/s it commands, and the row layout shares one column metric.

## Next steps, in order

1. **Supply**: put the module on a supply that holds 18–19 V under 5 A and does
   not hiccup, or confirm the 4.21 s cycle is the supply's own protection. Until
   then every "stutter" measurement below ~0.28 m/s is contaminated.
2. **Directional derate**: `supply_derate = 0` zeroes the whole velocity-loop
   limit. Rail high should cut only the regen/brake component, rail low only the
   driving component — a rail blip then costs torque in one direction instead of
   a torque hole. `derate_hi=20..30` per minute still shows real excursions.
3. **Timebase**: move `micros_64()` to `DWT->CYCCNT` (25 s wrap at 170 MHz,
   extended in software). Removes the 1 ms backwards step from every `dt`.
4. **Hall wrap**: measured and cleared — 699 wraps/run, no guard trip, no driver
   fault, no derate attributable to them, and the electrical angle is
   wrap-invariant because the pole count is an integer. Not the stutter.
5. **Above 0.28 m/s**: raise `simplefoc_driver_voltage_limit_v` and
   `simplefoc_motor_voltage_limit_v` first (17 V of phase is the real bound at
   0.64 m/s), then the rail — see the table above for what each target needs.
   The modulation headroom (0.9) is a bootstrap-refresh constraint on the
   DRV8328, not a knob to spend.

## Commits

- `firmware` `cd06e60` — measured EMF constant, filtered bus budgets, derate
  confirmation, angle-wrap counter (on the applied angle, not the free-run)
- `firmware` (this step) — per-phase loop timing (`K5`), timebase defect
  documented in `App/common.cpp`
- parent `80e2144` — `debug/bench.py` + bench scripts + the `K` counter list in
  `CLAUDE.md`
- earlier: normalized -1..1 speed commands with a configurable full scale
  (`firmware 0e0cfb2`), tuner row alignment and normalized targets (`8a47695`)
