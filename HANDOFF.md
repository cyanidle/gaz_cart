# HANDOFF — wheel module bench bring-up (2026-11-12, closed)

All software items from the previous handoff are implemented. The only remaining
hardware limitation is the 18.5 V bench supply: it cannot show true `Q10`
operation or speeds above its live electrical ceiling.

## Final state

- **Timebase:** `micros_64()` now uses `DWT->CYCCNT`, extended across its 32-bit
  wrap in software. TIM7 interrupt latency can no longer move time backwards.
  `cycle_counter` host tests cover repeated wraps.
- **Directional derate:** rail-high limits only regenerative torque; rail-low
  limits only driving torque (including launch from standstill). The opposite
  torque sign remains available. Open-loop voltage keeps its magnitude derate.
- **Voltage ceilings:** driver cap 60 V (below the DRV8323's 65 V absolute
  maximum), motor phase cap 30 V. The live bus and 0.9 modulation headroom still
  apply. `Q10` needs at least about 22.3 V; at 18.5 V the applied q-voltage is
  about 8.3 V.
- **Wheel/config:** live EEPROM wheel diameter is 0.32 m. The compiled defaults
  are full-scale speed 0.5 m/s and the validated SimpleFOC tuning (Kp 2.1,
  Ki 1.11, Kd 0, velocity LPF 0.05 s, speed ramp 0.75 m/s², torque ramp 50 A/s,
  brake current 2.2 A); the bench module's record now holds the values tuned in
  the next bullet. The sustained overcurrent guard stays at 10 A / 250 ms
  because the host safety test rejects a threshold below the drive's 8 A
  current ceiling.
- **Q10 bench range:** `debug/tune.lua` defaults to `vmax=10` and
  `debug/bench_voltage_hold.txt` holds `Q10` / `Q-10`.
- **Serial identity:** `I` returns the full firmware Git commit (`-dirty` when
  appropriate), backend, and C0/C16/C1/C2/C3. The commit header is regenerated
  at every build without changing its timestamp when the commit is unchanged.
- **Disable safety:** automatic gate-driver fault recovery can no longer
  re-enable a motor after an explicit `E0`.
- **EEPROM wear and the 14 ms stall:** the wheel-config record is written only
  when the drive is idle, once per debounce window, and only if a value really
  differs from what the record holds. Measured on the bench: a `C` write under
  a running drive used to stall the loop 13.9 ms (entirely `K5 persist`, 2.8
  speed-loop periods with the bridge on a stale vector); now it logs
  `eeprom: save deferred until the drive is idle` and writes at the next stop.
- **Live bench config (EEPROM v5 on the test module):** `C16 = 0.345 m/s`
  (the ceiling is 0.3507-0.3510 and tracks the bus; anything at or above it
  clamps every full-scale command and resets the regulator), `C12 = 0.030 s`
  velocity LPF (halves the full-scale jitter against 0.015 s without the
  voltage-limit episodes 0.05 s and 0.08 s produce), Kp 3.42 / Ki 2.69, ramp
  0.9 m/s², brake 3.1 A, wheel 0.32 m. Achieved 2.15 wheel rad/s (0.344 m/s,
  1314 rotor rpm at the 64:1 gearbox) both directions with `K1 bad=0`, no
  derate, no fault.

## Bench evidence

Release `Q10` hold, 40 s forward + 20 s reverse at the 18.5 V rail:

```
K1 hall bad=0 rate_bad=0
K2 guard_oc=0 stall=0 driver=0
K4 slow=0 max_us=631 heap=0/7
K5 pid=7 update=73 poll=161 persist=10 cyphal=568
```

The old 4.21 s supply-hiccup pattern did not appear in this run. Two high-rail
derates occurred during the final coast, not during either steady hold.

Release full-scale speed hold with C0=0.32 m and C16=0.5 m/s: the live ceiling
was about 0.3515 m/s (~2.2 wheel rad/s), held in both directions with:

```
K1 bad=0
K2 guard_oc=0 stall=0 derate_hi=0 derate_lo=0 driver=0
K4 slow=0 max_us=624 heap=0/7
```

The earlier one-off 65 ms trace-capture poll maximum did not recur in this final
run. One implausible Hall-rate sample was counted during the final coast; there
was no associated guard, derate, driver fault, or loop overrun.

Debug does reproduce stutters on the same `Q10` script: 15 over-budget loops,
worst 3.159 ms, one low-rail derate and one gate-driver fault. Its Cyphal phase
reached 2.903 ms, versus 0.568 ms in Release. Use Release for motor bench tests
and deployment; Debug's unoptimized Cyphal work can hold the bridge on a stale
vector long enough to cause the event.

## Open items (needs the bench, not code)

Measured state is green: `ctest --test-dir firmware/tests/build` 7/7, Debug and
Release both build, the cart-side Lua self-tests pass, and `mods/config_defs.lua`
ids still match `App/config_ids.h` (C0..C16).

1. **Max speed — the rail is the limit; two reservations sit on top of it.**
   Measured 2026-11-12 (Release, wheel free, `Q` sweep): `uq 2.0/4.0/6.0/8.0/
   8.30 V` give `0.663/1.383/2.085/2.748/2.845` wheel rad/s, i.e. **0.455 m/s,
   27 wheel rpm, 1738 rotor rpm** at the rail's full applied voltage, dead steady
   (3.5 % spread over 128 samples). The chain is `18.5 V → SinePWM phase limit
   Vdc/2 = 9.25 V → ×0.9 bootstrap headroom = 8.325 V → / 0.0456 V per rotor
   rad/s = 182 rad/s`. Effective EMF 0.0456-0.0472 V per rotor rad/s
   (KV ≈ 210-220 rpm/V against a 230 nameplate), so `ppairs=4`,
   `gear_ratio=64` and `torque_const=0.0478` all agree with the measurement —
   there is no stray factor. What the numbers *do* hide is the software
   reservation, and it is what to reason about before any modulation work:

   | | modulation ceiling | open loop (no speed loop) | speed mode (cart, `C16` max) |
   |---|---|---|---|
   | SinePWM today | 8.325 V | 0.455 m/s, 1738 rpm | (8.325−1.6)/3.059 = 0.352 m/s, 1343 rpm |
   | SVPWM (Vdc/√3) | 9.613 V | ~0.53 m/s, ~2010 rpm | (9.613−1.6)/3.059 = 0.419 m/s, 1601 rpm |

   The ~23 % between the open-loop and speed-mode columns is
   `simplefoc_current_loop_v_floor` (1.6 V of headroom so the current loops keep
   authority over the phase current, `App/motor_limits.h`). Speed mode can never
   use the whole applied voltage. (The open-loop column divides by the *measured*
   EMF, 8.30 V / 2.845 rad/s = 2.918 V per wheel rad/s; the speed-mode and SVPWM
   columns use the compiled `torque_const × 64 = 3.059`, so those two are ~5 %
   pessimistic.)

   **2 k rpm in speed mode needs ~11.6 V applied**, i.e. a rail of ~22.3 V with
   SVPWM or ~25.8 V with SinePWM; on this rail SVPWM alone buys 1601 rpm
   (+19 % over today's 1343). Switching `foc_modulation = SinePWM` →
   space-vector is a change in the vendored Arduino-FOC path
   (`App/backends/motor_backend_simplefoc.cpp`) and has to re-clear the
   bootstrap-refresh constraint that produced the 0.9 headroom. Left for a later
   session on purpose; a ≥22 V supply is the other route.
2. **Cyphal phase ~2.9 ms in Debug — bounded now, cause is codegen.** A pass can
   no longer drain 8 RX frames and the whole TX queue (`can_loop` serves one
   frame and one queue item), and `K5` now splits the phase into `can=`,
   `report=`, `beat=`. On the same `Q10` script Release measured 568 us against
   Debug's 2903 us, so the remainder is unoptimised libcanard transfer
   reassembly/CRC, not a logic defect. Verification: run the hold in Debug, read
   `K5`; if `can=` still reaches ~2 ms with one frame per pass, bench in Release
   and treat Debug as a diagnostics-only build.
3. **Rare sudden stop — one candidate removed, no reproducer yet.** The 13.9 ms
   EEPROM stall on every config write (fixed above: writes now wait for an idle
   drive) was a plausible in-motion stall and is gone. What remains is named by
   the counter that moves: run `K` before, reproduce, run `K` after and diff
   `K1`..`K5`; `W1` auto-dumps the 51 ms ring on the next hall_bad / rate_bad /
   guard trip / derate entry / driver fault / target clamp, and `W` dumps it on
   demand. Note that the dump itself costs ~65 ms in `K5 poll` (the ~12 kB takes
   the serial link over), so arm `W1` only when you want the capture. The only
   trace so far is one implausible Hall-rate sample in the tail of the final
   coast, with no guard, derate, fault or overrun behind it — that Hall rate is
   the first thing to look at, and the `I` line says exactly which image
   produced the log.

## Verification / reproduction

```sh
cmake --build firmware/tests/build
ctest --test-dir firmware/tests/build --output-on-failure   # 8/8
ninja -C firmware/build                                     # Debug
ninja -C firmware/build-release                             # Release

python3 firmware/scripts/upload.py firmware/build-release/vbcore.elf
python3 debug/bench.py send I 1
python3 debug/bench.py script debug/bench_voltage_hold.txt
python3 debug/bench.py script debug/bench_maxspeed_hold.txt
```

Both scripts finish with zero commands and `E0`. Never run them on a powered
cart; the wheel must be free.

For speeds above ~0.35 m/s or a true 10 V q-voltage, connect a supply of at
least 22.3 V that can hold the load and sink or tolerate braking energy. The
compiled 60 V / 30 V caps no longer prevent that test.
