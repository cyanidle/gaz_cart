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
- **Wheel/config:** live EEPROM wheel diameter is 0.32 m and full-scale speed is
  0.5 m/s. The tuner and firmware defaults now use the validated SimpleFOC
  tuning: Kp 2.1, Ki 1.11, Kd 0, velocity LPF 0.05 s, speed ramp 0.75 m/s²,
  torque ramp 50 A/s, brake current 2.2 A. The sustained overcurrent guard stays
  at 10 A / 250 ms because the host safety test rejects a threshold below the
  drive's 8 A current ceiling.
- **Q10 bench range:** `debug/tune.lua` defaults to `vmax=10` and
  `debug/bench_voltage_hold.txt` holds `Q10` / `Q-10`.
- **Serial identity:** `I` returns the full firmware Git commit (`-dirty` when
  appropriate), backend, and C0/C16/C1/C2/C3. The commit header is regenerated
  at every build without changing its timestamp when the commit is unchanged.
- **Disable safety:** automatic gate-driver fault recovery can no longer
  re-enable a motor after an explicit `E0`.

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

1. **Max speed — blocked by the supply, nothing left in software.** The compiled
   caps are 60 V / 30 V now; what binds is the 18.5 V rail, which regulates about
   0.3515 m/s (~2.2 wheel rad/s) and clamps `Q10` to ~8.3 V applied. To close
   this, connect a supply of at least 22.3 V that can hold the load and sink
   braking energy, then run `debug/bench_maxspeed_hold.txt` and
   `debug/bench_voltage_hold.txt` (`Q10`/`Q-10`) and compare `K` before/after:
   expect `K1 bad=0`, `K2 derate_hi=derate_lo=0 driver=0`, `K4 slow=0`. Arm
   `W1` before the run so an event captures itself.
2. **Cyphal phase ~2.9 ms in Debug — bounded now, cause is codegen.** A pass can
   no longer drain 8 RX frames and the whole TX queue (`can_loop` serves one
   frame and one queue item), and `K5` now splits the phase into `can=`,
   `report=`, `beat=`. On the same `Q10` script Release measured 568 us against
   Debug's 2903 us, so the remainder is unoptimised libcanard transfer
   reassembly/CRC, not a logic defect. Verification: run the hold in Debug, read
   `K5`; if `can=` still reaches ~2 ms with one frame per pass, bench in Release
   and treat Debug as a diagnostics-only build.
3. **Rare sudden stop — no reproducer, tooling is in place.** The mechanism is
   named by the counter that moves: run `K` before, reproduce, run `K` after and
   diff `K1`..`K5`; `W1` auto-dumps the 51 ms ring on the next hall_bad /
   rate_bad / guard trip / derate entry / driver fault / target clamp, and `W`
   dumps it on demand. The only trace of this so far is one implausible Hall-rate
   sample in the tail of the final coast, with no guard, derate, fault or
   overrun behind it — that Hall rate is the first thing to look at, and the
   `I` line now says exactly which image produced the log.

## Verification / reproduction

```sh
cmake --build firmware/tests/build
ctest --test-dir firmware/tests/build --output-on-failure   # 7/7
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
