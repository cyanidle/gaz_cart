# CLAUDE.md

Guidance for working in the `gaz_cart` repository.

## What this is

Firmware + control stack for a differential-drive robot cart. Four BLDC wheel
modules (STM32G474, `SixStep/`) talk to a Raspberry Pi over Cyphal/CAN. The Pi
runs [radapter](radapter/) Lua scripts (`main.lua`) to drive the cart and
compute odometry. A separate host runs `gui.lua` to tune each module's runtime
config over a websocket.

## Layout

- `SixStep/` — STM32 wheel-module firmware (git submodule). Uses `libvoltbro`
  (`SixStep/Drivers/libvoltbro`, also a submodule) and `libcxxcanard`.
- `radapter/` — the radapter engine (git submodule). See `radapter/CLAUDE.md`.
- `main.lua` — cart runtime: drives the 4 modules and integrates odometry.
- `odometry.lua` — diff-drive odometry from per-wheel linear velocity.
- `config_defs.lua` — single source of truth for runtime-tunable config values,
  shared by `main.lua` and `gui.lua`. **Keep the `id`s in sync with the
  `ConfigId` enum in `SixStep/App/app.cpp`.**
- `gui.lua` — config GUI; connects to `main.lua`'s websocket from another host.

## Submodules are vendored forks

`SixStep/`, `radapter/`, and the nested `libvoltbro` / `libcxxcanard` are our own
forks, not pristine upstream. **Fix bugs directly in the submodule source** —
don't work around library bugs from the app side. Commit the fix in the
submodule, then bump the pointer in the parent repo.

## Building the firmware

When working with the driver code, build with **`ninja -C build`** run from the
`SixStep/` subdirectory. The toolchain (`arm-none-eabi-gcc`) and the configured
`build/` directory are already present.

## Cyphal ports

Each module derives its ports from its (DIP-switch) node id:

| Port base | + node id | direction | type                              | meaning              |
|-----------|-----------|-----------|-----------------------------------|----------------------|
| 7100      | encoder   | module →  | `uavcan.primitive.scalar.Natural32` | raw hall count       |
| 7200      | velocity  | module →  | `uavcan.primitive.scalar.Real32`    | shaft angular vel, rad/s |
| 7300      | linear    | module →  | `uavcan.primitive.scalar.Real32`    | wheel linear vel, m/s |
| 4000      | speed cmd | → module  | `uavcan.primitive.scalar.Real32`    | target wheel speed, m/s |
| 4050      | direct cmd| → module  | `uavcan.primitive.scalar.Real32`    | open-loop voltage, V |
| 4100      | config    | → module  | `uavcan.primitive.array.Integer32`  | `{id, numerator, denominator}` |
