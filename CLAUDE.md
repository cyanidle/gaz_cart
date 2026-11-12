# CLAUDE.md

Guidance for working in the `gaz_cart` repository.

## What this is

Firmware + control stack for a differential-drive robot cart. Four BLDC wheel
modules (STM32G474, `firmware/`) talk to a Raspberry Pi over Cyphal/CAN. The Pi
runs [radapter](radapter/) Lua scripts (`cart.lua`) to drive the cart and
compute odometry. A separate host runs `gui.lua` to tune each module's runtime
config over a websocket.

## Commit intermediate work

Commit as you go. A long bring-up or debugging session must never end with the
tree dirty — every coherent step lands in a commit before you move on:

- One logical step per commit, in the repo that owns it: the submodule first
  (`firmware/`, `firmware/Drivers/libvoltbro`, `libcxxcanard`, `radapter/`), then
  the parent pointer bump here as a separate commit.
- Bench-driven fixes commit **with their evidence**: the measured register dump,
  `S` telemetry samples or `v` values that prove the change belong in the
  message, because the hardware is not repeatable for a future reader.
- Intermediate is fine, broken is not: the tree must configure and build
  (`ninja -C build`, or the matching build dir) and the host tests
  (`ctest --test-dir tests/build`) must pass in every commit you leave behind.
- Leave no debugging leftovers: probe scripts, one-off logs and flash images go
  to the scratchpad, not into the repository.

## Layout

- `firmware/` — STM32 wheel-module firmware (git submodule). Uses `libvoltbro`
  (`firmware/Drivers/libvoltbro`, also a submodule) and `libcxxcanard`.
- `radapter/` — the radapter engine (git submodule). See `radapter/CLAUDE.md`.
- `cart.lua` — cart runtime: drives the 4 modules and integrates odometry.
- `mods/odometry.lua` — diff-drive odometry from per-wheel linear velocity.
- `mods/config_defs.lua` — single source of truth for runtime-tunable config values,
  shared by `cart.lua` and `gui.lua`. **Keep the `id`s in sync with the
  `ConfigId` enum in `firmware/App/config_ids.h`** (`tests/config_defs.lua`
  checks the table against it).
- `mods/drive_mux.lua` — arbiter every drive source goes through: the last source
  that commanded a non-zero twist owns the wheels, and the joystick stop organ
  gates all of them. `mods/joystick_axes.lua` — axis shaping (deadzone, inversion)
  and the stop organ itself, shared by `nodes/joystick.lua` and `debug/motor.lua`.
- `gui.lua` — config GUI; connects to `cart.lua`'s websocket from another host.
- `joystick/` — native `gaz_joystick` plugin: a HID **source** worker for the
  Universal RC Joystick (`/dev/hidraw*`) that emits `{axes, buttons, raw}`.
  `nodes/joystick.lua` turns it into a drive source (throttle on one axis, turn
  on the other, the slider as a stop organ) once `cart.lua`'s `JOYSTICK` tunable
  names its hidraw node. `joystick/install-udev.sh` installs the plugdev access
  rule, and it is not part of the `gaz-cart` DEB yet.

## Submodules are vendored forks

`firmware/`, `radapter/`, and the nested `libvoltbro` / `libcxxcanard` are our own
forks, not pristine upstream. **Fix bugs directly in the submodule source** —
don't work around library bugs from the app side. Commit the fix in the
submodule, then bump the pointer in the parent repo.

## Building the firmware

When working with the driver code, build with **`ninja -C build`** run from the
`firmware/` subdirectory. The toolchain (`arm-none-eabi-gcc`) and the configured
`build/` directory are already present. Flash through the repository scripts:
start `firmware/scripts/openocd.sh`, then run
`python3 firmware/scripts/upload.py firmware/build/vbcore.elf` (an ELF carries
its own load address; a BIN is written at `0x08000000`). The upload script waits
for each OpenOCD command and verifies the image before resetting the target.

`VB_MOTOR_IMPL` picks the wheel
controller: `simplefoc` (default, hall FOC with d/q current control over the
board shunts on the vendored Arduino-FOC tree), `sixstep` (libvoltbro six-step
commutation + external velocity PID) or `voltbro-foc` (libvoltbro FOC); all
three sit behind `App/motor_backend.h` and expose the same Cyphal ports and
config ids.

## Packaging

`GAZ_PACKAGE_TARGET` selects the unified package:

- `cart` — the merged native plugin DEB (frames + nav + slam). Native parts
  only: the engine ships as the separate `radapter-jit-headless` DEB and the Lua
  runtime is deployed from a repository checkout (the RPi)
- `gui` — GUI radapter + `gui.lua` + QML UI (the controller PC)
- `none` — no unified package; plugins are packaged separately (development)
- `auto` (default) — `cart` for a clean headless all-plugin configure, `gui`
  when `RADAPTER_GUI=ON`, `none` otherwise

`scripts/packages.sh [x64|arm64]` (default `arm64`) builds the whole stack
from the vendored submodule pins and drops three artifacts into `${OUT:-out}`:

- `radapter-jit-headless_<ver>_<arch>.deb` — the engine (`/usr/bin/radapter`,
  `/usr/lib/libradapter-sdk.so`) on the LuaJIT runtime, built by
  `radapter/scripts/Dockerfile.cross` (or `.native` for x64)
- `radapter-ros_<ver>_<arch>.deb` — the ROS2 bridge plugin
  (`/usr/lib/radapter/plugins/libradapter_ros.so`), built from a ROS Jazzy
  sysroot via `radapter/scripts/Dockerfile.ros`; its dependencies pull in (and
  thus verify) a matching ROS 2 Jazzy installation on the target
- `gaz-cart_<ver>_<arch>.deb` — the merged native plugins, built by
  `scripts/Dockerfile.cross` (or `.native` for x64)

All three are built against LuaJIT (`RADAPTER_JIT=ON`), so the engine DEB the
plugins depend on is the one the script ships and `dpkg -i out/*.deb` accepts
them together. The plugin DEBs carry no Lua ABI of their own.

The Lua cart runtime is not packaged: deploy a repository checkout and run
`radapter /path/to/gaz_cart/cart.lua <can-device> [ros-plugin-dir]`.
Individual DEBs can also be built directly with
`docker buildx build -f <dockerfile> --target <target> --output=out .`
(`headless-pkg`, `cross-deb-pkg`/`native-deb-pkg`, `pkg`). The arm64 sysroot
stages execute arm64 apt, so the builder needs arm64 binfmt/QEMU or native
arm64 execution. Do not change host binfmt registration without approval.
`docker buildx build --check -f <dockerfile> .` checks a recipe without
executing its build stages. Dependency names inside the containers are pinned
to the container distribution with `GAZ_DEB_DISTRIBUTION=bookworm`; do not
reuse them for another distro.

Native packaging requires Debian/Ubuntu, `dpkg-dev`, and the Qt6/Ceres/Eigen/
TBB development packages (boost serialization is built from source by CPM as
a static PIC library embedded into the slam plugin, so no system boost and no
versioned `libboost-serialization*` runtime dependency). It uses
`dpkg-shlibdeps` (>= 1.17) to derive dependencies from the native binaries,
not Bookworm package names; `libradapter-sdk.so` is resolved from the build
tree, so the engine DEB does not need to be installed first. Non-Debian
development builds remain supported, but native DEB creation is not supported
there.

```sh
cmake -S . -B build-headless -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr -DGAZ_PACKAGE_TARGET=cart \
    -DRADAPTER_GUI=OFF -DRADAPTER_SDK_ONLY=OFF -DRADAPTER_STATIC=OFF \
    -DRADAPTER_JIT=OFF -DRADAPTER_JIT_STATIC=OFF \
    -DGAZ_BUILD_FRAMES=ON -DGAZ_BUILD_NAV=ON -DGAZ_BUILD_SLAM=ON
cmake --build build-headless -j 4
ctest --test-dir build-headless -R '^packaging_smoke' --output-on-failure
cpack --config build-headless/CPackConfig.cmake -B build-headless -G DEB
```

For the controller PC, configure the same way with `-DRADAPTER_GUI=ON`
(target defaults to `gui`) to produce `gaz-cart-gui`. Use separate build
directories for development GUI and deployment builds.

**Wipe the build directory after a radapter update that touches its CPM
bootstrap.** radapter bootstraps CPM through `radapter/cmake/get_cpm.cmake`,
which downloads CPM into the build tree; a cached `CPM_FILE` pointing at the
retired vendored `radapter/cmake/CPM.cmake` makes CPM abort early and every
`CPMAddPackage` call fails with `Unknown CMake command "CPMAddPackage"`.

**Drop a stale cached Boost with `BOOST_CACHE_BUST=1 scripts/packages.sh
arm64`.** The DEB images keep `CPM_SOURCE_CACHE` in one build-cache mount
(`/cpm`) shared by both architectures, so a Boost tree fetched once is reused
by every later build; `BOOST_CACHE_BUST` (any non-empty value) deletes
`/cpm/boost` before configure and lets CPM re-download it, leaving the other
cached packages in place. CPM stores a package under its lowercased name, so
Boost lives in `/cpm/boost` regardless of the case of `CPMAddPackage(NAME
Boost ...)`.

**That re-download only extracts under a UTF-8 locale.** `boost-1.92.0-cmake.7z`
stores its pathnames as UTF-16LE and libarchive converts them through the
locale, so under the base image's default POSIX locale extraction aborts with
`Pathname cannot be converted from UTF-16LE to current locale` — which is why
the bookworm builder stages do `ENV LANG=C.UTF-8`. Before that, `/cpm/boost`
had only ever been populated by the ROS Jazzy image (which sets `LANG=C.UTF-8`
itself) through the shared mount, and the pure-bookworm builds silently reused
it.

radapter fetches Boost through CPM for its fiber support, and CPM ignores a
second `CPMAddPackage` under the same name. A project embedding radapter as a
subdirectory must therefore extend `BOOST_INCLUDE_LIBRARIES` before
`add_subdirectory(radapter)` rather than fetching Boost again — that is how
slam gets `serialization` from the same static PIC Boost, so the plugin still
embeds it and needs no `libboost-serialization*` runtime package.

Cart deployment on the RPi composes three pieces:

- `radapter-jit-headless` DEB: `/usr/bin/radapter`, `/usr/lib/libradapter-sdk.so`.
- `gaz-cart` DEB: `/usr/lib/radapter/plugins/libgaz_{frames,nav,slam}.so`;
  depends on `radapter-jit-headless | radapter-jit-gui` and
  `libqt6serialbus6-plugins`; conflicts with the separate gaz plugin packages
  and `gaz-cart-gui`.
- the Lua runtime from a repository checkout (`cart.lua`, `mods/`, `nodes/`);
  `require` finds `mods`/`nodes` next to `cart.lua`, no `LUA_PATH` needed.
- optional `radapter-ros` DEB: `/usr/lib/radapter/plugins/libradapter_ros.so`;
  pass `/usr/lib/radapter/plugins` as `cart.lua`'s ros-plugin-dir argument.

Installed layout (`gaz-cart-gui`): `/usr/bin/radapter` (GUI build),
`/usr/bin/gaz-gui` launcher, the shared SDK, `/usr/share/gaz-gui/gui.lua`,
`qml/*.qml`, and `mods/config_defs.lua`. Sent-config state is persisted under
`$XDG_DATA_HOME/gaz-gui/config.json` (the `GAZ_GUI_STATE` override), never
under `/usr/share`.

Only named runtime/plugin components are packaged, never `Unspecified` headers,
static archives, GUI/QML, firmware, or the radapter test plugin. LuaSocket and
LuaFileSystem are embedded in the engine, which links the system LuaJIT
(`libluajit-5.1-2` or `libluajit2-5.1-2`) in the packaged builds; no Lua modules
need to be installed. QtGui is required for nav image processing, not a
desktop/display. Qt's SocketCAN backend (`libqt6serialbus6-plugins`) is an
explicit dependency of `gaz-cart`.

Install with `sudo apt install ./radapter-jit-headless_*.deb ./gaz-cart_*.deb`
(plus `./radapter-ros_*.deb` on ROS machines). Installation does **not** start
the cart, install a service, or configure CAN/serial hardware. After
configuring the correct CAN interface/bitrate and serial permissions, launch
manually from the repository checkout. Bare plugin names use radapter's system
plugin directory. Do not store writable state under `/usr/share`.

Safe installed-package verification (from a repository checkout):

```sh
timeout 15s radapter /path/to/gaz_cart/tests/packaging/smoke.lua
```

The `packaging_smoke_cart` / `packaging_smoke_gui` CTests stage only package
components under `build-headless/packaging-smoke/usr`, check required/forbidden
files (the cart package must contain neither the engine nor Lua; the gui
package must contain no cart plugins), and — for `cart` — run the build's
`radapter` on the source-tree smoke script from an unrelated cwd. It sets
`QT_PLUGIN_PATH` only to relocate the staged plugin directory, without
`LD_LIBRARY_PATH` or Lua paths. The smoke loads all plugins and imports Lua
modules but never evaluates `cart.lua` or creates hardware workers. The gui
target is layout-verified only (QML needs a display). **Never use cart startup
as an install test:** the hardware runtime can send wheel configuration and
motor commands. Restrict access to the control websocket (default port 6080);
package installation does not add authentication, command arbitration, or an
emergency stop.

## Cyphal ports

Each module derives its ports from its (DIP-switch) node id:

| Port base | + node id | direction | type                              | meaning              |
|-----------|-----------|-----------|-----------------------------------|----------------------|
| 7100      | travel    | module →  | `uavcan.si.unit.length.Scalar`      | accumulated wheel travel since boot, m (signed) |
| 7200      | velocity  | module →  | `uavcan.primitive.scalar.Real32`    | shaft angular vel, rad/s |
| 7300      | linear    | module →  | `uavcan.primitive.scalar.Real32`    | wheel linear vel, m/s |
| 4000      | speed cmd | → module  | `uavcan.primitive.scalar.Real32`    | wheel speed command, -1..1 of the module's full-scale speed (config id 16) |
| 4050      | direct cmd| → module  | `uavcan.primitive.scalar.Real32`    | open-loop voltage, V |
| 4100      | config    | → module  | `uavcan.primitive.array.Integer32`  | `{id, numerator, denominator}` |

The travel topic is the odometry input: `nodes/odo.lua` differences it per wheel
and feeds the deltas to `mods/odometry.lua`. It is a running total from module
boot (not a wrapped count), so a module restart resets it to zero and the node
discards the resulting step.

## Serial bench channel (USART2)

`App/serial_line.cpp` runs a SimpleFOC-style text line protocol on USART2
(`PA2` TX / `PA3` RX, 115200 8N1, IRQ priority 5), independently of Cyphal.
Cyphal remains the cart's control path; commands received here drive the same
`MotorBackend`, so the channel is for bring-up, tuning and a terminal on a bench.

- One ASCII request per line, `\n`-terminated (`\r` ignored, blank lines
  skipped); every reply ends with `\r\n`:
  `T<float>` target wheel speed (-1..1 of the module's full-scale speed,
  clamped; `G16` reads that full scale back), `Q<float>` open-loop voltage (V),
  `E<0|1>` driver enable, `X` zero both set-points, `M<rate>` telemetry rate in
  Hz (0 = off, max 100), `S` one status line, `I` the firmware Git commit,
  backend and basic config, `K` the stop/limit counters,
  `W`/`W<0|1>` dump / arm the loop trace, `?` command list. Anything else
  answers `ERR <line>`.
- Status and telemetry share one format: `S t=<-1..1> v=<rad/s> e=<0|1> f=<FAULT>`,
  where `t` is the commanded normalized set-point (the module's commanded m/s
  over its full-scale speed, whatever source issued it). Backends with a
  current measurement (SimpleFOC's d/q loop) append ` i=<A>`, the measured
  q-axis current. With `V1` the telemetry tick sends ` D t=... v=... sp=...
  iq=... id=... im=... uq=... ud=... b=... n=... f=... e=...` instead
  (`App/serial_line.h` lists the fields); there `t` is the applied physical
  set-point in m/s, i.e. the ramp's output.
- **Speed is normalized end to end**: every command source (serial `T`, Cyphal
  4000, `cart.lua`) sends -1..1 of the module's configured full-scale wheel
  speed (config id 16, `max_speed`, persisted in the EEPROM record version 5),
  and the module alone maps it to m/s — wheel radius, gearbox ratio, back-EMF
  constant and modulation ceiling. The façade keeps the last normalized value,
  so editing config id 16 re-maps the stored command: lowering the full scale
  slows a running drive without waiting for the source's next command. A fresh module boots with the speed its own
  rail can regulate (`App/motor_limits.h`) and the live ceiling still bounds
  whatever the configuration says, so no source can command a speed the drive
  has no voltage for. Telemetry, travel (odometry) and the cart's internal
  wheel-speed plumbing stay physical: m/s and m.
- **Speed commands are clamped to what the rail can regulate.** `App/motor_limits.h`
  derives the reachable wheel rate from the live modulation ceiling and the
  drive's own EMF constant (`DriveInfo::torque_const * gear_ratio`, volts per
  wheel rad/s) with the current loop's voltage floor as headroom, and the
  `MotorBackend` façade holds the set-point it ramps towards there. Past the
  ceiling the current loops have no voltage authority, the phase current is set
  by the back-EMF instead, and on the bench that ended at the gate driver's own
  latched fault. The same EMF constant now sizes the d/q current loops' voltage
  limit (a fixed V per rad/s cap trailed the EMF and turned into a
  speed-dependent brake, which is what made the speed loop hunt). A clamped
  target reports `# simplefoc: target clamped, ceiling (m/s) <value>`.
- **`App/loop_trace.h` is the bench black box.** The 100 Hz `D` line cannot show
  why a drive stutters — a stutter is a few control periods long. The trace
  keeps the last ~51 ms of loop state (angle error, tracked angle, wheel rate,
  d/q and phase currents, d/q and bus voltages, raw Hall state, event bits) at
  2.5 kHz in a 128-sample pre-trigger ring, plus counters for the events a
  single six-step step cannot explain: Hall transitions (`hall_bad`), Hall
  edges, rate samples outside the drive's EMF range (`rate_bad`), guard trips,
  supply-derate entries, gate-driver faults and speed-target clamps. `K` prints
  the counters,
  `W` dumps the ring, `W1` arms an automatic dump on the next such event. A dump
  is ~12 kB and takes the link over for about a second: it is emitted a few
  lines per poll pass into the 1 kB transmit ring, and the telemetry waits until
  it is through.
- Floats are formatted by hand: the link line uses `--specs=nano.specs` without
  `_printf_float`, so `%f` prints nothing on this target.
- A line that overruns the buffer, or a burst that overruns the 128-byte RX
  ring, is discarded up to the next newline rather than executed half-parsed.
- Setup is spread over `Src/usart.c` (init + MSP), `Src/main.c`,
  `Src/stm32g4xx_it.c`, `Inc/stm32g4xx_hal_conf.h` and both `.ioc` files.
  Enabling `HAL_UART_MODULE_ENABLED` also made libvoltbro's UART configurator
  compile, so `CMakeLists.txt` excludes `voltbro/config/serial/` (it needs
  `nanoprintf`, which is not vendored, and its REPL is unused here).
- The HAL UART driver files are vendored at the same HAL version (V1.2.5) as the
  rest of `Drivers/STM32G4xx_HAL_Driver`; keep them in step when updating the HAL.
- `tests/serial_line.cpp` covers the protocol on the host (mock HAL UART in
  `tests/hal`); run `ctest --test-dir tests/build`.
- From radapter use `Serial{framing = "line", protocol = "text", ...}`: each
  received line arrives in Lua as a string and a sent string is terminated with
  `\n`. See `radapter/examples/serial/serial_line.lua`.
- Interactive terminal:
  `radapter radapter/examples/serial/serial_repl.lua /dev/ttyACM0` forwards
  every stdin line to the port and prints the replies to stdout (empty line =
  `S`, `:q` quits). Shutdown zeroes the set-points and disables the driver.
- Bench scripts live in `debug/`: `bench.py` is the line client
  (`python3 debug/bench.py script debug/bench_speed_sweep.txt`, port from
  `$BENCH_PORT`), `bench_speed_sweep.txt` holds steady speeds to fit the EMF
  constant against the applied q-voltage, `bench_maxspeed_hold.txt` holds full
  scale both directions with the trace armed, and `bench_voltage_hold.txt` does
  the same in open loop (`Q`). All three park the drive at the end.
- `K` answers one counter per mechanism that can stop the drive or take
  authority from a regulator (`K1` hall: illegal transitions, counted edges,
  angle calls, implausible rates, sector pulls, 0/2pi wraps; `K2` stop: guard
  overcurrent/stall, derate rail-high/rail-low, driver faults, coasts; `K3`
  limit: target clamps, current-limit and voltage-limit episodes, brake clamp,
  regulator resets; `K4` loop: periods over budget, the worst one, rate
  timeouts and free-store allocations made while the loop ran; `K5` phase: the
  worst duration of each phase of the app loop). Read it before and after an
  event and the counter that moved names the mechanism. Sector pulls track the
  edge count (~1 per edge) by construction; 0/2pi wraps run at one per rotor
  revolution. The loop makes no heap allocations — `heap=` reports the count
  and the worst call, measured through the allocator wrappers.
