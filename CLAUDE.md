# CLAUDE.md

Guidance for working in the `gaz_cart` repository.

## What this is

Firmware + control stack for a differential-drive robot cart. Four BLDC wheel
modules (STM32G474, `SixStep/`) talk to a Raspberry Pi over Cyphal/CAN. The Pi
runs [radapter](radapter/) Lua scripts (`cart.lua`) to drive the cart and
compute odometry. A separate host runs `gui.lua` to tune each module's runtime
config over a websocket.

## Layout

- `SixStep/` — STM32 wheel-module firmware (git submodule). Uses `libvoltbro`
  (`SixStep/Drivers/libvoltbro`, also a submodule) and `libcxxcanard`.
- `radapter/` — the radapter engine (git submodule). See `radapter/CLAUDE.md`.
- `cart.lua` — cart runtime: drives the 4 modules and integrates odometry.
- `mods/odometry.lua` — diff-drive odometry from per-wheel linear velocity.
- `mods/config_defs.lua` — single source of truth for runtime-tunable config values,
  shared by `cart.lua` and `gui.lua`. **Keep the `id`s in sync with the
  `ConfigId` enum in `SixStep/App/app.cpp`.**
- `gui.lua` — config GUI; connects to `cart.lua`'s websocket from another host.

## Submodules are vendored forks

`SixStep/`, `radapter/`, and the nested `libvoltbro` / `libcxxcanard` are our own
forks, not pristine upstream. **Fix bugs directly in the submodule source** —
don't work around library bugs from the app side. Commit the fix in the
submodule, then bump the pointer in the parent repo.

## Building the firmware

When working with the driver code, build with **`ninja -C build`** run from the
`SixStep/` subdirectory. The toolchain (`arm-none-eabi-gcc`) and the configured
`build/` directory are already present.

## Packaging the cart

`scripts/Dockerfile.cross` builds one `gaz-cart_0.1.0_arm64.deb` for a
**64-bit Debian/Raspberry Pi OS Bookworm** RPi4. It includes headless radapter,
the shared SDK, all three plugins, and the Lua runtime. Build with:

    docker buildx build -f scripts/Dockerfile.cross --target pkg --output=out .

`scripts/packages.sh` wraps that command (`OUT=out` by default). The sysroot
stage executes arm64 apt, so the builder needs arm64 binfmt/QEMU or native
arm64 execution. Do not change host binfmt registration without approval.
`docker buildx build --check -f scripts/Dockerfile.cross .` checks the recipe
without executing its build stages. Cross dependencies are explicitly selected
with `GAZ_DEB_CROSS_DISTRIBUTION=bookworm`; do not reuse them for another distro.

Native packaging requires Debian/Ubuntu, `dpkg-dev`, and the Qt6/Boost/Ceres/
Eigen/TBB development packages. It uses `dpkg-shlibdeps` to derive dependencies
from the native binaries, not Bookworm package names. Non-Debian development
builds remain supported, but native DEB creation is not supported there.

```sh
cmake -S . -B build-headless -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr -DGAZ_PACKAGE_RUNTIME=ON \
    -DRADAPTER_GUI=OFF -DRADAPTER_SDK_ONLY=OFF -DRADAPTER_STATIC=OFF \
    -DRADAPTER_JIT=OFF -DRADAPTER_JIT_STATIC=OFF \
    -DGAZ_BUILD_FRAMES=ON -DGAZ_BUILD_NAV=ON -DGAZ_BUILD_SLAM=ON
cmake --build build-headless -j 4
ctest --test-dir build-headless -R '^packaging_smoke$' --output-on-failure
cpack --config build-headless/CPackConfig.cmake -B build-headless -G DEB
```

Fresh root builds default to headless with `GAZ_PACKAGE_RUNTIME=ON`. Explicit
GUI, SDK-only, static or partial-plugin configurations default that option OFF;
when changing an existing cache, also set `GAZ_PACKAGE_RUNTIME=OFF` explicitly.
Unified packaging requires headless shared radapter and all three plugins.
With runtime packaging OFF, the selected plugins are packaged separately.
Use separate build directories for development GUI and deployment builds.

Installed layout:

- `/usr/bin/radapter`: headless executable; `/usr/bin/gaz-cart`: manual launcher.
- `/usr/lib/libradapter-sdk.so`: shared engine, found through install RPATH.
- `/usr/lib/radapter/plugins/libgaz_{frames,nav,slam}.so`: native plugins.
- `/usr/share/gaz-cart/cart.lua`, `mods/*.lua`, `nodes/*.lua`: runtime scripts.
- `/usr/share/gaz-cart/packaging-smoke.lua`: hardware-free package smoke test.

Only named runtime/plugin components are packaged, never `Unspecified` headers,
static archives, GUI/QML, firmware, or the radapter test plugin. The package
conflicts with separately installed radapter and gaz plugin packages. Lua 5.4,
LuaSocket and LuaFileSystem are embedded with JIT OFF; no system Lua modules
are needed. QtGui is required for nav image processing, not a desktop/display.
Qt's SocketCAN backend (`libqt6serialbus6-plugins`) is an explicit dependency.

Install with `sudo apt install ./gaz-cart_0.1.0_arm64.deb`. Installation does
**not** start the cart, install a service, or configure CAN/serial hardware.
After configuring the correct CAN interface/bitrate and serial permissions,
launch manually with `gaz-cart` (arguments pass directly to `cart.lua`). The
launcher works from any cwd without `LUA_PATH`, `LD_LIBRARY_PATH`, or Qt plugin
environment setup. Bare plugin names use radapter's system plugin directory.
Do not store writable state under `/usr/share/gaz-cart`.

Safe installed-package verification:

```sh
timeout 15s radapter /usr/share/gaz-cart/packaging-smoke.lua
```

The `packaging_smoke` CTest stages only package components under
`build-headless/packaging-smoke/usr`, checks required/forbidden files, and runs
that smoke script from an unrelated cwd. It sets `QT_PLUGIN_PATH` only to
relocate the staged plugin directory, without `LD_LIBRARY_PATH` or Lua paths.
The smoke loads all plugins and imports Lua modules but never evaluates
`cart.lua` or creates hardware workers. **Never use cart startup as an install
test:** the hardware runtime can send wheel configuration and motor commands.
Restrict access to the control websocket (default port 6080); package installation
does not add authentication, command arbitration, or an emergency stop.

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
