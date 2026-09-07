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

## Packaging

`GAZ_PACKAGE_TARGET` selects the unified package:

- `cart` — the merged native plugin DEB (frames + nav + slam). Native parts
  only: the engine ships as the separate `radapter-headless` DEB and the Lua
  runtime is deployed from a repository checkout (the RPi)
- `gui` — GUI radapter + `gui.lua` + QML UI (the controller PC)
- `none` — no unified package; plugins are packaged separately (development)
- `auto` (default) — `cart` for a clean headless all-plugin configure, `gui`
  when `RADAPTER_GUI=ON`, `none` otherwise

`scripts/packages.sh [x64|arm64]` (default `arm64`) builds the whole stack
from the vendored submodule pins and drops three artifacts into `${OUT:-out}`:

- `radapter-headless_<ver>_<arch>.deb` — the engine (`/usr/bin/radapter`,
  `/usr/lib/libradapter-sdk.so`), built by `radapter/scripts/Dockerfile.cross`
  (or `.native` for x64)
- `radapter-ros_<ver>_<arch>.deb` — the ROS2 bridge plugin
  (`/usr/lib/radapter/plugins/libradapter_ros.so`), built from a ROS Jazzy
  sysroot via `radapter/scripts/Dockerfile.ros`; its dependencies pull in (and
  thus verify) a matching ROS 2 Jazzy installation on the target
- `gaz-cart_<ver>_<arch>.deb` — the merged native plugins, built by
  `scripts/Dockerfile.cross` (or `.native` for x64)

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

Cart deployment on the RPi composes three pieces:

- `radapter-headless` DEB: `/usr/bin/radapter`, `/usr/lib/libradapter-sdk.so`.
- `gaz-cart` DEB: `/usr/lib/radapter/plugins/libgaz_{frames,nav,slam}.so`;
  depends on `radapter-headless | radapter-gui` and
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
static archives, GUI/QML, firmware, or the radapter test plugin. Lua 5.4,
LuaSocket and LuaFileSystem are embedded in the engine with JIT OFF; no system
Lua modules are needed. QtGui is required for nav image processing, not a
desktop/display. Qt's SocketCAN backend (`libqt6serialbus6-plugins`) is an
explicit dependency of `gaz-cart`.

Install with `sudo apt install ./radapter-headless_*.deb ./gaz-cart_*.deb`
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
| 7100      | encoder   | module →  | `uavcan.primitive.scalar.Natural32` | raw hall count       |
| 7200      | velocity  | module →  | `uavcan.primitive.scalar.Real32`    | shaft angular vel, rad/s |
| 7300      | linear    | module →  | `uavcan.primitive.scalar.Real32`    | wheel linear vel, m/s |
| 4000      | speed cmd | → module  | `uavcan.primitive.scalar.Real32`    | target wheel speed, m/s |
| 4050      | direct cmd| → module  | `uavcan.primitive.scalar.Real32`    | open-loop voltage, V |
| 4100      | config    | → module  | `uavcan.primitive.array.Integer32`  | `{id, numerator, denominator}` |
