# Joystick: native HID joystick source

`gaz_joystick` is a native radapter plugin with one **source** worker: it reads
raw HID reports from a hidraw node and pushes the current controls into a Lua
pipeline. It is the prototype step towards teleoperating the cart; it holds no
cart state and sends no commands.

The device is a Universal RC Joystick
([VBCores/8-Axis-Universal-RC-Joystick](https://github.com/VBCores/8-Axis-Universal-RC-Joystick)),
an STM32F103 board (VID:PID `0483:5710`) that presents RC channels as a USB HID
joystick.

## Report layout

The firmware's HID report descriptor is:

```text
8 axes   Generic Desktop X, Y, Z, Rx, Ry, Rz, Slider, Dial
         logical 0..32767, 16 bits each, little-endian
1 button bit 0 = pressed, 7 padding bits
```

so the 8-axis build delivers **17 bytes** per report: `axis[i]` at bytes
`2i..2i+1`, buttons in the last byte. RC channels are clamped by the firmware
to `1000..2000` and scaled to `0..0x7fff`, so the plugin recovers the stick
position as `2 * raw / 0x7fff - 1`.

The worker does not hard-code the axis count: a report of `n` bytes is read as
`(n - 1) / 2` 16-bit axes plus one button byte.

## Worker

```lua
load_plugin("gaz_joystick")

local joystick = Joystick { path = "/dev/hidraw5" }
```

| config | meaning |
|---|---|
| `path` | hidraw device node; `/dev/hidraw5` by default |
| `name`, `category` | inherited from `WorkerConfig` |

Each report emits one message on the worker's data channel:

```lua
{
    axes = { x = 0.0, y = -0.42, z = -1.0, rx = 0.05, ry = 0.0,
             rz = -1.0, slider = -1.0, dial = -1.0 },  -- -1..1
    buttons = { 1 },                                  -- pressed, 1-based
    raw = { axes = { 16383, 9264, ... }, buttons = 1 },
}
```

Axes beyond the eight named ones appear as `axis9`, `axis10`, …. `raw.buttons`
is the button bitmask exactly as received.

The worker is read-only and refuses pipeline input (it logs a warning).

## Build

```sh
cmake -S . -B build && cmake --build build --target gaz_joystick
```

`libgaz_joystick.so` lands in `build/bin`, which `load_plugin("gaz_joystick")`
searches (the executable directory). `GAZ_BUILD_JOYSTICK=OFF` drops it from the
build. The plugin is not part of the `gaz-cart` DEB yet: `cmake/Packaging.cmake`
still packages only frames, nav and slam.

## Device permissions

`/dev/hidraw*` is root-only by default. Install the udev rule once:

```sh
sudo joystick/install-udev.sh    # idempotent, reloads and re-triggers udev
```

It grants `plugdev` read access to `0483:5710`; replug the joystick afterwards.

## Prototype script

```sh
build/bin/radapter joystick/joystick.lua [/dev/hidraw5]
```

Prints one line per change and nothing while the sticks are still:

```text
x=+0.00 y=-0.42 z=-1.00 rx=+0.05 ry=+0.00 rz=-1.00 slider=-1.00 dial=-1.00 buttons=-
```

## Debug teleop

`debug/motor.lua` drives one module over its USART2 bench channel from this
plugin: `axis=y` (0.05 deadzone, rescaled) → ±`max` m/s, `slider <= -0.8` parks
the drive, `slider >= -0.5` enables it again, 0.5 s without reports holds the
wheel at zero. Run `build/bin/radapter debug/motor.lua help` for the options.

The firmware declares a single button bit (`NUM_BUTTONS 1`), and on our
transmitter that channel chatters around the 1750 threshold at ~10 Hz, so a
button is not usable as a stop control.

## Cart teleop

`nodes/joystick.lua` is the same input for the whole cart: the throttle axis
(`y`) sets the forward speed, the other axis of that stick (`x`) the turn rate,
and the stop organ (`slider <= -0.8` parks, `slider >= -0.5` releases) gates
every drive source. Reports stop arriving for 0.5 s hold the cart at zero.

Enable it by naming the device in `cart.lua`'s `JOYSTICK` tunable (nil by
default — the cart also runs on robots without the joystick):

```lua
local JOYSTICK = "/dev/hidraw5"   -- hidraw node of the URCJ; nil = no joystick
```

`mods/drive_mux.lua` arbitrates the drive sources (planner, GUI keyboard teleop,
joystick, ROS bridge): the last source that commanded a non-zero twist owns the
wheels, so the GUI's standstill stream cannot fight the stick, and centring the
stick (or releasing the keys) hands control to the next source. The GUI teleop
tab mirrors the accepted command and the source that issued it.

The handover is per command, not sticky: while another source is actively
streaming its own non-zero twist (the planner, a second GUI with keys held) the
two alternate, so drive with one source at a time.

The organ latches the whole cart parked at start-up until the stop axis reports
where it is, and while it is engaged the config tab's per-wheel speed and
direct-voltage commands are refused as well — they spin motors without going
through the arbiter.

## Not done yet

Expo curves, per-axis trimming and a config-file override of the cart tunables
are deliberately out of scope: axis inversion, the deadzone and the stop
thresholds are node defaults today.
