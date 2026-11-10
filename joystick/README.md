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

## Not done yet

Deadzone, expo, axis inversion, throttle/steer mapping and the cart-side
teleop worker are deliberately out of scope for the prototype.
