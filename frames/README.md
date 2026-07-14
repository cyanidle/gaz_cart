# Frames: a small 2-D tf-style tree

`Frames` is a native radapter worker that owns named 2-D reference frames. It
answers the question: **“How do coordinates expressed in frame A relate to
frame B?”**

The cart normally uses this tree:

```text
map -> odom -> base_link
```

- `odom -> base_link` is continuously produced by wheel odometry.
- `map -> odom` is owned by localization or by an operator “set pose” action.
- `base_link` is the robot body: +x forward, +y left, positive yaw is
  counter-clockwise. Positions are metres and angles are radians.

## The important convention

```lua
frames:set(parent, child, transform)
```

stores a transform from `child` **into** `parent`, conventionally written
`parent <- child`. It tells Frames where the child-frame origin is when
described in the parent frame.

For example, this says that the robot (`base_link`) is 3 metres ahead of the
odometry origin:

```lua
frames:set("odom", "base_link", { x = 3, y = 0, theta = 0 })
```

Every child has one parent. Calling `set` again for the same child replaces its
previous parent/transform. Cycles are rejected when looked up.

## Creating it

```lua
load_plugin(SCRIPT_DIR .. "/build/frames/libgaz_frames")

local frames = Frames { name = "frames" }
```

`cart.lua` already creates one shared worker and passes it to odometry and
navigation. Normally, application nodes receive that worker in `cfg.frames`
rather than creating another one.

## `set(parent, child, transform)`

Add or update one edge. This is the lowest-level operation.

```lua
-- Start with map and odom coincident.
frames:set("map", "odom", { x = 0, y = 0, theta = 0 })

-- Wheel odometry says base_link is at x=1.2 m, y=0.4 m, yaw=0.1 rad in odom.
frames:set("odom", "base_link", { x = 1.2, y = 0.4, theta = 0.1 })
```

The result is that `map` and `odom` currently agree, so looking up either pose
gives the same coordinates. If SLAM later finds that the odometry origin is
wrong, it should change only `map -> odom`; it must not reset wheel odometry.

## `lookup(target, source)`

Return the transform `target <- source`, or `nil` when the frames are in
separate trees.

```lua
frames:set("map", "odom", { x = 2, y = 1, theta = math.pi / 2 })
frames:set("odom", "base_link", { x = 3, y = 0, theta = 0 })

local in_map = assert(frames:lookup("map", "base_link"))
-- in_map is approximately { x = 2, y = 4, theta = pi/2 }
-- The robot's +3 m odom-x direction points along map +y after the 90° turn.
```

This does not modify the tree.

## `transform_pose(target, source, pose)`

Convert one pose from `source` coordinates to `target` coordinates.

```lua
-- A lidar hit / object pose described in base_link coordinates.
local object_in_base = { x = 0.8, y = 0.2, theta = 0 }
local object_in_map = frames:transform_pose("map", "base_link", object_in_base)
```

Use `lookup` when you need the transform itself; use `transform_pose` when you
already have a pose to convert.

## `transform_odometry(odometry, target_frame)`

Convert the *pose portion* of a ROS-Odometry-like message. It requires
`odometry.frame_id`; it changes that field to `target_frame`, rotates the pose
covariance, and leaves the body-frame twist alone.

```lua
local wheel_odom = {
    timestamp = socket.gettime(),
    frame_id = "odom",
    child_frame_id = "base_link",
    pose = { x = 1.2, y = 0.4, theta = 0.1 },
    twist = { linear = 0.3, angular = 0.0 },
    pose_covariance = { xx = 0.01, xy = 0, xtheta = 0,
                        yy = 0.01, ytheta = 0, thetatheta = 0.02 },
    twist_covariance = { linear = 0.01, linear_angular = 0, angular = 0.02 },
}

local map_odom = frames:transform_odometry(wheel_odom, "map")
-- map_odom.frame_id == "map"
-- map_odom.child_frame_id == "base_link"  -- unchanged
-- map_odom.twist is unchanged: it is measured in the robot body frame.
```

This is what `nodes/nav.lua` uses before it passes wheel odometry to SLAM and
the planners.

## `reanchor(parent, source, source_pose, desired_pose)`

This is the safe “set robot pose” operation. It computes and stores the parent
to source transform that makes `source_pose` appear as `desired_pose`, without
changing the producer of `source_pose`.

```lua
-- Wheel odometry currently says the robot is here in odom.
local raw = { x = 1.2, y = 0.4, theta = 0.1 }

-- The operator says that same physical robot pose is actually here in map.
local desired = { x = 10, y = 5, theta = math.pi / 2 }

frames:reanchor("map", "odom", raw, desired)

-- The raw wheel pose now transforms exactly to desired.
local check = frames:transform_pose("map", "odom", raw)
```

As wheel odometry keeps changing `odom -> base_link`, the new `map -> odom`
transform keeps the correction applied. This is why navigation no longer needs
its own `poseOffset` math.

## Pipeline input

The worker may also be called through a radapter pipe. It accepts only this
message shape and produces no data message:

```lua
frames {
    set = {
        parent = "base_link",
        child = "lidar",
        transform = { x = 0.18, y = 0.0, theta = 0.0 },
    },
}
```

This is useful for static sensor mounting transforms. For reads and conversion,
use the methods above.

## Current scope

Frames is deliberately 2-D: `(x, y, theta)` only. It has no timestamps,
interpolation, or 3-D roll/pitch/z handling. A lookup across disconnected trees
returns `nil`; conversion methods raise a Lua error because they cannot produce
a meaningful result.
