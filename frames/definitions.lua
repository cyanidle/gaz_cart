---@meta gaz_frames
-- Lua LSP definitions for the native gaz_frames plugin (libgaz_frames).

---A pose of a child frame/object expressed in its parent/source frame.
---@class FramePose
---@field x number metres along +x
---@field y number metres along +y
---@field theta number radians, counter-clockwise

---@class FramesSetCommand
---@field parent string frame coordinates are transformed into
---@field child string frame coordinates are transformed from; has one parent
---@field transform FramePose parent <- child

---@class FramesInput
---@field set FramesSetCommand

---@class FramesConfig : WorkerConfig

---@class Frames : Worker<FramesInput, any>
---@field set fun(self: Frames, parent: string, child: string, transform: FramePose) stores `parent <- child`; replaces the child's previous edge
---@field lookup fun(self: Frames, target: string, source: string): FramePose? returns `target <- source`, or nil for separate trees
---@field transform_pose fun(self: Frames, target: string, source: string, pose: FramePose): FramePose converts a pose from source coordinates to target coordinates
---@field reanchor fun(self: Frames, parent: string, source: string, source_pose: FramePose, desired_pose: FramePose) changes parent <- source so source_pose appears as desired_pose
---@field transform_odometry fun(self: Frames, odometry: CartOdometry, target_frame: string): CartOdometry converts pose/covariance to target_frame; leaves body twist unchanged

---@param cfg FramesConfig
---@return Frames
function Frames(cfg) end
