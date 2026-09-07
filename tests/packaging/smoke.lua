-- No cart.lua evaluation, CAN workers, serial devices or motor commands.
-- The Lua runtime is deployed from the repository, so find mods/ and nodes/
-- relative to this file (tests/packaging/../../).
package.path = SCRIPT_DIR .. "/../../?.lua;" .. package.path

for _, plugin in ipairs { "gaz_frames", "gaz_nav", "gaz_slam" } do
    load_plugin(plugin)
end
assert(Frames and CostmapServer and GlobalPlanner and LocalPlanner and Lidar and Slam)
assert(type(require("socket").gettime()) == "number")
assert(type(require("lfs").currentdir()) == "string")
for _, name in ipairs {
    "mods.config_defs", "mods.odometry", "mods.diff_drive", "mods.rational",
} do
    assert(type(require(name)) == "table", name)
end
for _, name in ipairs { "nodes.odo", "nodes.nav", "nodes.teleop", "nodes.ros" } do
    assert(type(require(name)) == "function", name)
end
log.info("gaz-cart packaging smoke OK")
shutdown()
