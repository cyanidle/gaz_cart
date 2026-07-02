-- Self-checking smoke test for the gaz_nav plugin.
--   radapter/build/bin/radapter nav/test_nav.lua [plugin_build_dir]
-- Exits 0 on success.

load_plugin((args[1] or SCRIPT_DIR .. "/build") .. "/libgaz_nav")

local costmap = CostmapServer {
    update_rate_ms = 20,
    width = 101, height = 151, resolution = 0.02,
    inflate = { robot_safe_radius = 0.1 },
}
local gp = GlobalPlanner { update_rate_ms = 20 }
local lp = LocalPlanner { tick_rate = 25 }

pipe(costmap, gp)
pipe(costmap, lp)
pipe(gp, lp)
pipe(lp, gp)

local got_costmap, got_path, got_cmd = false, false, false
local costmap_bytes

on(costmap, "costmap", function(bytes)
    assert(type(bytes) == "userdata", "costmap must be immutable bytes")
    assert(#bytes == 16 + 101 * 151, "unexpected costmap size: " .. #bytes)
    local ok = pcall(function() bytes[1] = 0 end)
    assert(not ok, "costmap bytes must be immutable")
    costmap_bytes = bytes
    got_costmap = true
end)

on(gp, "path", function(path)
    if #path > 0 then
        got_path = true
        local last = path[#path]
        assert(math.abs(last.x - 1.0) < 0.1 and math.abs(last.y - 1.0) < 0.1,
            ("path must end near the target, ends at %g,%g"):format(last.x, last.y))
    end
end)

on(lp, "cmd_vel", function(cmd)
    if cmd.x ~= 0 or cmd.y ~= 0 or cmd.theta ~= 0 then got_cmd = true end
end)

-- an obstacle wall with a gap, so A* actually has to steer
for y = 0, 100 do
    if y < 40 or y > 60 then
        costmap { point = { x = 1.0, y = y * 0.02 } }
    end
end

local pos = { x = 0.2, y = 0.2, theta = 0 }
gp { position = pos }
lp { position = pos }
gp { target = { x = 1.0, y = 1.0, theta = 0.5 } }

after(1500, function()
    assert(got_costmap, "no costmap emitted")
    assert(got_path, "no path planned")
    assert(got_cmd, "no cmd_vel emitted")

    -- Reload must accept the same param table shape as the constructor
    costmap:Reload { width = 101, height = 151, resolution = 0.02 }
    gp:Reload { update_rate_ms = 30 }
    lp:Reload { tick_rate = 10 }

    -- a consumer must be able to reinterpret the emitted buffer: header check
    assert(costmap_bytes:byte(1) == 0x50 and costmap_bytes:byte(4) == 0x47,
        "bad magic in costmap header")

    log.info("nav smoke test OK")
    shutdown()
end)
