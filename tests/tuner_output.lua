-- Self-checking test of the diagnostics scrollback the tuner keeps for its
-- second window: the module's own `#` lines (counters, trace dump, boot) are
-- ring-buffered, the oldest are dropped past the limit, and the view is not
-- re-sent once per line while a dump is arriving.
--
--     build/bin/radapter tests/tuner_output.lua
--
-- No transport and no window: the ring and the push throttle are plain Lua on a
-- tuner object, so this runs on the host with nothing attached.

package.path = SCRIPT_DIR .. "/../?.lua;" .. package.path

local Tuner = require "mods.tuner"

local pushes = {}
local sink = setmetatable({}, {
    __call = function(_, value) pushes[#pushes + 1] = value.text or "" end,
})

---A tuner with no transport: only what the scrollback touches.
local function bare_tuner()
    return setmetatable({ output = {}, diag_view = sink }, Tuner)
end

-- ---- the ring keeps the newest lines, the oldest fall off ---------------------
local tuner = bare_tuner()
for i = 1, 500 do tuner:_output_line(("# line %d"):format(i)) end
assert(#tuner.output == 400, ("the scrollback kept %d lines, expected 400"):format(#tuner.output))
assert(tuner.output[1] == "# line 101", "the oldest line survived: " .. tostring(tuner.output[1]))
assert(tuner.output[#tuner.output] == "# line 500", "the newest line is missing")

-- ---- a dump does not re-send the whole ring per line -------------------------
sleep(300)  -- let the throttled push land
assert(#pushes > 0, "the scrollback never reached the view")
assert(#pushes < 50, ("%d pushes for 500 lines: the throttle is not working"):format(#pushes))
local last = pushes[#pushes]
assert(last:find("# line 500", 1, true), "the last push does not carry the newest line")
assert(not last:find("# line 100\n", 1, true), "the last push still carries a dropped line")

-- ---- one more line after the quiet period pushes again ----------------------
local before = #pushes
tuner:_output_line("# line 501")
sleep(300)
assert(#pushes > before, "a line after the quiet period was never pushed")
assert(pushes[#pushes]:find("# line 501", 1, true), "the new line is missing from the push")
assert(#tuner.output == 400, "the ring grew past its limit")

print(("tuner_output: ok (%d lines kept, %d pushes for 500 lines)"):format(#tuner.output, #pushes))
os.exit(0)
