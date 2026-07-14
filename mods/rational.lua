-- Numeric transport helpers. Kept out of cart setup so Cyphal payload details
-- do not obscure the node wiring.

local Rational = {}

local function gcd(a, b)
    a, b = math.abs(a), math.abs(b)
    while b > 0.5 do a, b = b, a % b end
    return a
end

---Approximate a number as an Integer32 numerator/denominator pair.
---@param value number
---@param precision integer? denominator before reduction; default 1,000,000
---@return integer numerator
---@return integer denominator
function Rational.from_number(value, precision)
    local denominator = precision or 1000000
    local numerator = math.floor(value * denominator + (value >= 0 and 0.5 or -0.5))
    local divisor = math.max(1, gcd(numerator, denominator))
    return math.floor(numerator / divisor), math.floor(denominator / divisor)
end

return Rational
