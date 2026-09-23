--!strict
-- NVGN.RubberBand -- straighten a grid outline without letting it leave the floor.
--
-- Cocosulx's idea. A diagonal edge on a grid is a staircase, and the real edge
-- lies somewhere in the band one cell wide between the staircase and a cell in.
-- Pull a rubber band through that band: it touches only the corners it has to
-- and runs straight between them.
--
-- Done exactly with the funnel the path tester uses. Every staircase corner
-- becomes a short gate from the corner itself (where the floor ends) to a point
-- one cell inward, and the band is pulled tight through the gates. By
-- construction the result:
--   * never leaves the band -- always within a cell of the real edge, never
--     beyond it, so never into a wall;
--   * cannot cross another ring or produce spikes and knots, which the old
--     simplify / merge / dejog / clean passes kept having to repair;
--   * keeps a real corner, because the band itself turns there.
--
-- Stretches marked `cut` (a tile border running through connected floor) are
-- kept exactly as they are, so neighbouring tiles still meet along a straight
-- line.

local RubberBand = {}

-- positive when c is LEFT of a->b, Y up (left of +X is -Z)
local function area2(a: Vector3, b: Vector3, c: Vector3): number
	return (c.X - a.X) * (b.Z - a.Z) - (b.X - a.X) * (c.Z - a.Z)
end

local function leftOf(d: Vector3): Vector3
	-- Y x d, flattened: the side the floor is on when d is travelled
	return Vector3.new(d.Z, 0, -d.X)
end

-- simple stupid funnel through gates given as { left, right }
local function funnel(gates: { { Vector3 } }): { Vector3 }
	local P = gates
	local path = { P[1][1] }
	local apex, left, right = P[1][1], P[1][1], P[1][2]
	local ai, li, ri = 1, 1, 1
	local i, guard = 2, 0
	while i <= #P and guard < 200000 do
		guard += 1
		local pl, pr = P[i][1], P[i][2]
		if area2(apex, right, pr) >= 0 then
			if apex == right or area2(apex, left, pr) < 0 then
				right = pr; ri = i
			else
				path[#path + 1] = left
				apex = left; ai = li
				left, right = apex, apex; li, ri = ai, ai
				i = ai + 1
				continue
			end
		end
		if area2(apex, left, pl) <= 0 then
			if apex == left or area2(apex, right, pl) > 0 then
				left = pl; li = i
			else
				path[#path + 1] = right
				apex = right; ai = ri
				left, right = apex, apex; li, ri = ai, ai
				i = ai + 1
				continue
			end
		end
		i += 1
	end
	local last = P[#P][1]
	if path[#path] ~= last then path[#path + 1] = last end
	return path
end

-- `pts` is a closed ring (region on the left), `kinds[i]` the kind of the run
-- that starts at pts[i]. `width` is the band width: one cell.
-- Returns the new ring and, per point, the kind of the run that starts there,
-- so the caller still knows which stretches are cuts.
function RubberBand.pull(pts: { Vector3 }, kinds: { string }, width: number): ({ Vector3 }, { string })
	local n = #pts
	if n < 4 then return pts, kinds end

	-- anchors: both ends of every cut run; with none, the extreme vertex, which
	-- any tight outline of the band must still pass through
	local anchor = {}
	local anyCut = false
	for i = 1, n do
		if kinds[i] == "cut" then
			anchor[i] = true
			anchor[i % n + 1] = true
			anyCut = true
		end
	end
	if not anyCut then
		local best = 1
		for i = 2, n do
			local p, q = pts[i], pts[best]
			if p.X < q.X - 1e-6 or (math.abs(p.X - q.X) <= 1e-6 and p.Z < q.Z) then best = i end
		end
		anchor[best] = true
	end

	-- the gate at a staircase vertex: the vertex itself (where the floor ends,
	-- on the RIGHT of travel) and a point a cell inward (on the LEFT)
	local function gate(k: number): { Vector3 }
		local p = pts[k]
		local din = (p - pts[(k - 2) % n + 1]) * Vector3.new(1, 0, 1)
		local dout = (pts[k % n + 1] - p) * Vector3.new(1, 0, 1)
		local nin = din.Magnitude > 1e-9 and leftOf(din.Unit) or Vector3.zero
		local nout = dout.Magnitude > 1e-9 and leftOf(dout.Unit) or Vector3.zero
		local inward = nin + nout
		if inward.Magnitude < 1e-6 then inward = nin end
		if inward.Magnitude < 1e-6 then return { p, p } end
		local inner = p + inward.Unit * width
		return { Vector3.new(inner.X, p.Y, inner.Z), p }
	end

	-- walk the ring from the first anchor, emitting cut runs verbatim and
	-- pulling every bound run tight between its two anchors
	local start = 1
	for i = 1, n do if anchor[i] then start = i; break end end
	local out, outKinds = {}, {}
	local i = start
	local walked = 0
	while walked < n do
		if kinds[i] == "cut" then
			out[#out + 1] = pts[i]
			outKinds[#outKinds + 1] = "cut"
			i = i % n + 1
			walked += 1
		else
			-- bound run from anchor i to the next anchor j
			local j = i % n + 1
			local steps = 1
			while not anchor[j] and steps < n do j = j % n + 1; steps += 1 end
			local gates = { { pts[i], pts[i] } }
			local k = i % n + 1
			for _ = 1, steps - 1 do
				gates[#gates + 1] = gate(k)
				k = k % n + 1
			end
			gates[#gates + 1] = { pts[j], pts[j] }
			local line = funnel(gates)
			for m = 1, #line - 1 do
				out[#out + 1] = line[m]
				outKinds[#outKinds + 1] = "bound"
			end
			i = j
			walked += steps
		end
	end
	return out, outKinds
end

return RubberBand
