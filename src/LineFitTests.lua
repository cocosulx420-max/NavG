--!strict
-- Tests for NavGen.LineFit. Every case is a literal cell array; each isolates
-- one behaviour of the two break rules.
--
-- ON BUILDING THE CHAINS. The cases are specified as treads — "y=0 for x=0..24,
-- y=1 for x=25..30". Written out literally that is 8-CONNECTED: nothing joins
-- (24,0) to (25,1). The stage's contract promises 4-connected input, so a RISER
-- cell is inserted at each tread change, at the last x of the lower tread and
-- the y of the upper one: ... (24,0), (24,1), (25,1) ...
--
-- This matters for the quoted deviation figures. Those were computed on the
-- 8-connected list and shift slightly once the riser cell exists. The SEGMENTATION
-- is what this stage owes the caller, so the assertions are on the vertices; the
-- deviations are reported alongside rather than asserted to a stale value.
--
-- Case I is the exception and is used exactly as written, diagonal steps and
-- all, because its quoted deviations are load-bearing to what it proves.

local LineFit = require(script.Parent.LineFit)

local T = {}

type Cell = LineFit.Cell

local function cell(x: number, y: number): Cell return { x = x, z = y } end

-- Treads to a 4-connected chain, inserting risers as described above.
local function chain(treads: { { y: number, x0: number, x1: number } }): { Cell }
	local out: { Cell } = {}
	local cx, cy = nil, nil
	for _, t in ipairs(treads) do
		if cy == nil then
			cx, cy = t.x0, t.y
			table.insert(out, cell(cx :: number, cy :: number))
		else
			-- riser: climb in unit steps at the current x
			local stepY = (t.y > (cy :: number)) and 1 or -1
			while cy ~= t.y do
				cy = (cy :: number) + stepY
				table.insert(out, cell(cx :: number, cy :: number))
			end
		end
		while (cx :: number) < t.x1 do
			cx = (cx :: number) + 1
			table.insert(out, cell(cx :: number, cy :: number))
		end
	end
	return out
end

local function xs(cells: { Cell }, idx: { number }): { number }
	local o = {}
	for _, i in ipairs(idx) do table.insert(o, cells[i].x) end
	return o
end

local function fmt(cells: { Cell }, idx: { number }): string
	local parts = {}
	for _, i in ipairs(idx) do
		table.insert(parts, string.format("(%d,%d)", cells[i].x, cells[i].z))
	end
	return table.concat(parts, " ")
end

-- Peak perpendicular deviation of the run start..k from the chord start->k.
-- Reporting only; never in the decision path.
local function peakDev(cells: { Cell }, a: number, k: number): number
	local p, q = cells[a], cells[k]
	local dx, dz = q.x - p.x, q.z - p.z
	local dd = dx * dx + dz * dz
	if dd == 0 then return 0 end
	local best = 0
	for i = a + 1, k - 1 do
		local r = cells[i]
		local cr = math.abs(dx * (r.z - p.z) - dz * (r.x - p.x))
		local d = cr / math.sqrt(dd)
		if d > best then best = d end
	end
	return best
end

local results = {}
local function check(name: string, ok: boolean, detail: string)
	table.insert(results, { name = name, ok = ok, detail = detail })
end

--------------------------------------------------------------------------

function T.run(): string
	results = {}

	-- A — flat run into a staircase (rollback case).
	do
		local c = chain({
			{ y = 0, x0 = 0,  x1 = 24 },
			{ y = 1, x0 = 25, x1 = 30 },
			{ y = 2, x0 = 31, x1 = 36 },
			{ y = 3, x0 = 37, x1 = 42 },
		})
		local r = LineFit.fit(c, { closed = false })
		local v = xs(c, r.vertices)
		-- The 25-cell flat run must end with zero verticality: the first vertex
		-- after the start sits at x=24, the last cell of the tread.
		check("A vertex at x=24", v[2] == 24,
			string.format("vertices %s", fmt(c, r.vertices)))
		-- Report the deviations the doc quotes as ~0.8 / ~1.5.
		local acc, rej = 0, 0
		for i, cc in ipairs(c) do
			if cc.x == 30 and cc.z == 1 then acc = peakDev(c, 1, i) end
			if cc.x == 30 and cc.z == 2 then rej = peakDev(c, 1, i) end
		end
		check("A deviation straddles tolerance 1", acc < 1 and rej > 1,
			string.format("last accepted %.3f, first rejected %.3f", acc, rej))
	end

	-- B — uniform staircase (accept case): exactly one segment.
	do
		local c = chain({
			{ y = 0, x0 = 0,  x1 = 4 },
			{ y = 1, x0 = 5,  x1 = 9 },
			{ y = 2, x0 = 10, x1 = 14 },
		})
		local r = LineFit.fit(c, { closed = false })
		check("B one segment", #r.vertices == 2,
			string.format("vertices %s, peak dev %.3f", fmt(c, r.vertices), peakDev(c, 1, #c)))
	end

	-- C — direction flip (hard break, no rollback).
	do
		local c = chain({
			{ y = 0, x0 = 0,  x1 = 4 },
			{ y = 1, x0 = 5,  x1 = 9 },
			{ y = 2, x0 = 10, x1 = 14 },
			{ y = 1, x0 = 15, x1 = 19 },
		})
		local r = LineFit.fit(c, { closed = false })
		local v = xs(c, r.vertices)
		-- The break must land on the last cell of the ascending staircase, with
		-- the staircase intact. Emphatically NOT rolled back into it.
		local ok = v[2] == 14 and c[r.vertices[2]].z == 2
		check("C breaks at the flip, no rollback", ok,
			string.format("vertices %s, reasons %s", fmt(c, r.vertices), table.concat(r.reasons, ",")))
	end

	-- D — tread length change: treads of 5 x4, then a flat run of 20.
	do
		local c = chain({
			{ y = 0, x0 = 0,  x1 = 4 },
			{ y = 1, x0 = 5,  x1 = 9 },
			{ y = 2, x0 = 10, x1 = 14 },
			{ y = 3, x0 = 15, x1 = 19 },
			{ y = 4, x0 = 20, x1 = 39 },
		})
		local r = LineFit.fit(c, { closed = false })
		check("D splits staircase from flat run", #r.vertices == 3,
			string.format("vertices %s", fmt(c, r.vertices)))
	end

	-- E — degenerate inputs: none may crash or emit an empty segment.
	do
		local cases = {
			{ cell(0, 0) },
			{ cell(0, 0), cell(1, 0) },
			chain({ { y = 0, x0 = 0, x1 = 30 } }),
			chain({
				{ y = 0, x0 = 0, x1 = 0 }, { y = 1, x0 = 1, x1 = 1 },
				{ y = 2, x0 = 2, x1 = 2 }, { y = 3, x0 = 3, x1 = 3 },
				{ y = 4, x0 = 4, x1 = 4 }, { y = 5, x0 = 5, x1 = 5 },
			}),
		}
		local ok, detail = true, {}
		for i, c in ipairs(cases) do
			local good, res = pcall(LineFit.fit, c, { closed = false })
			if not good then
				ok = false
				table.insert(detail, string.format("case %d errored: %s", i, tostring(res)))
			else
				table.insert(detail, string.format("case %d -> %d vertices", i, #(res :: any).vertices))
				if #(res :: any).vertices == 0 then ok = false end
			end
		end
		check("E degenerate inputs survive", ok, table.concat(detail, "; "))
	end

	-- G — mid-run and on-corner seeds on an open chain.
	do
		local c = chain({
			{ y = 0, x0 = 0,  x1 = 24 },
			{ y = 1, x0 = 25, x1 = 30 },
			{ y = 2, x0 = 31, x1 = 36 },
			{ y = 3, x0 = 37, x1 = 42 },
		})
		local base = fmt(c, LineFit.fit(c, { closed = false, seed = 1 }).vertices)
		local mid  = fmt(c, LineFit.fit(c, { closed = false, seed = 13 }).vertices)
		local onc  = fmt(c, LineFit.fit(c, { closed = false, seed = 25 }).vertices)
		check("G seed does not move the vertices", base == mid and base == onc,
			string.format("seed1 [%s] seed13 [%s] seed25 [%s]", base, mid, onc))
	end

	-- I — seed two cells from a corner, adjacent to a steeper run.
	-- Used verbatim from the spec, in backward-travel coordinates from the seed.
	-- This is the adversarial case: the backward walk has only 2 cells of
	-- evidence before it crosses the corner, so its corridor is still wide and
	-- it absorbs the corner without breaking. It finally breaks at (5,3), and
	-- max-deviation rollback is what puts the anchor back on the true corner.
	do
		local c = { cell(0,0), cell(1,0), cell(2,0), cell(3,1), cell(4,2), cell(5,3), cell(6,4) }
		local r = LineFit._growRun(c, 1, 1, { tolNum = 1, tolDen = 1, closed = false })
		local hit = c[r.endIdx]
		check("I anchors on the true corner (2,0)", hit.x == 2 and hit.z == 0,
			string.format("anchor (%d,%d) reason %s", hit.x, hit.z, r.reason))
		check("I does NOT land on (4,2)", not (hit.x == 4 and hit.z == 2),
			string.format("anchor (%d,%d)", hit.x, hit.z))
		local d3, d4, d5 = peakDev(c, 1, 4), peakDev(c, 1, 5), peakDev(c, 1, 6)
		check("I deviations match the spec", math.abs(d3 - 0.632) < 0.01
			and math.abs(d4 - 0.894) < 0.01 and math.abs(d5 - 1.029) < 0.01,
			string.format("(3,1) %.3f  (4,2) %.3f  (5,3) %.3f", d3, d4, d5))
	end

	-- F — seed independence on a closed loop.
	-- Case D's profile, closed by dropping to y=-3, running back along the
	-- bottom, and climbing to the start. Every cell is distinct.
	do
		local c = chain({
			{ y = 0, x0 = 0,  x1 = 4 },
			{ y = 1, x0 = 5,  x1 = 9 },
			{ y = 2, x0 = 10, x1 = 14 },
			{ y = 3, x0 = 15, x1 = 19 },
			{ y = 4, x0 = 20, x1 = 39 },
		})
		for y = 3, -3, -1 do table.insert(c, cell(39, y)) end
		for x = 38, 0, -1 do table.insert(c, cell(x, -3)) end
		for y = -2, -1 do table.insert(c, cell(0, y)) end

		local function vertexSet(res): string
			local o = {}
			for _, i in ipairs(res.vertices) do
				table.insert(o, string.format("%d,%d", c[i].x, c[i].z))
			end
			table.sort(o)
			return table.concat(o, " | ")
		end

		local base = vertexSet(LineFit.fit(c, { closed = true, seed = 1 }))
		local bad, badSeed = 0, nil
		for s = 1, #c do
			local ok, res = pcall(LineFit.fit, c, { closed = true, seed = s })
			if not ok or vertexSet(res) ~= base then
				bad += 1
				if badSeed == nil then badSeed = s end
			end
		end
		check("F seed independence over all " .. #c .. " seeds", bad == 0,
			string.format("%d seeds disagreed (first %s); base set: %s",
				bad, tostring(badSeed), base))
	end

	-- H — determinism.
	do
		local c = chain({
			{ y = 0, x0 = 0,  x1 = 24 },
			{ y = 1, x0 = 25, x1 = 30 },
			{ y = 2, x0 = 31, x1 = 36 },
		})
		local a = fmt(c, LineFit.fit(c, { closed = false, seed = 7 }).vertices)
		local b = fmt(c, LineFit.fit(c, { closed = false, seed = 7 }).vertices)
		check("H determinism", a == b, a)
	end

	--------------------------------------------------------------------------

	local out, pass, fail = {}, 0, 0
	for _, r in ipairs(results) do
		if r.ok then pass += 1 else fail += 1 end
		table.insert(out, string.format("%s  %-42s  %s", r.ok and "PASS" or "FAIL", r.name, r.detail))
	end
	table.insert(out, string.format("-- %d passed, %d failed", pass, fail))
	return table.concat(out, "\n")
end

return T
