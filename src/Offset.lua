--!strict
-- NVGN.Offset -- from where the floor ends to where an agent's centre may go.
--
-- Up to here the polygons describe geometry: the line where floor physically
-- stops. A pathfinder cannot use that, because an agent has width and its
-- CENTRE is what the polygon has to constrain. This pass moves each edge inward
-- so that a centre anywhere inside is a body entirely on real floor.
--
-- ONE RULE PER EDGE KIND, and the kinds come from EdgeKind:
--
--   wall  -- moved inward, GRADED by how much ground is behind it
--   drop  -- not moved. An agent may stand on the lip of a ledge, and standing
--            off from every ledge as though it were masonry cost 12.4% of the
--            old map's cells, from exactly the places worth keeping
--   seam  -- not moved, for a DIFFERENT reason than drop: a seam is not an edge
--            of the floor at all, it is where one region's plane bucket ends.
--            Moving it pulls apart two polygons meant to share an edge and
--            opens a gap down the middle of continuous ground
--   none  -- moved, as a wall
--
-- AND AN EDGE WHOSE VOTE WAS CLOSE IS MOVED AS A WALL WHATEVER IT VOTED. Both
-- fallbacks lean the same way for the same reason: offsetting something that
-- turns out to be a ledge costs a sliver of floor, which is erosion and safe,
-- while failing to offset a real wall puts an agent inside geometry. Take the
-- expensive answer over the broken one.
--
-- GRADED, NOT SWITCHED. A wall bordering a wide floor is moved the full radius;
-- one bordering a narrow ledge is moved only what the ledge can spare. A switch
-- -- full radius above some width, nothing below it -- makes two adjacent
-- polygons either side of the threshold move by different amounts and stop
-- sharing an edge, which is the one thing this pass must not do.

local Offset = {}

-- Half a 2 stud character, plus 0.2 so a centre on the boundary is never
-- exactly touching the wall. Cocosulx's number. Measured cost at 1.2 is 0.4
-- points above the bare 1.0 on case5, so the margin is nearly free.
Offset.agentRadius = 1.2

-- The half-width the offset refuses to erode below. A strip this thin keeps
-- nothing back: at 0.25, one cell of a 0.5 step survives as a line rather than
-- disappearing. Raising it deletes narrow ground instead of thinning it.
Offset.margin = 0.25

-- An edge whose kind carried less than this share of its raw nodes is moved as
-- a wall regardless of what it voted. Matches EdgeKind.mixedBelow; kept here as
-- the OFFSET's own policy, since EdgeKind only reports the number.
Offset.trustBelow = 0.8

-- Past this multiple of the offset distance, an acute corner's intersection is
-- thrown away and the two moved edges are joined straight across instead. An
-- intersection at a sharp corner runs away to infinity, and a spike is worse
-- than a bevel: it claims floor in a direction nothing was measured in.
Offset.miterLimit = 2.5

local MOVED = { wall = true, none = true }

-- Does the low-trust rule apply to an edge that voted SEAM.
--
-- The rule exists because failing to offset a real wall puts an agent inside
-- geometry. That argument does not carry to a seam: a seam means LocalGrid
-- found live floor at the same height on the far side, so there is no wall
-- there to be inside of. What moving it does instead is pull two regions apart
-- and cut the graph -- measured on case3, where eight low-trust seams severed
-- the main component into 2067 + 1709 cells.
Offset.offsetMixedSeams = false

-- How far this edge may move.
--
-- `maxD` is the LARGEST ground thickness among the cells this edge was traced
-- from, which is half the local width of the floor behind it. A wide room
-- reports a large number and gives up the full radius; a one cell ledge reports
-- 0.25 and gives up almost nothing.
--
-- Largest, not smallest. The thickness at a rim cell is always about half a
-- step -- that is what being at the rim means -- so a minimum would read every
-- edge as paper thin and move nothing anywhere.
local function reach(L: any, i: number): number
	local nodes = L.edgeNodes and L.edgeNodes[i]
	if not nodes or #nodes == 0 then return 0 end
	local maxD = 0
	for _, k in ipairs(nodes) do
		local cell = L.polyCell and L.polyCell[k]
		local t = cell and cell.thick
		if t == math.huge then return Offset.agentRadius end
		if t and t > maxD then maxD = t end
	end
	return math.clamp(maxD - Offset.margin, 0, Offset.agentRadius)
end

-- Intersect two offset lines, each given as a point and a direction.
local function cross(p1: Vector3, d1: Vector3, p2: Vector3, d2: Vector3,
	up: Vector3): Vector3?
	-- solved in the plane, against the normal, so a shared vertical component
	-- cannot make two in-plane parallel lines look like they meet
	local n2 = up:Cross(d2)
	local denom = d1:Dot(n2)
	if math.abs(denom) < 1e-6 then return nil end
	local t = (p2 - p1):Dot(n2) / denom
	return p1 + d1 * t
end

-- Offset one loop. Writes `L.offset` (the moved corners) and `L.offsetDist`.
--
-- The edges are moved FIRST and the corners recomputed from them. Moving the
-- corners directly would be wrong wherever two adjacent edges move by different
-- amounts, which after grading is most of them.
local function loopOffset(L: any, stats: any): boolean
	local pts, up = L.pts, L.regionUp or L.up
	local n = #pts
	if n < 3 or not L.closed or not L.edgeKind then return false end

	local dist = table.create(n)
	local base = table.create(n)
	local dir = table.create(n)
	local moved = 0
	for i = 1, n do
		local a = pts[i]
		local b = pts[(i % n) + 1]
		local d = b - a
		local len = d.Magnitude
		if len < 1e-6 then
			-- a zero length edge has no direction to move along; leave it put
			dist[i] = 0
			base[i] = a
			dir[i] = Vector3.xAxis
			stats.degenerate += 1
			continue
		end
		local u = d / len
		local kind = L.edgeKind[i] or "none"
		local purity = L.edgePurity and L.edgePurity[i] or 1
		local lowTrust = purity < Offset.trustBelow
		if kind == "edge" and not Offset.offsetMixedSeams then lowTrust = false end
		local asWall = MOVED[kind] or lowTrust
		local amount = 0
		if asWall then
			amount = reach(L, i)
			if amount > 0 then moved += 1 end
			if not MOVED[kind] then stats.lowTrust += 1 end
		end
		-- faces are wound region-on-the-left, so up x direction points INTO the
		-- floor. This is the same convention the boundary nodes were inset with.
		local inward = up:Cross(u)
		dist[i] = amount
		base[i] = a + inward * amount
		dir[i] = u
		stats.edges += 1
		if amount > 0 then
			stats.movedEdges += 1
			stats.moveSum += amount
			if amount > stats.moveMax then stats.moveMax = amount end
		else
			stats.heldEdges += 1
		end
	end

	local out = table.create(n)
	for i = 1, n do
		local prev = ((i - 2) % n) + 1
		-- NEITHER EDGE MOVED, SO NEITHER DOES THE CORNER. Recomputing it from
		-- the two unmoved lines returns the same point only to within float
		-- error, and a rim cell centre lies exactly ON the polygon edge, where
		-- a crossing test is a coin toss. A 1e-6 wobble there flips cells in
		-- both directions: case5 reported 424 cells the offset had "added",
		-- on a bake where 521 of 530 edges held.
		if dist[prev] == 0 and dist[i] == 0 then
			out[i] = pts[i]
			continue
		end
		local p = cross(base[prev], dir[prev], base[i], dir[i], up)
		if p then
			-- a runaway intersection at an acute corner is replaced by the
			-- moved corner itself, which bevels instead of spiking
			local limit = math.max(dist[prev], dist[i]) * Offset.miterLimit
			if limit > 0 and (p - pts[i]).Magnitude > limit then
				p = nil
				stats.mitered += 1
			end
		end
		if not p then
			-- no usable crossing: take the nearer of the two moved lines at the
			-- original corner, which is never outside either of them
			local q1 = base[prev] + dir[prev] * (pts[i] - base[prev]):Dot(dir[prev])
			local q2 = base[i] + dir[i] * (pts[i] - base[i]):Dot(dir[i])
			p = ((q1 - pts[i]).Magnitude <= (q2 - pts[i]).Magnitude) and q1 or q2
			stats.fallback += 1
		end
		out[i] = p
	end

	L.offset = out
	L.offsetDist = dist
	stats.loops += 1
	if moved > 0 then stats.loopsMoved += 1 end
	return true
end

-- Offset every closed loop. Ground thickness must already be on the cells.
function Offset.apply(loops: {any}): any
	local stats = { loops = 0, loopsMoved = 0, edges = 0, movedEdges = 0,
		heldEdges = 0, lowTrust = 0, mitered = 0, fallback = 0, degenerate = 0,
		skipped = 0, moveSum = 0, moveMax = 0 }
	for _, L in ipairs(loops) do
		if not loopOffset(L, stats) then
			L.offset = nil
			stats.skipped += 1
		end
	end
	stats.moveMean = stats.movedEdges > 0 and (stats.moveSum / stats.movedEdges) or 0
	return stats
end

function Offset.report(stats: any): string
	return ("offset    %d loops (%d moved), %d edges -- %d moved, %d held; mean %.2f max %.2f studs\n")
		:format(stats.loops, stats.loopsMoved, stats.edges, stats.movedEdges,
			stats.heldEdges, stats.moveMean, stats.moveMax)
		.. ("  %d moved on low trust, %d corners mitered, %d fell back, %d loops skipped")
			:format(stats.lowTrust, stats.mitered, stats.fallback, stats.skipped)
end

-- A test for the severance check: did this cell survive the offset.
--
-- A cell survives when its centre is inside its region's offset outer rim and
-- outside every offset hole in it. Handing this to Severance is what turns the
-- offset from "the lines moved" into "and here is what it cost", which is the
-- only way a closed corridor is ever seen.
--
-- Loops the offset could not move fall back to their original corners rather
-- than dropping out, so a skipped loop reads as "nothing lost here" instead of
-- silently deleting a region.
--
-- `useOriginal` tests against the UNMOVED polygon instead, and the severance
-- baseline must be taken that way. A polygon already excludes about 11% of the
-- cells it was traced from before anything moves: boundary nodes sit on the
-- outermost cell centres, so those centres lie exactly ON the edge where a
-- crossing test is a coin toss, and simplification then moves the line up to a
-- stud. Comparing raw cells against offset-contained cells charges all of that
-- to the offset and reports a catastrophe on a bake that moved 11 edges. Taking
-- both snapshots through the same test cancels it, and what is left is the
-- offset's own cost.
function Offset.keepTest(loops: {any}, useOriginal: boolean?): (any) -> boolean
	local byRegion: { [any]: any } = {}
	for _, L in ipairs(loops) do
		local pts = (useOriginal and L.pts) or L.offset or L.pts
		if not pts or #pts < 3 or not L.closed then continue end
		local r = L.region
		local e = byRegion[r]
		if not e then
			local up = L.regionUp or L.up
			local ax, ay, az = math.abs(up.X), math.abs(up.Y), math.abs(up.Z)
			local seed = (ax <= ay and ax <= az) and Vector3.xAxis
				or (ay <= az and Vector3.yAxis or Vector3.zAxis)
			local e1 = seed:Cross(up)
			if e1.Magnitude < 1e-6 then
				e1 = (math.abs(up.Y) < 0.9 and Vector3.yAxis or Vector3.xAxis):Cross(up)
			end
			e1 = e1.Unit
			e = { e1 = e1, e2 = up:Cross(e1), outer = {}, holes = {} }
			byRegion[r] = e
		end
		local flat = table.create(#pts)
		for i, q in ipairs(pts) do
			flat[i] = Vector2.new(q:Dot(e.e1), q:Dot(e.e2))
		end
		if L.kind == "hole" then
			e.holes[#e.holes + 1] = flat
		else
			e.outer[#e.outer + 1] = flat
		end
	end

	local function inside(poly: {Vector2}, x: number, y: number): boolean
		local n = #poly
		local hit = false
		local a = poly[n]
		for i = 1, n do
			local b = poly[i]
			if (a.Y > y) ~= (b.Y > y) then
				local t = (y - a.Y) / (b.Y - a.Y)
				if x < a.X + t * (b.X - a.X) then hit = not hit end
			end
			a = b
		end
		return hit
	end

	return function(cell: any): boolean
		local e = byRegion[cell.region]
		-- no offset polygon for this region at all: nothing was moved, so
		-- nothing was lost. Refusing the cell here would report a severance the
		-- offset did not cause.
		if not e then return true end
		local x, y = cell.pos:Dot(e.e1), cell.pos:Dot(e.e2)
		local ok = false
		for _, poly in ipairs(e.outer) do
			if inside(poly, x, y) then ok = true; break end
		end
		if not ok then return false end
		for _, poly in ipairs(e.holes) do
			if inside(poly, x, y) then return false end
		end
		return true
	end
end

return Offset
