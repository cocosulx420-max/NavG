--!strict
-- NVGN.Rings -- which loop is a surface and which is a hole punched in one.
--
-- Boundary.trace hands back a flat list of closed loops per region and says
-- nothing about how they relate. A room and the pillar standing in the middle
-- of it come out as two rings of the same kind. Everything downstream that
-- wants AREA rather than OUTLINE -- the offset, and triangulation after it --
-- has to know which is which, or it fills the pillar in with floor.
--
-- THE ANSWER IS ALREADY IN THE WINDING, so this module does not have to invent
-- it. Boundary.faces emits every face wound region-on-the-LEFT (Boundary:128).
-- Walk a loop with the floor on your left and an outer rim goes counter-
-- clockwise about the surface normal while a hole goes clockwise, because the
-- floor is on the far side of it. So the sign of the enclosed area IS the
-- classification, exactly, with no tolerance and no nesting search.
--
-- Containment is then computed anyway, and disagreements are REPORTED rather
-- than repaired. The sign is a local fact about one ring; containment is a
-- global one about a pair. When a trace defect makes them disagree, that is a
-- defect in the trace, and silently trusting whichever answer looks nicer is
-- how it would stay hidden. Same rule as the severance check.

local Rings = {}

-- How far inside its own edge a ring's containment probe sits. A vertex lies ON
-- the boundary, where a crossing test is a coin toss, so the probe steps in off
-- one. Small against a 0.5 stud cell and large against float noise.
Rings.probeInset = 0.01

-- Rings enclosing less than this are reported rather than classified. A sign
-- needs an area to have a sign, and what this catches is the known small-region
-- collapse: a region of a few border cells yields two coincident edges
-- enclosing nothing.
Rings.minArea = 1e-4

export type Kind = "outer" | "hole" | "open" | "degenerate"

-- A right-handed in-plane basis, chosen deterministically from `up` alone. The
-- seed axis is the world axis LEAST aligned with up, so the cross product never
-- collapses on a floor that happens to face straight down a world axis.
local function basis(up: Vector3): (Vector3, Vector3)
	local ax, ay, az = math.abs(up.X), math.abs(up.Y), math.abs(up.Z)
	local seed = (ax <= ay and ax <= az) and Vector3.xAxis
		or (ay <= az and Vector3.yAxis or Vector3.zAxis)
	local e1 = seed:Cross(up)
	local m = e1.Magnitude
	if m < 1e-6 then
		e1 = (math.abs(up.Y) < 0.9 and Vector3.yAxis or Vector3.xAxis):Cross(up)
		m = e1.Magnitude
	end
	e1 = e1 / m
	return e1, up:Cross(e1)
end

-- Exported so Triangulate measures in the SAME frame Rings classified in. Two
-- copies of this could disagree on handedness, and a flipped basis silently
-- turns every outer rim into a hole.
Rings.basis = basis

-- Shoelace, in the plane of `up`. POSITIVE means counter-clockwise about up,
-- which by the region-on-the-left winding means an OUTER rim.
--
-- Summed about the ring's first vertex rather than the world origin. A region a
-- thousand studs out would otherwise subtract two huge numbers to recover a
-- small one, and the sign of a narrow ring is exactly what gets lost there.
local function signedArea(pts: {Vector3}, e1: Vector3, e2: Vector3): number
	local n = #pts
	if n < 3 then return 0 end
	local o = pts[1]
	local sum = 0
	local px, py = 0, 0
	for i = 2, n do
		local d = pts[i] - o
		local qx, qy = d:Dot(e1), d:Dot(e2)
		sum += px * qy - qx * py
		px, py = qx, qy
	end
	-- The closing term runs back to the origin vertex, which is (0,0) and
	-- contributes nothing, so it is left out rather than computed as zero.
	return sum * 0.5
end

-- A point just INSIDE the ring, for the containment test.
--
-- Taken at the midpoint of the first real edge and stepped to the LEFT of
-- travel, which is the same invariant the winding classification rests on. For
-- an outer rim, left is onto the floor. For a hole, left is also onto the
-- floor, which is OUTSIDE the hole but INSIDE the rim that owns it, and that is
-- the question containment is being asked.
local function probe(pts: {Vector3}, up: Vector3, inset: number): Vector3?
	local n = #pts
	for i = 1, n do
		local a = pts[i]
		local b = pts[(i % n) + 1]
		local d = b - a
		if d.Magnitude > 1e-6 then
			return (a + b) * 0.5 + up:Cross(d.Unit) * inset
		end
	end
	return nil
end

-- Crossing-number test in the plane of `up`. Counts the edges that straddle the
-- probe's own horizontal, with a half-open rule on the vertical span so a
-- vertex sitting exactly on that horizontal is counted once rather than twice
-- or not at all.
local function contains(pts: {Vector3}, e1: Vector3, e2: Vector3, q: Vector3): boolean
	local n = #pts
	if n < 3 then return false end
	local qx, qy = q:Dot(e1), q:Dot(e2)
	local inside = false
	local ax, ay = pts[n]:Dot(e1), pts[n]:Dot(e2)
	for i = 1, n do
		local bx, by = pts[i]:Dot(e1), pts[i]:Dot(e2)
		if (ay > qy) ~= (by > qy) then
			local t = (qy - ay) / (by - ay)
			if qx < ax + t * (bx - ax) then inside = not inside end
		end
		ax, ay = bx, by
	end
	return inside
end

-- Classify every loop, in place.
--
-- Adds to each loop: `kind`, `area` (signed, in the REGION's basis), `parent`
-- (the index in `loops` of the ring a hole sits in, or nil) and `depth`.
-- Returns statistics and a list of complaints.
--
-- ONE BASIS PER REGION, not per loop. Signs are only comparable when they are
-- measured about the same normal, and a region is planar to within
-- regionPlanarity so its loops genuinely share one. The region's normal is
-- taken from its LARGEST ring, which is the one whose own normal is best
-- supported by the cells underneath it.
function Rings.classify(loops: {any}): any
	local stats = { outer = 0, hole = 0, open = 0, degenerate = 0, regions = 0 }
	local complaints = {}

	-- group by region, keeping the order the loops arrived in
	local order, byRegion = {}, {}
	for i, L in ipairs(loops) do
		local r = L.region
		if not byRegion[r] then
			byRegion[r] = {}
			order[#order + 1] = r
		end
		local g = byRegion[r]
		g[#g + 1] = i
	end

	for _, r in ipairs(order) do
		stats.regions += 1
		local idxs = byRegion[r]

		-- The region's normal: the up of its largest ring, measured in that
		-- ring's own frame because there is no region basis yet to measure in.
		local regionUp, best = nil, -1
		for _, i in ipairs(idxs) do
			local L = loops[i]
			if L.closed and #L.pts >= 3 then
				local u1, u2 = basis(L.up)
				local a = math.abs(signedArea(L.pts, u1, u2))
				if a > best then
					best = a
					regionUp = L.up
				end
			end
		end
		if not regionUp then regionUp = loops[idxs[1]].up end

		local e1, e2 = basis(regionUp)
		local outers, holes = {}, {}

		for _, i in ipairs(idxs) do
			local L = loops[i]
			L.regionUp = regionUp
			L.parent = nil
			L.depth = 0
			if not L.closed then
				-- AN OPEN PATH ENCLOSES NOTHING. No area, so no sign, so no
				-- answer here. Labelled rather than guessed at: a path silently
				-- called an outer rim is a room invented out of a wall the
				-- trace failed to close.
				L.kind = "open" :: Kind
				L.area = 0
				stats.open += 1
			elseif #L.pts < 3 then
				L.kind = "degenerate" :: Kind
				L.area = 0
				stats.degenerate += 1
			else
				local a = signedArea(L.pts, e1, e2)
				L.area = a
				if math.abs(a) < Rings.minArea then
					L.kind = "degenerate" :: Kind
					stats.degenerate += 1
				elseif a > 0 then
					L.kind = "outer" :: Kind
					outers[#outers + 1] = i
					stats.outer += 1
				else
					L.kind = "hole" :: Kind
					holes[#holes + 1] = i
					stats.hole += 1
				end
				-- A ring's own normal has to agree with the region's, or the
				-- sign just computed was measured about the wrong axis.
				local d = math.clamp(L.up:Dot(regionUp), -1, 1)
				if d < 0.87 then
					complaints[#complaints + 1] =
						("r%03d loop%d: normal %.0f deg off the region's")
							:format(r, L.index, math.deg(math.acos(d)))
				end
			end
		end

		-- A REGION IS ONE CONNECTED SURFACE, so it has exactly one outer rim.
		-- Anything else is a trace defect, and it gets said out loud.
		if #outers == 0 and #holes > 0 then
			complaints[#complaints + 1] =
				("r%03d: %d holes and no outer rim"):format(r, #holes)
		elseif #outers > 1 then
			complaints[#complaints + 1] =
				("r%03d: %d outer rims, expected 1"):format(r, #outers)
		end

		-- Parent each hole by containment, and cross-check the winding against
		-- it. The smallest containing rim wins, so this still behaves if a
		-- region ever does come back with more than one.
		for _, i in ipairs(holes) do
			local L = loops[i]
			local q = probe(L.pts, regionUp, Rings.probeInset)
			local parent, parentArea = nil, math.huge
			if q then
				for _, j in ipairs(outers) do
					local P = loops[j]
					if contains(P.pts, e1, e2, q) and math.abs(P.area) < parentArea then
						parent = j
						parentArea = math.abs(P.area)
					end
				end
			end
			if parent then
				L.parent = parent
				L.depth = 1
			else
				complaints[#complaints + 1] =
					("r%03d loop%d: wound as a hole, inside no outer rim"):format(r, L.index)
			end
		end

		-- The same cross-check the other way: an outer rim must not sit inside
		-- another ring of its own region.
		for _, i in ipairs(outers) do
			local L = loops[i]
			local q = probe(L.pts, regionUp, Rings.probeInset)
			if q then
				for _, j in ipairs(outers) do
					if j ~= i and contains(loops[j].pts, e1, e2, q) then
						complaints[#complaints + 1] =
							("r%03d loop%d: wound as an outer rim, inside loop%d")
								:format(r, L.index, loops[j].index)
					end
				end
			end
		end
	end

	return { stats = stats, complaints = complaints }
end

-- One line, plus every complaint. Nothing is summarised away: a disagreement
-- between winding and containment is the whole reason this pass computes both.
function Rings.report(res: any): string
	local s = res.stats
	local lines = {
		("rings     %d regions, %d outer, %d holes, %d open, %d degenerate")
			:format(s.regions, s.outer, s.hole, s.open, s.degenerate),
	}
	for _, c in ipairs(res.complaints) do
		lines[#lines + 1] = "  ! " .. c
	end
	return table.concat(lines, "\n")
end

return Rings
