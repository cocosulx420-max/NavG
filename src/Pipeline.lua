--!strict
-- NVGN.Pipeline -- one bake, root to polygons, from a configuration that lives
-- in git rather than in the call that ran it.
--
-- THE POINT OF THIS MODULE IS THAT A RESULT CAN BE REPRODUCED. Every pass here
-- was previously driven by options typed into a throwaway call, so a drawing
-- could be inspected and approved and then never rebuilt, because the numbers
-- that made it were gone. So: OVERRIDES is the one place tuning is written
-- down, and every run stamps the parameters it actually used into the result
-- and into the DataModel, where a Studio save keeps them.
--
-- The stages, in order:
--   LocalGrid.build   parts -> cells -> regions      (Floor runs inside it)
--   Boundary.trace    cells -> directed faces -> closed loops
--   PathSimplify      loop -> corners -> merged -> dejogged -> bevels collapsed
--                     -> closed, if the trace left it open
local Pipeline = {}

local LocalGrid = require(script.Parent:WaitForChild("LocalGrid"))
local Boundary = require(script.Parent:WaitForChild("Boundary"))
local PathSimplify = require(script.Parent:WaitForChild("PathSimplify"))
local Rings = require(script.Parent:WaitForChild("Rings"))
local Severance = require(script.Parent:WaitForChild("Severance"))
local Thickness = require(script.Parent:WaitForChild("Thickness"))
local Erode = require(script.Parent:WaitForChild("Erode"))
local Triangulate = require(script.Parent:WaitForChild("Triangulate"))
local CDT = require(script.Parent:WaitForChild("CDT"))
local SVOLocal = require(script.Parent:WaitForChild("SVOLocal"))
local FaceKind = require(script.Parent:WaitForChild("FaceKind"))
local EdgeKind = require(script.Parent:WaitForChild("EdgeKind"))
local Nodes = require(script.Parent:WaitForChild("Nodes"))
local Portals = require(script.Parent:WaitForChild("Portals"))
local Leaps = require(script.Parent:WaitForChild("Leaps"))
-- drop and jump links (one-way), see Leaps
Pipeline.leaps = true

-- TUNING LIVES IN THE MODULES, NOT HERE. A number in OVERRIDES is a deliberate
-- departure from a module's own default, and the module comment next to that
-- default is where its reasoning stays. Duplicating the defaults into this
-- table would give every constant two homes and let them disagree, so the table
-- is empty until something genuinely needs to differ.
--
-- `root` is the exception: it has no sensible default, and a bake of the wrong
-- model is the one mistake that silently produces a plausible answer.
Pipeline.OVERRIDES = {
	-- 30, against PathSimplify's own 12. Merging harder is free here and the
	-- measurement says so: 12 gives 677 corners and 30 gives 621, both at the
	-- same 0.671 stud worst deviation and the same two edges over a step. 40
	-- gives 580 but doubles the edges over a step, so the cost starts there.
	--
	-- This is only safe because mergeMax bounds the drift absolutely. Loosening
	-- the angle when the drift ceiling was 2.0 studs is exactly what let a 4 stud
	-- link fold into a 35 stud run and stop describing the floor.
	mergeAngle = 30,
	-- mergeMin, NOT mergeMax, is what bounds a short span. The allowance is
	-- clamp(mergeRel * chord, mergeMin, mergeMax), and mergeRel of a six stud
	-- chord is 0.3, so every short span lands on the floor and mergeMax never
	-- enters into it. That is why loosening mergeMax alone changed nothing.
	--
	-- A quantised diagonal steps sideways about 0.6 studs, just over one cell, so
	-- at 0.35 all of those merges were refused and the line kept its stair
	-- pattern: 40 drift refusals on r001 loop3 alone, none from the raycast. 0.55
	-- clears them and leaves the worst deviation exactly where it was, at 0.671.
	-- Above 0.7 the deviation itself starts to climb.
	mergeMin = 0.55,
	-- Raised only so the clamp stays coherent once mergeMin moves. Still far below
	-- the 1.77 studs that let a 4 stud link fold into a 35 stud run.
	mergeMax = 1.2,
	-- 1.5, below minWidth's 2.0. minWidth is a standing agent's shoulders; this
	-- decides whether a region is worth tracing at all, and the crawl spaces this
	-- gate exists to keep are narrower than shoulders by definition. case5's is
	-- 1.5 studs across, so at 2.0 it was discarded and its loop never drawn.
	traceMinWidth = 1.5,
	-- 1.5, a 3-cell footprint. Cocosulx's call 2026-09-23: keep strips 3 cells
	-- thick and wider. Stair treads (1.5 deep) come back one region each; 1 stud
	-- rails and wall caps stay pruned.
	minWidth = 1.5,
	-- Cleaner lines, paid for in floor along the border, never toward a wall.
	-- case6: 13590 -> 13135 corners at 1.0; 1.5 buys only 19 more.
	inwardMax = 1.0,
} :: { [string]: any }

-- The parameters each stage reads. Used to snapshot what a run actually used,
-- so a result is self-describing.
local BAKE_KEYS = {
	"step", "maxSlope", "clearCap", "minClearance", "flushTol", "probeRadius",
	"minWidth", "traceMinWidth", "regionAngle", "regionPlanarity", "bandHeight", "standHeight",
	"crouchHeight", "connectivity", "faceAngle",
}
local SIMPLIFY_KEYS = {
	"windowSize", "dotThreshold", "epsilonSq", "strict",
	"mergeAngle", "mergeRel", "mergeMin", "mergeMax",
	"jogMax", "jogParallel",
	"bevelMax", "bevelSquare", "bevelRunRatio", "bevelRunMin", "bevelTravel",
	"closeMaxGap", "closeAngleMin", "closeTravel", "closeMergeMax", "closeMergeRun",
	"inwardMax", "inwardAngle",
	"clusterRadius", "spikeAngle", "spikeArea", "crossCutMax",
}

-- How the raycast validator probes. These belong to the pipeline rather than to
-- PathSimplify, which never casts anything itself.
local VALIDATE = {
	-- Cast the edge test at ankle height rather than along the boundary itself.
	-- On the surface a ray grazes the floor it is tracing and reports a hit for
	-- every edge.
	rayLift = 0.35,
	-- Start the floor probe above the proposed point and reach below it, so a
	-- point sitting a hair under the surface still finds the floor it is on.
	floorRise = 0.8,
	floorDrop = 1.8,
}

-- A CLOSED RING ENCLOSING LESS THAN THIS IS NOT A BOUNDARY, it is debris.
-- Squared studs, and a tenth of one 0.5-stud cell -- far under any hole a real
-- lattice can describe, so nothing real is ever this small.
Pipeline.degenArea = 0.05

Pipeline.debugName = "NVGN_Debug"
-- An ABSOLUTE location, not `script.Parent`. The module is routinely required
-- from a throwaway clone to get past Luau's require cache, and a stamp written
-- beside the clone dies with it, leaving the previous run's numbers in place
-- looking current. A stale stamp is worse than none.
Pipeline.stampParent = game:GetService("ServerScriptService")
Pipeline.stampName = "NVGN_LastBake"

local function resolve(overrides: any?): any
	local c = {}
	for k, v in pairs(Pipeline.OVERRIDES) do c[k] = v end
	if overrides then for k, v in pairs(overrides) do if v ~= nil then c[k] = v end end end
	return c
end

-- Every number this run will use, resolved against each module's defaults. This
-- is what makes a result reproducible: it names the values, not the intent.
function Pipeline.effective(cfg: any?): any
	local c = resolve(cfg)
	local snap = { root = c.root and c.root:GetFullName() or nil, cw = c.cw or false }
	local bake = {}
	for _, k in ipairs(BAKE_KEYS) do bake[k] = c[k] end
	local simp = {}
	for _, k in ipairs(SIMPLIFY_KEYS) do
		local v = c[k]
		if v == nil then v = (PathSimplify :: any)[k] end
		simp[k] = v
	end
	-- Only the overrides are known before a bake; run() replaces this with the
	-- values LocalGrid actually resolved.
	snap.bake = bake
	snap.simplify = simp  -- fully resolved; PathSimplify owns every default
	snap.validate = { rayLift = VALIDATE.rayLift, floorRise = VALIDATE.floorRise,
		floorDrop = VALIDATE.floorDrop }
	return snap
end

-- A geometry test for the passes that move a line: PathSimplify calls it with
-- two points to test one edge, or with a proposed corner and the two run ends
-- that will meet it.
--
-- IT ANSWERS "DOES THIS EDGE CROSS SOMETHING", NOT "IS THIS STILL THE FLOOR".
-- Drifting off the side of a floor hits nothing at all, so this can never stand
-- in for a drift bound; that job belongs to mergeMax and epsilonSq.
function Pipeline.validator(up: Vector3, debugRoot: Instance?): (any) -> boolean
	local rp = RaycastParams.new()
	rp.FilterType = Enum.RaycastFilterType.Exclude
	rp.FilterDescendantsInstances = debugRoot and { debugRoot } or {}
	rp.IgnoreWater = true
	local lift = up * VALIDATE.rayLift
	local rise, drop = up * VALIDATE.floorRise, -up * VALIDATE.floorDrop

	local function clear(a: Vector3, b: Vector3): boolean
		local d = b - a
		if d.Magnitude < 1e-4 then return true end
		return workspace:Raycast(a + lift, d, rp) == nil
	end
	return function(p: Vector3, q: Vector3, r: Vector3?): boolean
		if r == nil then return clear(p, q) end
		if workspace:Raycast(p + rise, drop, rp) == nil then return false end
		return clear(q, p) and clear(p, r)
	end :: any
end

-- Cells, regions and boundary loops. Returns LocalGrid's data table with
-- `boundary` filled in.
-- Re-decide wall / drop / seam from SVOLocal after the trace, per RAW FACE.
--
-- OFF: superseded by EdgeKind, which asks the same question of the polygon
-- edges directly and so needs no provenance chain to get the answer back to
-- them. Kept wired because it is the independent second opinion the EdgeKind
-- verdicts are cross-checked against, and because it is the only thing that
-- labels the raw faces for `drawRawFaces`.
Pipeline.faceKind = false

-- Label and split the polygon boundary edges as part of building the mesh.
--
-- OFF. Portals need neither of its answers: inside a region a shared polygon
-- edge is proof of adjacency on its own, and across one Severance has already
-- decided reachability with a gate it validated against case3's own treads. The
-- classification also carries a measured defect -- 213 of 1369 `wall` samples
-- have nothing in the probe box, from voxel over-claim -- which there is no
-- reason to inherit. Turning it off also skips `SVOLocal.fromParts`, which is
-- most of a second of the bake and is built for nothing else here.
--
-- Turn it back on to draw `drawMeshKind` or to work on the classification
-- itself. Nothing in the mesh or the portals changes when it does.
Pipeline.edgeKind = false

function Pipeline.bake(cfg: any?): (any, any)
	local c = resolve(cfg)
	assert(c.root, "Pipeline: cfg.root is required -- name the model to bake")
	local data = LocalGrid.build(c)
	-- EROSION IS OFF BY DEFAULT. It was on until 2026-09-12, when the call was
	-- made that the mesh should be a correct representation of the map: the
	-- boundary is where the floor actually ends, and clearance is the runtime's
	-- problem. Erosion shrinks the floor by a node so an agent's centre can never
	-- touch a wall, which is a real guarantee, but it pays for it by deleting
	-- ground -- case3 empties 17 of its regions outright at one node.
	--
	-- `erode = true` turns it back on at Erode.radius, `erode = <studs>` at a
	-- radius of your choosing. Kept wired because the eroded and un-eroded traces
	-- are worth comparing, and because a later pathfinder may want the bake-time
	-- guarantee after all.
	local estats = nil
	if c.erode then
		estats = Erode.apply(data, typeof(c.erode) == "number" and c or nil)
	end
	local _, bstats = Boundary.trace(data, c)
	if estats then bstats.erode = estats end

	-- WHY THE FLOOR STOPS, decided per raw face against a tree per PART rather
	-- than against the merged global octree. See FaceKind for the measurements
	-- that forced this; the short version is that `drop` used to be the `else`
	-- of `wall`, so every failed probe read as open ground.
	--
	-- Runs here, after the trace, because it labels FACES and the faces only
	-- exist once Boundary has emitted them. `classifyNodes` is deliberately left
	-- alone -- edgeMask, stepMask, cell.wall and the erosion all still read the
	-- masks it writes, and seams are the one verdict it already got right.
	if Pipeline.faceKind then
		local t0 = os.clock()
		local trees, ttotals = SVOLocal.fromParts(data.parts, FaceKind.leaf, 0.01)
		data.localTrees = trees
		local kstats = FaceKind.build(data, trees)
		kstats.buildSeconds = os.clock() - t0
		kstats.trees = ttotals.parts
		kstats.treeNodes = ttotals.nodes
		bstats.kind = kstats
	end
	return data, bstats
end

-- A loop from Boundary.chain is a list of FACE INDICES, not points. Turning it
-- into a polyline fixes where a boundary node sits, and that choice is as much
-- a part of the result as any tolerance.
--
-- MIDPOINT, THEN INSET HALF A STEP INTO THE REGION. The face's own corners
-- zigzag by half a step at every cell, which is noise the simplifier then has to
-- undo; the midpoint is already the smoother polyline. Inset because a node on
-- the face sits exactly on the boundary of the floor, where a character cannot
-- stand and a downward probe is a coin toss; half a step puts it on the centre
-- line of the outermost cells, clear of the edge and clear of any wall.
--
-- Faces are wound region-on-the-left, so up x direction points INTO the region.
--
-- Duplicates are dropped. At a convex corner one cell contributes two faces
-- whose midpoints both inset onto that cell's centre, so the same position would
-- otherwise appear twice and give the simplifier a zero-length segment.
-- `faceOf` comes back alongside the points: which face each raw node was made
-- from. Two of the steps above lose that correspondence -- a zero length face is
-- skipped and a convex corner's two faces collapse onto one point -- so it
-- cannot be recovered afterwards by counting, and it is what carries a face's
-- wall/drop/edge verdict forward to the finished polygon.
local function polyline(entry: any, loop: any, step: number): ({Vector3}, Vector3, {number})
	local F = loop.faces
	local pts = table.create(#F)
	local faceOf = table.create(#F)
	local up = Vector3.yAxis
	local inset = step * 0.5
	for i, fi in ipairs(F) do
		local f = entry.faces[fi]
		if i == 1 then up = f.up end
		local d = f.b - f.a
		if d.Magnitude > 1e-9 then
			local p = (f.a + f.b) * 0.5 + f.up:Cross(d.Unit) * inset
			local prev = pts[#pts]
			if not prev or (prev - p).Magnitude > 1e-3 then
				pts[#pts + 1] = p
				faceOf[#pts] = fi
			end
		end
	end
	if #pts > 1 and loop.closed and (pts[1] - pts[#pts]).Magnitude < 1e-3 then
		faceOf[#pts] = nil
		pts[#pts] = nil
	end
	return pts, up, faceOf
end

-- WHAT EACH SIMPLIFIED EDGE IS MADE OF.
--
-- Boundary.faces already decides the only thing a portal builder needs to know
-- -- whether the floor stops here because of a WALL or because it simply runs
-- out -- and every pass after it threw that away, so a finished outline said
-- "floor ends here" and nothing more. A wall and a stair nose looked identical.
--
-- BY PROVENANCE, NEVER BY PROXIMITY. Every corner knows the raw node it came
-- from, so the span of raw nodes an edge covers is known exactly and the faces
-- under that span are the faces the edge is made of. Matching an edge to nearby
-- faces by distance instead needs a tolerance, and any tolerance wide enough to
-- survive the 0.671 stud simplification deviation is also wide enough to pick up
-- the wall on the FAR side of a doorway, which is the one answer that must never
-- be wrong.
--
-- The span is half open, [a, b): the raw node at b belongs to the next edge.
-- WHICH REPAIR TIER PUT AN EDGE THERE, alongside what kind of boundary it is.
--
-- `kind` and repair origin are different questions and the first cannot answer
-- the second: `Boundary.bridge` stamps its chords `kind = "none"`, which is the
-- SAME label a genuine "the floor runs out here" face carries, so an edge that
-- the pipeline invented out of a nearest-neighbour pairing is indistinguishable
-- by kind from real open boundary. Hence a second, separate label.
--
-- Read off the raw faces the edge spans, in the walk this function was already
-- doing -- no second traversal. Worst wins, because one invented chord in a span
-- is what matters: bridge > weld > invented > none.
local function edgeKinds(entry: any, poly: {Vector3}, faceOf: {number},
	rawIdx: {number}, pts: {Vector3}, closed: boolean): ({string}, {number}, {string})
	local nRaw = #poly
	local n = #pts
	local last = closed and n or math.max(n - 1, 0)
	local kinds, wallFrac, repair = {}, {}, {}
	for i = 1, last do
		local a = rawIdx[i]
		local b = rawIdx[(i % n) + 1]
		-- A corner the closing pass invented has no raw node behind it, so the
		-- edges either side of it are this pipeline's own work and get said so
		-- rather than guessed at.
		if a == nil or b == nil or nRaw == 0 then
			kinds[i] = "invented"
			wallFrac[i] = 0
			repair[i] = "invented"
		else
			local tally, count, wall = {}, 0, 0
			local sawBridge, sawWeld = false, false
			local j = a
			for _ = 1, nRaw do
				if j == b then break end
				local fi = faceOf[j]
				local f = fi and entry.faces[fi]
				if f then
					count += 1
					tally[f.kind] = (tally[f.kind] or 0) + 1
					if f.kind == "wall" then wall += 1 end
					if f.bridged then sawBridge = true end
					if f.welded then sawWeld = true end
				end
				j = (j % nRaw) + 1
			end
			repair[i] = sawBridge and "bridge"
				or sawWeld and "weld"
				or (count == 0 and "invented" or "none")
			if count == 0 then
				kinds[i] = "invented"
				wallFrac[i] = 0
			else
				wallFrac[i] = wall / count
				if wall == count then
					kinds[i] = "wall"
				elseif wall > 0 then
					-- MIXED IS NOT ROUNDED TO THE MAJORITY. An edge merged across a
					-- doorjamb is part wall and part opening, and calling it whichever
					-- won on count either invents a doorway through a wall or seals a
					-- real one. Whoever cuts portals out of these has to see the split.
					kinds[i] = "mixed"
				else
					local best, bn = "none", -1
					for k, v in pairs(tally) do
						if v > bn or (v == bn and k < best) then best, bn = k, v end
					end
					kinds[i] = best
				end
			end
		end
	end
	return kinds, wallFrac, repair
end

-- Every traced loop, simplified to corners. One loop in, one entry out; a loop
-- the closing pass could not shut keeps `closed = false` and stays a path.
-- Enclosed area of a ring, signed, in the plane of `up`.
--
-- Boundary winds every loop with the floor on the LEFT, so the sign says which
-- kind of ring this is before anything has classified it: positive is an outer
-- rim and negative is a hole. Rings computes the same thing later and in more
-- detail, but it runs AFTER simplification, and simplification is exactly where
-- the difference has to be known.
local function ringArea(pts: { Vector3 }, up: Vector3): number
	if #pts < 3 then return 0 end
	local e1, e2 = Rings.basis(up)
	local o = pts[1]
	local a = 0
	for i = 2, #pts - 1 do
		local p, q = pts[i] - o, pts[i + 1] - o
		a += (p:Dot(e1) * q:Dot(e2)) - (q:Dot(e1) * p:Dot(e2))
	end
	return a * 0.5
end

-- A HOLE IS NOT SIMPLIFIED AS HARD AS A RIM. Cocosulx's call, after watching a
-- pillar disappear: at 195 corners case3 lost three of its five holes outright,
-- and a hole that collapses is not a cosmetic loss, it is a pillar turning into
-- walkable floor. A rim that simplifies badly costs a sliver of ground; a hole
-- that simplifies badly puts an NPC inside a column. Holes therefore ignore the
-- run's overrides and use the tight baseline.
Pipeline.holeTight = true

-- A ring must keep this much of the area it enclosed before simplification, or
-- the simplification is thrown away and redone tight.
--
-- This is the other half of the same failure. Aggressive settings collapsed six
-- of case3's rings to degenerate and three whole regions went untriangulated
-- for want of an outer rim -- small steps, mostly. Losing 30% of a ring's area
-- is not a simplification, it is a deletion, and it gets refused.
Pipeline.keepArea = 0.7

-- Knots, spikes and self-crossings at corners, cleaned after collapseBevels.
-- See PathSimplify.cleanCorners; every move it makes passes the validator.
Pipeline.cleanCorners = true

-- A SMALL HOLE WITH NOTHING IN IT IS A GAP, NOT AN OBSTACLE. Rows of them sit
-- along seams where a few cells went missing, and each one is a hole CDT has to
-- cut around. A hole is filled only when it is small AND proven empty:
--   * every chord between two of its corners is clear at ankle height (the
--     validator's own ray), so no post, panel or pillar stands inside it; and
--   * floor lies under its centre and under every chord midpoint, so it is not
--     a pit.
-- A pillar fails the first test and a hole in the floor fails the second.
Pipeline.gapFill = true
-- Loosened 2026-09-23 on request. The emptiness test is what keeps it honest:
-- of case6's small holes left at the tighter limits, 77 had a solid inside.
Pipeline.gapArea = 4.0      -- square studs; any hole this small qualifies
Pipeline.gapWidth = 1.5     -- studs; or one this narrow in its own plane...
Pipeline.gapAreaMax = 16.0  -- ...up to this area, so a long seam sliver qualifies
Pipeline.gapMaxCorners = 24 -- chords are n^2; a bigger ring is not a sliver

-- OPEN PIECES OF ONE REGION ARE ONE BOUNDARY. When the trace breaks a rim it
-- breaks it into several open chains, and `close` only ever joins a chain to
-- ITSELF -- so two halves of one rim 5 studs apart both stay open. Join a
-- piece's end to another piece's start (winding says which end meets which)
-- when they are within joinMax, the gap is ray-clear and floor lies under its
-- middle; a chain whose own ends then meet within joinMax is closed.
Pipeline.joinOpen = true
Pipeline.joinMax = 6.0 -- studs

-- AN END THAT OVERSHOOTS ITS OWN START. The trace can run a tail back along
-- the rim it began on, so the ends look 6 studs apart while the path passed
-- within a cell of its start: r001's rim came back to 0.63 studs, then ran on
-- 6.6 studs over its own first edge. Trim each end by at most trimMax of path
-- to the pair of raw nodes that meet, and close there.
Pipeline.trimMax = 8.0  -- studs of path either end may lose
Pipeline.trimMeet = 1.0 -- studs; how close the trimmed ends must come

local function trimOverlap(entry: any, faces: { number }, step: number,
	gapOK: (Vector3, Vector3) -> boolean): ({ number }, boolean)
	local pts, _, faceOf = polyline(entry, { faces = faces, closed = false }, step)
	local n = #pts
	if n < 8 then return faces, false end
	local head, tail = { 1 }, { n }
	local run = 0
	for i = 2, n do
		run += (pts[i] - pts[i - 1]).Magnitude
		if run > Pipeline.trimMax then break end
		head[#head + 1] = i
	end
	run = 0
	for i = n - 1, 1, -1 do
		run += (pts[i + 1] - pts[i]).Magnitude
		if run > Pipeline.trimMax then break end
		tail[#tail + 1] = i
	end
	local best, bh, bt = Pipeline.trimMeet, nil, nil
	for _, h in ipairs(head) do
		for _, t in ipairs(tail) do
			if t - h > 4 then
				local d = (pts[t] - pts[h]).Magnitude
				if d <= best then best, bh, bt = d, h, t end
			end
		end
	end
	if not bh or not bt or not gapOK(pts[bt], pts[bh]) then return faces, false end
	local pos = {}
	for k, fi in ipairs(faces) do if pos[fi] == nil then pos[fi] = k end end
	local from, to = pos[faceOf[bh]], pos[faceOf[bt]]
	if not from or not to or to <= from then return faces, false end
	return table.move(faces, from, to, 1, {}), true
end

-- A SHADOW IS NOT A BOUNDARY. The top of a thin part (a 0.4 to 1 stud wall or
-- panel) sitting flush with a floor and overlapping its edge joins the floor's
-- region, and its own rim is traced a fraction of a stud inside the floor's
-- rim, running the same way. Nothing connects to its ends, so it can never
-- close. Every open piece left on case6 after joining was one of these: r002's
-- wall, r046/r049's panels, r198's, r221's DestructibleWallTest. An open piece
-- whose faces mostly run beside a same-direction face of one of the region's
-- CLOSED rings is dropped before joining, so it is never joined into a ring
-- that does not fit either.
Pipeline.shadowGap = 0.3   -- studs between a face and the rim face it shadows
Pipeline.shadowShare = 0.5 -- share of the piece's faces that must shadow

local function isShadow(entry: any, L: any, closedIdx: { [string]: { any } }): boolean
	local n, hit = 0, 0
	for _, fi in ipairs(L.faces) do
		local f = entry.faces[fi]
		local d = f.b - f.a
		if d.Magnitude > 1e-9 then
			n += 1
			d = d.Unit
			local m = (f.a + f.b) * 0.5
			local bx, bz = math.floor(m.X), math.floor(m.Z)
			local found = false
			for ox = -1, 1 do
				for oz = -1, 1 do
					for _, g in ipairs(closedIdx[(bx + ox) .. ":" .. (bz + oz)] or {}) do
						if (g.m - m).Magnitude <= Pipeline.shadowGap and g.d:Dot(d) > 0.9 then
							found = true
							break
						end
					end
					if found then break end
				end
				if found then break end
			end
			if found then hit += 1 end
		end
	end
	return n > 0 and hit >= Pipeline.shadowShare * n
end

local function joinOpen(entry: any, step: number, debugRoot: Instance?): ({ any }, number, number, number)
	local open, out = {}, {}
	for _, L in ipairs(entry.loops) do
		if L.closed then out[#out + 1] = L else open[#open + 1] = L end
	end
	if #open == 0 then return entry.loops, 0, 0, 0 end
	local shadows = 0
	do
		local idx: { [string]: { any } } = {}
		for _, L in ipairs(out) do
			for _, fi in ipairs(L.faces) do
				local f = entry.faces[fi]
				local d = f.b - f.a
				if d.Magnitude > 1e-9 then
					local m = (f.a + f.b) * 0.5
					local k = math.floor(m.X) .. ":" .. math.floor(m.Z)
					local b = idx[k]
					if not b then b = {}; idx[k] = b end
					b[#b + 1] = { m = m, d = d.Unit }
				end
			end
		end
		local keep = {}
		for _, L in ipairs(open) do
			if #out > 0 and isShadow(entry, L, idx) then shadows += 1 else keep[#keep + 1] = L end
		end
		open = keep
	end
	if #open == 0 then return out, 0, 0, shadows end
	local chains = {}
	for _, L in ipairs(open) do
		local pts, up = polyline(entry, L, step)
		if #pts >= 1 then
			chains[#chains + 1] = { faces = table.clone(L.faces), a = pts[1], b = pts[#pts], up = up, n = 1 }
		end
	end
	local joined, closedN = 0, 0
	local check = Pipeline.validator(chains[1] and chains[1].up or Vector3.yAxis, debugRoot)
	local function gapOK(p: Vector3, q: Vector3): boolean
		return check((p + q) * 0.5, p, q)
	end
	local refused = {}
	while true do
		local best, bi, bj = Pipeline.joinMax, nil, nil
		for i, A in ipairs(chains) do
			for j, B in ipairs(chains) do
				if i ~= j then
					local d = (B.a - A.b).Magnitude
					if d <= best and not refused[tostring(A.b) .. tostring(B.a)] then
						best, bi, bj = d, i, j
					end
				end
			end
		end
		if not bi then break end
		local A, B = chains[bi], chains[bj]
		if gapOK(A.b, B.a) then
			for _, f in ipairs(B.faces) do A.faces[#A.faces + 1] = f end
			A.b = B.b
			A.n += B.n
			table.remove(chains, bj)
			joined += 1
		else
			refused[tostring(A.b) .. tostring(B.a)] = true
		end
	end
	for _, C in ipairs(chains) do
		-- a lone piece whose gap `close` can handle is left to `close`, which
		-- recovers the corner rather than drawing a chord
		local gap = (C.a - C.b).Magnitude
		local closed = gap <= Pipeline.joinMax
			and (C.n > 1 or gap > PathSimplify.closeMaxGap)
			and gapOK(C.b, C.a)
		local faces = C.faces
		if not closed and gap > PathSimplify.closeMaxGap then
			faces, closed = trimOverlap(entry, faces, step, gapOK)
		end
		if closed then closedN += 1 end
		out[#out + 1] = { faces = faces, closed = closed, joined = closed }
	end
	return out, joined, closedN, shadows
end

-- Narrowest extent of a ring in its plane, over a 15 degree caliper sweep.
local function ringWidth(pts: { Vector3 }, up: Vector3): number
	local e1, e2 = Rings.basis(up)
	local best = math.huge
	for a = 0, 165, 15 do
		local r = math.rad(a)
		local ax = e1 * math.cos(r) + e2 * math.sin(r)
		local lo, hi = math.huge, -math.huge
		for _, p in ipairs(pts) do
			local d = p:Dot(ax)
			if d < lo then lo = d end
			if d > hi then hi = d end
		end
		if hi - lo < best then best = hi - lo end
	end
	return best
end

-- "filled", a reason it was kept, or nil when the hole is not small enough to ask.
local function gapVerdict(poly: { Vector3 }, up: Vector3, rawArea: number, debugRoot: Instance?): string?
	local a = math.abs(rawArea)
	if a > Pipeline.gapAreaMax then return nil end
	if a > Pipeline.gapArea and ringWidth(poly, up) > Pipeline.gapWidth then return nil end
	-- corners, not raw nodes: a raw hole ring is a staircase of half-step faces
	local pts = PathSimplify.simplify(poly, { closed = true })
	if #pts < 3 then pts = poly end
	if #pts > Pipeline.gapMaxCorners then return "too many corners" end

	local rp = RaycastParams.new()
	rp.FilterType = Enum.RaycastFilterType.Exclude
	rp.FilterDescendantsInstances = debugRoot and { debugRoot } or {}
	rp.IgnoreWater = true
	local lift = up * VALIDATE.rayLift
	local rise, drop = up * VALIDATE.floorRise, -up * VALIDATE.floorDrop
	local function floorAt(p: Vector3): boolean
		return workspace:Raycast(p + rise, drop, rp) ~= nil
	end

	local c = Vector3.zero
	for _, p in ipairs(pts) do c += p end
	c /= #pts
	if not floorAt(c) then return "no floor" end
	local n = #pts
	for i = 1, n do
		for j = i + 1, n do
			local p, q = pts[i], pts[j]
			if (q - p).Magnitude > 1e-3 then
				if workspace:Raycast(p + lift, q - p, rp) then return "solid" end
				if j ~= i + 1 and not (i == 1 and j == n) and not floorAt((p + q) * 0.5) then
					return "no floor"
				end
			end
		end
	end
	return "filled"
end

function Pipeline.simplify(data: any, cfg: any?): ({any}, any)
	local c = resolve(cfg)
	local o = {}
	for _, k in ipairs(SIMPLIFY_KEYS) do o[k] = c[k] end
	-- the same options WITHOUT this run's overrides, for holes and for any ring
	-- that the run's settings would have destroyed
	local tight = {}
	for _, k in ipairs(SIMPLIFY_KEYS) do tight[k] = Pipeline.OVERRIDES[k] end
	local debugRoot = workspace:FindFirstChild(Pipeline.debugName)
	local step = data.config.step

	local out = {}
	local stats = { loops = 0, open = 0, raw = 0, corners = 0,
		holesHeld = 0, rescued = 0, collapsed = 0, degenerate = 0,
		gapFilled = 0, gapKept = {},
		clean = { crossCut = 0, crossRefused = 0, spikes = 0, clusters = 0, clusterRefused = 0, reverted = 0, notches = 0 },
		joined = 0, joinClosed = 0, shadows = 0,
		edges = 0, wallEdges = 0, openEdges = 0, mixedEdges = 0, inventedEdges = 0,
		repair = {},
		closedBy = { merge = 0, intersect = 0, straight = 0, ["already closed"] = 0 } }

	-- Region order follows Boundary's table, which LocalGrid numbers largest
	-- region first; sort so a result lists loops the same way every run.
	local regions = {}
	for r in pairs(data.boundary) do regions[#regions + 1] = r end
	table.sort(regions)

	for _, r in ipairs(regions) do
		local entry = data.boundary[r]
		local loopsIn = entry.loops
		if Pipeline.joinOpen then
			local j, jc, sh
			loopsIn, j, jc, sh = joinOpen(entry, step, debugRoot)
			stats.joined += j
			stats.joinClosed += jc
			stats.shadows += sh
		end
		for li, L in ipairs(loopsIn) do
			local poly, up, faceOf = polyline(entry, L, step)
			local rawArea = ringArea(poly, up)
			local isHole = L.closed and rawArea < 0

			-- A CLOSED RING THAT ENCLOSES NOTHING IS NOT A BOUNDARY. `Boundary.chain`
			-- can close a walk over coincident faces -- the small-region collapse,
			-- where a couple of border cells yield two edges lying on top of one
			-- another -- and what comes back is a ring of zero area.
			--
			-- It survived because the rescue below is gated on `rawArea > 1e-6`, so
			-- the one case it most needed to catch was the one case it skipped.
			-- case5 shipped 14 of these as ONE-POINT rings, each of them a CDT
			-- complaint and a polygon describing no floor. Dropping one removes
			-- nothing by definition: it encloses no area. Counted, so the loss stays
			-- visible instead of becoming a gap in the loop numbering.
			if L.closed and math.abs(rawArea) <= Pipeline.degenArea then
				stats.degenerate += 1
				continue
			end

			if isHole and Pipeline.gapFill then
				local verdict = gapVerdict(poly, up, rawArea, debugRoot)
				if verdict == "filled" then
					stats.gapFilled += 1
					continue
				elseif verdict then
					stats.gapKept[verdict] = (stats.gapKept[verdict] or 0) + 1
				end
			end

			-- `ri` is the third return: the raw node each finished corner came
			-- from. Threaded rather than recovered, because collapseBevels can
			-- replace two corners with one and no amount of counting afterwards
			-- says which raw nodes that new corner speaks for.
			local function run(base: any)
				local opts = table.clone(base)
				opts.closed = L.closed
				opts.up = up
				opts.validate = Pipeline.validator(up, debugRoot)
				local p, i = PathSimplify.simplify(poly, opts)
				-- `merge` and `dejog` BOTH return a third value saying what they
				-- refused and why, and it was being dropped on the floor here.
				-- A jog this pipeline saw and declined is invisible in every
				-- report unless it is carried out, and that refusal is exactly
				-- what a one-cell notch surviving into the output looks like.
				local ms, js
				p, i, ms = PathSimplify.merge(p, i, poly, opts)
				p, i, js = PathSimplify.dejog(p, i, poly, opts)
				local q, _, map = PathSimplify.collapseBevels(p, opts)
				local ri = table.create(#q)
				for j = 1, #q do ri[j] = i[map[j]] end
				local cs
				if Pipeline.cleanCorners then
					-- a cleanup that deletes the ring is not a cleanup: undone whole
					local cq, cri
					cq, cri, cs = PathSimplify.cleanCorners(q, ri, opts)
					local before = math.abs(ringArea(q, up))
					if before < 1e-6 or math.abs(ringArea(cq, up)) >= Pipeline.keepArea * before then
						q, ri = cq, cri
					else
						cs = { crossCut = 0, crossRefused = 0, spikes = 0, clusters = 0,
							clusterRefused = 0, reverted = 1 }
					end
				end
				return q, opts, ri, { merge = ms, jog = js, clean = cs }
			end

			local opts
			local pts, rawIdx, simpStats
			pts, opts, rawIdx, simpStats = run((isHole and Pipeline.holeTight) and tight or o)
			if isHole and Pipeline.holeTight then stats.holesHeld += 1 end

			-- Refuse a simplification that deleted the ring rather than
			-- simplifying it, and try again with the tight settings. Checked on
			-- CLOSED rings only: an open path encloses nothing, so it has no
			-- area to lose and the test would fire on every one of them.
			if L.closed and math.abs(rawArea) > 1e-6 then
				local kept = math.abs(ringArea(pts, up)) / math.abs(rawArea)
				if #pts < 3 or kept < Pipeline.keepArea then
					local retry, ropts, rri, rss = run(tight)
					local rkept = math.abs(ringArea(retry, up)) / math.abs(rawArea)
					if #retry >= 3 and rkept > kept then
						pts, opts, rawIdx, simpStats = retry, ropts, rri, rss
						stats.rescued += 1
					else
						stats.collapsed += 1
					end
				end
			end

			-- And again after simplification: the tight retry above can still fail
			-- to keep three corners, and `stats.collapsed` only COUNTED that while
			-- emitting the ring anyway. Fewer than three corners is not a polygon.
			if L.closed and (#pts < 3 or math.abs(ringArea(pts, up)) <= Pipeline.degenArea) then
				stats.degenerate += 1
				continue
			end

			local closed, method = L.closed, nil
			if not closed then
				local before = pts
				local ok, cs
				pts, ok, cs = PathSimplify.close(pts, opts)
				closed = ok
				method = ok and cs.method or nil
				if ok then stats.closedBy[cs.method] = (stats.closedBy[cs.method] or 0) + 1 end
				-- Realign provenance across the closing pass. `close` moves or adds
				-- corners rather than deleting them, so a corner it left alone is
				-- bit identical to the one it came from; anything else has no raw
				-- node behind it and is left nil, which edgeKinds reports as
				-- invented. Matching by position is safe HERE and nowhere else --
				-- these are the same floats, not nearby ones.
				if pts ~= before then
					local at = {}
					for j, p in ipairs(before) do at[tostring(p)] = rawIdx[j] end
					local moved = table.create(#pts)
					for j, p in ipairs(pts) do moved[j] = at[tostring(p)] end
					rawIdx = moved
				end
				-- A ring the closing pass just closed never had its corners
				-- cleaned: cleanCorners only takes closed rings. The knot sits
				-- exactly at the join, so clean it now.
				if closed and Pipeline.cleanCorners then
					local co = table.clone(opts)
					co.closed = true
					local cq, cri, cs2 = PathSimplify.cleanCorners(pts, rawIdx, co)
					local a0 = math.abs(ringArea(pts, up))
					if a0 < 1e-6 or math.abs(ringArea(cq, up)) >= Pipeline.keepArea * a0 then
						pts, rawIdx = cq, cri
						simpStats = simpStats or {}
						local prior = simpStats.clean
						if prior then
							for k, v in pairs(cs2) do prior[k] = (prior[k] or 0) + v end
						else
							simpStats.clean = cs2
						end
					end
				end
			end

			local kinds, wallFrac, repair = edgeKinds(entry, poly, faceOf, rawIdx, pts, closed)
			for _, k in pairs(kinds) do
				stats.edges += 1
				if k == "wall" then stats.wallEdges += 1
				elseif k == "mixed" then stats.mixedEdges += 1
				elseif k == "invented" then stats.inventedEdges += 1
				else stats.openEdges += 1 end
			end
			-- How much of the finished boundary a repair tier put there. Counted
			-- separately from `kinds` because a bridge chord is `kind = "none"`
			-- and would otherwise be filed as ordinary open boundary.
			for _, k in pairs(repair) do
				stats.repair[k] = (stats.repair[k] or 0) + 1
			end

			out[#out + 1] = { region = r, index = li, up = up,
				poly = poly, pts = pts, closed = closed, closedBy = method,
				faceOf = faceOf, rawIdx = rawIdx, rawArea = rawArea,
				edgeKind = kinds, edgeWall = wallFrac, edgeRepair = repair,
				simp = simpStats, joined = L.joined and closed or nil }
			if simpStats and simpStats.clean then
				for k, v in pairs(simpStats.clean) do stats.clean[k] += v end
			end
			stats.loops += 1
			stats.raw += #poly
			stats.corners += #pts
			if not closed then stats.open += 1 end
		end
	end
	-- Label every ring outer or hole before anything downstream sees it. This
	-- runs here rather than as its own call because it is not a measurement:
	-- `measure` is optional and reports on a result, this WRITES the structure
	-- the offset and the triangulation both read.
	stats.rings = Rings.classify(out)

	-- A JOIN THAT PRODUCED A RING THAT DOES NOT FIT IS UNDONE. A second outer
	-- rim in its region, or a hole inside no rim, means the chord closed the
	-- wrong thing; the piece goes back to open, as it was before the join.
	local reopened = 0
	local outersOf = {}
	for _, L in ipairs(out) do
		if L.kind == "outer" then outersOf[L.region] = (outersOf[L.region] or 0) + 1 end
	end
	for _, L in ipairs(out) do
		if L.joined and ((L.kind == "outer" and outersOf[L.region] > 1)
			or (L.kind == "hole" and L.parent == nil)) then
			if L.kind == "outer" then outersOf[L.region] -= 1 end
			L.closed = false
			L.joined = nil
			reopened += 1
		end
	end
	if reopened > 0 then
		stats.joinReopened = reopened
		stats.joinClosed -= reopened
		stats.open += reopened
		stats.rings = Rings.classify(out)
	end

	return out, stats
end

-- Bake and simplify, and record what it took to do so.
--
-- WHAT A FULL BAKE COSTS, case6 measured end to end 2026-09-16: SVO 53s,
-- Floor.extract 4.6s, LocalGrid.fromFloor 293s, Boundary.trace 80s,
-- simplify 0.4s, CDT.build 233s, Portals.build 2.6s -- 14.6 min including the
-- mesh and the portals. It was 29.5 min before the raycast-filter and adaptive
-- grid work; `fromFloor` and CDT are still two thirds of it.
--
-- Pass `cfg.onProgress` from a coroutine for anything this size: it is what
-- makes the four divisible stages yield. `Boundary.trace` and `CDT.build` do
-- not yield at all and will block Studio for minutes apiece.
function Pipeline.run(cfg: any?): any
	local t0 = os.clock()
	local data, bstats = Pipeline.bake(cfg)
	local tBake = os.clock() - t0

	local t1 = os.clock()
	local loops, sstats = Pipeline.simplify(data, cfg)
	local tSimplify = os.clock() - t1

	local effective = Pipeline.effective(cfg)
	-- LocalGrid resolves every bake default it was not given, so read the values
	-- back off the bake rather than reporting the handful that were overridden.
	local baked = {}
	for k, v in pairs(data.config) do
		baked[k] = typeof(v) == "Instance" and v:GetFullName() or v
	end
	effective.bake = baked

	local result = {
		config = effective,
		data = data,
		loops = loops,
		stats = { boundary = bstats, simplify = sstats,
			bakeSeconds = tBake, simplifySeconds = tSimplify },
	}
	Pipeline.stamp(result)
	return result
end

-- Write the effective configuration into the DataModel so a Studio save keeps
-- it. A drawing in the workspace and a stamp beside the code then describe the
-- same run, which is the thing that was missing before.
function Pipeline.stamp(result: any): Instance
	local parent = Pipeline.stampParent
	local v = parent:FindFirstChild(Pipeline.stampName)
	if not v or not v:IsA("StringValue") then
		if v then v:Destroy() end
		v = Instance.new("StringValue")
		v.Name = Pipeline.stampName
		v.Parent = parent
	end
	local ok, encoded = pcall(function()
		return game:GetService("HttpService"):JSONEncode({
			at = os.date("!%Y-%m-%dT%H:%M:%SZ"),
			config = result.config,
			loops = result.stats.simplify.loops,
			corners = result.stats.simplify.corners,
			open = result.stats.simplify.open,
		})
	end)
	;(v :: StringValue).Value = ok and encoded or "encode failed"
	return v
end

-- WORST DISTANCE FROM THE RAW BOUNDARY TO THE SIMPLIFIED ONE.
--
-- The number that says whether the polygons still describe the floor. Every
-- other statistic counts things; this one measures error, and a merge that
-- wanders off a floor edge shows up here and nowhere else -- not in the corner
-- count, and not in the raycast validator, because leaving a floor hits nothing.
--
-- MEASURED PER RAW NODE AGAINST THE WHOLE POLYGON, never per edge against a span
-- of raw nodes. Pairing a simplified edge with the raw nodes it covers needs the
-- two to run in the same order, and they do not: where a boundary pinches, the
-- raw walk goes out along a spur and back, and consecutive corners land on raw
-- indices that run backwards. case3 has such a pinch, and the span walk wrapped
-- almost the whole loop there and reported 29.9 studs of error on a 2.5 stud
-- edge. Asking instead how far each raw node sits from the nearest simplified
-- edge needs no correspondence at all and cannot be fooled by one.
--
-- O(raw nodes * corners) per loop, so it is a separate call and not part of run.
function Pipeline.measure(result: any): any
	local worst, where, over = 0, nil, 0
	local step = result.config.bake.step or 0.5
	local limit = step * 1.1
	for _, L in ipairs(result.loops) do
		local poly, pts, n = L.poly, L.pts, #L.pts
		local last = L.closed and n or n - 1
		for _, q in ipairs(poly) do
			local best = math.huge
			for i = 1, last do
				local a = pts[i]
				local d = pts[(i % n) + 1] - a
				local dd = d:Dot(d)
				local t = dd > 1e-12 and math.clamp((q - a):Dot(d) / dd, 0, 1) or 0
				local dist = (q - (a + d * t)).Magnitude
				if dist < best then best = dist end
				if best <= 1e-6 then break end
			end
			if best > limit then over += 1 end
			if best > worst then
				worst = best
				where = ("r%03d loop%d, raw node at (%.1f,%.1f,%.1f)"):format(L.region, L.index, q.X, q.Y, q.Z)
			end
		end
	end
	-- `over` counts RAW NODES beyond a step, not edges
	return { worst = worst, where = where, overStep = over }
end

-- GROUND THICKNESS per cell, written to `cell.thick`, plus a histogram.
--
-- A separate call for now because nothing consumes it yet. It is cheap enough
-- to fold into `run` when the offset lands: 0.4s on case5 against a 32s bake.
function Pipeline.thickness(result: any): (any, any)
	local stats = Thickness.build(result.data)
	return stats, Thickness.histogram(result.data)
end

-- CONNECTIVITY OF THE WALKABLE CELLS, as a snapshot to compare against later.
--
-- A separate call and not part of `run`, for the same reason `measure` is: it
-- costs about two seconds on case5 and nothing downstream needs it yet. What
-- needs it is the offset, which has to be measured across, so the baseline has
-- to be taken BEFORE it and kept.
--
-- `keep` is the filter that makes the second snapshot: given a cell, answer
-- whether it survives. Pass nothing for the baseline.
--
-- NOT FREE, and not cached: with no `keep` this recomputes the very snapshot
-- `Pipeline.portals` has already stored on `result.severance`. Measured at 105s
-- on case6, a seventh of the whole bake, spent twice. Read `result.severance`
-- instead unless a `keep` predicate makes it a genuinely different snapshot.
function Pipeline.connectivity(result: any, keep: ((any) -> boolean)?): any
	return Severance.snapshot(result.data, keep)
end

local function segment(a: Vector3, b: Vector3, thick: number, colour: Color3,
	name: string, parent: Instance)
	local d = b - a
	local len = d.Magnitude
	if len < 1e-4 then return end
	local p = Instance.new("Part")
	p.Anchored = true; p.CanCollide = false; p.CanQuery = false; p.CanTouch = false
	p.Size = Vector3.new(thick, thick, len)
	p.CFrame = CFrame.lookAt(a + d * 0.5, b)
	p.Color = colour
	p.Material = Enum.Material.Neon
	p.Name = name
	p.Parent = parent
end

-- Draw the simplified polygons. Folder names carry region, loop, raw node count
-- and corner count, and say how a loop was closed, so the explorer alone is
-- enough to find a loop worth looking at.
function Pipeline.draw(result: any, opts: any?): Instance
	local o = opts or {}
	local lift = o.lift or 0.3
	local LINE = o.lineColor or Color3.fromRGB(60, 230, 255)
	local CORN = o.cornerColor or Color3.fromRGB(255, 200, 40)
	local MARK = o.closureColor or Color3.fromRGB(60, 255, 120)
	-- A hole is drawn in its own colour rather than annotated in the folder
	-- name alone. Outer and hole is the one distinction that has to be readable
	-- from the camera, because a hole drawn like a rim looks like a second
	-- floor sitting inside the first.
	local HOLE = o.holeColor or Color3.fromRGB(255, 80, 160)

	local old = workspace:FindFirstChild(Pipeline.debugName)
	if old then old:Destroy() end
	local root = Instance.new("Folder")
	root.Name = Pipeline.debugName
	root.Parent = workspace
	local simp = Instance.new("Folder")
	simp.Name = "Simplify"
	simp.Parent = root
	local openF: Folder? = nil

	for _, L in ipairs(result.loops) do
		local pts, up = L.pts, L.up
		local off = up * lift
		local n = #pts
		local base = (L.kind == "hole") and HOLE or LINE
		local f = Instance.new("Folder")
		f.Name = ("r%03d_loop%d_%s_%dto%d%s"):format(L.region, L.index,
			L.kind or "unlabelled", #L.poly, n,
			L.closedBy and ("_CLOSED_" .. L.closedBy) or (L.closed and "" or "_OPEN"))
		-- open loops get their own folder and a prefix, so the Explorer finds
		-- the few that matter among thousands of closed ones
		if L.closed then
			f.Parent = simp
		else
			if not openF then
				openF = Instance.new("Folder")
				openF.Name = "OPEN_LOOPS"
				openF.Parent = root
			end
			f.Name = "OPEN_" .. f.Name
			f.Parent = openF
		end
		local lines = Instance.new("Folder"); lines.Name = "line"; lines.Parent = f
		for i = 1, (L.closed and n or n - 1) do
			-- a merged closure moved the edges either side of its node; an
			-- intersected one added the last edge. Either way they are the
			-- edges this pipeline invented rather than traced.
			local made = (L.closedBy == "merge" and (i == n or i == 1))
				or (L.closedBy ~= nil and L.closedBy ~= "merge" and i == n)
			segment(pts[i] + off, pts[(i % n) + 1] + off, made and 0.2 or 0.14,
				made and MARK or base, made and "closure" or ("seg" .. i), lines)
		end
		local balls = Instance.new("Folder"); balls.Name = "corners"; balls.Parent = f
		for i = 1, n do
			local made = L.closedBy == "merge" and i == 1
			local size = made and 0.5 or 0.3
			local b = Instance.new("Part")
			b.Anchored = true; b.CanCollide = false; b.CanQuery = false; b.CanTouch = false
			b.Shape = Enum.PartType.Ball
			b.Size = Vector3.new(size, size, size)
			b.Color = made and MARK or CORN
			b.Material = Enum.Material.Neon
			b.CFrame = CFrame.new(pts[i] + off)
			b.Name = made and "mergedNode" or ("c" .. i)
			b.Parent = balls
		end
	end
	return root
end

-- Draw the boundary coloured by WHAT EACH EDGE IS MADE OF, not by which loop it
-- belongs to. This is the drawing that says where a portal could go: red is a
-- wall and nothing can cross it, green is floor simply running out, yellow is an
-- edge the simplifier merged across a doorjamb so it is part of each.
--
-- The one thing to look for is green where there is plainly a wall, or red
-- across an opening. Either means the provenance chain is misaligned, and no
-- amount of portal logic downstream will survive it.
function Pipeline.drawEdgeKinds(result: any, opts: any?): (Instance, string)
	local o = opts or {}
	local lift = o.lift or 0.3
	local COLOUR = {
		wall = Color3.fromRGB(255, 60, 60),
		drop = Color3.fromRGB(60, 255, 120),
		edge = Color3.fromRGB(60, 200, 255),
		none = Color3.fromRGB(200, 200, 200),
		mixed = Color3.fromRGB(255, 210, 40),
		invented = Color3.fromRGB(255, 0, 255),
	}

	local old = workspace:FindFirstChild(Pipeline.debugName)
	if old then old:Destroy() end
	local root = Instance.new("Folder")
	root.Name = Pipeline.debugName
	root.Parent = workspace
	local byKind = {}
	local tally = {}

	for _, L in ipairs(result.loops) do
		local pts, up = L.pts, L.up
		local n = #pts
		local off = up * lift
		for i = 1, (L.closed and n or n - 1) do
			local k = (L.edgeKind and L.edgeKind[i]) or "none"
			local f = byKind[k]
			if not f then
				f = Instance.new("Folder")
				f.Name = k
				f.Parent = root
				byKind[k] = f
				tally[k] = 0
			end
			tally[k] += 1
			-- wall edges drawn thinner: what is being looked for is the
			-- openings, and on a closed map the walls are most of the drawing
			segment(pts[i] + off, pts[(i % n) + 1] + off,
				(k == "wall") and 0.10 or 0.18, COLOUR[k] or COLOUR.none,
				("r%03d_l%d_e%d"):format(L.region, L.index, i), f)
		end
	end

	local parts = {}
	for k, v in pairs(tally) do parts[#parts + 1] = ("%s %d"):format(k, v) end
	table.sort(parts)
	return root, table.concat(parts, ", ")
end

-- Every RAW boundary face, coloured by its own verdict.
--
-- THIS IS WHERE THE VERDICT ACTUALLY LIVES, and it is not the line the mesh is
-- drawn on. `drawEdgeKinds` colours the SIMPLIFIED edges, which is a derived
-- curve sitting up to half a stud off these faces and carrying at best one
-- label per edge -- so a run merged across a doorjamb comes back "mixed" and
-- says nothing a portal builder can act on. A face has exactly one kind,
-- because a face is one cell edge. There is no mixed here and there cannot be.
--
-- Drawn at cell resolution, so this is the staircase the simplifier smooths,
-- not the outline. Expect roughly seven times as many segments as the outline
-- has edges, and expect them to zigzag: that is the input, faithfully.
function Pipeline.drawRawFaces(result: any, opts: any?): (Instance, string)
	local o = opts or {}
	local lift = o.lift or 0.3
	local COLOUR = {
		wall = Color3.fromRGB(255, 60, 60),
		step = Color3.fromRGB(255, 170, 40),
		drop = Color3.fromRGB(60, 255, 120),
		ledge = Color3.fromRGB(170, 60, 220),
		edge = Color3.fromRGB(60, 200, 255),
		none = Color3.fromRGB(120, 120, 120),
	}

	local old = workspace:FindFirstChild(Pipeline.debugName)
	if old then old:Destroy() end
	local root = Instance.new("Folder")
	root.Name = Pipeline.debugName
	root.Parent = workspace
	local byKind, tally = {}, {}

	-- Walked through the LOOPS rather than through entry.faces, so only the
	-- faces a traced ring actually uses are drawn. A face the trace never
	-- reached is a defect worth seeing separately, not worth hiding in here.
	local regions = {}
	for r in pairs(result.data.boundary) do regions[#regions + 1] = r end
	table.sort(regions)

	for _, r in ipairs(regions) do
		local entry = result.data.boundary[r]
		for li, L in ipairs(entry.loops) do
			for _, fi in ipairs(L.faces) do
				local f = entry.faces[fi]
				if f and (f.b - f.a).Magnitude > 1e-9 then
					local k = f.kind or "none"
					local folder = byKind[k]
					if not folder then
						folder = Instance.new("Folder")
						folder.Name = k
						folder.Parent = root
						byKind[k] = folder
						tally[k] = 0
					end
					tally[k] += 1
					local off = f.up * lift
					-- wall faces drawn thinner: what is being looked for is the
					-- openings, and on a closed map the walls are most of the
					-- drawing
					segment(f.a + off, f.b + off,
						(k == "wall") and 0.08 or 0.16, COLOUR[k] or COLOUR.none,
						("r%03d_l%d_f%d"):format(r, li, fi), folder)
				end
			end
		end
	end

	local parts, total = {}, 0
	for k, v in pairs(tally) do
		parts[#parts + 1] = ("%s %d"):format(k, v)
		total += v
	end
	table.sort(parts)
	return root, ("raw faces %d: %s"):format(total, table.concat(parts, ", "))
end

-- Cells to rectangle nodes. Cached, like the triangulation, so a draw and a
-- report describe the same graph.
function Pipeline.nodes(result: any, cfg: any?): any
	if not result.nodes then
		result.nodes = Nodes.build(result.data, cfg)
	end
	return result.nodes
end

-- Draw the node graph: a plate over each rectangle, a ball at each centre, and
-- a line along every link.
--
-- THE PLATE IS THE POINT. A ball on its own says where a node is and hides how
-- much floor it speaks for, which is the only thing worth looking at in a
-- decomposition. Plates are sized to the rectangle and tinted by node id, so two
-- nodes that should have been one are visible as a seam.
--
-- Links are drawn white when flat and orange when they climb, so a staircase
-- reads as a chain of orange rungs and a mistaken link across a wall does too.
function Pipeline.drawNodes(result: any, opts: any?): (Instance, string)
	local o = opts or {}
	local lift = o.lift or 0.25
	local res = Pipeline.nodes(result, o.cfg)
	local step = (result.data.config and result.data.config.step) or 0.5

	local old = workspace:FindFirstChild(Pipeline.debugName)
	if old then old:Destroy() end
	local root = Instance.new("Folder")
	root.Name = Pipeline.debugName
	root.Parent = workspace
	local plates = Instance.new("Folder"); plates.Name = "plates"; plates.Parent = root
	local balls = Instance.new("Folder"); balls.Name = "centres"; balls.Parent = root
	local wires = Instance.new("Folder"); wires.Name = "links"; wires.Parent = root

	for _, n in ipairs(res.nodes) do
		local up = n.normal or Vector3.yAxis
		-- The rectangle's own axes, recovered from the grid rather than guessed:
		-- a plate built on a world-aligned frame sits crooked on a rotated slab.
		local g = result.data.grids[n.grid]
		local e1 = g.u or g.e1 or Vector3.xAxis
		local e2 = g.v or g.e2 or up:Cross(e1)
		local centre = n.pos + up * lift
		-- The BALL sits on a real cell, so the plate has to be shifted off it to
		-- the rectangle's geometric middle. On an even-sided rectangle no cell is
		-- at the middle, and drawing the plate around the nearest one leaves it
		-- half a cell out of line with the floor it covers.
		local plateAt = centre
			+ e1 * ((n.midU - n.ui) * step)
			+ e2 * ((n.midV - n.vi) * step)
		local p = Instance.new("Part")
		p.Anchored = true; p.CanCollide = false; p.CanQuery = false; p.CanTouch = false
		p.Size = Vector3.new(n.width, 0.08, n.height)
		p.CFrame = CFrame.fromMatrix(plateAt, e1, up)
		p.Color = Color3.fromHSV((n.id * 0.618034) % 1, 0.55, 1)
		p.Transparency = 0.45
		p.Material = Enum.Material.SmoothPlastic
		p.Name = ("n%04d_r%03d_%dx%d_deg%d")
			:format(n.id, n.region, n.u1 - n.u0 + 1, n.v1 - n.v0 + 1, n.degree)
		p.Parent = plates

		local b = Instance.new("Part")
		b.Anchored = true; b.CanCollide = false; b.CanQuery = false; b.CanTouch = false
		b.Shape = Enum.PartType.Ball
		b.Size = Vector3.new(0.4, 0.4, 0.4)
		b.Color = (n.degree == 0) and Color3.fromRGB(255, 40, 40)
			or Color3.fromRGB(255, 255, 255)
		b.Material = Enum.Material.Neon
		b.CFrame = CFrame.new(centre)
		b.Name = ("n%04d"):format(n.id)
		b.Parent = balls
	end

	local FLAT = Color3.fromRGB(240, 240, 240)
	local RISE = Color3.fromRGB(255, 150, 40)
	for i, L in ipairs(res.links) do
		local a, b = res.nodes[L.a], res.nodes[L.b]
		local ua = (a.normal or Vector3.yAxis) * lift
		local ub = (b.normal or Vector3.yAxis) * lift
		segment(a.pos + ua, b.pos + ub, L.rise > step and 0.14 or 0.09,
			L.rise > step and RISE or FLAT,
			("l%04d_%d_%d_x%d"):format(i, L.a, L.b, L.pairs), wires)
	end

	return root, Nodes.report(res)
end

-- Rings to triangles. Cached on the result so a draw and a report see the same
-- mesh, and so re-running the triangulator is a deliberate act.
function Pipeline.triangulate(result: any): any
	if not result.tri then
		result.tri = Triangulate.build(result.loops)
	end
	return result.tri
end

-- Rings to a convex polygon mesh: constrained Delaunay, Ruppert refinement,
-- then Hertel-Mehlhorn. This is the navmesh; `Pipeline.triangulate` above is the
-- older boundary-only ear clip, kept so the two can be drawn against each other.
--
-- `data` goes in because the Steiner points need the region's FLOOR plane, and
-- only the cells know where that is.
function Pipeline.mesh(result: any): any
	if not result.mesh then
		result.mesh = CDT.build(result.loops, result.data)
		-- Labelled in the same breath as it is built. A mesh whose edges have
		-- not been asked about is one a portal builder would read as all-wall,
		-- and the failure would look like a map with no doors rather than an
		-- error.
		if Pipeline.edgeKind then
			local trees = result.data.localTrees
			if not trees then
				trees = SVOLocal.fromParts(result.data.parts, EdgeKind.leaf, 0.01)
				result.data.localTrees = trees
			end
			local t0 = os.clock()
			result.meshKind = EdgeKind.build(result.mesh, result.data, trees)
			result.meshKind.seconds = os.clock() - t0
		end
	end
	return result.mesh
end

-- Draw the triangles as wireframe plus a ball at each centroid.
--
-- THE CENTROID IS DRAWN BECAUSE IT IS THE SEARCH NODE. Everything downstream
-- treats a triangle as one node at its centre, so a drawing that shows only the
-- edges hides the thing the pathfinder actually walks on. A region's triangles
-- share a colour, picked from the region id, so a surface that came out as two
-- surfaces is visible without opening a single folder.
function Pipeline.drawTriangles(result: any, opts: any?): (Instance, string)
	local o = opts or {}
	local lift = o.lift or 0.35
	-- `cdt` draws the refined convex mesh instead of the ear clip. Same drawing
	-- either way: both emit n-gons with a centroid, and the centroid is the node.
	local tri = o.cdt and Pipeline.mesh(result) or Pipeline.triangulate(result)

	local old = workspace:FindFirstChild(Pipeline.debugName)
	if old then old:Destroy() end
	local root = Instance.new("Folder")
	root.Name = Pipeline.debugName
	root.Parent = workspace
	local folders = {}

	local function hue(r: number): Color3
		-- golden-ratio stride, so consecutive regions are never near-neighbours
		return Color3.fromHSV((r * 0.618034) % 1, 0.65, 1)
	end

	for i, t in ipairs(tri.tris) do
		local f = folders[t.region]
		if not f then
			f = Instance.new("Folder")
			f.Name = ("r%03d"):format(t.region)
			f.Parent = root
			folders[t.region] = f
		end
		local c = hue(t.region)
		local off = t.up * lift
		local g = Instance.new("Folder")
		local verts = t.verts or { t.a, t.b, t.c }
		g.Name = ("f%04d_%dgon_%.1fsq"):format(i, #verts, t.area)
		g.Parent = f
		for k = 1, #verts do
			segment(verts[k] + off, verts[(k % #verts) + 1] + off,
				0.1, c, "e" .. k, g)
		end
		local n = Instance.new("Part")
		n.Anchored = true; n.CanCollide = false; n.CanQuery = false; n.CanTouch = false
		n.Shape = Enum.PartType.Ball
		n.Size = Vector3.new(0.35, 0.35, 0.35)
		n.Color = Color3.fromRGB(255, 255, 255)
		n.Material = Enum.Material.Neon
		n.CFrame = CFrame.new(t.centre + off)
		n.Name = "node"
		n.Parent = g
	end

	return root, (o.cdt and CDT.report or Triangulate.report)(tri, result.loops)
end

-- The convex mesh with every boundary stretch coloured by what is on the other
-- side of it. One folder per kind, so any one of them can be isolated.
--
-- This is the whole point of baking the verdict onto the edges: the mesh a
-- pathfinder walks and the reason it may not leave are the same drawing.
-- `drawRawFaces` shows the same information at cell resolution on a curve that
-- does not line up with the polygons.
-- Draw the mesh SOLID rather than as a wireframe.
--
-- A convex polygon is a fan of triangles about its first vertex, and a triangle
-- is TWO RIGHT-ANGLED WEDGES sharing the foot of an altitude -- the only way to
-- put an arbitrary triangle on screen without an EditableMesh. Rotate the corners
-- so the longest edge is the base, drop the altitude from the apex onto it, and
-- each half is a WedgePart whose sloped face is one half of the triangle.
--
-- Semi-transparent and lifted clear of the floor, because the question this
-- answers is what the mesh COVERS -- which needs the map visible underneath it.
-- One hue per region, as the wireframe uses, so the two can be read together.
--
-- case6: 4243 polygons become 18116 wedges in 1.4 seconds.
function Pipeline.drawFilled(result: any, opts: any?): (Instance, string)
	local o = opts or {}
	local lift = o.lift or 0.25
	local thick = o.thickness or 0.12
	local trans = o.transparency or 0.25
	local mesh = Pipeline.mesh(result)

	local dbg = workspace:FindFirstChild(Pipeline.debugName)
	if not dbg then
		dbg = Instance.new("Folder"); dbg.Name = Pipeline.debugName; dbg.Parent = workspace
	end
	local prev = dbg:FindFirstChild("Filled")
	if prev then prev:Destroy() end
	local root = Instance.new("Folder"); root.Name = "Filled"; root.Parent = dbg

	local made = 0
	local function tri(a: Vector3, b: Vector3, c: Vector3, colour: Color3, parent: Instance)
		local ab, ac, bc = b - a, c - a, c - b
		local d1, d2, d3 = ab:Dot(ab), ac:Dot(ac), bc:Dot(bc)
		-- put the apex opposite the longest edge, so both halves are right-angled
		if d1 > d2 and d1 > d3 then a, c = c, a
		elseif d2 > d1 and d2 > d3 then a, b = b, a end
		ab, ac, bc = b - a, c - a, c - b
		local right = ac:Cross(ab)
		if right.Magnitude < 1e-6 then return end
		right = right.Unit
		local up = bc:Cross(right).Unit
		local back = bc.Unit
		local h = math.abs(ab:Dot(up))
		if h < 1e-4 then return end
		local function wedge(sz: Vector3, cf: CFrame)
			local w = Instance.new("WedgePart")
			w.Anchored = true; w.CanCollide = false; w.CanQuery = false; w.CanTouch = false
			w.Material = Enum.Material.SmoothPlastic
			w.Color = colour; w.Transparency = trans
			w.Size = sz; w.CFrame = cf; w.Parent = parent
			made += 1
		end
		wedge(Vector3.new(thick, h, math.abs(ab:Dot(back))),
			CFrame.fromMatrix((a + b) * 0.5, right, up, back))
		wedge(Vector3.new(thick, h, math.abs(ac:Dot(back))),
			CFrame.fromMatrix((a + c) * 0.5, -right, up, -back))
	end

	local folders = {}
	for _, poly in ipairs(mesh.tris) do
		local r = poly.region or 0
		local f = folders[r]
		if not f then
			f = Instance.new("Folder"); f.Name = ("r%03d"):format(r); f.Parent = root
			folders[r] = f
		end
		local colour = Color3.fromHSV((r * 0.61803398875) % 1, 0.85, 1)
		local v, n = poly.verts, poly.n
		local off = (poly.up or Vector3.yAxis) * lift
		for i = 2, n - 1 do
			tri(v[1] + off, v[i] + off, v[i + 1] + off, colour, f)
		end
	end
	return root, ("filled %d polygons as %d wedges"):format(#mesh.tris, made)
end

function Pipeline.drawMeshKind(result: any, opts: any?): (Instance, string)
	local o = opts or {}
	local lift = o.lift or 0.35
	local mesh = Pipeline.mesh(result)
	local COLOUR = {
		internal = Color3.fromRGB(70, 70, 80),
		wall = Color3.fromRGB(255, 60, 60),
		step = Color3.fromRGB(255, 170, 40),
		drop = Color3.fromRGB(60, 255, 120),
		ledge = Color3.fromRGB(170, 60, 220),
		none = Color3.fromRGB(130, 130, 130),
	}

	local old = workspace:FindFirstChild(Pipeline.debugName)
	if old then old:Destroy() end
	local root = Instance.new("Folder")
	root.Name = Pipeline.debugName
	root.Parent = workspace
	local byKind, tally, len = {}, {}, {}

	for fi, f in ipairs(mesh.tris) do
		local off = f.up * lift
		for j = 1, f.n do
			local k = (f.edgeKind and f.edgeKind[j]) or "none"
			local folder = byKind[k]
			if not folder then
				folder = Instance.new("Folder")
				folder.Name = k
				folder.Parent = root
				byKind[k] = folder
				tally[k], len[k] = 0, 0
			end
			local A = f.verts[j] + off
			local B = f.verts[j % f.n + 1] + off
			tally[k] += 1
			len[k] += (B - A).Magnitude
			-- internal edges drawn thin: they are floor, and what is being
			-- looked at is where the floor stops
			local fall = f.edgeFall and f.edgeFall[j]
			segment(A, B, (k == "internal") and 0.06 or 0.16, COLOUR[k] or COLOUR.none,
				(fall and fall ~= math.huge)
					and ("r%03d_f%03d_e%d_%.2f"):format(f.region, fi, j, fall)
					or ("r%03d_f%03d_e%d"):format(f.region, fi, j),
				folder)
		end
	end

	local parts = {}
	for k, v in pairs(tally) do
		parts[#parts + 1] = ("%s %d (%.0f studs)"):format(k, v, len[k])
	end
	table.sort(parts)
	return root, ("mesh edges: "):format() .. table.concat(parts, ", ")
end

-- Draw two traced boundaries against each other, in two folders and nothing
-- else, clearing the debug root first.
--
-- For the eroded pipeline there is no separate offset polygon to draw: the
-- traced boundary IS the offset boundary, which is the whole point of eroding
-- first. So the comparison is between two TRACES of the same bake -- the raw
-- cells and the eroded ones -- rather than between a polygon and a moved copy
-- of itself.
-- The links that make the polygons a mesh rather than a pile of faces.
--
-- Cached on the result like `Pipeline.mesh`, so a draw and a report describe the
-- same graph. The Severance snapshot is taken here rather than passed in
-- because the pairs must come from the SAME data the mesh was cut from -- a
-- snapshot of anything else would link polygons through cells that are not
-- underneath them.
function Pipeline.portals(result: any): any
	if not result.portals then
		local mesh = Pipeline.mesh(result)
		local snap = result.severance
		if not snap then
			snap = Severance.snapshot(result.data)
			result.severance = snap
		end
		result.portals = Portals.build(mesh, result.data, snap)
		if Pipeline.leaps then
			-- rays must not hit our own drawings, markers or characters
			local ex = {}
			for _, n in ipairs({ Pipeline.debugName, "NVGN_Path", "PathStart", "PathEnd", "NVGN_Follower" }) do
				local x = workspace:FindFirstChild(n)
				if x then ex[#ex + 1] = x end
			end
			local root = result.data.config and result.data.config.root
			if typeof(root) == "Instance" then
				for _, h in ipairs(root:GetDescendants()) do
					if h:IsA("Humanoid") and h.Parent then ex[#ex + 1] = h.Parent end
				end
			end
			Leaps.build(mesh, result.data, result.portals, ex)
		end
		Pipeline.measure_(result)
	end
	return result.portals
end

-- MEASUREMENTS, NOT VERDICTS. One bake serves every NPC profile (see Agents),
-- so each polygon and link carries what is actually there and a profile filters
-- at path time:
--   polygon  slope (deg), headroom (lowest clearance of any cell under it),
--            width (twice the largest distance from a cell under it to a wall or
--            drop: the widest NPC that fits anywhere in it; capped, since open
--            floor has no bound)
--   link     rise (signed height change a -> b), span (usable width), gap
--            (horizontal distance crossed), type walk / step
-- Drop and jump links (one-way) carry the same fields.
Pipeline.widthCap = 64

function Pipeline.measure_(result: any)
	local mesh, res = result.mesh, result.portals
	local data = result.data
	-- WALLS ONLY. Distance to a drop or a step edge would make a 6.5 stud wide
	-- staircase read 1.5 wide (the tread's own edge is 0.75 away) -- and a step
	-- is walked across, not bumped into. A narrow ledge between a wall and a
	-- drop is left to the stuck fallback.
	if not data.wallDistMeasured then
		Thickness.build(data, nil, { seedWallOnly = true, field = "wallDist" })
		data.wallDistMeasured = true
	end
	local head, wide, n = {}, {}, {}
	for cell, pi in pairs(res.polyOf) do
		local c = cell.clearance or math.huge
		if head[pi] == nil or c < head[pi] then head[pi] = c end
		local t = cell.wallDist
		if t and t ~= math.huge then
			if wide[pi] == nil or t > wide[pi] then wide[pi] = t end
		elseif t == math.huge then
			wide[pi] = Pipeline.widthCap
		end
		n[pi] = (n[pi] or 0) + 1
	end
	-- PORTAL CLEARANCE is the width that decides whether a wide NPC gets
	-- through, not a polygon's own width: triangulation cuts open floor into
	-- thin triangles along walls, and a third of case6's polygons measured under
	-- 2 studs by themselves while sitting in wide rooms. Along the portal, the
	-- widest point to cross at is twice the largest wall distance of the cells
	-- under it; a corridor is narrow at every point, a room is not.
	local H = 1.0
	local cellsAt: { [string]: { any } } = {}
	for cell in pairs(res.polyOf) do
		local p = cell.pos
		local k = math.floor(p.X / H) .. ":" .. math.floor(p.Z / H)
		local b = cellsAt[k]
		if not b then b = {}; cellsAt[k] = b end
		b[#b + 1] = cell
	end
	local function clearAt(p: Vector3, reach: number): number
		local best = 0
		local bx, bz = math.floor(p.X / H), math.floor(p.Z / H)
		for dx = -1, 1 do
			for dz = -1, 1 do
				for _, cell in ipairs(cellsAt[(bx + dx) .. ":" .. (bz + dz)] or {}) do
					local d = cell.pos - p
					if Vector3.new(d.X, 0, d.Z).Magnitude <= reach and math.abs(d.Y) < 2.5 then
						local t = cell.wallDist
						if t == math.huge then return Pipeline.widthCap end
						if t and t > best then best = t end
					end
				end
			end
		end
		return best
	end
	for i, f in ipairs(mesh.tris) do
		local up = f.up or Vector3.yAxis
		f.slope = math.deg(math.acos(math.clamp(up.Y, -1, 1)))
		f.headroom = head[i] or 0
		f.width = math.min(Pipeline.widthCap, 2 * (wide[i] or 0))
		f.cells = n[i] or 0
	end
	local step = require(script.Parent:WaitForChild("Agents")).envelope().step
	-- height of a polygon's plane under a point
	local function heightAt(f: any, p: Vector3): number
		local c, up = f.centre, f.up or Vector3.yAxis
		if math.abs(up.Y) < 1e-3 then return c.Y end
		return c.Y - ((p.X - c.X) * up.X + (p.Z - c.Z) * up.Z) / up.Y
	end
	for _, L in ipairs(res.links) do
		-- FROM GEOMETRY, a -> b. The Severance drop's sign follows whichever
		-- polygon was numbered lower, so up a staircase it alternated +1/-1.
		local pa = L.centre
		local pb = (L.bLeft and L.bRight) and (L.bLeft + L.bRight) * 0.5 or L.centre
		L.rise = heightAt(mesh.tris[L.b], pb) - heightAt(mesh.tris[L.a], pa)
		L.gap = L.gap or 0
		-- along the portal, both sides of it
		local best = 0
		local len = (L.right - L.left).Magnitude
		local m = math.max(1, math.floor(len / 0.5))
		for k = 0, m do
			local t = k / m
			best = math.max(best, clearAt(L.left:Lerp(L.right, t), 0.75))
			if L.bLeft and L.bRight then best = math.max(best, clearAt(L.bRight:Lerp(L.bLeft, t), 0.75)) end
			if best >= Pipeline.widthCap then break end
		end
		L.clear = math.min(Pipeline.widthCap, 2 * best)
		local d = math.abs(L.rise)
		L.type = L.type or ((d <= 0.25) and "walk" or (d <= step) and "step" or "steep")
		L.oneWay = L.oneWay or false
	end
end

-- A bar across every portal, plus the polygon centres it joins.
--
-- ONE FOLDER PER KIND, because the three are found by completely different means
-- and a defect in one says nothing about the others: `shared` links are exact
-- polygon edges, `seam` links are fitted through Severance's cross-region cell
-- pairs, and `bridge` links cross floor that was never traced at all. Being able
-- to hide two and look at the third is the whole point.
--
-- Orphan polygons get a marker of their own -- a polygon with no link is a hole
-- in the mesh and should be findable without reading a report.
function Pipeline.drawPortals(result: any, opts: any?): (Instance, string)
	local o = opts or {}
	local lift = o.lift or 0.45
	local res = Pipeline.portals(result)
	local mesh = Pipeline.mesh(result)

	local old = workspace:FindFirstChild(Pipeline.debugName)
	if old then old:Destroy() end
	local root = Instance.new("Folder")
	root.Name = Pipeline.debugName
	root.Parent = workspace

	local function folder(name: string): Folder
		local f = Instance.new("Folder")
		f.Name = name
		f.Parent = root
		return f
	end
	local byKind = {
		shared = folder("shared"),
		seam = folder("seam"),
		bridge = folder("bridge"),
		drop = folder("drop"),
		jump = folder("jump"),
	}

	-- ONE COLOUR PER KIND, chosen clear of the outline drawing (cyan rims, pink
	-- holes, yellow corners, green closures):
	--   blue    shared  -- the exact edge two polygons of one region share
	--   green   seam    -- between regions, flush (height change <= 0.25)
	--   orange  seam    -- between regions, a step up to the largest step
	--   purple  bridge  -- across floor too narrow to trace
	--   red     drop    -- one-way, off an edge
	--   teal    jump    -- one-way, up onto a ledge or across a gap
	--   dark red        a walk link steeper than any step: should not exist
	--   white   ball    -- a polygon with no link at all
	-- A portal with two sides is drawn as TWO bars, the overlap on each edge,
	-- with arrows between them: two for a two-way crossing, one for one-way.
	local SHARED = o.sharedColor or Color3.fromRGB(40, 110, 255)
	local FLUSH  = o.flushColor or Color3.fromRGB(60, 255, 90)
	local STEP   = o.stepColor or Color3.fromRGB(255, 140, 20)
	local BRIDGE = o.bridgeColor or Color3.fromRGB(170, 70, 255)
	local DROP   = o.dropColor or Color3.fromRGB(255, 40, 40)
	local JUMP   = o.jumpColor or Color3.fromRGB(0, 230, 220)
	local STEEP  = o.steepColor or Color3.fromRGB(140, 0, 0)
	local STEPMAX = require(script.Parent:WaitForChild("Agents")).envelope().step

	local function arrow(from: Vector3, to: Vector3, colour: Color3, parent: Instance)
		local d = to - from
		local len = d.Magnitude
		if len < 0.05 then return end
		local head = math.min(0.5, len * 0.45)
		local dir = d / len
		segment(from, to - dir * head * 0.5, 0.07, colour, "shaft", parent)
		-- a V for the head, in the plane that holds the arrow and the world up
		-- (or world X when the arrow is vertical, as a drop's is)
		local side = dir:Cross(math.abs(dir.Y) > 0.9 and Vector3.xAxis or Vector3.yAxis)
		if side.Magnitude < 1e-3 then side = Vector3.zAxis end
		side = side.Unit
		segment(to, to - dir * head + side * head * 0.6, 0.07, colour, "head", parent)
		segment(to, to - dir * head - side * head * 0.6, 0.07, colour, "head", parent)
	end

	for i, L in ipairs(res.links) do
		local into = byKind[L.kind] or byKind.seam
		local up = mesh.tris[L.a].up
		local off = up * lift
		local g = Instance.new("Folder")
		local rise = L.rise or -(L.drop or 0)
		g.Name = ("%sp%04d_f%04d-f%04d_%.1fw_%+.2frise"):format(
			(L.kind ~= "shared" and not (L.bLeft and L.bRight)) and "FITTED_" or "",
			i, L.a, L.b, L.span, rise)
		g.Parent = into

		local d = math.abs(rise)
		local colour
		if L.kind == "shared" then colour = SHARED
		elseif L.kind == "drop" then colour = DROP
		elseif L.kind == "jump" then colour = JUMP
		elseif d > STEPMAX then colour = STEEP
		elseif L.kind == "bridge" then colour = BRIDGE
		else colour = (d <= 0.25) and FLUSH or STEP end

		local function bar(a: Vector3, b: Vector3, name: string)
			if (b - a).Magnitude > 1e-4 then
				segment(a + off, b + off, 0.2, colour, name, g)
			else
				-- a one-cell-pair portal has no width to draw, and a missing bar
				-- would read as a missing link
				local ball = Instance.new("Part")
				ball.Anchored = true; ball.CanCollide = false; ball.CanQuery = false; ball.CanTouch = false
				ball.Shape = Enum.PartType.Ball
				ball.Size = Vector3.new(0.3, 0.3, 0.3)
				ball.Color = colour; ball.Material = Enum.Material.Neon
				ball.CFrame = CFrame.new(a + off)
				ball.Name = name
				ball.Parent = g
			end
		end
		bar(L.left, L.right, "gateA")
		if L.bLeft and L.bRight then
			bar(L.bLeft, L.bRight, "gateB")
			-- L.left pairs with L.bRight and L.right with L.bLeft (B's edge runs the
			-- other way), so the point at fraction t on A faces 1 - t on B
			local function at(t: number): (Vector3, Vector3)
				return L.left:Lerp(L.right, t) + off, L.bRight:Lerp(L.bLeft, t) + off
			end
			if L.oneWay then
				local pa, pb = at(0.5)
				if L.over then
					-- FORWARD, THEN DOWN (or up, then across, for a jump-up): the way
					-- the NPC actually goes, so the arrow never cuts through the ledge
					local ov = L.over + off
					segment(pa, ov, 0.07, colour, "shaft", g)
					arrow(ov, pb, colour, g)
				else
					arrow(pa, pb, colour, g)
				end
			else
				local pa, pb = at(1 / 3)
				arrow(pa, pb, colour, g)
				local qa, qb = at(2 / 3)
				arrow(qb, qa, colour, g)
			end
		end
	end

	local orphans = 0
	local fOrphan: Folder? = nil
	local degree = {}
	for _, L in ipairs(res.links) do
		degree[L.a] = true; degree[L.b] = true
	end
	for i, f in ipairs(mesh.tris) do
		if not degree[i] then
			if not fOrphan then fOrphan = folder("NO_LINK_POLYGONS") end
			orphans += 1
			local b = Instance.new("Part")
			b.Anchored = true; b.CanCollide = false; b.CanQuery = false; b.CanTouch = false
			b.Shape = Enum.PartType.Ball
			b.Size = Vector3.new(0.9, 0.9, 0.9)
			b.Color = o.orphanColor or Color3.fromRGB(255, 255, 255)
			b.Material = Enum.Material.Neon
			b.CFrame = CFrame.new(f.centre + f.up * lift)
			-- searchable: type NOLINK in the Explorer filter
			b.Name = ("NOLINK_f%04d_r%03d_%.0fsq"):format(i, f.region, f.area or 0)
			b.Parent = fOrphan
		end
	end

	return root, Portals.report(res)
end

-- QUALITY OF THE PORTALS, one line per defect class. Reports only.
--   fragmented  more than one gate of one kind between the same two polygons
--   off polygon a gate more than 1 stud from one of the polygons it joins
--   along       a gate within 18 degrees of pointing the way you walk
--   blocked     a chest-height ray from the gate to either polygon's centre
--               hits geometry (2 studs up, so step risers do not count)
--   narrow      under 1 stud wide -- description, not a defect
function Pipeline.auditPortals(result: any): string
	local mesh, links = Pipeline.mesh(result), Pipeline.portals(result).links
	local rp = RaycastParams.new()
	rp.FilterType = Enum.RaycastFilterType.Exclude
	local dbg = workspace:FindFirstChild(Pipeline.debugName)
	rp.FilterDescendantsInstances = dbg and { dbg } or {}
	local function distToPoly(P: any, q: Vector3): number
		local v, best = P.verts, math.huge
		for i = 1, #v do
			local a, b = v[i], v[i % #v + 1]
			local d = b - a
			local dd = d:Dot(d)
			local t = dd > 1e-9 and math.clamp((q - a):Dot(d) / dd, 0, 1) or 0
			best = math.min(best, (q - (a + d * t)).Magnitude)
		end
		return best
	end
	local byPair, kinds = {}, {}
	local far, along, blocked, narrow, point = {}, {}, {}, 0, 0
	local edgeN, fitN = 0, 0
	for i, L in ipairs(links) do
		kinds[L.kind] = (kinds[L.kind] or 0) + 1
		if L.kind ~= "shared" then
			if L.edge then edgeN += 1 else fitN += 1 end
		end
		local k = L.kind .. ":" .. L.a .. ":" .. L.b
		local t = byPair[k]
		if not t then t = {}; byPair[k] = t end
		t[#t + 1] = i
		local A, B = mesh.tris[L.a], mesh.tris[L.b]
		local dA = math.min(distToPoly(A, L.left), distToPoly(A, L.right), distToPoly(A, L.centre))
		local dB = math.min(distToPoly(B, L.left), distToPoly(B, L.right), distToPoly(B, L.centre))
		if math.max(dA, dB) > 1.0 then far[#far + 1] = { math.max(dA, dB), i } end
		if L.span > 0.5 then
			local tr = B.centre - A.centre
			tr -= A.up * tr:Dot(A.up)
			if tr.Magnitude > 1e-3 and math.abs((L.right - L.left).Unit:Dot(tr.Unit)) > 0.95 then
				along[#along + 1] = i
			end
		else
			point += 1
		end
		if L.span < 1.0 then narrow += 1 end
		local c = L.centre + A.up * 2.0
		local h = workspace:Raycast(c, (A.centre + A.up * 2.0) - c, rp)
			or workspace:Raycast(c, (B.centre + B.up * 2.0) - c, rp)
		if h then blocked[#blocked + 1] = ("p%04d(%s)"):format(i, h.Instance.Name) end
	end
	local multi, extra, worst = 0, 0, {}
	for k, t in pairs(byPair) do
		if #t > 1 then
			multi += 1
			extra += #t - 1
			worst[#worst + 1] = { #t, k }
		end
	end
	table.sort(worst, function(x, y) return x[1] > y[1] end)
	table.sort(far, function(x, y) return x[1] > y[1] end)
	local ws = {}
	for i = 1, math.min(6, #worst) do ws[#ws + 1] = ("%s x%d"):format(worst[i][2], worst[i][1]) end
	local bs = {}
	for i = 1, math.min(8, #blocked) do bs[#bs + 1] = blocked[i] end
	return table.concat({
		("links     %d (shared %d, seam %d, bridge %d); of seam+bridge %d on polygon edges, %d still fitted")
			:format(#links, kinds.shared or 0, kinds.seam or 0, kinds.bridge or 0, edgeN, fitN),
		("fragment  %d polygon pairs with more than one gate of one kind, %d extra gates; worst %s")
			:format(multi, extra, table.concat(ws, ", ")),
		("off poly  %d gates more than 1 stud from a polygon they join%s")
			:format(#far, #far > 0 and (", worst p%04d at %.1f studs"):format(far[1][2], far[1][1]) or ""),
		("along     %d gates within 18 deg of the direction of travel"):format(#along),
		("blocked   %d gates with geometry between them and a polygon centre at chest height%s")
			:format(#blocked, #bs > 0 and (": " .. table.concat(bs, " ")) or ""),
		("narrow    %d gates under 1 stud wide, %d of them single cell pairs"):format(narrow, point),
	}, "\n")
end

-- ATTRIBUTE A STRAIGHTENED RING'S EDGES BACK TO THE LOOP'S OWN EDGES.
--
-- CDT tests the ring AFTER `Triangulate.straighten`, so ring edge `i` is NOT
-- loop edge `i`: straighten deletes collinear corners and every deletion shifts
-- the rest. It only ever deletes, though, so each ring point is still one of
-- `L.pts`, and matching on the float is exact rather than approximate -- these
-- are the same numbers, not nearby ones. That is the same rule
-- `Pipeline.simplify` uses to realign provenance across the closing pass, and it
-- is safe for the same reason and no other.
--
-- A ring edge spans one or more loop edges, so worst wins, as in `edgeKinds`.
local function ringRepair(L: any, src: { Vector3 }): { string }
	local rep = {}
	local nSrc, nPts = #src, #L.pts
	local er = L.edgeRepair
	if not er then
		for i = 1, nSrc do rep[i] = "?" end
		return rep
	end
	local at = {}
	for k, p in ipairs(L.pts) do at[tostring(p)] = k end
	for i = 1, nSrc do
		local a0 = at[tostring(src[i])]
		local b0 = at[tostring(src[i % nSrc + 1])]
		if not a0 or not b0 then
			rep[i] = "?"
			continue
		end
		local best = "none"
		local j = a0
		for _ = 1, nPts do
			if j == b0 then break end
			local r = er[j]
			if r == "bridge" then best = "bridge"; break end
			if r == "weld" then best = "weld"
			elseif r == "invented" and best == "none" then best = "invented" end
			j = (j % nPts) + 1
		end
		rep[i] = best
	end
	return rep
end

-- WHERE A RING CROSSES ITSELF, drawn so it can be found.
--
-- CDT already says this out loud -- "r045 ring1: edge 3 crosses edge 11, the
-- ring is not simple" -- but a complaint naming two edge numbers on a ring of
-- several hundred corners is not something anyone can find in a workspace. 62 of
-- case6's 97 complaints are that one sentence, and a crossing ring costs real
-- output: `insertSegment` resolves a crossing by flipping, a constrained edge is
-- never flipped, so the hole is never cut and polygons end up over ground their
-- own cells do not claim.
--
-- THE TEST IS CDT'S OWN, OVER CDT'S OWN PROJECTION: the same outer-and-holes
-- grouping, the same `Rings.basis` off the outer rim's up, the same origin, the
-- same `Triangulate.straighten`, the same coincident-corner dedupe and the same
-- ring order -- so `ring1` here is `ring1` in the complaint and the edge numbers
-- agree. Reimplementing any of it would draw a different set of crossings from
-- the one being complained about, which is worse than drawing none: the marker
-- would be somewhere the mesh never objected to.
--
-- It does NOT build a mesh. A crossing is a property of the ring alone, so this
-- reads `result.loops` straight off `Pipeline.run`.
--
-- Its own subfolder, cleared on its own, because the question is where the
-- crossings sit relative to the polygons -- which needs the mesh drawing left
-- standing.
function Pipeline.drawCrossings(result: any, opts: any?): (Instance, string)
	local o = opts or {}
	local lift = o.lift or 0.5
	local RING = o.ringColor or Color3.fromRGB(120, 120, 130)
	local HOT = o.crossColor or Color3.fromRGB(255, 45, 45)
	local MARK = o.markColor or Color3.fromRGB(255, 230, 40)
	local REPAIR = {
		bridge = Color3.fromRGB(255, 0, 255),
		weld = Color3.fromRGB(255, 150, 40),
		invented = Color3.fromRGB(60, 230, 255),
	}
	local tally = {}

	local loops = result.loops
	if not loops then return workspace, "no loops on this result" end

	local function cross2(ax: number, ay: number, bx: number, by: number,
		cx: number, cy: number): number
		return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
	end

	local root = workspace:FindFirstChild(Pipeline.debugName)
	if not root then
		root = Instance.new("Folder")
		root.Name = Pipeline.debugName
		root.Parent = workspace
	end
	local name = o.name or "crossings"
	local prev = root:FindFirstChild(name)
	if prev then prev:Destroy() end
	local out = Instance.new("Folder")
	out.Name = name
	out.Parent = root

	local byRegion, order = {}, {}
	for i, L in ipairs(loops) do
		if not byRegion[L.region] then
			byRegion[L.region] = {}
			order[#order + 1] = L.region
		end
		local g = byRegion[L.region]
		g[#g + 1] = i
	end

	local nCross, nRings, nRegions = 0, 0, 0
	local found = {}

	for _, r in ipairs(order) do
		local idxs = byRegion[r]
		local outer, holes = nil, {}
		for _, i in ipairs(idxs) do
			local L = loops[i]
			if L.kind == "outer" then
				if outer then outer = false elseif outer == nil then outer = i end
			elseif L.kind == "hole" then
				holes[#holes + 1] = i
			end
		end
		-- A region CDT never meshes is a region it never tested, so it is not
		-- tested here either. This draws what CDT complains about, not what it
		-- would have complained about had it got that far.
		if outer == nil or outer == false then continue end

		local up = loops[outer].regionUp or loops[outer].up
		local e1, e2 = Rings.basis(up)
		local origin = loops[outer].pts[1]
		local rise = up * lift

		local ringIdx, ringRep, uniq, seen = {}, {}, {}, {}
		local function addRing(L: any)
			local src = (CDT.collinear > 0)
				and Triangulate.straighten(L.pts, CDT.collinear) or L.pts
			local ring = {}
			for _, p in ipairs(src) do
				local d = p - origin
				local x, y = d:Dot(e1), d:Dot(e2)
				local key = ("%.5f,%.5f"):format(x, y)
				local k = seen[key]
				if not k then
					uniq[#uniq + 1] = { x, y, p }
					k = #uniq
					seen[key] = k
				end
				ring[#ring + 1] = k
			end
			ringIdx[#ringIdx + 1] = ring
			-- Computed per ring, against that ring's own loop. `uniq` is shared
			-- across the region's rings and would hand a shared corner whichever
			-- ring reached it first.
			ringRep[#ringIdx] = ringRepair(L, src)
		end
		addRing(loops[outer])
		for _, i in ipairs(holes) do addRing(loops[i]) end

		local hits, bad = {}, {}
		for ri, ring in ipairs(ringIdx) do
			local n = #ring
			for i = 1, n do
				for j = i + 2, n do
					if not (i == 1 and j == n) then
						local a, b = uniq[ring[i]], uniq[ring[i % n + 1]]
						local c, d = uniq[ring[j]], uniq[ring[j % n + 1]]
						local d1 = cross2(c[1], c[2], d[1], d[2], a[1], a[2])
						local d2 = cross2(c[1], c[2], d[1], d[2], b[1], b[2])
						local d3 = cross2(a[1], a[2], b[1], b[2], c[1], c[2])
						local d4 = cross2(a[1], a[2], b[1], b[2], d[1], d[2])
						if ((d1 > 0) ~= (d2 > 0)) and ((d3 > 0) ~= (d4 > 0)) then
							-- The meeting point, not a midpoint: `d3` and `d4`
							-- are the signed areas that just disagreed, so the
							-- zero between them lands exactly on AB.
							local rep = ringRep[ri]
							hits[#hits + 1] = { ri, i, j, a, b, c, d,
								d3 / (d3 - d4),
								rep and rep[i] or "?", rep and rep[j] or "?" }
							bad[ri] = (bad[ri] or 0) + 1
						end
					end
				end
			end
		end
		if #hits == 0 then continue end

		nRegions += 1
		local fr = Instance.new("Folder")
		fr.Name = ("r%03d_x%d"):format(r, #hits)
		fr.Parent = out

		for ri = 1, #ringIdx do
			local cnt = bad[ri]
			if not cnt then continue end
			nRings += 1
			local ring = ringIdx[ri]
			local n = #ring
			local fring = Instance.new("Folder")
			fring.Name = ("ring%d_%dpts_x%d"):format(ri, n, cnt)
			fring.Parent = fr

			-- The whole ring in grey first, so the two red edges are read in the
			-- context of the loop they belong to rather than as a floating X.
			for i = 1, n do
				segment(uniq[ring[i]][3] + rise, uniq[ring[i % n + 1]][3] + rise,
					0.08, RING, ("e%d"):format(i), fring)
			end

			for _, h in ipairs(hits) do
				if h[1] ~= ri then continue end
				nCross += 1
				local i, j = h[2], h[3]
				local a, b, c, d, t = h[4], h[5], h[6], h[7], h[8]
				local repI, repJ = h[9], h[10]
				-- The colour IS the finding: a magenta edge is a bridge chord, an
				-- orange one a welded seam, cyan a corner the closing pass
				-- invented, red an edge real faces stand behind.
				segment(a[3] + rise, b[3] + rise, 0.20, REPAIR[repI] or HOT,
					("x_e%d_%s"):format(i, repI), fring)
				segment(c[3] + rise, d[3] + rise, 0.20, REPAIR[repJ] or HOT,
					("x_e%d_%s"):format(j, repJ), fring)
				-- Worst of the two, because one invented chord is enough to
				-- explain a crossing the other edge merely met.
				local worst = (repI == "bridge" or repJ == "bridge") and "bridge"
					or (repI == "weld" or repJ == "weld") and "weld"
					or (repI == "invented" or repJ == "invented") and "invented"
					or (repI == "?" or repJ == "?") and "?"
					or "none"
				tally[worst] = (tally[worst] or 0) + 1

				local px = c[1] + (d[1] - c[1]) * t
				local py = c[2] + (d[2] - c[2]) * t
				local m = Instance.new("Part")
				m.Anchored = true; m.CanCollide = false
				m.CanQuery = false; m.CanTouch = false
				m.Shape = Enum.PartType.Ball
				m.Size = Vector3.new(1.2, 1.2, 1.2)
				m.Color = MARK
				m.Material = Enum.Material.Neon
				m.CFrame = CFrame.new(origin + e1 * px + e2 * py + rise)
				-- Named as CDT words the complaint, so the sentence in the
				-- report can be pasted into the Explorer's search box. The
				-- attribution is a SUFFIX for that reason -- a substring search
				-- for the complaint still finds it.
				m.Name = ("r%03d_ring%d_e%d_x_e%d_%s"):format(r, ri, i, j, worst)
				m.Parent = fring
				found[#found + 1] = m.Name
			end
		end
	end

	local head = ("%d crossings on %d rings in %d regions")
		:format(nCross, nRings, nRegions)
	if nCross == 0 then
		return out, head .. " -- the rings are simple, nothing drawn"
	end
	-- THE ATTRIBUTION IS THE POINT OF THE DRAWING, so it leads the summary.
	local by = {}
	for _, k in ipairs({ "bridge", "weld", "invented", "none", "?" }) do
		if tally[k] then by[#by + 1] = ("%s %d"):format(k, tally[k]) end
	end
	return out, head .. "  [" .. table.concat(by, ", ") .. "]\n"
		.. table.concat(found, "\n")
end

-- EVERY RING TESTED FOR SELF-CROSSING, INDEPENDENTLY OF CLASSIFICATION.
--
-- `drawCrossings` mirrors CDT exactly, which means it inherits CDT's blind spot:
-- CDT only tests a region that resolved to exactly ONE outer rim, and it only
-- tests the rings it meshes. `Rings.classify` decides `outer` against `hole` on
-- SIGNED AREA ALONE, and a self-crossing ring's two lobes carry opposite sign and
-- partly cancel -- so a crossing ring can come back negative and be filed
-- `hole`, or cancel below `Rings.minArea` and be filed `degenerate` and dropped.
-- Neither raises a complaint. A ring that disappears that way is invisible to
-- CDT, to the complaint list and to the drawing alike.
--
-- So this asks the question the other way round: test every CLOSED ring there
-- is, then report what each crossing ring was labelled and whether its region
-- was meshed at all. It says whether the 62 CDT reports is the number or a
-- fraction of it.
--
-- REPORTS ONLY. It must not change how anything is classified -- if the answer
-- is that classification is hiding rings, that is a finding to act on
-- deliberately, not something to paper over from a debug call.
--
-- Uses each ring's OWN `up` for the projection rather than the region basis,
-- because a ring filed `degenerate` may be the only ring its region has and
-- there is then no outer rim to take a basis from. Self-crossing is a property
-- of the ring in its own plane, so this is the honest frame for the question.
function Pipeline.auditCrossings(result: any): any
	local loops = result.loops
	if not loops then return { error = "no loops on this result" } end

	-- `CDT.build` returns its convex polygons under `tris`, not `polys`; the name
	-- predates the merge step that stopped them being triangles.
	local meshed = {}
	if result.mesh and result.mesh.tris then
		for _, p in ipairs(result.mesh.tris) do meshed[p.region] = true end
	end

	local out = { rings = 0, tested = 0, crossing = 0, pairs_ = 0,
		byKind = {}, unmeshed = 0, worst = nil, regions = {} }

	for _, L in ipairs(loops) do
		out.rings += 1
		local kind = L.kind or "unclassified"
		if not L.closed or #L.pts < 4 then continue end
		out.tested += 1

		local up = L.regionUp or L.up
		local e1, e2 = Rings.basis(up)
		local origin = L.pts[1]
		local src = (CDT.collinear > 0)
			and Triangulate.straighten(L.pts, CDT.collinear) or L.pts
		local xs, ys = {}, {}
		for i, p in ipairs(src) do
			local d = p - origin
			xs[i], ys[i] = d:Dot(e1), d:Dot(e2)
		end

		local n = #src
		local hits = 0
		for i = 1, n do
			for j = i + 2, n do
				if not (i == 1 and j == n) then
					local i2, j2 = i % n + 1, j % n + 1
					local d1 = (xs[j2] - xs[j]) * (ys[i] - ys[j]) - (ys[j2] - ys[j]) * (xs[i] - xs[j])
					local d2 = (xs[j2] - xs[j]) * (ys[i2] - ys[j]) - (ys[j2] - ys[j]) * (xs[i2] - xs[j])
					local d3 = (xs[i2] - xs[i]) * (ys[j] - ys[i]) - (ys[i2] - ys[i]) * (xs[j] - xs[i])
					local d4 = (xs[i2] - xs[i]) * (ys[j2] - ys[i]) - (ys[i2] - ys[i]) * (xs[j2] - xs[i])
					if ((d1 > 0) ~= (d2 > 0)) and ((d3 > 0) ~= (d4 > 0)) then
						hits += 1
					end
				end
			end
		end

		if hits > 0 then
			out.crossing += 1
			out.pairs_ += hits
			out.byKind[kind] = (out.byKind[kind] or 0) + 1
			if not meshed[L.region] then out.unmeshed += 1 end
			out.regions[#out.regions + 1] =
				("r%03d l%d %s %dpts x%d%s"):format(L.region, L.index, kind,
					#src, hits, meshed[L.region] and "" or " UNMESHED")
		end
	end
	return out
end

-- WHAT KIND OF DEFECT EACH SELF-CROSSING RING IS, in one table.
--
-- Every fact below was already being computed and none of it was ever read.
-- `closedBy` has been stored per loop since the closing pass was written;
-- `merge` and `dejog` each return a refusal count that `Pipeline.simplify` threw
-- away. The 62 crossings were treated as one defect through two failed passes
-- because nothing ever asked whether they were the same SHAPE of defect.
--
-- They are not. Measured off the drawing, three signatures separate cleanly:
--
--   A  a tiny steep fragment, 2-3 studs across at 20-37 degrees, whose ring is
--      longer than the patch perimeter -- a scribble, not an outline
--   B  a one-cell notch on a huge FLAT region: two ~0.5 stud edges then a long
--      run back across them, repeating. `dejog` exists to remove exactly this
--      and `jogMax` is 1.5, so it SAW these and refused them
--   C  a closure chord: `close` joins two ends with an absolute cap that is
--      reasonable on an 800 stud rim and is a third of the whole ring on a small
--      one. r2370's crossing edge is 3.00 studs against `closeMaxGap` of 3.0
--
-- REPORTS ONLY. It resolves nothing; the point is to stop guessing which fix is
-- worth writing.
function Pipeline.auditRings(result: any): any
	local loops = result.loops
	if not loops then return { error = "no loops on this result" } end

	local byRegion = {}
	for _, L in ipairs(loops) do
		local g = byRegion[L.region]
		if not g then g = {}; byRegion[L.region] = g end
		g[#g + 1] = L
	end

	local rows, tot = {}, { rings = 0, crossings = 0,
		closedBy = {}, jogRefused = 0, mergeRefused = 0, rescued = 0 }

	for _, L in ipairs(loops) do
		if not L.closed or #L.pts < 4 then continue end
		local up = L.regionUp or L.up
		local e1, e2 = Rings.basis(up)
		local origin = L.pts[1]
		local src = (CDT.collinear > 0)
			and Triangulate.straighten(L.pts, CDT.collinear) or L.pts
		local xs, ys = {}, {}
		for i, p in ipairs(src) do
			local d = p - origin
			xs[i], ys[i] = d:Dot(e1), d:Dot(e2)
		end
		local n = #src
		local hits = 0
		for i = 1, n do
			for j = i + 2, n do
				if not (i == 1 and j == n) then
					local i2, j2 = i % n + 1, j % n + 1
					local d1 = (xs[j2] - xs[j]) * (ys[i] - ys[j]) - (ys[j2] - ys[j]) * (xs[i] - xs[j])
					local d2 = (xs[j2] - xs[j]) * (ys[i2] - ys[j]) - (ys[j2] - ys[j]) * (xs[i2] - xs[j])
					local d3 = (xs[i2] - xs[i]) * (ys[j] - ys[i]) - (ys[i2] - ys[i]) * (xs[j] - xs[i])
					local d4 = (xs[i2] - xs[i]) * (ys[j2] - ys[i]) - (ys[i2] - ys[i]) * (xs[j2] - xs[i])
					if ((d1 > 0) ~= (d2 > 0)) and ((d3 > 0) ~= (d4 > 0)) then hits += 1 end
				end
			end
		end
		if hits == 0 then continue end

		-- Extent and slope in WORLD axes, the same frame the drawing was
		-- measured in, so a row here and a folder in the workspace agree.
		local lo, hi = L.pts[1], L.pts[1]
		local perim, longest = 0, 0
		local m = #L.pts
		for i = 1, m do
			lo = lo:Min(L.pts[i]); hi = hi:Max(L.pts[i])
			local len = (L.pts[i % m + 1] - L.pts[i]).Magnitude
			perim += len
			if len > longest then longest = len end
		end
		local d = hi - lo
		local flat = math.max(d.X, d.Z)
		local slope = math.deg(math.atan2(d.Y, math.max(flat, 1e-6)))
		-- A box around the ring is the SHORTEST any honest outline of it could
		-- be, so perimeter over box perimeter above ~1.5 means the ring doubles
		-- back on itself rather than going round.
		local box = 2 * (d.X + d.Z)
		local waste = box > 1e-6 and perim / box or 0

		local jog = L.simp and L.simp.jog
		local mrg = L.simp and L.simp.merge
		local jr = jog and (jog.refusedDrift + jog.refusedRay) or 0
		local mr = mrg and (mrg.refusedDrift + mrg.refusedRay) or 0
		tot.jogRefused += jr
		tot.mergeRefused += mr
		tot.rings += 1
		tot.crossings += hits
		local cb = L.closedBy or "traced"
		tot.closedBy[cb] = (tot.closedBy[cb] or 0) + 1

		rows[#rows + 1] = { hits, ("r%03d x%-2d %3dpts raw%-5d %6.1fx%-6.1f slope%3.0f  perim%7.1f waste%4.1f  longest%6.2f (%2.0f%%)  closedBy:%-10s jogRef %d/%d  mergeRef %d/%d")
			:format(L.region, hits, m, #L.poly, flat, math.max(d.X, d.Z) == d.X and d.Z or d.X,
				slope, perim, waste, longest, 100 * longest / math.max(perim, 1e-6),
				cb, jr, jog and jog.input or 0, mr, mrg and mrg.input or 0) }
	end

	table.sort(rows, function(a, b) return a[1] > b[1] end)
	local out = {}
	for _, r in ipairs(rows) do out[#out + 1] = r[2] end
	local cbs = {}
	for k, v in pairs(tot.closedBy) do cbs[#cbs + 1] = ("%s=%d"):format(k, v) end
	table.sort(cbs)
	return { rows = out, totals = ("%d rings, %d crossings | closedBy[%s] | jog refusals %d, merge refusals %d")
		:format(tot.rings, tot.crossings, table.concat(cbs, " "),
			tot.jogRefused, tot.mergeRefused) }
end

-- WHAT A HIGHER `traceMinWidth` WOULD COST, before anyone sets one.
--
-- `Boundary.liveRegions` keeps a region only if a solid k-by-k square of lattice
-- slots fits inside it, k = ceil(traceMinWidth / step). At the shipped
-- minWidth of 2 and step 0.5 that is k = 4, a 2x2 stud square -- which a ragged
-- 2.4 stud scribble passes by a hair while being useless to an agent.
--
-- Raising it DELETES FLOOR, so this says how much, per candidate value, without
-- changing anything. Counted on the finished rings rather than the lattice: a
-- region whose outer rim's shorter extent is under the candidate would not have
-- survived the gate.
function Pipeline.traceWidthCost(result: any, values: {number}?): any
	local loops = result.loops
	if not loops then return { error = "no loops on this result" } end
	local vals = values or { 2.0, 2.5, 3.0, 3.5 }

	local outerOf = {}
	for _, L in ipairs(loops) do
		if L.kind == "outer" then
			local r = L.region
			if outerOf[r] == nil then outerOf[r] = L else outerOf[r] = false end
		end
	end

	local lines = {}
	for _, v in ipairs(vals) do
		local drop, area, crossDrop = 0, 0, 0
		for r, L in pairs(outerOf) do
			if L then
				-- IN THE REGION'S OWN PLANE, not world X/Z. A ramp or a
				-- wall-like surface has a small world footprint on one axis
				-- while being wide on the surface an agent actually walks, and
				-- measuring it in world axes reported those as narrow.
				local e1, e2 = Rings.basis(L.regionUp or L.up)
				local o = L.pts[1]
				local loU, hiU, loV, hiV = math.huge, -math.huge, math.huge, -math.huge
				for _, p in ipairs(L.pts) do
					local d = p - o
					local u, w = d:Dot(e1), d:Dot(e2)
					if u < loU then loU = u end
					if u > hiU then hiU = u end
					if w < loV then loV = w end
					if w > hiV then hiV = w end
				end
				if math.min(hiU - loU, hiV - loV) < v then
					drop += 1
					area += math.abs(L.area or 0)
				end
			end
		end
		lines[#lines + 1] = ("traceMinWidth %.1f -> drops %d regions, %.0f sq studs"):format(v, drop, area)
	end
	return table.concat(lines, "\n")
end

function Pipeline.drawCompare(rawLoops: {any}, cutLoops: {any}, opts: any?): (Instance, string)
	local o = opts or {}
	local lift = o.lift or 0.3
	local OLD = o.oldColor or Color3.fromRGB(130, 130, 140)
	local NEW = o.newColor or Color3.fromRGB(60, 235, 255)

	-- `o.name` puts this comparison in its own subfolder and clears only that
	-- one, so two maps can be on screen at once. Without it the debug root is
	-- wiped, which is what you want when re-running a single map.
	local old = workspace:FindFirstChild(Pipeline.debugName)
	if not o.name then
		if old then old:Destroy() end
		old = nil
	end
	if not old then
		old = Instance.new("Folder")
		old.Name = Pipeline.debugName
		old.Parent = workspace
	end
	local root = old
	if o.name then
		local prev = old:FindFirstChild(o.name)
		if prev then prev:Destroy() end
		root = Instance.new("Folder")
		root.Name = o.name
		root.Parent = old
	end
	local fOld = Instance.new("Folder"); fOld.Name = "original"; fOld.Parent = root
	local fNew = Instance.new("Folder"); fNew.Name = "offset"; fNew.Parent = root

	local function paint(loops, colour, thick, parent)
		local n = 0
		for _, L in ipairs(loops) do
			local pts, up = L.pts, L.up
			local c = #pts
			local rise = up * lift
			for i = 1, (L.closed and c or c - 1) do
				segment(pts[i] + rise, pts[(i % c) + 1] + rise, thick, colour,
					("r%03d_l%d_e%d"):format(L.region, L.index, i), parent)
				n += 1
			end
		end
		return n
	end
	local a = paint(rawLoops, OLD, 0.10, fOld)
	local b = paint(cutLoops, NEW, 0.16, fNew)
	return root, ("original %d edges (grey), offset %d edges (cyan)"):format(a, b)
end

-- Draw the global SVO's solid space -- the voxels the wall test actually asks
-- about when nothing in the cell grids knows.
--
-- EACH LEAF AT ITS OWN SIZE. The octree collapses a fully solid region into one
-- big node, and expanding those back down to leaf size is how a single baseplate
-- node becomes 32768 parts. Drawn as they are stored, case5 is 52439 boxes
-- instead.
--
-- Semi-transparent, and added ALONGSIDE the offset drawing rather than clearing
-- it, because the question being asked is where solid space sits relative to the
-- boundary lines -- which needs both visible at once.
function Pipeline.drawSolid(result: any, opts: any?): (Instance, string)
	local o = opts or {}
	local maxParts = o.maxParts or 20000
	local svo = result.data and result.data.svo
	if not svo then return workspace, "no SVO on this bake" end

	local old = workspace:FindFirstChild(Pipeline.debugName)
	if not old then
		old = Instance.new("Folder")
		old.Name = Pipeline.debugName
		old.Parent = workspace
	end
	local prev = old:FindFirstChild("solid")
	if prev then prev:Destroy() end
	local root = Instance.new("Folder")
	root.Name = "solid"
	root.Parent = old

	-- Optionally keep only what sits in the band the wall probe reads, which is
	-- the band that decides wall against dropoff. Everything else is scenery.
	local band = o.nearFloor
	local lookup
	if band then
		local hash = {}
		for _, g in ipairs(result.data.grids) do
			for _, cell in ipairs(g.cells) do
				if cell.region then
					local p = cell.pos
					local k = ("%d:%d"):format(math.floor(p.X / 4), math.floor(p.Z / 4))
					local t = hash[k]
					if not t then t = {}; hash[k] = t end
					t[#t + 1] = p
				end
			end
		end
		lookup = function(c: Vector3): boolean
			local x, z = math.floor(c.X / 4), math.floor(c.Z / 4)
			for ox = -1, 1 do for oz = -1, 1 do
				for _, p in ipairs(hash[("%d:%d"):format(x + ox, z + oz)] or {}) do
					if (Vector3.new(p.X - c.X, 0, p.Z - c.Z)).Magnitude <= band
						and c.Y > p.Y - 1 and c.Y < p.Y + 4 then return true end
				end
			end end
			return false
		end
	end

	local kept, skipped = 0, 0
	local leaves = {}
	svo:forEachSolidLeaf(function(c, h)
		if lookup and not lookup(c) then skipped += 1; return end
		leaves[#leaves + 1] = { c, h }
	end)
	local stride = math.max(1, math.ceil(#leaves / maxParts))
	for i = 1, #leaves, stride do
		local c, h = leaves[i][1], leaves[i][2]
		local e = h * 2
		local p = Instance.new("Part")
		p.Anchored = true; p.CanCollide = false; p.CanQuery = false; p.CanTouch = false
		p.Size = Vector3.new(e, e, e)
		p.CFrame = CFrame.new(c)
		p.Transparency = o.transparency or 0.65
		p.Material = Enum.Material.SmoothPlastic
		-- bigger nodes are lighter, so the collapse structure reads at a glance
		p.Color = Color3.fromHSV(0.08, 0.55, math.clamp(0.35 + math.log(e, 2) * 0.18, 0, 1))
		p.Name = ("s%d"):format(e)
		p.Parent = root
		kept += 1
	end

	return root, ("%d solid leaves drawn%s%s"):format(kept,
		stride > 1 and (" (1 in " .. stride .. ")") or "",
		skipped > 0 and (", " .. skipped .. " outside the floor band") or "")
end

-- Draw the connectivity snapshot: one colour per connected component.
--
-- SAMPLED, ON PURPOSE. case5 has 202k cells and one part each would be a
-- drawing nobody can move a camera through. The budget is spent where the
-- answer is: a component small enough to be suspicious is drawn WHOLE, and only
-- the mainland gets strided, because what this drawing is for is deciding
-- whether the little islands are real ground or trace debris.
--
-- The stride is reported rather than hidden. A sampled component looks sparse,
-- and sparse is exactly what a broken one would look like too.
function Pipeline.drawConnectivity(snap: any, opts: any?): (Instance, string)
	local o = opts or {}
	local maxParts = o.maxParts or 12000
	local fullUnder = o.fullUnder or 3000
	local size = o.size or 0.4
	local lift = o.lift or 0.25

	local old = workspace:FindFirstChild(Pipeline.debugName)
	if old then
		local prev = old:FindFirstChild("Connectivity")
		if prev then prev:Destroy() end
	else
		old = Instance.new("Folder")
		old.Name = Pipeline.debugName
		old.Parent = workspace
	end
	local root = Instance.new("Folder")
	root.Name = "Connectivity"
	root.Parent = old

	-- cells per component, in index order so the drawing is reproducible
	local byComp = {}
	for i = 1, snap.cellCount do
		local c = snap.comp[i]
		local t = byComp[c]
		if not t then t = {}; byComp[c] = t end
		t[#t + 1] = i
	end

	-- Budget: the small components cost what they cost, and whatever is left is
	-- shared among the big ones in proportion to their size.
	local spent, bigTotal = 0, 0
	for id = 1, snap.pieces do
		local n = #byComp[id]
		if n <= fullUnder then spent += n else bigTotal += n end
	end
	local left = math.max(maxParts - spent, 1000)

	local strided = {}
	for id = 1, snap.pieces do
		local idxs = byComp[id]
		local n = #idxs
		local stride = 1
		if n > fullUnder then
			local share = math.max(math.floor(left * (n / bigTotal)), 1)
			stride = math.max(math.ceil(n / share), 1)
			if stride > 1 then strided[#strided + 1] = ("c%d 1 in %d"):format(id, stride) end
		end
		-- golden-ratio hue, so adjacent component ids never share a colour
		local hue = (id * 0.6180339887) % 1
		local colour = Color3.fromHSV(hue, 0.75, 1)
		local f = Instance.new("Folder")
		f.Name = ("c%03d_%dcells%s"):format(id, n, stride > 1 and ("_1in" .. stride) or "")
		f.Parent = root
		for k = 1, n, stride do
			local cell = snap.cells[idxs[k]]
			local up = cell.normal or Vector3.yAxis
			local p = Instance.new("Part")
			p.Anchored = true; p.CanCollide = false; p.CanQuery = false; p.CanTouch = false
			p.Size = Vector3.new(size, size, size)
			p.CFrame = CFrame.new(cell.pos + up * lift)
			p.Color = colour
			p.Material = Enum.Material.Neon
			p.Name = "c" .. id
			p.Parent = f
		end
	end

	local note = ("%d components drawn"):format(snap.pieces)
	if #strided > 0 then
		note ..= ", SAMPLED: " .. table.concat(strided, ", ")
	else
		note ..= ", every cell drawn"
	end
	return root, note
end

-- One line per stage, for a console that has to be read at a glance.
function Pipeline.report(result: any): string
	local b, s = result.stats.boundary, result.stats.simplify
	local by = {}
	for k, v in pairs(s.closedBy) do
		if v > 0 then by[#by + 1] = ("%s %d"):format(k, v) end
	end
	table.sort(by)
	-- appended one at a time: a nil in the middle of a table constructor
	-- silently truncates everything after it in table.concat
	local lines = {
		("root      %s"):format(result.config.root or "?"),
		("bake      %d regions, %d boundary faces, %.1fs"):format(b.regions, b.faces, result.stats.bakeSeconds),
	}
	if b.erode then lines[#lines + 1] = Erode.report(b.erode) end
	lines[#lines + 1] = ("trace     %d loops, %d closed, %d broken, %d seam stitches"):format(b.loops, b.closed, b.broken, b.stitched)
	lines[#lines + 1] = ("simplify  %d raw nodes -> %d corners, %.1fs"):format(s.raw, s.corners, result.stats.simplifySeconds)
	if s.rescued > 0 or s.collapsed > 0 or s.holesHeld > 0 then
		lines[#lines + 1] = ("  %d holes held tight, %d rings rescued, %d still collapsed")
			:format(s.holesHeld, s.rescued, s.collapsed)
	end
	if s.gapFilled then
		local kept = {}
		for k, v in pairs(s.gapKept) do kept[#kept + 1] = ("%s %d"):format(k, v) end
		table.sort(kept)
		lines[#lines + 1] = ("gaps      %d filled, kept: %s"):format(s.gapFilled,
			#kept > 0 and table.concat(kept, ", ") or "none")
	end
	if s.clean then
		local cl = s.clean
		lines[#lines + 1] = ("corners   %d crossings cut (%d refused), %d spikes, %d clusters (%d refused), %d notches filled, %d rings reverted")
			:format(cl.crossCut, cl.crossRefused, cl.spikes, cl.clusters, cl.clusterRefused, cl.notches, cl.reverted)
		lines[#lines + 1] = ("joins     %d shadow pieces dropped, %d open pieces joined, %d rings closed by a join, %d reopened"):format(s.shadows or 0, s.joined or 0, s.joinClosed or 0, s.joinReopened or 0)
	end
	lines[#lines + 1] = ("closing   %s, %d still open"):format(#by > 0 and table.concat(by, ", ") or "nothing to close", s.open)
	if s.edges and s.edges > 0 then
		lines[#lines + 1] = ("edges     %d: %d wall, %d open, %d mixed, %d invented")
			:format(s.edges, s.wallEdges, s.openEdges, s.mixedEdges, s.inventedEdges)
	end
	if result.stats.boundary and result.stats.boundary.kind then
		lines[#lines + 1] = FaceKind.report(result.stats.boundary.kind)
	end
	lines[#lines + 1] = Rings.report(s.rings)
	if result.tri then
		lines[#lines + 1] = Triangulate.report(result.tri, result.loops)
	end
	if result.mesh then
		lines[#lines + 1] = CDT.report(result.mesh, result.loops)
	end
	if result.meshKind then
		lines[#lines + 1] = EdgeKind.report(result.meshKind)
	end
	if result.portals then
		lines[#lines + 1] = Portals.report(result.portals)
	end
	return table.concat(lines, "\n")
end

return Pipeline
