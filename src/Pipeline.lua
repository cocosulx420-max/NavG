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
local EdgeKind = require(script.Parent:WaitForChild("EdgeKind"))
local Offset = require(script.Parent:WaitForChild("Offset"))

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
function Pipeline.bake(cfg: any?): (any, any)
	local c = resolve(cfg)
	assert(c.root, "Pipeline: cfg.root is required -- name the model to bake")
	local data = LocalGrid.build(c)
	local _, bstats = Boundary.trace(data, c)
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
-- The face's KIND travels with its node. Boundary already knows whether a face
-- is a wall, a dropoff or a region seam, and that verdict dies here unless it is
-- carried: the simplifier takes an array of Vector3 and gives corners back.
-- The offset needs it per edge, because a wall is pushed inward and a ledge is
-- not, so losing it means treating every ledge as masonry.
local function polyline(entry: any, loop: any, step: number): ({Vector3}, Vector3, {string}, {any})
	local F = loop.faces
	local pts = table.create(#F)
	local kinds = table.create(#F)
	-- the CELL behind each node, so the offset can read its ground thickness
	local cells = table.create(#F)
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
				kinds[#kinds + 1] = f.kind or "none"
				cells[#cells + 1] = f.cell
			end
		end
	end
	if #pts > 1 and loop.closed and (pts[1] - pts[#pts]).Magnitude < 1e-3 then
		pts[#pts] = nil
		kinds[#kinds] = nil
		cells[#cells] = nil
	end
	return pts, up, kinds, cells
end

-- Every traced loop, simplified to corners. One loop in, one entry out; a loop
-- the closing pass could not shut keeps `closed = false` and stays a path.
function Pipeline.simplify(data: any, cfg: any?): ({any}, any)
	local c = resolve(cfg)
	local o = {}
	for _, k in ipairs(SIMPLIFY_KEYS) do o[k] = c[k] end
	local debugRoot = workspace:FindFirstChild(Pipeline.debugName)
	local step = data.config.step

	local out = {}
	local stats = { loops = 0, open = 0, raw = 0, corners = 0,
		closedBy = { merge = 0, intersect = 0, straight = 0, ["already closed"] = 0 } }

	-- Region order follows Boundary's table, which LocalGrid numbers largest
	-- region first; sort so a result lists loops the same way every run.
	local regions = {}
	for r in pairs(data.boundary) do regions[#regions + 1] = r end
	table.sort(regions)

	for _, r in ipairs(regions) do
		local entry = data.boundary[r]
		for li, L in ipairs(entry.loops) do
			local poly, up, polyKind, polyCell = polyline(entry, L, step)
			local opts = table.clone(o)
			opts.closed = L.closed
			opts.up = up
			opts.validate = Pipeline.validator(up, debugRoot)

			local pts, idx = PathSimplify.simplify(poly, opts)
			pts, idx = PathSimplify.merge(pts, idx, poly, opts)
			pts, idx = PathSimplify.dejog(pts, idx, poly, opts)
			pts = PathSimplify.collapseBevels(pts, opts)

			local closed, method = L.closed, nil
			if not closed then
				local ok, cs
				pts, ok, cs = PathSimplify.close(pts, opts)
				closed = ok
				method = ok and cs.method or nil
				if ok then stats.closedBy[cs.method] = (stats.closedBy[cs.method] or 0) + 1 end
			end

			out[#out + 1] = { region = r, index = li, up = up,
				poly = poly, polyKind = polyKind, polyCell = polyCell,
				pts = pts, closed = closed, closedBy = method }
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
	-- Boundary's wall/drop/seam verdict, carried onto the simplified edges. The
	-- offset reads it per edge: a wall moves inward, a ledge does not.
	stats.edgeKind = EdgeKind.assign(out)

	return out, stats
end

-- Bake and simplify, and record what it took to do so.
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

-- THE OFFSET, and what it cost.
--
-- Runs the ground thickness first because the grading reads it, takes a
-- connectivity snapshot on each side, and hands back the severance verdict
-- alongside the offset statistics. The two belong together: "the lines moved"
-- is not a result until it is paired with "and nothing was cut off".
function Pipeline.offset(result: any): (any, any)
	Thickness.build(result.data)
	-- BOTH snapshots go through the containment test, the baseline against the
	-- unmoved polygon. The polygon already excludes cells the offset never
	-- touched; charging those to the offset reported case3 severed into nine
	-- pieces on a bake that moved eleven edges.
	local stats = Offset.apply(result.loops, result.data)
	local before = Severance.snapshot(result.data, Offset.keepTest(result.loops, true))
	local after = Severance.snapshot(result.data, Offset.keepTest(result.loops))
	return stats, Severance.compare(before, after)
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

	for _, L in ipairs(result.loops) do
		local pts, up = L.pts, L.up
		local off = up * lift
		local n = #pts
		local base = (L.kind == "hole") and HOLE or LINE
		local f = Instance.new("Folder")
		f.Name = ("r%03d_loop%d_%s_%dto%d%s"):format(L.region, L.index,
			L.kind or "unlabelled", #L.poly, n,
			L.closedBy and ("_CLOSED_" .. L.closedBy) or (L.closed and "" or "_OPEN"))
		f.Parent = simp
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

-- Draw the offset polygon against the one it came from.
--
-- Almost every edge holds -- 9 of 530 move on case5 -- so a drawing that treats
-- both outlines equally is two coincident lines and nothing to see. The
-- original is drawn dim and the offset bright, and the handful of edges that
-- actually moved get their own folder with a tie line from each original
-- corner to where it went. That folder IS the result; the rest is context.
function Pipeline.drawOffset(result: any, opts: any?): (Instance, string)
	local o = opts or {}
	local lift = o.lift or 0.3
	local OLD = o.oldColor or Color3.fromRGB(120, 120, 130)
	local NEW = o.newColor or Color3.fromRGB(80, 255, 180)
	local HOT = o.movedColor or Color3.fromRGB(255, 130, 40)

	local old = workspace:FindFirstChild(Pipeline.debugName)
	if old then
		local prev = old:FindFirstChild("Offset")
		if prev then prev:Destroy() end
	else
		old = Instance.new("Folder")
		old.Name = Pipeline.debugName
		old.Parent = workspace
	end
	local root = Instance.new("Folder")
	root.Name = "Offset"
	root.Parent = old
	local fOld = Instance.new("Folder"); fOld.Name = "original"; fOld.Parent = root
	local fNew = Instance.new("Folder"); fNew.Name = "offset"; fNew.Parent = root
	local fHot = Instance.new("Folder"); fHot.Name = "moved"; fHot.Parent = root

	local moved, total = 0, 0
	for _, L in ipairs(result.loops) do
		local pts, off, up = L.pts, L.offset, L.up
		local n = #pts
		if not off then continue end
		local rise = up * lift
		local last = L.closed and n or n - 1
		for i = 1, last do
			local j = (i % n) + 1
			local d = L.offsetDist and L.offsetDist[i] or 0
			total += 1
			segment(pts[i] + rise, pts[j] + rise, 0.10, OLD, "seg" .. i, fOld)
			segment(off[i] + rise, off[j] + rise, d > 0 and 0.22 or 0.13,
				d > 0 and HOT or NEW, "seg" .. i, fNew)
			if d > 0 then
				moved += 1
				local g = Instance.new("Folder")
				g.Name = ("r%03d_l%d_e%d_%s_%.2f"):format(L.region, L.index, i,
					L.edgeKind and L.edgeKind[i] or "?", d)
				g.Parent = fHot
				segment(pts[i] + rise, pts[j] + rise, 0.14, OLD, "was", g)
				segment(off[i] + rise, off[j] + rise, 0.24, HOT, "now", g)
				-- tie lines, so the direction and size of the move is readable
				segment(pts[i] + rise, off[i] + rise, 0.12, HOT, "shiftA", g)
				segment(pts[j] + rise, off[j] + rise, 0.12, HOT, "shiftB", g)
			end
		end
	end

	return root, ("%d edges drawn, %d moved (orange, in the `moved` folder); original is grey, offset is green")
		:format(total, moved)
end

-- Draw every polygon edge in the colour of its kind.
--
-- GROUPED BY KIND, not by loop. The question this drawing answers is "is
-- anything labelled wrong", and that is asked one kind at a time: hide every
-- folder but `wall` and what is left should be masonry and nothing else. A
-- per-loop tree cannot be filtered that way.
--
-- Mixed edges get a marker at their midpoint. They are the ones where a merge
-- flattened masonry and ledge into a single straight edge, so they are where a
-- wrong answer is most likely and hardest to see from the colour alone.
function Pipeline.drawEdgeKinds(result: any, opts: any?): (Instance, string)
	local o = opts or {}
	local lift = o.lift or 0.35
	local thick = o.thick or 0.18
	local COLOUR = {
		wall = o.wallColor or Color3.fromRGB(255, 80, 50),
		drop = o.dropColor or Color3.fromRGB(60, 200, 255),
		edge = o.seamColor or Color3.fromRGB(170, 255, 70),
		none = o.noneColor or Color3.fromRGB(210, 70, 255),
	}

	local old = workspace:FindFirstChild(Pipeline.debugName)
	if old then
		local prev = old:FindFirstChild("EdgeKinds")
		if prev then prev:Destroy() end
	else
		old = Instance.new("Folder")
		old.Name = Pipeline.debugName
		old.Parent = workspace
	end
	local root = Instance.new("Folder")
	root.Name = "EdgeKinds"
	root.Parent = old

	local bucket = {}
	for _, k in ipairs({ "wall", "drop", "edge", "none" }) do
		local f = Instance.new("Folder")
		f.Name = k == "edge" and "seam" or k
		f.Parent = root
		bucket[k] = f
	end
	local mixedFolder = Instance.new("Folder")
	mixedFolder.Name = "mixed"
	mixedFolder.Parent = root

	local counts = { wall = 0, drop = 0, edge = 0, none = 0 }
	local mixed = 0
	for _, L in ipairs(result.loops) do
		local pts, up = L.pts, L.up
		local off = up * lift
		local n = #pts
		local ek, ep = L.edgeKind, L.edgePurity
		if not ek then continue end
		for i = 1, #ek do
			local kind = ek[i] or "none"
			local a = pts[i] + off
			local b = pts[(i % n) + 1] + off
			segment(a, b, thick, COLOUR[kind] or COLOUR.none,
				("r%03d_l%d_e%d_%d%%"):format(L.region, L.index, i, math.floor((ep[i] or 0) * 100)),
				bucket[kind] or bucket.none)
			counts[kind] = (counts[kind] or 0) + 1
			if (ep[i] or 1) < 0.8 then
				mixed += 1
				local m = Instance.new("Part")
				m.Anchored = true; m.CanCollide = false; m.CanQuery = false; m.CanTouch = false
				m.Shape = Enum.PartType.Ball
				m.Size = Vector3.new(0.7, 0.7, 0.7)
				m.Color = Color3.fromRGB(255, 255, 255)
				m.Material = Enum.Material.Neon
				m.Transparency = 0.35
				m.CFrame = CFrame.new((a + b) * 0.5)
				m.Name = ("r%03d_l%d_e%d_%s_%d%%"):format(L.region, L.index, i, kind,
					math.floor((ep[i] or 0) * 100))
				m.Parent = mixedFolder
			end
		end
	end

	return root, ("wall %d (red), drop %d (blue), seam %d (green), none %d (purple); %d mixed marked white")
		:format(counts.wall, counts.drop, counts.edge, counts.none, mixed)
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
	return table.concat({
		("root      %s"):format(result.config.root or "?"),
		("bake      %d regions, %d boundary faces, %.1fs"):format(b.regions, b.faces, result.stats.bakeSeconds),
		("trace     %d loops, %d closed, %d broken, %d seam stitches"):format(b.loops, b.closed, b.broken, b.stitched),
		("simplify  %d raw nodes -> %d corners, %.1fs"):format(s.raw, s.corners, result.stats.simplifySeconds),
		("closing   %s, %d still open"):format(#by > 0 and table.concat(by, ", ") or "nothing to close", s.open),
		Rings.report(s.rings),
		EdgeKind.report(s.edgeKind),
	}, "\n")
end

return Pipeline
