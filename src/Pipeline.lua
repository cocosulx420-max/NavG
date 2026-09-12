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
local Nodes = require(script.Parent:WaitForChild("Nodes"))

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
local function polyline(entry: any, loop: any, step: number): ({Vector3}, Vector3)
	local F = loop.faces
	local pts = table.create(#F)
	local up = Vector3.yAxis
	local inset = step * 0.5
	for i, fi in ipairs(F) do
		local f = entry.faces[fi]
		if i == 1 then up = f.up end
		local d = f.b - f.a
		if d.Magnitude > 1e-9 then
			local p = (f.a + f.b) * 0.5 + f.up:Cross(d.Unit) * inset
			local prev = pts[#pts]
			if not prev or (prev - p).Magnitude > 1e-3 then pts[#pts + 1] = p end
		end
	end
	if #pts > 1 and loop.closed and (pts[1] - pts[#pts]).Magnitude < 1e-3 then
		pts[#pts] = nil
	end
	return pts, up
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
			local poly, up = polyline(entry, L, step)
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
				poly = poly, pts = pts, closed = closed, closedBy = method }
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
	local tri = Pipeline.triangulate(result)

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

	return root, Triangulate.report(tri, result.loops)
end

-- Draw two traced boundaries against each other, in two folders and nothing
-- else, clearing the debug root first.
--
-- For the eroded pipeline there is no separate offset polygon to draw: the
-- traced boundary IS the offset boundary, which is the whole point of eroding
-- first. So the comparison is between two TRACES of the same bake -- the raw
-- cells and the eroded ones -- rather than between a polygon and a moved copy
-- of itself.
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
	lines[#lines + 1] = ("closing   %s, %d still open"):format(#by > 0 and table.concat(by, ", ") or "nothing to close", s.open)
	lines[#lines + 1] = Rings.report(s.rings)
	if result.tri then
		lines[#lines + 1] = Triangulate.report(result.tri, result.loops)
	end
	return table.concat(lines, "\n")
end

return Pipeline
