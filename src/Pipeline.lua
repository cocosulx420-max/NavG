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

-- WORST PERPENDICULAR DISTANCE FROM THE RAW BOUNDARY TO THE SIMPLIFIED ONE.
--
-- The number that says whether the polygons still describe the floor. Every
-- other statistic counts things; this one measures error, and a merge that
-- wanders off a floor edge shows up here and nowhere else -- not in the corner
-- count, and not in the raycast validator, because leaving a floor hits nothing.
--
-- O(corners * raw nodes) per loop, so it is a separate call and not part of run.
function Pipeline.measure(result: any): any
	local worst, where, over = 0, nil, 0
	local step = result.config.bake.step or 0.5
	for _, L in ipairs(result.loops) do
		local poly, pts, nP, n = L.poly, L.pts, #L.poly, #L.pts
		local function nearest(q: Vector3): number
			local bi, bd = 1, math.huge
			for i = 1, nP do
				local d = (poly[i] - q).Magnitude
				if d < bd then bi, bd = i, d end
			end
			return bi
		end
		for i = 1, (L.closed and n or n - 1) do
			local A, B = pts[i], pts[(i % n) + 1]
			local d = B - A
			if d.Magnitude > 1e-6 then
				local e = d.Unit
				local k, stop, w, guard = nearest(A), nearest(B), 0, 0
				repeat
					local rr = poly[k] - A
					local dist = (rr - e * rr:Dot(e)).Magnitude
					if dist > w then w = dist end
					k = (k % nP) + 1
					guard += 1
				until k == stop or guard > nP
				if w > step * 1.1 then over += 1 end
				if w > worst then
					worst = w
					where = ("r%03d loop%d edge%d, len %.1f"):format(L.region, L.index, i, d.Magnitude)
				end
			end
		end
	end
	return { worst = worst, where = where, overStep = over }
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
		local f = Instance.new("Folder")
		f.Name = ("r%03d_loop%d_%dto%d%s"):format(L.region, L.index, #L.poly, n,
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
				made and MARK or LINE, made and "closure" or ("seg" .. i), lines)
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
	}, "\n")
end

return Pipeline
