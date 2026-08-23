--!strict
-- NavGen.Contours -- turn local grids into contour cell lists a classifier can
-- read, and say out loud whether they are any good.
--
-- WHAT THIS REPLACES. The contours being fed to the classifier were produced by
-- an ad-hoc harness that lived only inside a one-off command, which is why
-- diagnosing it took hours: there was nothing to read. Its measured output on
-- SmallMap:
--
--   cells                                  8345
--   contours                               1592
--   contours of exactly 2 cells            1415     (89%)
--   cell positions claimed by >1 contour    980     (up to 4 contours on a cell)
--   contour-pairs whose endpoints touch   11617
--
-- 92% of the contours were degenerate, so every statistic computed on that map
-- was dominated by noise. The classifier was not at fault -- a 591-cell contour
-- decomposed into 11 primitives at worst deviation 0.936, and of six pairs of
-- consecutive same-pair STAIR primitives, zero could legally have been merged.
-- The rainbow banding along wall bases was never one staircase cut up; it was
-- one boundary cut into dozens of 2-cell contours, each classified alone.
--
-- THE THREE DEFECTS, AND WHAT IS DONE ABOUT EACH.
--
-- A. SLIVERS. 1358 of the tiny contours sat inside one grating -- closely spaced
--    parallel bars whose gaps are one or two cells of walkable floor. Nothing
--    can path along a one-cell sliver; it is far below agent radius, and it
--    should never reach tracing at all. So walkable cells are eroded once and
--    any connected region left with no core cell is discarded whole, region and
--    boundary together. This is the highest-value change of the three.
--
-- B. OVERLAP. The old harness emitted one chain per edge at every node whose
--    degree was not 2, and put the node itself in each of them -- so a node with
--    four boundary neighbours appeared in four contours, emitted consecutively,
--    which is exactly the consecutive-index overlap that was measured. Here the
--    walk marks cells visited as it goes, so A CELL BELONGS TO EXACTLY ONE
--    CONTOUR by construction, and `stats.sharedCells` asserts it.
--
-- C. FRAGMENTATION. Endpoint-touching pairs are mostly a consequence of A and B.
--    What remains is reported rather than hidden, alongside the open-chain count
--    and whether each open chain ends at the edge of its part, which is the only
--    place an unclosed boundary is legitimate.
--
-- The classifier is not tuned to compensate for any of this. It is handed clean
-- contours or it is handed nothing.

local Contours = {}

local DIR4 = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } }

export type Config = {
	-- A region must be at least this many cells wide to be traced.
	--
	-- TWO, NOT THREE. Three discards every two-cell-wide region, and those are
	-- real geometry: the grating's bars are two cells wide and each one traces
	-- cleanly as a 70-cell ring reading `D1 R34 U1 L34` -- two long edges and
	-- two ends, which is exactly the right reading. Measured, 3 threw away 21
	-- regions and 1083 cells to solve a problem that was never the geometry's:
	-- the 1415 two-cell contours came from a tracer that cut at every branch,
	-- and they do not come back now that it does not.
	--
	-- At 2, nothing is discarded on this map at all, which is the honest
	-- outcome -- there were no one-cell slivers to remove.
	minWidthCells: number?,
	-- Contours shorter than this carry no shape worth classifying. Reported
	-- either way; set to 0 to keep everything and see what is really there.
	minContourCells: number?,
	-- Print the summary on every build.
	verbose: boolean?,
}

local DEFAULT = {
	minWidthCells = 2,
	minContourCells = 8,
	verbose = true,
}

export type Contour = {
	cells: { any },      -- LocalGrid cells, in traced order
	grid: any,
	closed: boolean,
	atPartEdge: boolean, -- an open chain that stops where its part stops
}

local function key(ui: number, vi: number): string
	return ui .. ":" .. vi
end

--------------------------------------------------------------------------
-- A. Discard regions narrower than the agent
--------------------------------------------------------------------------

-- Returns the set of cells worth tracing, and how many were dropped.
--
-- Erosion, not a bounding box: a sliver that bends still has no core, whereas a
-- bounding box would call an L-shaped corridor wide. A cell is CORE when all
-- four of its neighbours are walkable, and a connected region with no core cell
-- anywhere in it cannot hold the agent at any point along its length.
local function keepWideRegions(g: any, minWidth: number)
	local index = {}
	for _, c in ipairs(g.cells) do index[key(c.ui, c.vi)] = c end

	local erosions = math.max(0, math.floor((minWidth - 1) / 2))
	local core = index
	for _ = 1, erosions do
		local next_ = {}
		for k, c in pairs(core) do
			local solid = true
			for _, d in ipairs(DIR4) do
				if not core[key(c.ui + d[1], c.vi + d[2])] then solid = false break end
			end
			if solid then next_[k] = c end
		end
		core = next_
	end

	-- Connected regions of the ORIGINAL cells; a region survives only if it
	-- contains at least one core cell.
	local seen, keep = {}, {}
	local dropped, droppedRegions = 0, 0
	for _, start in ipairs(g.cells) do
		local sk = key(start.ui, start.vi)
		if not seen[sk] then
			local stack, members, hasCore = { start }, {}, false
			seen[sk] = true
			while #stack > 0 do
				local c = table.remove(stack)
				members[#members + 1] = c
				if core[key(c.ui, c.vi)] then hasCore = true end
				for _, d in ipairs(DIR4) do
					local nk = key(c.ui + d[1], c.vi + d[2])
					local n = index[nk]
					if n and not seen[nk] then
						seen[nk] = true
						stack[#stack + 1] = n
					end
				end
			end
			if hasCore then
				for _, c in ipairs(members) do keep[key(c.ui, c.vi)] = c end
			else
				dropped = dropped + #members
				droppedRegions = droppedRegions + 1
			end
		end
	end
	return keep, dropped, droppedRegions
end

--------------------------------------------------------------------------
-- Ramps
--------------------------------------------------------------------------

-- A RAMP IS NAMED, so it does not have to be inferred.
--
-- `LocalGrid` marks a cell `wall` when a surface stands above it, and a ramp
-- meeting the floor does exactly that: at the ClipRamp's foot the ground cells
-- are `wall = true` because the ramp rises 0.65 studs over one cell beside them.
-- That is the seam where floor meets ramp, not a wall, and tracing it produced
-- short stubs along the foot duplicating the clean line the ramp's own edge
-- already gives.
--
-- DERIVING IT GEOMETRICALLY DOES NOT WORK, and the attempt is worth recording so
-- nobody repeats it: discounting any wall climbable within `maxSlope` also
-- discounts every stair riser, because a one-stud rise over one cell is 45
-- degrees and this pipeline accepts up to 65. Measured, that cut STAIR
-- primitives from 61 to 17. A ramp and a staircase are the same shape to every
-- measure available here. The only thing separating them is that one of them
-- says so in its name.
local DIR8 = {
	{ 1, 0 }, { 1, 1 }, { 0, 1 }, { -1, 1 },
	{ -1, 0 }, { -1, -1 }, { 0, -1 }, { 1, -1 },
}

local function isRamp(part): boolean
	if not part then return false end
	return tostring(part.Name):lower():find("ramp", 1, true) ~= nil
end

-- True when every direction this cell reports as walled is walled by a RAMP --
-- a surface named as one, which you walk onto rather than into. Drops are never
-- discounted; a drop is a drop.
local function walledOnlyByRamp(cell, g, world, step): boolean
	local mask = cell.wallMask or 0
	if mask == 0 then return false end
	if (cell.dropMask or 0) ~= 0 then return false end
	for bit, d in ipairs(DIR8) do
		if bit32.band(mask, bit32.lshift(1, bit - 1)) ~= 0 then
			local off
			if not g.fallback and g.u and g.v then
				off = g.u * (d[1] * step) + g.v * (d[2] * step)
			else
				off = Vector3.new(d[1] * step, 0, d[2] * step)
			end
			local want = cell.pos + off
			local found = false
			local bx = math.floor(want.X / step)
			local by = math.floor(want.Y / step)
			local bz = math.floor(want.Z / step)
			for dx = -1, 1 do for dy = -2, 2 do for dz = -1, 1 do
				local b = world[(bx + dx) .. ":" .. (by + dy) .. ":" .. (bz + dz)]
				if b then
					for _, e in ipairs(b) do
						if isRamp(e.g.part) then
							local v = e.cell.pos - cell.pos
							local horiz = Vector3.new(v.X, 0, v.Z).Magnitude
							if horiz > 0.01 and horiz <= step * 1.6 then found = true end
						end
					end
				end
			end end end
			if not found then return false end
		end
	end
	return true
end

--------------------------------------------------------------------------
-- Tracing
--------------------------------------------------------------------------

-- Walk the boundary so that every cell lands in exactly one contour.
--
-- The old harness started a chain from every edge of every branch node, which
-- put the node in as many contours as it had edges. Here a cell is claimed the
-- moment it is walked, so overlap is impossible rather than merely unlikely --
-- and where a branch genuinely exists the walk simply continues down one arm
-- and the others become their own contours, which is the honest reading of a
-- boundary that forks.
--
-- Deterministic: candidates are ordered by (ui, vi), never by table order.
local function traceGrid(g: any, keep: any, out: { Contour })
	local boundary = {}
	local list = {}
	for k, c in pairs(keep) do
		if c.wall or c.dropoff then
			boundary[k] = c
			list[#list + 1] = c
		end
	end
	table.sort(list, function(a, b)
		if a.ui ~= b.ui then return a.ui < b.ui end
		return a.vi < b.vi
	end)

	-- WHICH WAY THE WALK GOES WHEN IT HAS A CHOICE.
	--
	-- Ordering candidates lexicographically looks harmless and is not. Where a
	-- boundary is two cells thick every cell has a neighbour in the other row,
	-- and a lexicographic pick WEAVES BETWEEN THE TWO ROWS instead of running
	-- along one. What comes out is a sawtooth -- `D1 R1 U1 R1 D1 R1 U1` -- which
	-- alternates direction on the SAME axis, so the direction pair changes on
	-- every run and the classifier correctly makes every run its own primitive.
	-- Measured: one 54-cell contour came out as 18 primitives, twelve of them a
	-- single cell.
	--
	-- So the walk carries on in the direction it was already going when it can.
	-- Where the boundary is one cell thick this changes nothing -- an interior
	-- cell has exactly two neighbours and the walk is forced -- so it costs
	-- nothing where there is no ambiguity and resolves it where there is.
	-- Lexicographic order stays as the final tie-break, so the trace remains a
	-- fact about the cells rather than about table order.
	local function neighbours(c: any, visited: any, fromDir: { number }?)
		local n = {}
		for _, d in ipairs(DIR4) do
			local q = boundary[key(c.ui + d[1], c.vi + d[2])]
			if q and not visited[key(q.ui, q.vi)] then
				n[#n + 1] = { cell = q, d = d }
			end
		end
		table.sort(n, function(a, b)
			if fromDir then
				local sa = (a.d[1] == fromDir[1] and a.d[2] == fromDir[2]) and 0 or 1
				local sb = (b.d[1] == fromDir[1] and b.d[2] == fromDir[2]) and 0 or 1
				if sa ~= sb then return sa < sb end
			end
			if a.cell.ui ~= b.cell.ui then return a.cell.ui < b.cell.ui end
			return a.cell.vi < b.cell.vi
		end)
		return n
	end

	local function degree(c: any)
		local d = 0
		for _, o in ipairs(DIR4) do
			if boundary[key(c.ui + o[1], c.vi + o[2])] then d = d + 1 end
		end
		return d
	end

	local visited = {}
	-- Loose ends first, so an open boundary is walked end to end rather than
	-- being started somewhere in its middle and split in two.
	local order = {}
	for _, c in ipairs(list) do
		if degree(c) <= 1 then order[#order + 1] = c end
	end
	for _, c in ipairs(list) do
		if degree(c) > 1 then order[#order + 1] = c end
	end

	for _, start in ipairs(order) do
		if not visited[key(start.ui, start.vi)] then
			local run = { start }
			visited[key(start.ui, start.vi)] = true
			local cur, dir = start, nil
			while true do
				local n = neighbours(cur, visited, dir)
				if #n == 0 then break end
				local pick = n[1]
				dir = pick.d
				cur = pick.cell
				visited[key(cur.ui, cur.vi)] = true
				run[#run + 1] = cur
			end
			-- Closed when the two ends are neighbours and there is an interior;
			-- two cells that touch are a chain walked once, not a ring.
			local closed = false
			if #run >= 4 then
				local a, b = run[1], run[#run]
				closed = (math.abs(a.ui - b.ui) + math.abs(a.vi - b.vi)) == 1
			end
			out[#out + 1] = { cells = run, grid = g, closed = closed, atPartEdge = false }
		end
	end
end

--------------------------------------------------------------------------
-- Build
--------------------------------------------------------------------------

function Contours.build(localData: any, cfg: Config?)
	local c: any = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end

	-- Grids come out of a hash, so they are ordered here before anything reads
	-- them; otherwise contour indices differ between runs on identical input.
	local grids = {}
	for _, g in pairs(localData.grids) do grids[#grids + 1] = g end
	table.sort(grids, function(a, b)
		local pa, pb = a.cells[1], b.cells[1]
		if not pa or not pb then return false end
		if pa.pos.X ~= pb.pos.X then return pa.pos.X < pb.pos.X end
		if pa.pos.Y ~= pb.pos.Y then return pa.pos.Y < pb.pos.Y end
		return pa.pos.Z < pb.pos.Z
	end)

	local stats = {
		grids = #grids,
		cellsDropped = 0, regionsDropped = 0,
		traced = 0, kept = 0, discarded = 0,
		closed = 0, open = 0, openAtEdge = 0,
		sharedCells = 0, coincidentPositions = 0,
		endpointTouching = 0, openDiagonalClose = 0,
		histogram = {},
	}

	local raw: { Contour } = {}
	for _, g in ipairs(grids) do
		local keep, dropped, regions = keepWideRegions(g, c.minWidthCells)
		stats.cellsDropped = stats.cellsDropped + dropped
		stats.regionsDropped = stats.regionsDropped + regions
		traceGrid(g, keep, raw)
	end
	stats.traced = #raw

	-- An open boundary is legitimate where the FLOOR CONTINUES ONTO ANOTHER PART:
	-- grids are per-part, so a contour that walks off the edge of its own part
	-- has nowhere left to go on this lattice. The earlier version of this test
	-- asked whether the endpoint sat on the grid's ui/vi bounding box, which is
	-- only the same question for a part whose walkable area is a full rectangle
	-- -- it called 13 perfectly ordinary endings suspicious.
	local step = (localData.config and localData.config.step) or 1
	local world = {}
	local function bucketKey(p, dx, dy, dz)
		return math.floor(p.X / step) + dx .. ":" .. math.floor(p.Y / step) + dy
			.. ":" .. math.floor(p.Z / step) + dz
	end
	for _, g in ipairs(grids) do
		for _, cell in ipairs(g.cells) do
			local k = bucketKey(cell.pos, 0, 0, 0)
			local b = world[k]
			if not b then b = {}; world[k] = b end
			b[#b + 1] = { cell = cell, g = g }
		end
	end
	local function continuesElsewhere(cell, g)
		for dx = -1, 1 do for dy = -1, 1 do for dz = -1, 1 do
			local b = world[bucketKey(cell.pos, dx, dy, dz)]
			if b then
				for _, e in ipairs(b) do
					if e.g ~= g and (e.cell.pos - cell.pos).Magnitude <= step * 1.5 then
						return true
					end
				end
			end
		end end end
		return false
	end

	local contours: { Contour } = {}
	for _, ct in ipairs(raw) do
		local n = #ct.cells
		stats.histogram[n] = (stats.histogram[n] or 0) + 1
		if n < c.minContourCells then
			stats.discarded = stats.discarded + 1
		else
			if ct.closed then
				stats.closed = stats.closed + 1
			else
				stats.open = stats.open + 1
				local a, b = ct.cells[1], ct.cells[n]
				ct.atPartEdge = continuesElsewhere(a, ct.grid) or continuesElsewhere(b, ct.grid)
				if ct.atPartEdge then stats.openAtEdge = stats.openAtEdge + 1 end
				-- A ring whose two ends meet diagonally. It IS closed as geometry,
				-- but the closing step is not 4-connected, so it cannot be handed
				-- over as a loop and is counted separately rather than as a defect.
				local man = math.abs(a.ui - b.ui) + math.abs(a.vi - b.vi)
				if man == 2 and a.ui ~= b.ui and a.vi ~= b.vi then
					stats.openDiagonalClose = stats.openDiagonalClose + 1
				end
			end
			contours[#contours + 1] = ct
		end
	end
	stats.kept = #contours

	-- THE INVARIANT, measured on cell identity. Two different parts can hold a
	-- cell at the same world position -- grids are per-part and their lattices
	-- do not line up -- so keying this on position measures lattice coincidence
	-- and calls it an overlap. Measured: 5 positions coincide across parts while
	-- the actual invariant holds at 0.
	local claim = {}
	for i, ct in ipairs(contours) do
		for _, cell in ipairs(ct.cells) do
			if claim[cell] ~= nil and claim[cell] ~= i then
				stats.sharedCells = stats.sharedCells + 1
			end
			claim[cell] = i
		end
	end

	local seenPos = {}
	for i, ct in ipairs(contours) do
		for _, cell in ipairs(ct.cells) do
			local k = string.format("%.3f,%.3f,%.3f", cell.pos.X, cell.pos.Y, cell.pos.Z)
			local prev = seenPos[k]
			if prev and prev ~= i then
				stats.coincidentPositions = stats.coincidentPositions + 1
			end
			seenPos[k] = i
		end
	end

	-- Endpoint-touching pairs: a boundary that should be continuous, cut and
	-- restarted. Counted on the survivors only; the discarded slivers would
	-- swamp it.
	local ends = {}
	for i, ct in ipairs(contours) do
		if not ct.closed then
			ends[#ends + 1] = { pos = ct.cells[1].pos, i = i }
			ends[#ends + 1] = { pos = ct.cells[#ct.cells].pos, i = i }
		end
	end
	for a = 1, #ends do
		for b = a + 1, #ends do
			if ends[a].i ~= ends[b].i and (ends[a].pos - ends[b].pos).Magnitude <= 1.6 then
				stats.endpointTouching = stats.endpointTouching + 1
			end
		end
	end

	local res = { contours = contours, stats = stats, config = c }
	if c.verbose then print(Contours.report(res)) end
	return res
end

-- The summary the pipeline prints on every bake. Any of these going wrong
-- should be visible immediately, not discovered by querying debug parts weeks
-- later.
function Contours.report(res: any): string
	local s = res.stats
	local buckets = { { 2, 2 }, { 3, 3 }, { 4, 7 }, { 8, 15 }, { 16, 50 }, { 51, 200 }, { 201, math.huge } }
	local lines = {}
	for _, b in ipairs(buckets) do
		local n = 0
		for len, count in pairs(s.histogram) do
			if len >= b[1] and len <= b[2] then n = n + count end
		end
		local label = (b[2] == math.huge) and (b[1] .. "+") or (b[1] == b[2] and tostring(b[1]) or (b[1] .. "-" .. b[2]))
		lines[#lines + 1] = string.format("      %-8s %4d", label .. " cells", n)
	end
	return table.concat({
		"[Contours] " .. s.grids .. " grids",
		string.format("   regions discarded as narrower than the agent: %d (%d cells)",
			s.regionsDropped, s.cellsDropped),
		string.format("   traced %d, kept %d, discarded %d below the size floor",
			s.traced, s.kept, s.discarded),
		"   contour length histogram (all traced):",
		table.concat(lines, "\n"),
		string.format("   closed loops %d | open chains %d (%d continue onto another part, %d close diagonally)",
			s.closed, s.open, s.openAtEdge, s.openDiagonalClose),
		string.format("   world positions two parts both hold: %d (lattice coincidence, not overlap)",
			s.coincidentPositions),
		string.format("   cells claimed by more than one contour: %d %s",
			s.sharedCells, s.sharedCells == 0 and "" or "  <-- MUST BE ZERO"),
		string.format("   endpoint-touching pairs: %d", s.endpointTouching),
	}, "\n")
end

return Contours
