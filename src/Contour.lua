--!strict
-- NVGN.Contour -- Recast stage 4, part one: turn a region's cells into LINES.
--
-- Input is one REGION's cells (strictly coplanar, from NodeWalk.regions), given
-- as the drawn debug Parts or as {cf=CFrame} entries. Output is:
--   * loops    -- the region's boundary, traced as closed rings of cells
--   * tangent  -- a direction per border cell, fitted from its neighbours
--   * lines    -- runs of consistent direction, one per straight stretch
--
-- On case5 region #40 (90836 cells) this turns a 4480-step staircase boundary
-- into 91 lines, median 18 cells, longest 470.
--
-- WHY DIRECTION FITTING RATHER THAN CHORD SIMPLIFICATION.
-- The obvious approach -- greedily extend a chord while it stays inside the
-- region -- was tried and rejected. It handles convex boundaries beautifully
-- (a 400-step diagonal collapsed to one 199-stud edge) but cannot straighten a
-- CONCAVE staircase, because any chord across a notch leaves the region. On
-- region #40 that left 71% of its output edges at one stud or shorter. Fitting
-- a direction field and cutting where the direction turns has no such blind
-- spot: a staircase has a constant tangent whichever way it bends.

local Contour = {}

local DEFAULT = {
	leaf = 0.5,

	-- PCA window for the tangent fit, in studs. Measured on region #40:
	--   1.5 -> mean linearity 0.973, 20 low-linearity cells
	--   3.0 -> 0.955,  99
	--   5.0 -> 0.946, 145
	-- Bigger windows round off corners. 1.5-3.0 is the usable band.
	window = 1.5,

	-- Start a new line when the tangent leaves the line's ANCHOR direction by
	-- more than this. Must be compared against the anchor, never a running mean
	-- -- a running mean drifts along a curve and swallows whole loops.
	tol = 20,

	minCells = 5,   -- a line shorter than this is a corner fragment, not a line
	mergeCap = 20,  -- never merge a fragment into a neighbour further off than this
	thickness = 1,  -- border band depth, in cells

	-- How far a short stretch must diverge from BOTH neighbours before it is
	-- treated as a line in its own right rather than a corner fragment. This is
	-- deliberately much larger than mergeCap: gating on mergeCap (20) keeps every
	-- fragment the merge pass refused, which on region 40 gave 190 lines with 105
	-- of them under 5 cells -- the exact failure the distribution pass exists to
	-- prevent. The stubs worth keeping are near perpendicular to their neighbours
	-- (measured: 45-79 degrees on line 69, ~50 on line 8), not merely off-axis.
	keepAngle = 45,

	-- A loop no wider than this (in CELLS) skips the tangent field entirely and
	-- goes through Contour.smallLoop. Defaults to the tangent window, because
	-- that is exactly the point at which the fit stops meaning anything -- see
	-- the note on smallLoop below.
	smallLoop = 3,

	-- U-turn fold detection: how many cells either side of a point to compare,
	-- and how strongly they must oppose. -0.7 keeps ordinary corners out -- they
	-- turn, they do not reverse.
	foldWindow = 3,
	foldDot = -0.7,

	-- U-turn pairing. A U-turn is two lines running ALONGSIDE each other, so it
	-- is identified from the pair as a whole -- near-parallel axes, a small
	-- perpendicular separation, and a real overlap -- and then a connector is
	-- emitted at EACH end where both lines terminate.
	--
	-- Requiring the connector itself to be perpendicular (the first attempt) only
	-- works when the two rows are the same length. They usually are not: on case5
	-- the pairs were 12 vs 10 cells and 71 vs 399, so one end squares off and the
	-- other is STAGGERED along the axis. The staggered end failed the
	-- perpendicularity test (measured 2.0 degrees), got no connector, and
	-- Contour.connect then welded the two parallel rows straight to each other --
	-- reinstating the 180-degree reversal as a 5.7-degree spike.
	sepMax = 2.5,    -- perpendicular separation, in cells, for a side-by-side pair
	joinMax = 3.0,   -- furthest apart two ends may be and still get a connector
	overlapMin = 2,  -- cells of overlap before a pair counts as running alongside

	-- Shortest edge worth keeping, in studs. A 1-stud chamfer is a real feature
	-- and the segmentation is right to find it, but at this scale it is a corner
	-- wearing a line's clothes: it costs an edge, two joints and a colour to say
	-- what a single corner says. Below this the edge is dissolved and its two
	-- neighbours are extended to meet each other -- but ONLY if the replacement
	-- chords stay inside the region, since cutting a corner is exactly how a
	-- boundary ends up running through a wall. Set to 0 to keep every edge.
	minEdge = 1.5,
	-- How far off the lattice a chord may stray before it counts as having left
	-- the region, in cells. 1 allows the ordinary case of a chord running along
	-- the outside of the border band.
	chordSlack = 1,
	-- How many times a line may be split trying to keep its chord inside. A
	-- boundary that needs more than this is not a line under any subdivision.
	splitDepth = 8,

	-- Line-of-sight validation. The lattice test only knows whether a chord
	-- passes over cells of this region; it cannot see a wall standing between
	-- two cells at the same height. A ray between the two corner nodes can.
	rayLift = 0.6,     -- studs along the surface normal, to clear the floor itself
	-- Shortest chord worth casting along. This exists to skip sub-cell noise,
	-- NOT to protect small loops -- at 1.5 it exempted every 2-to-3 cell line,
	-- and a 1-stud line crosses a thin wall perfectly well. Small loops are
	-- protected by rayMinCells and by refusing to shred what cannot be walked.
	rayMinChord = 0.75,
	rayMinCells = 2,   -- never walk a line below this many cells
}

local function merged(cfg)
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

-- Angles live on a 180-degree axis (a line has no head or tail), so ordinary
-- arithmetic on them is wrong. A linear max-minus-min calls 179 and 1 a spread
-- of 178 when they are 2 degrees apart -- that mistake once reported 32 broken
-- lines where there were 12. Always use these two.
local function angDiff(a: number, b: number): number
	local d = math.abs(a - b) % 180
	return (d > 90) and (180 - d) or d
end

local function meanAngle(list, tangent): number
	local sx, sy = 0, 0
	for _, k in ipairs(list) do
		local t = tangent[k]
		if t then sx += math.cos(2 * math.rad(t)); sy += math.sin(2 * math.rad(t)) end
	end
	return (math.deg(math.atan2(sy, sx)) / 2) % 180
end

local N4 = { {1,0}, {-1,0}, {0,1}, {0,-1} }
local function K(i: number, j: number): number
	-- biased so negative lattice coordinates still pack safely
	return (i + 50000) * 100000 + (j + 50000)
end

-- Project a region's cells onto its own plane and index them.
--
-- THE ANCHOR CELL IS CHOSEN, NOT TAKEN. Using parts[1] makes the whole lattice
-- depend on iteration order: every coordinate is rounded relative to that cell,
-- so a different first cell shifts the grid. The shift is usually a whole number
-- of cells and nothing downstream notices, but when the origin lands near a
-- rounding boundary a few cells fall into different (i, j) and the contour comes
-- out slightly different. Measured on region 40: shuffling the input left the
-- cell count identical at 90836 every time, yet one shuffle in six produced a
-- different set of edges. Pick the anchor from the GEOMETRY instead -- the
-- lexicographically smallest position -- so the lattice is a property of the
-- region rather than of the order its parts happened to arrive in. Exact
-- comparison, never an accumulated sum: adding coordinates in a different order
-- is itself order-dependent in floating point.
function Contour.lattice(parts, cfg)
	local c = merged(cfg)
	local first = parts[1]
	local fp = first.Position or (first.cf and first.cf.Position)
	for _, p in ipairs(parts) do
		local q = p.Position or p.cf.Position
		if q.X < fp.X
			or (q.X == fp.X and q.Y < fp.Y)
			or (q.X == fp.X and q.Y == fp.Y and q.Z < fp.Z) then
			first = p; fp = q
		end
	end
	local cf0 = first.CFrame or first.cf
	local up, uAx, vAx = cf0.UpVector, cf0.RightVector, cf0.LookVector
	local origin = fp

	local occ, partAt, coord = {}, {}, {}
	for _, p in ipairs(parts) do
		local pos = p.Position or p.cf.Position
		local d = pos - origin
		local i = math.round(d:Dot(uAx) / c.leaf)
		local j = math.round(d:Dot(vAx) / c.leaf)
		local k = K(i, j)
		occ[k] = true; partAt[k] = p; coord[k] = { i, j }
	end
	return { occ = occ, partAt = partAt, coord = coord,
	         origin = origin, up = up, u = uAx, v = vAx, leaf = c.leaf }
end

-- Border cells, 4-CONNECTED.
-- 8-connectivity was measured and rejected for this purpose: it adds the inner
-- corners of staircases (+630 cells on region #40) and makes the tangent fit
-- slightly WORSE (linearity 0.973 -> 0.959), because those cells sit off the
-- line the rest of the stretch defines. It is the right rule for erosion or for
-- catching corner-to-corner pinch points -- just not for tracing, where a loop
-- is built from cell FACES and a diagonal contact has no face to contribute.
function Contour.border(L, cfg)
	local c = merged(cfg)
	local seed = {}
	for k, cc in pairs(L.coord) do
		for _, d in ipairs(N4) do
			if not L.occ[K(cc[1]+d[1], cc[2]+d[2])] then seed[k] = true; break end
		end
	end
	-- optional inward band
	local depth, q = {}, {}
	for k in pairs(seed) do depth[k] = 1; table.insert(q, k) end
	local head = 1
	while head <= #q do
		local k = q[head]; head += 1
		if depth[k] < c.thickness then
			local cc = L.coord[k]
			for _, d in ipairs(N4) do
				local nk = K(cc[1]+d[1], cc[2]+d[2])
				if L.occ[nk] and not depth[nk] then depth[nk] = depth[k] + 1; table.insert(q, nk) end
			end
		end
	end
	return seed, depth
end

-- Trace closed loops. Every cell face that borders a non-region cell becomes a
-- directed edge, wound so the region is always on the LEFT; chaining them
-- end-to-start yields one loop per boundary component -- outer ring first, then
-- one per hole. No heuristic, just connectivity.
function Contour.loops(L)
	local edges, starts = {}, {}
	local function add(ax, ay, bx, by, cell)
		local e = { ax, ay, bx, by, cell }
		edges[#edges+1] = e
		local kk = K(ax, ay)
		local b = starts[kk]; if not b then b = {}; starts[kk] = b end
		table.insert(b, e)
	end
	for k, cc in pairs(L.coord) do
		local i, j = cc[1], cc[2]
		if not L.occ[K(i, j-1)] then add(i, j, i+1, j, k) end
		if not L.occ[K(i+1, j)] then add(i+1, j, i+1, j+1, k) end
		if not L.occ[K(i, j+1)] then add(i+1, j+1, i, j+1, k) end
		if not L.occ[K(i-1, j)] then add(i, j+1, i, j, k) end
	end
	local used, out = {}, {}
	for _, e in ipairs(edges) do
		if not used[e] then
			local seq = {}
			local cur = e
			while cur and not used[cur] do
				used[cur] = true
				if seq[#seq] ~= cur[5] then seq[#seq+1] = cur[5] end
				local cand = starts[K(cur[3], cur[4])]
				local nxt = nil
				if cand then for _, x in ipairs(cand) do if not used[x] then nxt = x; break end end end
				cur = nxt
			end
			if #seq > 3 then table.insert(out, seq) end
		end
	end
	table.sort(out, function(a, b) return #a > #b end)
	return out
end

-- A direction per border cell: principal axis of its neighbourhood.
-- On region #40 adjacent cells agree to within 2 degrees at the 75th percentile,
-- and the angle histogram shows clean peaks on the real edge directions rather
-- than the staircase's jitter.
function Contour.tangents(L, seed, cfg)
	local c = merged(cfg)
	local R = c.window / c.leaf
	local CELL = 8
	local hash = {}
	for k in pairs(seed) do
		local cc = L.coord[k]
		local hh = math.floor(cc[1]/CELL) * 100000 + math.floor(cc[2]/CELL)
		local b = hash[hh]; if not b then b = {}; hash[hh] = b end
		table.insert(b, k)
	end
	local tangent, linearity = {}, {}
	local rc = math.ceil(R / CELL)
	for k in pairs(seed) do
		local cc = L.coord[k]
		local nb = {}
		for di = -rc, rc do for dj = -rc, rc do
			local b = hash[math.floor(cc[1]/CELL + di) * 100000 + math.floor(cc[2]/CELL + dj)]
			if b then for _, o in ipairs(b) do
				local oc = L.coord[o]
				local dx, dy = oc[1]-cc[1], oc[2]-cc[2]
				if dx*dx + dy*dy <= R*R then table.insert(nb, o) end
			end end
		end end
		if #nb >= 3 then
			local m = #nb
			local mx, my = 0, 0
			for _, o in ipairs(nb) do mx += L.coord[o][1]; my += L.coord[o][2] end
			mx, my = mx/m, my/m
			local sxx, sxy, syy = 0, 0, 0
			for _, o in ipairs(nb) do
				local dx, dy = L.coord[o][1]-mx, L.coord[o][2]-my
				sxx += dx*dx; sxy += dx*dy; syy += dy*dy
			end
			local tr, det = sxx + syy, sxx*syy - sxy*sxy
			local disc = math.max(0, tr*tr/4 - det)
			local l1, l2 = tr/2 + math.sqrt(disc), tr/2 - math.sqrt(disc)
			local ang
			if math.abs(sxy) > 1e-9 then ang = math.atan2(l1 - sxx, sxy)
			else ang = (sxx >= syy) and 0 or math.pi/2 end
			tangent[k] = math.deg(ang) % 180
			linearity[k] = (l1 > 1e-9) and (1 - l2/l1) or 0
		end
	end
	return tangent, linearity
end

-- SMALL LOOPS DO NOT HAVE TANGENTS. When a loop is no bigger than the PCA
-- window, every cell's neighbourhood wraps the whole ring, so the fit returns
-- the ring's DIAMETER rather than any local direction. Measured on the 8-cell
-- ring around case5's 1-stud Target (extent 1.55 studs, window 1.5): cells on
-- OPPOSITE sides of the obstacle came back with identical tangents, and the
-- segmentation duly grouped them into one line whose chord ran straight through
-- the obstacle. The fragment-distribution pass cannot rescue it either -- that
-- is guarded by #segs > 2 and such a loop only ever yields two.
--
-- The raw cell-to-cell step still carries the truth, so use it directly: walk
-- the ring and take each MAXIMAL RUN OF EQUAL-DIRECTION STEPS as one edge.
--
-- The one special case is a lone DIAGONAL step between two runs: that is a
-- corner, not an edge, so it is dropped and its two cells stay with the faces
-- either side. This distinction is what the rule turns on, and it is easy to
-- get wrong -- cutting only at diagonals looks right on the Target ring (whose
-- 8-degree rotation makes every corner a diagonal step) but returns an
-- axis-aligned ring as one single edge, because there its corners are axis
-- steps that merely turn. Both are covered here:
--   * rotated ring   -- steps alternate axis/diagonal; four length-1 axis runs
--                       become the four faces, four diagonals are dropped
--   * axis-aligned   -- every step is an axis step; direction changes four
--                       times, giving the four faces
--   * diamond        -- one long diagonal run per side, kept as real edges,
--                       since a run of diagonals is a 45-degree face
function Contour.smallLoop(L, seq)
	local function sgn(x) return (x > 0 and 1) or (x < 0 and -1) or 0 end
	local dir = {}
	for i = 1, #seq do
		local ca, cb = L.coord[seq[i]], L.coord[seq[i % #seq + 1]]
		dir[i] = { sgn(cb[1] - ca[1]), sgn(cb[2] - ca[2]) }
	end

	-- start the walk at a direction change so runs are never split by the seam
	local start = 1
	for i = 1, #seq do
		local p = dir[(i - 2) % #seq + 1]
		if dir[i][1] ~= p[1] or dir[i][2] ~= p[2] then start = i; break end
	end

	local edges, run = {}, nil
	local function flush()
		if not run then return end
		local d = dir[run[1]]
		local diagonal = (d[1] ~= 0 and d[2] ~= 0)
		if not (diagonal and #run == 1) then
			local cells = {}
			for _, i in ipairs(run) do table.insert(cells, seq[i]) end
			table.insert(cells, seq[run[#run] % #seq + 1])
			table.insert(edges, cells)
		end
		run = nil
	end
	for x = 0, #seq - 1 do
		local i = (start - 1 + x) % #seq + 1
		if run and (dir[i][1] ~= dir[run[1]][1] or dir[i][2] ~= dir[run[1]][2]) then flush() end
		run = run or {}
		table.insert(run, i)
	end
	flush()
	return edges
end

local function loopExtent(L, seq)
	local mni, mxi, mnj, mxj = math.huge, -math.huge, math.huge, -math.huge
	for _, k in ipairs(seq) do
		local cc = L.coord[k]
		mni = math.min(mni, cc[1]); mxi = math.max(mxi, cc[1])
		mnj = math.min(mnj, cc[2]); mxj = math.max(mxj, cc[2])
	end
	return math.max(mxi - mni, mxj - mnj)
end

-- Cut each loop into lines wherever the direction turns.
function Contour.lines(L, loops, tangent, cfg)
	local c = merged(cfg)
	local out, stats = {}, { distributed = 0, refused = 0, smallLoops = 0, uturns = 0, kept = 0 }

	for _, seq in ipairs(loops) do
		if loopExtent(L, seq) <= c.smallLoop then
			for _, e in ipairs(Contour.smallLoop(L, seq)) do table.insert(out, e) end
			stats.smallLoops += 1
			continue
		end
		-- CANONICAL START. Greedy segmentation depends on where you begin, so
		-- begin at the loop's sharpest turn: a real corner, found independently of
		-- the order the edge chain happened to be built in. This is what makes the
		-- result reproducible instead of an artefact of the seed cell.
		local bestTurn, bestAt = -1, 1
		for x = 1, #seq do
			local a, b = tangent[seq[x]], tangent[seq[(x % #seq) + 1]]
			if a and b then
				local d = angDiff(a, b)
				if d > bestTurn then bestTurn = d; bestAt = x end
			end
		end
		local start = (bestAt % #seq) + 1

		-- U-TURNS ARE INVISIBLE TO angDiff. Tangents live on a 180-degree axis, so
		-- a boundary that doubles back on itself reads as no turn at all:
		-- angDiff(0, 180) is 0. Where the border runs out along one row of cells
		-- and returns along the row beside it, both runs carry the SAME tangent and
		-- the segmentation joins them into one line whose chord covers only the
		-- longer run. Measured on case5 line 1: 470 cells in two parallel rows 0.5
		-- studs apart, 400 out and 70 back, every one of them tangent 0, and the
		-- 70-cell return run left with no edge over it.
		--
		-- The loop ORDER still knows, so carry an orientation the tangent threw
		-- away: accumulate the segment's net step and cut when a new step opposes
		-- it. Net displacement rather than the previous step alone, because a
		-- staircase's steps alternate between two perpendicular axes and any
		-- step-to-step test would cut at every tread.
		local segs, cur, anchor = {}, nil, nil
		local prevK, accI, accJ = nil, 0, 0
		for x = 0, #seq - 1 do
			local k = seq[((start - 1 + x) % #seq) + 1]
			local t = tangent[k]
			if t then
				local reversed = false
				if prevK and cur then
					local ca, cb = L.coord[prevK], L.coord[k]
					local si, sj = cb[1] - ca[1], cb[2] - ca[2]
					if si * accI + sj * accJ < 0 then
						reversed = true
					else
						accI += si; accJ += sj
					end
				end
				if cur == nil or reversed or angDiff(t, anchor) > c.tol then
					cur = { k }; anchor = t; table.insert(segs, cur)
					accI, accJ = 0, 0
					if reversed then stats.uturns += 1 end
				else
					table.insert(cur, k)
				end
				prevK = k
			end
		end

		-- Absorb fragments, but ONLY into a neighbour pointing the same way.
		-- Uncapped merging drags a line's direction off and produced 12 lines
		-- bending past 45 degrees from their own mean.
		local locked, guard = {}, 0
		while #segs > 1 and guard < 20000 do
			guard += 1
			local smallest, at = math.huge, nil
			for i, s in ipairs(segs) do
				if not locked[s] and #s < smallest then smallest = #s; at = i end
			end
			if at == nil or smallest >= c.minCells then break end
			local me = meanAngle(segs[at], tangent)
			local prev = segs[(at - 2) % #segs + 1]
			local nxt  = segs[at % #segs + 1]
			local dp, dn = angDiff(me, meanAngle(prev, tangent)), angDiff(me, meanAngle(nxt, tangent))
			local target = (dp <= dn) and prev or nxt
			if math.min(dp, dn) > c.mergeCap then
				locked[segs[at]] = true; stats.refused += 1
			else
				for _, k in ipairs(segs[at]) do table.insert(target, k) end
				table.remove(segs, at)
			end
		end

		-- Whatever is still too short gets split CELL BY CELL between its two
		-- neighbours: each cell joins the side it actually agrees with, so a corner
		-- becomes the boundary between two lines rather than a 2-cell line of its
		-- own. Keeping fragments gave 193 lines of which 109 were under 5 cells;
		-- distributing gives 91 lines with one.
		-- ...but distribution must respect mergeCap too, which it originally did
		-- not. A stretch under minCells that agrees with NEITHER neighbour is not a
		-- corner fragment, it is a short LINE, and forcing it into a neighbour
		-- silently merges two real lines into one. Measured on case5: line 69 was a
		-- 21-cell run at tangent 45 with a 4-cell stub running perpendicular to it,
		-- emitted as a single line whose two-piece fit is 100% better than its
		-- one-piece fit; line 8 was the same failure at 14 + 2 cells. Both stubs
		-- were under minCells, so the uncapped pass swallowed them.
		local keep = {}
		local changed = true
		while changed do
			changed = false
			for at = #segs, 1, -1 do
				if #segs > 2 and #segs[at] < c.minCells and not keep[segs[at]] then
					local prev = segs[(at - 2) % #segs + 1]
					local nxt  = segs[at % #segs + 1]
					local mp, mn = meanAngle(prev, tangent), meanAngle(nxt, tangent)
					local me = meanAngle(segs[at], tangent)
					if math.min(angDiff(me, mp), angDiff(me, mn)) >= c.keepAngle then
						keep[segs[at]] = true
						stats.kept += 1
					else
						for _, k in ipairs(segs[at]) do
							if angDiff(tangent[k], mp) <= angDiff(tangent[k], mn) then
								table.insert(prev, k)
							else
								table.insert(nxt, k)
							end
							stats.distributed += 1
						end
						table.remove(segs, at)
						changed = true
					end
				end
			end
		end

		for _, s in ipairs(segs) do table.insert(out, s) end
	end
	return out, stats
end

-- A U-turn leaves two lines running alongside each other with no edge between
-- them, so the boundary cannot be walked: it arrives at the end of one row and
-- has nowhere to go. Emit the single lattice step across the gap as its own
-- edge, which turns one 180-degree reversal into two 90-degree turns.
--
-- IDENTIFYING THEM WITHOUT WINDING. The obvious test -- two lines whose
-- directions are antiparallel -- cannot be used, because a line's stored
-- direction comes from a fit and carries no head or tail. Measured on the
-- case5 U-turn, the two lines' fitted directions came out exactly PARALLEL
-- (dot +1.0000), and testing for antiparallel instead produced two false hits
-- elsewhere in the region. All four conditions below are undirected:
--   1. an endpoint of each line, within one leaf of each other
--   2. the two line axes near parallel      (undirected, <= 20 degrees)
--   3. the connector near perpendicular to both        (>= 70 degrees)
--   4. the lines OVERLAP along their shared axis -- they run alongside each
--      other rather than meeting end to end, which is what separates a U-turn
--      from an ordinary collinear hand-off
-- A LINE'S CELL LIST IS NOT IN ORDER. Both the merge and the distribution pass
-- append a fragment's cells onto the end of a neighbouring segment, so seg[1]
-- and seg[#seg] are whatever happened to land there, not the line's two ends.
-- That is harmless for painting, which is all lines were used for, but any
-- geometry built from the list order is wrong: taking the ends on trust left
-- only 100 of 200 endpoints joined on region 40, some of them 30+ studs adrift.
-- Always take the extremes along the line's own axis instead.
local function endpointsOf(L, seg)
	local n = #seg
	local mi, mj = 0, 0
	for _, k in ipairs(seg) do mi += L.coord[k][1]; mj += L.coord[k][2] end
	mi, mj = mi / n, mj / n
	local sxx, sxy, syy = 0, 0, 0
	for _, k in ipairs(seg) do
		local dx, dy = L.coord[k][1] - mi, L.coord[k][2] - mj
		sxx += dx*dx; sxy += dx*dy; syy += dy*dy
	end
	local tr = sxx + syy
	local disc = math.max(0, tr*tr/4 - (sxx*syy - sxy*sxy))
	local l1 = tr/2 + math.sqrt(disc)
	local ax, ay
	if math.abs(sxy) > 1e-9 then ax, ay = sxy, l1 - sxx
	elseif sxx >= syy then ax, ay = 1, 0
	else ax, ay = 0, 1 end
	local m = math.sqrt(ax*ax + ay*ay)
	if m < 1e-9 then ax, ay = 1, 0 else ax, ay = ax/m, ay/m end
	local lo, hi, kLo, kHi = math.huge, -math.huge, seg[1], seg[1]
	for _, k in ipairs(seg) do
		local t = (L.coord[k][1] - mi) * ax + (L.coord[k][2] - mj) * ay
		if t < lo then lo = t; kLo = k end
		if t > hi then hi = t; kHi = k end
	end
	return kLo, kHi, ax, ay
end

function Contour.connectors(L, lines, cfg)
	local c = merged(cfg)
	local function pca(seg)
		local n = #seg
		local mi, mj = 0, 0
		for _, k in ipairs(seg) do mi += L.coord[k][1]; mj += L.coord[k][2] end
		mi, mj = mi/n, mj/n
		local sxx, sxy, syy = 0, 0, 0
		for _, k in ipairs(seg) do
			local dx, dy = L.coord[k][1]-mi, L.coord[k][2]-mj
			sxx += dx*dx; sxy += dx*dy; syy += dy*dy
		end
		local tr = sxx + syy
		local disc = math.max(0, tr*tr/4 - (sxx*syy - sxy*sxy))
		local l1 = tr/2 + math.sqrt(disc)
		local ax, ay
		if math.abs(sxy) > 1e-9 then ax, ay = sxy, l1 - sxx
		elseif sxx >= syy then ax, ay = 1, 0
		else ax, ay = 0, 1 end
		local m = math.sqrt(ax*ax + ay*ay)
		if m < 1e-9 then return 1, 0, mi, mj end
		return ax/m, ay/m, mi, mj
	end
	local function undirected(ax, ay, bx, by)
		return math.deg(math.acos(math.clamp(math.abs(ax*bx + ay*by), -1, 1)))
	end

	local axis, ends = {}, {}
	for id, seg in ipairs(lines) do
		if #seg >= 2 then
			local ax, ay = pca(seg)
			axis[id] = { ax, ay }
			local kLo, kHi = endpointsOf(L, seg)
			ends[id] = { kLo, kHi }
		end
	end

	-- Overlap along a's axis, and how far b sits off it. Both in cells.
	local function relation(a, b)
		local ax, ay = axis[a][1], axis[a][2]
		local px, py = -ay, ax
		local ref = L.coord[lines[a][1]]
		local function proj(seg)
			local lo, hi, plo, phi = math.huge, -math.huge, math.huge, -math.huge
			for _, k in ipairs(seg) do
				local dx, dy = L.coord[k][1] - ref[1], L.coord[k][2] - ref[2]
				local t, s = dx*ax + dy*ay, dx*px + dy*py
				lo = math.min(lo, t); hi = math.max(hi, t)
				plo = math.min(plo, s); phi = math.max(phi, s)
			end
			return lo, hi, (plo + phi) / 2
		end
		local loA, hiA, sA = proj(lines[a])
		local loB, hiB, sB = proj(lines[b])
		return math.min(hiA, hiB) - math.max(loA, loB), math.abs(sB - sA)
	end

	local out, seen = {}, {}
	for a in pairs(axis) do
		for b in pairs(axis) do
			if a < b and undirected(axis[a][1], axis[a][2], axis[b][1], axis[b][2]) <= 20 then
				local overlap, sep = relation(a, b)
				if overlap > c.overlapMin and sep > 0.1 and sep <= c.sepMax then
					-- a genuine side-by-side pair: close whichever ends terminate together
					local cand = {}
					for ia, ka in ipairs(ends[a]) do
						for ib, kb in ipairs(ends[b]) do
							local ca, cb = L.coord[ka], L.coord[kb]
							local vx, vy = cb[1]-ca[1], cb[2]-ca[2]
							local gap = math.sqrt(vx*vx + vy*vy)
							-- The connector must CROSS the sliver. Dropping this test (tried,
							-- to reach the staggered end of an uneven pair) emits a stub lying
							-- along the row instead: on region 52 it produced (0,0)->(-1,0),
							-- both cells in the same row, parallel to the line it was meant to
							-- turn away from. That just moves the spike from row-to-row onto
							-- row-to-connector, at the very same angle.
							if gap > 1e-6 and gap <= c.joinMax
								and undirected(vx/gap, vy/gap, axis[a][1], axis[a][2]) >= 70 then
								table.insert(cand, { gap = gap, ka = ka, kb = kb, ia = ia, ib = ib })
							end
						end
					end
					table.sort(cand, function(x, y) return x.gap < y.gap end)
					-- at most one connector per end, and never reuse an endpoint
					local usedA, usedB = {}, {}
					for _, e in ipairs(cand) do
						if not usedA[e.ia] and not usedB[e.ib] then
							usedA[e.ia] = true; usedB[e.ib] = true
							local sig = math.min(e.ka, e.kb) .. "_" .. math.max(e.ka, e.kb)
							if not seen[sig] then
								seen[sig] = true
								table.insert(out, { e.ka, e.kb })
							end
						end
					end
				end
			end
		end
	end
	return out
end

-- CLOSING A U-TURN BY STEALING ITS CORNER.
--
-- Where the boundary doubles back, the two rows must be joined by a
-- perpendicular edge or the polygon cannot be walked. Building that edge BETWEEN
-- two lines' endpoints does not work, and the reason is worth keeping: at the
-- turn, ONE line usually owns BOTH corner cells. Measured on region 40, cells
-- (399,-1) and (399,0) -- the two rows' real ends -- were both in line 1, which
-- ran down one row, turned, and came back along the other. So there is no
-- cross-line pair at the true corner, and a search for one silently falls back
-- to the nearest pair that IS split, one cell behind the turn.
--
-- That same wrap breaks endpointsOf, which takes the extremes along a line's
-- fitted axis: on an L-shaped line the two corner cells sit at the SAME position
-- along that axis and differ only across it, so one is called an end and the
-- other is treated as interior.
--
-- Both problems dissolve if the corner is taken out and made its own line:
--   1. find the fold from the LOOP ORDER -- a window before and after the point
--      facing opposite ways -- then take the single perpendicular step inside it
--   2. remove those two cells from whatever line owns them
--   3. split any remnant left disconnected, which un-wraps the L
-- The stolen pair is perpendicular by construction, so no angle test is needed.
function Contour.folds(L, seq, cfg)
	local c = merged(cfg)
	local W = c.foldWindow
	local n = #seq
	local function co(x) return L.coord[seq[(x - 1) % n + 1]] end
	local hits = {}
	for i = 1, n do
		local a, b, cc = co(i - W), co(i + W), co(i)
		local d1x, d1y = cc[1] - a[1], cc[2] - a[2]
		local d2x, d2y = b[1] - cc[1], b[2] - cc[2]
		local m1 = math.sqrt(d1x*d1x + d1y*d1y)
		local m2 = math.sqrt(d2x*d2x + d2y*d2y)
		-- a U-turn REVERSES; an ordinary corner merely turns
		if m1 > 0 and m2 > 0 and (d1x*d2x + d1y*d2y) / (m1*m2) < c.foldDot then
			hits[#hits+1] = i
		end
	end
	local out, i = {}, 1
	while i <= #hits do
		local j = i
		while j < #hits and hits[j+1] - hits[j] <= 2 do j += 1 end
		local mid = hits[math.max(1, math.floor((i + j) / 2))]
		if mid then
			local ref, cc = co(mid - W), co(mid)
			local rx, ry = cc[1] - ref[1], cc[2] - ref[2]
			local best, bd = nil, math.huge
			for m = mid - W, mid + W do
				local p1, p2 = co(m), co(m + 1)
				local sx, sy = p2[1] - p1[1], p2[2] - p1[2]
				-- one cell, square across the run: that is the corner step
				if (sx*sx + sy*sy) == 1 and (sx*rx + sy*ry) == 0 then
					local d = math.abs(m - mid)
					if d < bd then bd = d; best = { seq[(m-1) % n + 1], seq[m % n + 1] } end
				end
			end
			if best then table.insert(out, best) end
		end
		i = j + 1
	end
	return out
end

-- 8-connected components of a cell set, used to split a line the steal cut in two
local function components(L, seg)
	local inSeg, comps, seen, byXY = {}, {}, {}, {}
	for _, k in ipairs(seg) do
		inSeg[k] = true
		byXY[L.coord[k][1] .. "," .. L.coord[k][2]] = k
	end
	for _, k in ipairs(seg) do
		if not seen[k] then
			local stack, comp = { k }, {}
			seen[k] = true
			while #stack > 0 do
				local cur = table.remove(stack)
				comp[#comp+1] = cur
				local cc = L.coord[cur]
				for dx = -1, 1 do for dy = -1, 1 do
					if dx ~= 0 or dy ~= 0 then
						local nk = byXY[(cc[1]+dx) .. "," .. (cc[2]+dy)]
						if nk and inSeg[nk] and not seen[nk] then
							seen[nk] = true; table.insert(stack, nk)
						end
					end
				end end
			end
			comps[#comps+1] = comp
		end
	end
	return comps
end

-- Steal every fold's corner pair, split what that disconnects, and append the
-- pairs as lines of their own. Returns the new line list and stats.
function Contour.stealUTurns(L, lines, loops, cfg)
	local c = merged(cfg)
	local steal, taken = {}, {}
	for _, seq in ipairs(loops) do
		-- small loops have their own path and no U-turns to speak of
		if loopExtent(L, seq) > c.smallLoop then
			for _, pair in ipairs(Contour.folds(L, seq, c)) do
				if not taken[pair[1]] and not taken[pair[2]] then
					taken[pair[1]] = true; taken[pair[2]] = true
					table.insert(steal, pair)
				end
			end
		end
	end
	local out, splits = {}, 0
	for _, seg in ipairs(lines) do
		local keep = {}
		for _, k in ipairs(seg) do if not taken[k] then table.insert(keep, k) end end
		if #keep == #seg then
			table.insert(out, keep)
		elseif #keep > 0 then
			local comps = components(L, keep)
			if #comps > 1 then splits += 1 end
			for _, comp in ipairs(comps) do table.insert(out, comp) end
		end
	end
	local firstLink = #out + 1
	for _, pair in ipairs(steal) do table.insert(out, { pair[1], pair[2] }) end
	return out, { links = #steal, splits = splits, firstLink = firstLink }
end

-- Stage 4 part two: turn the lines into a closed polygon by EXTENDING each one
-- along its own direction until it meets its neighbour, and welding both ends to
-- that crossing. Chords stop at cell centres, so consecutive lines end about a
-- cell apart and the boundary is not walkable until they actually share a
-- vertex. Measured on case5: 198 endpoints, every one with a partner inside
-- 1.118 studs, 196 of them mutual nearest -- so the pairing is unambiguous and
-- needs no search radius beyond one cell diagonal.
--
-- Two lines that are near PARALLEL have no usable crossing: it is either
-- nowhere or absurdly far outside the region. Those weld at the midpoint
-- instead, as does any pair whose crossing lands further than maxExtend away.
-- On case5 three junctions took the parallel path (down to 0.0 degrees apart)
-- and the distance cap never fired -- every real crossing was within 0.99 studs.
--
-- This is why U-turns had to be closed first (see Contour.connectors). At a
-- U-turn the two lines are parallel and their ends are adjacent, so extension
-- cannot join them -- there is no crossing to find. The connector supplies the
-- perpendicular edge that gives each side something to meet at right angles.
-- Where two lines cross, in the region's own plane.
local function planeMeet(L, p1, d1, p2, d2)
	local function planar(v) return v:Dot(L.u), v:Dot(L.v) end
	local a1, b1 = planar(d1)
	local a2, b2 = planar(d2)
	local den = a1 * b2 - b1 * a2
	if math.abs(den) < 1e-9 then return nil end
	local rx, ry = planar(p2 - p1)
	return p1 + d1 * ((rx * b2 - ry * a2) / den)
end

-- Does the straight run from `p` to `q` stay over the region?
--
-- This is the test the fitted edges never had. A line is fitted to cells that
-- follow the boundary, but it is DRAWN as the chord between its endpoints, and
-- a chord across a concavity leaves the region -- which is how a boundary ends
-- up crossing a wall. Sampling at half a cell is finer than any feature the
-- lattice can hold, and `slack` cells of tolerance allow the ordinary case of a
-- chord running just outside the border band it was fitted to.
function Contour.chordInside(L, p, q, slack)
	slack = slack or 1
	local d = q - p
	local len = d.Magnitude
	if len < 1e-6 then return true end
	local steps = math.max(1, math.ceil(len / (L.leaf * 0.5)))
	for x = 0, steps do
		local at = p + d * (x / steps)
		local rel = at - L.origin
		local i = math.round(rel:Dot(L.u) / L.leaf)
		local j = math.round(rel:Dot(L.v) / L.leaf)
		local hit = false
		for di = -slack, slack do
			for dj = -slack, slack do
				if L.occ[K(i + di, j + dj)] then hit = true; break end
			end
			if hit then break end
		end
		if not hit then return false end
	end
	return true
end

-- Validate each line against real geometry, and cut back the end that fails.
--
-- The lattice test (chordInside) only knows whether a chord passes over cells of
-- this region. It cannot see a WALL standing between two cells at the same
-- height, because both endpoints and everything between them are perfectly good
-- floor -- the wall is simply in the way. A ray between the two corner nodes
-- sees it.
--
-- When a line fails, the end to blame is the one whose own tangent disagrees
-- most with the line as a whole: that is the end that has bent away from the
-- straight run and dragged the chord off the surface. It is walked back ONE node
-- at a time, re-casting each time, rather than bisected -- a bisect overshoots
-- and shatters a line that needed to lose two cells. The nodes walked off are
-- not discarded: they become a line of their own and are validated in turn, so
-- the boundary keeps every cell it started with.
--
-- SMALL LOOPS ARE LEFT ALONE. A ring only a couple of studs across (the hole
-- around a crate, say) has no meaningful straight run in it: its chord is
-- shorter than the ray lift, the tangents wrap the whole ring, and walking it
-- back produces noise rather than lines. `rayMinChord` and `rayMinCells` keep
-- the pass out of them.
function Contour.validateLines(L, lines, tangent, cfg)
	local c = merged(cfg)
	local stats = { tested = 0, failed = 0, walked = 0, spawned = 0, refused = 0 }
	local rp = c.rayFilter
	if not rp then return lines, stats end

	local lift = (L.up or Vector3.yAxis) * c.rayLift
	local function world(k)
		local cc = L.coord[k]
		return L.origin + L.u * (cc[1] * L.leaf) + L.v * (cc[2] * L.leaf)
	end
	local function clear(a, b)
		local d = (b + lift) - (a + lift)
		if d.Magnitude < 1e-4 then return true end
		return workspace:Raycast(a + lift, d, rp) == nil
	end

	local out = {}
	local queue = {}
	for _, seg in ipairs(lines) do queue[#queue + 1] = seg end

	local guard = 0
	while #queue > 0 and guard < 20000 do
		guard += 1
		local seg = table.remove(queue, 1)
		if #seg < c.rayMinCells then
			if #seg > 0 then out[#out + 1] = seg end
			continue
		end
		local kLo, kHi = endpointsOf(L, seg)
		local a, b = world(kLo), world(kHi)
		if (b - a).Magnitude < c.rayMinChord then
			out[#out + 1] = seg
			continue
		end
		stats.tested += 1
		if clear(a, b) then
			out[#out + 1] = seg
			continue
		end
		stats.failed += 1

		-- Which end is to blame? The one whose tangent sits furthest from the
		-- line's own mean direction. Without tangents, fall back to the end
		-- whose neighbour turns the sharpest.
		local mean = meanAngle(seg, tangent)
		local tLo = tangent and tangent[kLo]
		local tHi = tangent and tangent[kHi]
		local badIsHi
		if tLo and tHi then
			badIsHi = angDiff(tHi, mean) >= angDiff(tLo, mean)
		else
			badIsHi = true
		end

		-- walk that end back one node at a time
		local work = {}
		for i, k in ipairs(seg) do work[i] = k end
		local dropped = {}
		local ok = false
		while #work >= c.rayMinCells do
			-- the sequence end nearest the bad corner is the one to shorten
			local headDist = (world(work[1]) - (badIsHi and b or a)).Magnitude
			local tailDist = (world(work[#work]) - (badIsHi and b or a)).Magnitude
			local k
			if headDist <= tailDist then
				k = table.remove(work, 1)
				table.insert(dropped, 1, k)
			else
				k = table.remove(work)
				dropped[#dropped + 1] = k
			end
			stats.walked += 1
			if #work < c.rayMinCells then break end
			local lo2, hi2 = endpointsOf(L, work)
			local a2, b2 = world(lo2), world(hi2)
			if (b2 - a2).Magnitude < c.rayMinChord or clear(a2, b2) then ok = true; break end
		end

		if ok and #work >= c.rayMinCells then
			out[#out + 1] = work
			if #dropped >= c.rayMinCells then
				queue[#queue + 1] = dropped
				stats.spawned += 1
			elseif #dropped > 0 then
				out[#out + 1] = dropped
			end
		elseif #seg >= 4 then
			-- Walking from one end could not clear it, which means the blame was
			-- not all at one end. Halve it and test both: on a boundary that
			-- doubles back, each half has a straight run the whole did not.
			local h = math.floor(#seg / 2)
			local first, second = {}, {}
			for i = 1, h do first[#first + 1] = seg[i] end
			for i = h, #seg do second[#second + 1] = seg[i] end
			queue[#queue + 1] = first
			queue[#queue + 1] = second
			stats.spawned += 1
		else
			-- two or three cells that still fail: too small to walk and too small
			-- to halve. Keep it rather than shredding the boundary, and say so.
			out[#out + 1] = seg
			stats.refused += 1
		end
	end
	for _, seg in ipairs(queue) do out[#out + 1] = seg end
	return out, stats
end

-- Split any line whose CHORD leaves the region.
--
-- A line is fitted to cells that follow the boundary, but it is drawn -- and
-- consumed downstream -- as the straight chord between its endpoints. Around a
-- concavity those are different things: the cells hug the notch, the chord cuts
-- straight across it, and the result is a boundary running over open space or
-- through a wall. Measured on case5: 5 edges, the worst of them 45 studs long.
--
-- The break goes at the cell FURTHEST from the chord, not at the midpoint of the
-- sequence. That is the corner the chord is cutting, so one split usually
-- suffices where a midpoint split would need several. The two halves share the
-- break cell, so they still meet.
--
-- This runs before connect, so the pieces are ordinary lines by the time
-- anything tries to weld them.
function Contour.splitOutside(L, lines, cfg)
	local c = merged(cfg)
	local stats = { split = 0, refused = 0 }
	local function world(k)
		local cc = L.coord[k]
		return L.origin + L.u * (cc[1] * L.leaf) + L.v * (cc[2] * L.leaf)
	end
	local out = {}
	local function process(seg, depth)
		if #seg < 4 or depth >= c.splitDepth then
			if #seg > 0 then out[#out + 1] = seg end
			return
		end
		local kLo, kHi = endpointsOf(L, seg)
		local a, b = world(kLo), world(kHi)
		if Contour.chordInside(L, a, b, c.chordSlack) then
			out[#out + 1] = seg
			return
		end
		-- furthest cell from the chord, measured in the region's own plane
		local d = b - a
		local len = d.Magnitude
		if len < 1e-6 then out[#out + 1] = seg; return end
		local dir = d / len
		local bestAt, bestOff = nil, -1
		for x = 2, #seg - 1 do
			local pcell = world(seg[x])
			local rel = pcell - a
			local along = rel:Dot(dir)
			local off = (rel - dir * along).Magnitude
			if off > bestOff then bestOff = off; bestAt = x end
		end
		if not bestAt then
			out[#out + 1] = seg
			stats.refused += 1
			return
		end
		local first, second = {}, {}
		for x = 1, bestAt do first[#first + 1] = seg[x] end
		for x = bestAt, #seg do second[#second + 1] = seg[x] end
		stats.split += 1
		process(first, depth + 1)
		process(second, depth + 1)
	end
	for _, seg in ipairs(lines) do process(seg, 0) end
	return out, stats
end

function Contour.connect(L, lines, cfg)
	local c = merged(cfg)
	local pairMax = c.pairMax or 1.5
	local minAngle = c.minAngle or 5
	local maxExtend = c.maxExtend or 3.0

	local function world(k)
		local cc = L.coord[k]
		return L.origin + L.u * (cc[1] * L.leaf) + L.v * (cc[2] * L.leaf)
	end

	local E = {}
	for id, seg in ipairs(lines) do
		if #seg >= 2 then
			local kLo, kHi = endpointsOf(L, seg)
			local a, b = world(kLo), world(kHi)
			local d = b - a
			if d.Magnitude > 1e-6 then
				table.insert(E, { id = id, a = a, b = b, dir = d.Unit })
			end
		end
	end

	-- ANGLE TIE-BREAK. After a U-turn corner is stolen, each row ends one cell
	-- back, which puts it the SAME distance (one leaf) from the link's end and
	-- from the other row's end. Distance alone then picks arbitrarily, and picking
	-- the row welds the two parallel rows together -- rebuilding the very U-turn
	-- the steal just removed, as a 0.5 degree spike. On a tie, take the partner
	-- that meets this line at the LARGER angle: the link, never the row.
	local function nearest(i, w)
		local pt = E[i][w]
		local best, bj, bw, bang = math.huge, nil, nil, -1
		for j, t in ipairs(E) do
			if j ~= i then
				for _, w2 in ipairs({ "a", "b" }) do
					local dd = (t[w2] - pt).Magnitude
					local ang = math.deg(math.acos(math.clamp(math.abs(E[i].dir:Dot(t.dir)), -1, 1)))
					if dd < best - 1e-6 or (math.abs(dd - best) <= 1e-6 and ang > bang) then
						best = dd; bj = j; bw = w2; bang = ang
					end
				end
			end
		end
		return bj, bw, best
	end
	local nb = {}
	for i in ipairs(E) do
		for _, w in ipairs({ "a", "b" }) do
			local j, w2, dd = nearest(i, w)
			-- A region can come out as a SINGLE line -- a lone strip, or the one
			-- stretch left after the small loops are dropped. `nearest` then has
			-- no candidate and returns nil, so there is no neighbour to record:
			-- leave the slot empty rather than storing a partner with no index.
			if j then nb[i .. w] = { j = j, w = w2, d = dd } end
		end
	end

	-- crossing of two lines in the region's own plane
	local function planar(v) return v:Dot(L.u), v:Dot(L.v) end
	local function meet(p1, d1, p2, d2)
		local a1, b1 = planar(d1)
		local a2, b2 = planar(d2)
		local den = a1 * b2 - b1 * a2
		if math.abs(den) < 1e-9 then return nil end
		local rx, ry = planar(p2 - p1)
		return p1 + d1 * ((rx * b2 - ry * a2) / den)
	end

	local stats = { welded = 0, parallel = 0, capped = 0, unpaired = 0, junction = 0, open = 0, blocked = 0 }
	-- is the edge still clear if this end moves to X?
	local rp = c.rayFilter
	local lift = (L.up or Vector3.yAxis) * (c.rayLift or 0.6)
	local function clearTo(e, w, X)
		if not rp then return true end
		local far = (w == "a") and e.b or e.a
		local d = (X + lift) - (far + lift)
		if d.Magnitude < 1e-4 then return true end
		return workspace:Raycast(far + lift, d, rp) == nil
	end
	local done = {}
	local placed = {}
	for i, s in ipairs(E) do
		for _, w in ipairs({ "a", "b" }) do
			local link = nb[i .. w]
			local back = link and nb[link.j .. link.w]
			-- An endpoint can have NO partner at all -- a line whose end faces
			-- nothing in the region. `key` used to be built before this was
			-- checked, so `link.j` threw on those. Nothing to weld: skip it.
			local key = link and (math.min(i, link.j) .. "|" .. ((i < link.j) and (w .. link.w) or (link.w .. w)))
			if link and link.d <= pairMax and back and back.j == i and back.w == w and not done[key] then
				done[key] = true
				local t = E[link.j]
				local w2 = link.w
				local ang = math.deg(math.acos(math.clamp(math.abs(s.dir:Dot(t.dir)), -1, 1)))
				local X = (ang >= minAngle) and meet(s[w], s.dir, t[w2], t.dir) or nil
				if X and math.max((X - s[w]).Magnitude, (X - t[w2]).Magnitude) > maxExtend then
					X = nil; stats.capped += 1
				elseif X and not clearTo(s, w, X) then
					-- extending to the crossing point would push this edge through
					-- a wall; the midpoint of two on-surface ends cannot
					X = nil; stats.blocked += 1
				elseif X and not clearTo(t, w2, X) then
					X = nil; stats.blocked += 1
				elseif not X then
					stats.parallel += 1
				end
				if not X then X = (s[w] + t[w2]) / 2 else stats.welded += 1 end
				s[w] = X; t[w2] = X
				placed[i .. w] = true
				placed[link.j .. link.w] = true
			elseif (not link) or link.d > pairMax then
				stats.unpaired += 1
			end
		end
	end

	-- SECOND PASS: close what mutual-nearest cannot.
	--
	-- The pairing above is a MATCHING: an end welds to one partner, and only if
	-- that partner picks it back. At a junction of three or more ends -- a stub
	-- meeting two longer lines at a corner -- only one pair can be mutual and
	-- the rest are left open. Ties make it worse: two ends of the same line are
	-- often exactly equidistant from a third, and since the angle tie-break
	-- compares line DIRECTIONS it scores both ends of a line identically, so
	-- iteration order decides which one wins and the other is abandoned.
	--
	-- An open end means the loop is not closed, and an unclosed loop is not a
	-- polygon, so nothing downstream can build portals from it. Measured on
	-- case5: 10 of 1086 ends, in 10 separate regions.
	--
	-- This pass only ever touches ends the first pass left alone. Every weld the
	-- tuned pairing made is preserved exactly, including the angle tie-break
	-- that keeps the two rows of a U-turn from being welded back together.
	local order = {}
	for i in ipairs(E) do
		for _, w in ipairs({ "a", "b" }) do
			if not placed[i .. w] then order[#order + 1] = { i = i, w = w } end
		end
	end
	for _, e in ipairs(order) do
		local i, w = e.i, e.w
		if not placed[i .. w] then
			local pt = E[i][w]
			-- nearest end of any OTHER line, preferring one already welded: an
			-- end that has been placed marks a junction that already exists, and
			-- joining it is what makes three lines meet at one point.
			local bj, bw, bd, bPlaced = nil, nil, math.huge, false
			for j, t in ipairs(E) do
				if j ~= i then
					-- NEAR-PARALLEL IS NOT A CORNER. Two ends whose lines run at
					-- the same angle are the two rows of a U-turn, and joining
					-- them at a single point rebuilds the 180 degree reversal
					-- that stealUTurns exists to remove -- as a zero degree
					-- coincident pair. Measured: without this the pass closed 20
					-- ends on case3 and created 4 such spikes.
					local ang = math.deg(math.acos(math.clamp(math.abs(E[i].dir:Dot(t.dir)), -1, 1)))
					if ang >= minAngle then
						for _, w2 in ipairs({ "a", "b" }) do
							local dd = (t[w2] - pt).Magnitude
							local isP = placed[j .. w2] == true
							if dd <= pairMax and (dd < bd - 1e-6 or (math.abs(dd - bd) <= 1e-6 and isP and not bPlaced)) then
								bj, bw, bd, bPlaced = j, w2, dd, isP
							end
						end
					end
				end
			end
			if not bj then
				stats.open += 1
			elseif bPlaced then
				-- snap onto the existing junction point
				E[i][w] = E[bj][bw]
				placed[i .. w] = true
				stats.junction += 1
			else
				-- two orphans facing each other: weld them the same way the
				-- first pass would have, had either picked the other back
				local t = E[bj]
				local ang = math.deg(math.acos(math.clamp(math.abs(E[i].dir:Dot(t.dir)), -1, 1)))
				local X = (ang >= minAngle) and meet(E[i][w], E[i].dir, t[bw], t.dir) or nil
				if X and math.max((X - E[i][w]).Magnitude, (X - t[bw]).Magnitude) > maxExtend then X = nil end
				if not X then X = (E[i][w] + t[bw]) / 2 end
				E[i][w] = X; t[bw] = X
				placed[i .. w] = true
				placed[bj .. bw] = true
				stats.junction += 2
			end
		end
	end

	return E, stats
end

-- Everything, for one region's parts.
function Contour.run(parts, cfg)
	local c = merged(cfg)
	local L = Contour.lattice(parts, c)
	local seed, depth = Contour.border(L, c)
	local loops = Contour.loops(L)
	local tangent, linearity = Contour.tangents(L, seed, c)
	local lines, stats = Contour.lines(L, loops, tangent, c)

	-- close the U-turns by stealing their corners; the links are ordinary lines
	local steal
	lines, steal = Contour.stealUTurns(L, lines, loops, c)

	-- break any line whose chord would cut across a concavity
	local splitStats
	lines, splitStats = Contour.splitOutside(L, lines, c)

	-- then test what is left against real geometry, walking back the bad end
	local rayStats
	lines, rayStats = Contour.validateLines(L, lines, tangent, c)

	-- quality: RMS deviation is the fair measure. Worst-cell deviation always
	-- looks bad because the worst cell in a line is, by construction, the corner
	-- cell at its end.
	local rms, worst, nb = 0, 0, 0
	for _, s in ipairs(lines) do
		local m = meanAngle(s, tangent)
		local acc, w = 0, 0
		for _, k in ipairs(s) do
			local d = angDiff(tangent[k], m)
			acc += d * d; w = math.max(w, d)
		end
		rms += math.sqrt(acc / #s)
		if w > 45 then nb += 1 end
		worst = math.max(worst, w)
	end
	local edges, joinStats = Contour.connect(L, lines, c)
	local dissolveStats
	edges, dissolveStats = Contour.dissolveShort(L, edges, c)

	return {
		lattice = L, seed = seed, depth = depth, loops = loops,
		tangent = tangent, linearity = linearity, lines = lines,
		edges = edges, joinStats = joinStats,
		stats = {
			cells = #parts, loops = #loops, lines = #lines,
			distributed = stats.distributed, refused = stats.refused,
			smallLoops = stats.smallLoops, uturns = stats.uturns, kept = stats.kept,
			links = steal.links, splits = steal.splits, firstLink = steal.firstLink,
			dissolved = dissolveStats.dissolved, dissolveRefused = dissolveStats.refused,
			dissolveOutside = dissolveStats.outside, dissolveCapped = dissolveStats.capped,
			chordSplits = splitStats.split, chordSplitRefused = splitStats.refused,
			rayTested = rayStats.tested, rayFailed = rayStats.failed,
			rayWalked = rayStats.walked, raySpawned = rayStats.spawned,
			rayRefused = rayStats.refused,
			rmsDeviation = rms / math.max(1, #lines),
			worstDeviation = worst, bent = nb,
		},
		config = c,
	}
end

-- Dissolve edges too short to be worth an edge, extending their neighbours to
-- meet instead.
--
-- Runs after connect, so every endpoint already coincides with its neighbour's
-- and adjacency is exact rather than a proximity guess. Shortest first, and an
-- edge whose neighbour was already dissolved is left alone, so a run of stubs
-- cannot cascade into one long chord across a curve.
--
-- REFUSED whenever the replacement would not stay over the region. Cutting a
-- corner is precisely how a boundary starts running through a wall, and the two
-- new chords are checked against the occupancy lattice before anything is
-- committed. A junction of more than two edges is also refused: which pair
-- should meet is not defined there.
function Contour.dissolveShort(L, E, cfg)
	local c = merged(cfg)
	local stats = { dissolved = 0, refused = 0, outside = 0, capped = 0 }
	-- maxExtend is not in DEFAULT; connect reads it with this same fallback
	local maxExtend = c.maxExtend or 3.0
	if not c.minEdge or c.minEdge <= 0 then return E, stats end

	local function key(v: Vector3): string
		return string.format("%d:%d:%d",
			math.round(v.X * 64), math.round(v.Y * 64), math.round(v.Z * 64))
	end

	local order = {}
	for idx, e in ipairs(E) do
		local len = (e.b - e.a).Magnitude
		if len < c.minEdge then order[#order + 1] = { idx = idx, len = len } end
	end
	table.sort(order, function(x, y) return x.len < y.len end)

	local dead = {}
	for _, o in ipairs(order) do
		local s = E[o.idx]
		if not dead[o.idx] then
			-- exactly one live neighbour at each end, or this is a junction
			local nA, nB, wA, wB, countA, countB = nil, nil, nil, nil, 0, 0
			local ka, kb = key(s.a), key(s.b)
			for j, t in ipairs(E) do
				if j ~= o.idx and not dead[j] then
					for _, w in ipairs({ "a", "b" }) do
						if key(t[w]) == ka then countA += 1; nA, wA = j, w end
						if key(t[w]) == kb then countB += 1; nB, wB = j, w end
					end
				end
			end
			if countA ~= 1 or countB ~= 1 or nA == nB then
				stats.refused += 1
			else
				local A, B = E[nA], E[nB]
				local X = planeMeet(L, A[wA], A.dir, B[wB], B.dir)
				-- CAP THE EXTENSION, exactly as connect does. Two neighbours of a
				-- stub are often near parallel -- the two sides of a narrow notch,
				-- or the ring around a small hole in the floor -- and near
				-- parallel lines meet a long way off. Uncapped, a 2-cell line with
				-- a half-stud chord came out drawn as an 8-stud spike shooting
				-- away from the surface. A corner replacement that has to travel
				-- further than maxExtend is not a corner.
				if X and math.max((X - A[wA]).Magnitude, (X - B[wB]).Magnitude) > maxExtend then
					X = nil
					stats.capped += 1
				elseif not X then
					stats.refused += 1
				end
				if not X then
					-- already counted above
				else
					-- the far end of each neighbour, which the new chord runs from
					local farA = (wA == "a") and A.b or A.a
					local farB = (wB == "a") and B.b or B.a
					if Contour.chordInside(L, farA, X, c.chordSlack)
						and Contour.chordInside(L, X, farB, c.chordSlack) then
						A[wA] = X; B[wB] = X
						dead[o.idx] = true
						stats.dissolved += 1
					else
						stats.outside += 1
					end
				end
			end
		end
	end

	if stats.dissolved == 0 then return E, stats end
	local out = {}
	for i, e in ipairs(E) do
		if not dead[i] then out[#out + 1] = e end
	end
	return out, stats
end

local function hueFor(i: number): Color3
	return Color3.fromHSV(((i * 0.61803398875) % 1), 0.85, 1)
end

-- Paint the region's own parts: one colour per line, or per loop.
function Contour.paint(res, mode: string?, base: Color3?)
	mode = mode or "line"
	local L = res.lattice
	local BASE = base or Color3.fromRGB(129, 69, 249)
	for _, p in pairs(L.partAt) do
		p.Color = BASE
		p.Material = Enum.Material.SmoothPlastic
		p:SetAttribute("line", nil); p:SetAttribute("loop", nil); p:SetAttribute("tangentDeg", nil)
	end
	local n = 0
	if mode == "loop" then
		for li, seq in ipairs(res.loops) do
			for _, k in ipairs(seq) do
				local p = L.partAt[k]
				if p then p.Color = hueFor(li); p.Material = Enum.Material.Neon; p:SetAttribute("loop", li); n += 1 end
			end
		end
	else
		for id, s in ipairs(res.lines) do
			for _, k in ipairs(s) do
				local p = L.partAt[k]
				if p then
					p.Color = hueFor(id); p.Material = Enum.Material.Neon
					p:SetAttribute("line", id)
					p:SetAttribute("tangentDeg", math.floor((res.tangent[k] or 0) * 10) / 10)
					n += 1
				end
			end
		end
	end
	return n
end

-- Convenience: pull one region's parts out of a NodeWalk region draw.
function Contour.partsOfRegion(folder: Instance, regionId: number)
	local out = {}
	for _, p in ipairs(folder:GetChildren()) do
		if p:GetAttribute("region") == regionId then table.insert(out, p) end
	end
	return out
end

return Contour

