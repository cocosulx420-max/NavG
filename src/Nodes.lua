--!strict
-- NVGN.Nodes -- cover the walkable cells with rectangles and call each one a node.
--
-- This replaces the whole trace-simplify-ring-triangulate path for pathfinding
-- purposes. That path exists to produce an exact polygon boundary, and the one
-- thing an exact boundary buys you is the funnel: a path that hugs real corners
-- instead of visiting node centres. We are not running a funnel. The node model
-- is centres, so the boundary is computed, simplified, classified, triangulated
-- and then discarded down to a set of points -- and the tracer is the stage that
-- does not survive contact with a hard map. case4 alone leaves 24 loops broken,
-- 10 still open and 28 rings degenerate, and every one of those defects is
-- downstream of the cells, which are fine.
--
-- SO GO STRAIGHT FROM THE CELLS. A rectangle of cells is convex, so a straight
-- line between the centres of two touching rectangles stays on the floor, which
-- is the same guarantee convex polygon merging was going to buy. The rectangle
-- lives in the grid's own lattice, and LocalGrid aligns that lattice to the
-- part's own frame, so a rectangle on a rotated slab is a real rectangle in
-- world space and not a staircase of cells.
--
-- ONE RECTANGLE FOR A RECTANGULAR REGION, several for a complex one, and no
-- branch anywhere that asks which it is. The shape decides the count.
--
-- ADJACENCY IS THE SAME QUESTION SEVERANCE ALREADY ANSWERS, so it is asked the
-- same way, with that module's own gate rather than a second copy of the
-- numbers. Two nodes are linked when any cell of one is close enough to any cell
-- of the other. Nothing is matched, intersected or snapped, so there is no
-- epsilon to get wrong, and a staircase links tread to tread for free.

local Nodes = {}

local Severance = require(script.Parent:WaitForChild("Severance"))

-- Rectangles narrower than this in cells are still emitted, but counted, because
-- a cover made mostly of one-cell slivers means the region's edges are ragged in
-- the lattice and the decomposition is not buying much.
Nodes.sliverWidth = 1

-- "largest" takes the biggest rectangle that fits, over and over, until the
-- cells run out. "runs" stacks equal-width row runs and glues the results
-- sideways. Runs is linear and largest is a full scan per rectangle emitted, and
-- on ragged edges runs fragments badly -- case3 came out half one-cell slivers.
Nodes.mode = "largest"

-- Let rectangles OVERLAP. A cover does not have to be a partition: a node is a
-- claim that some rectangle of floor is convex and walkable, and two nodes
-- claiming the same cell costs nothing. Partitioning is what produces the
-- slivers -- carve the big rectangle out of the middle and the fringe left
-- behind is one cell wide with nowhere to grow. Allowed to overlap, that fringe
-- rectangle grows back over the middle and becomes a real node.
Nodes.overlap = true

-- How thick a grown rectangle is allowed to get, in CELLS. Unbounded growth
-- was measured and it is the wrong trade: every fringe strip on case5 grew
-- across the whole open floor, the cover reached 6.94x redundant, links went
-- from 2586 to 14486 and the build took 81 seconds, all to remove 586 slivers.
--
-- 4 cells is 2 studs, which is the agent's width. A node thinner than the agent
-- is not somewhere the agent fits, so that is exactly the point where growing
-- stops paying. A rectangle that is already this thick is left alone.
Nodes.growTo = 4

-- 8 gives the graph diagonal links between rectangles that only touch at a
-- corner. Off by default: a corner touch is not somewhere an agent fits.
Nodes.diagonalLinks = false

-- Maximal horizontal runs of cells in one row of the lattice.
local function runs(us: { number }): { { number } }
	table.sort(us)
	local out = {}
	local i = 1
	while i <= #us do
		local a = us[i]
		local b = a
		while i < #us and us[i + 1] == b + 1 do
			i += 1
			b = us[i]
		end
		out[#out + 1] = { a, b }
		i += 1
	end
	return out
end

-- The biggest rectangle of set cells in a binary grid, by the histogram method:
-- for each row, treat the column of set cells above it as a bar, and find the
-- largest rectangle in that histogram with a monotonic stack. Linear per row, so
-- linear in the grid.
--
-- Returns x0, x1, y0, y1 in 0-based grid coordinates, or nil when empty.
local function largestRect(occ: { { boolean } }, W: number, H: number)
	local heights = table.create(W, 0)
	local stackX = table.create(W + 1, 0)
	local stackH = table.create(W + 1, 0)
	local bestA, bx0, bx1, by0, by1 = 0, nil, nil, nil, nil

	for y = 1, H do
		local row = occ[y]
		for x = 1, W do
			heights[x] = row[x] and (heights[x] + 1) or 0
		end
		-- one sentinel pass past the end flushes the stack
		local top = 0
		for x = 1, W + 1 do
			local h = (x <= W) and heights[x] or 0
			local start = x
			while top > 0 and stackH[top] > h do
				local ph = stackH[top]
				local px = stackX[top]
				top -= 1
				local area = ph * (x - px)
				if area > bestA then
					bestA = area
					bx0, bx1 = px, x - 1
					by0, by1 = y - ph + 1, y
				end
				start = px
			end
			if h > 0 and (top == 0 or stackH[top] < h) then
				top += 1
				stackX[top] = start
				stackH[top] = h
			end
		end
	end
	if not bx0 then return nil end
	return bx0, bx1, by0, by1, bestA
end

-- Largest-rectangle-first cover. Take the biggest rectangle that fits, claim its
-- cells, repeat.
--
-- WORTH THE EXTRA SCANS. The linear alternative below stacks row runs of equal
-- width, which is exact on a shape that really is a rectangle and falls apart on
-- one whose edges are ragged in the lattice: a boundary that steps in and out by
-- a cell breaks every run and case3 came out 123 slivers of 242 nodes. Taking
-- the biggest rectangle first puts the large nodes in the open middle and leaves
-- the raggedness to be mopped up at the edges, where it belongs.
local function coverLargest(rows: { [number]: { number } }): { any }
	local ulo, uhi, vlo, vhi = math.huge, -math.huge, math.huge, -math.huge
	local total = 0
	for v, us in pairs(rows) do
		if v < vlo then vlo = v end
		if v > vhi then vhi = v end
		for _, u in ipairs(us) do
			if u < ulo then ulo = u end
			if u > uhi then uhi = u end
			total += 1
		end
	end
	if total == 0 then return {} end
	local W, H = uhi - ulo + 1, vhi - vlo + 1

	-- Two masks: what is floor, and what is not yet spoken for. Without overlap
	-- they stay identical and this is a straight partition.
	local walk = table.create(H)
	local uncov = table.create(H)
	for y = 1, H do
		walk[y] = table.create(W, false)
		uncov[y] = table.create(W, false)
	end
	for v, us in pairs(rows) do
		local a, b = walk[v - vlo + 1], uncov[v - vlo + 1]
		for _, u in ipairs(us) do
			a[u - ulo + 1] = true
			b[u - ulo + 1] = true
		end
	end

	local function rowClear(y: number, x0: number, x1: number): boolean
		if y < 1 or y > H then return false end
		local row = walk[y]
		for x = x0, x1 do
			if not row[x] then return false end
		end
		return true
	end
	local function colClear(x: number, y0: number, y1: number): boolean
		if x < 1 or x > W then return false end
		for y = y0, y1 do
			if not walk[y][x] then return false end
		end
		return true
	end

	local out = {}
	while total > 0 do
		local x0, x1, y0, y1 = largestRect(uncov, W, H)
		if not x0 then break end

		if Nodes.overlap then
			-- Grow over ground that is already covered, never over ground that is
			-- not floor, and only up to `growTo` thick. A rectangle that already
			-- clears that in both directions is left exactly as it was, so the
			-- big interior nodes never overlap anything and only the thin fringe
			-- reaches back over its neighbour.
			local wantH = math.max(y1 - y0 + 1, Nodes.growTo)
			local wantW = math.max(x1 - x0 + 1, Nodes.growTo)
			while (y1 - y0 + 1) < wantH and rowClear(y0 - 1, x0, x1) do y0 -= 1 end
			while (y1 - y0 + 1) < wantH and rowClear(y1 + 1, x0, x1) do y1 += 1 end
			while (x1 - x0 + 1) < wantW and colClear(x0 - 1, y0, y1) do x0 -= 1 end
			while (x1 - x0 + 1) < wantW and colClear(x1 + 1, y0, y1) do x1 += 1 end
		end

		local claimed = 0
		for y = y0, y1 do
			local row = uncov[y]
			for x = x0, x1 do
				if row[x] then row[x] = false; claimed += 1 end
			end
		end
		-- A grown rectangle always claims the seed rectangle, so this cannot
		-- fail to make progress and the loop always ends.
		total -= claimed
		out[#out + 1] = {
			u0 = x0 + ulo - 1, u1 = x1 + ulo - 1,
			v0 = y0 + vlo - 1, v1 = y1 + vlo - 1,
		}
	end
	return out
end

-- Cover one group's cells with rectangles.
--
-- TWO PASSES, NOT A SEARCH. Finding the largest empty rectangle first would give
-- a slightly better cover, and it costs a full scan per rectangle emitted, which
-- on a twelve thousand cell component is tens of millions of operations for a
-- result nobody can see. Instead: cut every row into runs, stack runs of
-- identical extent into rectangles, then glue rectangles side by side where the
-- union is still a rectangle. Both passes are linear and both are exact on a
-- shape that really is one rectangle, which is the case that matters.
local function cover(rows: { [number]: { number } }): { any }
	-- vertical: stack identical runs from consecutive rows
	local strips = {}
	for v, us in pairs(rows) do
		for _, r in ipairs(runs(us)) do
			strips[#strips + 1] = { u0 = r[1], u1 = r[2], v = v }
		end
	end
	table.sort(strips, function(a, b)
		if a.u0 ~= b.u0 then return a.u0 < b.u0 end
		if a.u1 ~= b.u1 then return a.u1 < b.u1 end
		return a.v < b.v
	end)
	local rects = {}
	local cur = nil
	for _, s in ipairs(strips) do
		if cur and cur.u0 == s.u0 and cur.u1 == s.u1 and cur.v1 + 1 == s.v then
			cur.v1 = s.v
		else
			if cur then rects[#rects + 1] = cur end
			cur = { u0 = s.u0, u1 = s.u1, v0 = s.v, v1 = s.v }
		end
	end
	if cur then rects[#rects + 1] = cur end

	-- horizontal: glue rectangles that span the same rows and touch
	table.sort(rects, function(a, b)
		if a.v0 ~= b.v0 then return a.v0 < b.v0 end
		if a.v1 ~= b.v1 then return a.v1 < b.v1 end
		return a.u0 < b.u0
	end)
	local out = {}
	cur = nil
	for _, r in ipairs(rects) do
		if cur and cur.v0 == r.v0 and cur.v1 == r.v1 and cur.u1 + 1 == r.u0 then
			cur.u1 = r.u1
		else
			if cur then out[#out + 1] = cur end
			cur = r
		end
	end
	if cur then out[#out + 1] = cur end
	return out
end

-- Build the node graph from a bake.
--
-- Returns `{ nodes, links, stats }`. A node carries its world centre, its
-- normal, its extent in studs, the cells it covers and the grid and region it
-- came from. A link carries the two node ids and how many cell pairs support it,
-- which is a usable stand-in for how wide the opening is.
function Nodes.build(data: any, cfg: any?): any
	local c = cfg or {}
	local step = (data.config and data.config.step) or 0.5

	-- Group by GRID AND REGION. A rectangle has to be flat and has to live on
	-- one lattice, and those are the two things that guarantee it.
	local groups, order = {}, {}
	for gi, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			if cell.region then
				local k = gi * 1000000 + cell.region
				local grp = groups[k]
				if not grp then
					grp = { grid = gi, region = cell.region, rows = {}, at = {} }
					groups[k] = grp
					order[#order + 1] = k
				end
				local row = grp.rows[cell.vi]
				if not row then row = {}; grp.rows[cell.vi] = row end
				row[#row + 1] = cell.ui
				local col = grp.at[cell.vi]
				if not col then col = {}; grp.at[cell.vi] = col end
				col[cell.ui] = cell
			end
		end
	end

	local nodes = {}
	-- A CELL MAY BELONG TO SEVERAL NODES once rectangles are allowed to overlap,
	-- so this is a list and not a single id. Writing one id and letting the last
	-- rectangle win would silently strip the overlap out of the graph and leave
	-- the earlier node with nothing to link through.
	local nodesOf: { [any]: { number } } = {}
	local stats = { groups = 0, nodes = 0, cells = 0, covered = 0, slivers = 0,
		single = 0, biggest = 0, links = 0 }

	for _, k in ipairs(order) do
		local grp = groups[k]
		stats.groups += 1
		local rects = (Nodes.mode == "runs") and cover(grp.rows)
			or coverLargest(grp.rows)
		for _, r in ipairs(rects) do
			local w, h = r.u1 - r.u0 + 1, r.v1 - r.v0 + 1

			-- The node sits on the cell nearest the rectangle's middle, not on
			-- the arithmetic centre of the corners. A cell's `pos` is a point
			-- LocalGrid put on the actual surface; a computed midpoint is a
			-- point in the air above a ramp.
			local mu = (r.u0 + r.u1) * 0.5
			local mv = (r.v0 + r.v1) * 0.5
			local pick, best = nil, math.huge
			local cells = {}
			for v = r.v0, r.v1 do
				local col = grp.at[v]
				for u = r.u0, r.u1 do
					local cell = col and col[u]
					if cell then
						cells[#cells + 1] = cell
						local du, dv = u - mu, v - mv
						local d = du * du + dv * dv
						if d < best then best = d; pick = cell end
					end
				end
			end
			if not pick then continue end

			local id = #nodes + 1
			nodes[id] = {
				id = id,
				pos = pick.pos,
				ui = pick.ui, vi = pick.vi,
				midU = mu, midV = mv,
				normal = pick.normal,
				grid = grp.grid,
				region = grp.region,
				u0 = r.u0, u1 = r.u1, v0 = r.v0, v1 = r.v1,
				width = w * step,
				height = h * step,
				cells = cells,
				cellCount = #cells,
			}
			for _, cell in ipairs(cells) do
				local t = nodesOf[cell]
				if not t then t = {}; nodesOf[cell] = t end
				t[#t + 1] = id
			end
			stats.nodes += 1
			-- summed over nodes, so overlap counts twice; `cells` below is distinct
			stats.covered += #cells
			if #cells > stats.biggest then stats.biggest = #cells end
			if w <= Nodes.sliverWidth or h <= Nodes.sliverWidth then
				stats.slivers += 1
			end
			if #cells == 1 then stats.single += 1 end
		end
	end

	-- LINKS, THROUGH SEVERANCE'S OWN GATE. Read off that module rather than
	-- copied, so raising the stair tolerance in one place cannot leave the node
	-- graph disagreeing with the connectivity report about what is reachable.
	local plane2 = Severance.stepPlane * Severance.stepPlane
	local normTol = Severance.stepNormal
	local G = math.max(Severance.stepPlane, Severance.stepNormal)

	-- DISTINCT cells, not the union of the node cell lists: with overlap on, a
	-- shared cell appears in each node that covers it and would be hashed and
	-- tested once per copy.
	local all = {}
	for cell in pairs(nodesOf) do all[#all + 1] = cell end
	stats.cells = #all

	local hash: { [string]: { any } } = {}
	local function bucket(p: Vector3): string
		return ("%d,%d,%d"):format(
			math.floor(p.X / G), math.floor(p.Y / G), math.floor(p.Z / G))
	end
	for _, cell in ipairs(all) do
		local b = bucket(cell.pos)
		local t = hash[b]
		if not t then t = {}; hash[b] = t end
		t[#t + 1] = cell
	end

	local links = {}
	local seen: { [number]: any } = {}
	-- Two nodes covering the same cell are trivially reachable from each other,
	-- and no cell-to-cell test will ever say so because it is one cell. Link
	-- them here or an overlapping pair looks disconnected.
	local links = {}
	local seen: { [number]: any } = {}
	local function join(ia: number, ib: number, rise: number)
		if ia == ib then return end
		if ia > ib then ia, ib = ib, ia end
		local kk = ia * 1000000 + ib
		local L = seen[kk]
		if not L then
			L = { a = ia, b = ib, pairs = 0, rise = 0 }
			seen[kk] = L
			links[#links + 1] = L
		end
		L.pairs += 1
		if rise > L.rise then L.rise = rise end
	end
	for _, t in pairs(nodesOf) do
		for i = 1, #t do
			for j = i + 1, #t do join(t[i], t[j], 0) end
		end
	end

	for _, a in ipairs(all) do
		local ta = nodesOf[a]
		local p = a.pos
		local up = a.normal or Vector3.yAxis
		local bx, by, bz = math.floor(p.X / G), math.floor(p.Y / G), math.floor(p.Z / G)
		for ox = -1, 1 do
			for oy = -1, 1 do
				for oz = -1, 1 do
					local bucketCells = hash[("%d,%d,%d"):format(bx + ox, by + oy, bz + oz)]
					if bucketCells then
						for _, b in ipairs(bucketCells) do
							if b ~= a then
								local dv = b.pos - p
								local dn = dv:Dot(up)
								local flat = dv - up * dn
								if flat:Dot(flat) <= plane2 and math.abs(dn) <= normTol then
									local r = math.abs(dn)
									for _, ia in ipairs(ta) do
										for _, ib in ipairs(nodesOf[b]) do
											join(ia, ib, r)
										end
									end
								end
							end
						end
					end
				end
			end
		end
	end
	stats.links = #links

	-- Degree, so an orphan is visible without a second pass over the graph.
	local deg = table.create(#nodes, 0)
	for _, L in ipairs(links) do
		deg[L.a] += 1
		deg[L.b] += 1
	end
	stats.orphans = 0
	for i, n in ipairs(nodes) do
		n.degree = deg[i]
		if deg[i] == 0 then stats.orphans += 1 end
	end

	return { nodes = nodes, links = links, stats = stats, nodesOf = nodesOf }
end

function Nodes.report(res: any): string
	local s = res.stats
	local lines = {
		("nodes     %d nodes over %d groups, %d cells, %d links")
			:format(s.nodes, s.groups, s.cells, s.links),
		("  mean %.1f cells per node, biggest %d, %d single-cell, %d slivers")
			:format(s.nodes > 0 and (s.covered / s.nodes) or 0, s.biggest,
				s.single, s.slivers),
		("  overlap %.2fx -- %d cell slots over %d distinct cells")
			:format(s.cells > 0 and (s.covered / s.cells) or 0, s.covered, s.cells),
	}
	if s.orphans > 0 then
		lines[#lines + 1] = ("  ! %d nodes with no link at all"):format(s.orphans)
	end
	return table.concat(lines, "\n")
end

return Nodes
