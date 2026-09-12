--!strict
-- NVGN.Skeleton -- erode a region until only its middle is left, and put the
-- nodes there.
--
-- Cocosulx's idea, and it has a name: the medial axis. Erode a shape until what
-- remains cannot be eroded any further and you are left with a one-cell spine
-- running down the middle of it. A big open room thins to a dot, a corridor
-- thins to a line, a room with three doorways thins to a hub with three arms.
-- Nobody has to decide whether a region is simple enough for one node; the
-- shape decides, and a region that deserves one node produces one.
--
-- THE HARD PART IS THE STOPPING CONDITION, and Cocosulx named it before the
-- algorithm did: some parts of a region reach the middle before others, and
-- those must not then be eroded away. Two local rules are enough:
--
--   * never delete a cell whose removal would split its own neighbourhood in
--     two -- that is what keeps a corridor from being eaten from both ends
--   * never delete a cell that is the last one on an arm -- that is what keeps
--     the arm from retracting into the hub
--
-- Those two conditions are Zhang and Suen's thinning algorithm. Erosion then
-- stops by itself, and it stops in a different place for every part of the
-- shape, which is exactly the behaviour that was wanted.
--
-- NOT A PERCENTAGE. Percentage of what is the problem: a region can be a wide
-- hall with a narrow arm off it, and taking half the region's widest measure
-- off everything deletes the arm and barely touches the hall. The stopping
-- condition has to be local, and "stop before you break something" already is.
--
-- THINNED PER GRID, LINKED ACROSS. The lattice belongs to a part, so a region
-- crossing two parts is thinned once per part and the pieces are joined
-- afterwards through Severance's gate. That is a real seam and it can leave a
-- kink in the spine where the parts meet; it is honest about it rather than
-- pretending the two lattices are one.

local Skeleton = {}

local Severance = require(script.Parent:WaitForChild("Severance"))

-- An arm shorter than this that ends in a tip is boundary noise, not a limb. A
-- floor edge that steps in and out by one cell grows a little spike toward the
-- step, and without pruning every jagged rim sprouts them. In CELLS.
Skeleton.prune = 4

-- How far apart to put nodes along a plain stretch of spine, in CELLS. Junctions
-- and tips always get one regardless. 8 cells is 4 studs.
Skeleton.spacing = 8

-- 8-connectivity, clockwise from north. The thinning conditions below are
-- written against this exact order and index; do not reorder it.
local P = {
	{ 0, -1 }, { 1, -1 }, { 1, 0 }, { 1, 1 },
	{ 0, 1 }, { -1, 1 }, { -1, 0 }, { -1, -1 },
}

-- One Zhang-Suen sub-iteration. `phase` 1 and 2 differ only in which pair of
-- corners they protect, which is what makes the result symmetric instead of
-- drifting toward one side.
local function thinPass(m: { { boolean } }, W: number, H: number, phase: number): number
	local kill = {}
	for y = 1, H do
		local row = m[y]
		for x = 1, W do
			if row[x] then
				-- n[1..8] clockwise from north, then B is how many are set and A
				-- is how many times the ring goes from empty to filled. A == 1
				-- means the neighbourhood is a single unbroken run, so removing
				-- this cell cannot split it.
				local n = table.create(8, false)
				for i = 1, 8 do
					local yy = y + P[i][2]
					local xx = x + P[i][1]
					n[i] = yy >= 1 and yy <= H and xx >= 1 and xx <= W and m[yy][xx]
				end
				local B = 0
				for i = 1, 8 do if n[i] then B += 1 end end
				if B >= 2 and B <= 6 then
					local A = 0
					for i = 1, 8 do
						local j = (i % 8) + 1
						if not n[i] and n[j] then A += 1 end
					end
					if A == 1 then
						local N, E, S, Wst = n[1], n[3], n[5], n[7]
						local ok
						if phase == 1 then
							ok = not (N and E and S) and not (E and S and Wst)
						else
							ok = not (N and E and Wst) and not (N and S and Wst)
						end
						if ok then kill[#kill + 1] = { x, y } end
					end
				end
			end
		end
	end
	-- deleted together, after the whole pass. Deleting as we go would let one
	-- removal change the neighbourhood the next test reads, and the shape then
	-- thins unevenly depending on scan order.
	for _, c in ipairs(kill) do m[c[2]][c[1]] = false end
	return #kill
end

local function neighbours(m: { { boolean } }, W: number, H: number,
	x: number, y: number): number
	local k = 0
	for i = 1, 8 do
		local yy = y + P[i][2]
		local xx = x + P[i][1]
		if yy >= 1 and yy <= H and xx >= 1 and xx <= W and m[yy][xx] then k += 1 end
	end
	return k
end

-- Walk in from every tip and rub out any arm shorter than `prune` cells.
-- Repeated, because removing one spike can expose another behind it.
local function pruneSpurs(m: { { boolean } }, W: number, H: number, limit: number)
	if limit <= 0 then return end
	for _ = 1, limit do
		local tips = {}
		for y = 1, H do
			for x = 1, W do
				if m[y][x] and neighbours(m, W, H, x, y) == 1 then
					tips[#tips + 1] = { x, y }
				end
			end
		end
		if #tips == 0 then return end
		local cut = 0
		for _, t in ipairs(tips) do
			-- walk the arm, stopping at a junction or at the length limit
			local path = {}
			local x, y = t[1], t[2]
			local px, py = -1, -1
			while true do
				path[#path + 1] = { x, y }
				if #path > limit then break end
				local nx, ny, deg = nil, nil, 0
				for i = 1, 8 do
					local yy = y + P[i][2]
					local xx = x + P[i][1]
					if yy >= 1 and yy <= H and xx >= 1 and xx <= W and m[yy][xx]
						and not (xx == px and yy == py) then
						deg += 1
						nx, ny = xx, yy
					end
				end
				if deg ~= 1 then break end
				px, py = x, y
				x, y = nx :: number, ny :: number
				if neighbours(m, W, H, x, y) > 2 then break end
			end
			if #path <= limit then
				for _, c in ipairs(path) do m[c[2]][c[1]] = false end
				cut += 1
			end
		end
		if cut == 0 then return end
	end
end

-- Thin every region to its spine and place nodes on it.
--
-- Returns `{ nodes, links, spine, stats }`. `spine` is every surviving cell,
-- for drawing. A node carries its world position, normal, region, the kind of
-- place it sits (tip, junction or run) and its degree in the node graph.
function Skeleton.build(data: any, cfg: any?): any
	local c = cfg or {}
	local prune = c.prune or Skeleton.prune
	local spacing = c.spacing or Skeleton.spacing

	local groups, order = {}, {}
	for gi, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			if cell.region then
				local k = gi * 1000000 + cell.region
				local grp = groups[k]
				if not grp then
					grp = { grid = gi, region = cell.region, cells = {} }
					groups[k] = grp
					order[#order + 1] = k
				end
				grp.cells[#grp.cells + 1] = cell
			end
		end
	end

	local nodes = {}
	local spine = {}
	local ownerOf: { [any]: number } = {}   -- skeleton cell -> node id
	local links = {}
	local seen: { [number]: any } = {}
	local function join(a: number, b: number)
		if a == b or a == nil or b == nil then return end
		if a > b then a, b = b, a end
		local kk = a * 1000000 + b
		local L = seen[kk]
		if not L then
			L = { a = a, b = b, pairs = 0 }
			seen[kk] = L
			links[#links + 1] = L
		end
		L.pairs += 1
	end

	local stats = { groups = 0, cells = 0, spine = 0, nodes = 0, links = 0,
		tips = 0, junctions = 0, runs = 0, spanning = 0 }
	local gridsPerRegion = {}
	for _, k in ipairs(order) do
		local grp = groups[k]
		gridsPerRegion[grp.region] = (gridsPerRegion[grp.region] or 0) + 1
	end
	for _, n in pairs(gridsPerRegion) do
		if n > 1 then stats.spanning += 1 end
	end

	for _, k in ipairs(order) do
		local grp = groups[k]
		stats.groups += 1
		stats.cells += #grp.cells

		local ulo, uhi, vlo, vhi = math.huge, -math.huge, math.huge, -math.huge
		for _, cell in ipairs(grp.cells) do
			if cell.ui < ulo then ulo = cell.ui end
			if cell.ui > uhi then uhi = cell.ui end
			if cell.vi < vlo then vlo = cell.vi end
			if cell.vi > vhi then vhi = cell.vi end
		end
		local W, H = uhi - ulo + 1, vhi - vlo + 1
		local m, at = table.create(H), {}
		for y = 1, H do m[y] = table.create(W, false) end
		for _, cell in ipairs(grp.cells) do
			local x, y = cell.ui - ulo + 1, cell.vi - vlo + 1
			m[y][x] = true
			at[y * 100000 + x] = cell
		end

		-- thin to a fixed point
		local guard = 0
		while true do
			guard += 1
			local a = thinPass(m, W, H, 1)
			local b = thinPass(m, W, H, 2)
			if a + b == 0 or guard > 200 then break end
		end
		pruneSpurs(m, W, H, prune)

		-- classify what survived
		local live = {}
		for y = 1, H do
			for x = 1, W do
				if m[y][x] then live[#live + 1] = { x, y } end
			end
		end
		if #live == 0 then continue end
		stats.spine += #live
		for _, q in ipairs(live) do
			local cell = at[q[2] * 100000 + q[1]]
			if cell then spine[#spine + 1] = cell end
		end

		-- A NODE GOES WHERE THE SPINE MEANS SOMETHING: at a tip, at a junction,
		-- and every `spacing` cells along a plain run so a long corridor is not
		-- one enormous edge. A spine with neither tip nor junction is a closed
		-- loop or a single blob; it gets one node so the region is not lost.
		local isNode = {}
		local anyMarked = false
		for _, q in ipairs(live) do
			local d = neighbours(m, W, H, q[1], q[2])
			if d ~= 2 then
				isNode[q[2] * 100000 + q[1]] = true
				anyMarked = true
			end
		end
		if not anyMarked then
			isNode[live[1][2] * 100000 + live[1][1]] = true
		end
		-- space out the runs, in scan order, which is arbitrary but stable
		local since = 0
		for _, q in ipairs(live) do
			local kk = q[2] * 100000 + q[1]
			if isNode[kk] then
				since = 0
			else
				since += 1
				if since >= spacing then
					isNode[kk] = true
					since = 0
				end
			end
		end

		local idOfKey = {}
		for _, q in ipairs(live) do
			local kk = q[2] * 100000 + q[1]
			if isNode[kk] then
				local cell = at[kk]
				if cell then
					local d = neighbours(m, W, H, q[1], q[2])
					local kind = (d <= 1) and "tip" or (d >= 3) and "junction" or "run"
					local id = #nodes + 1
					nodes[id] = { id = id, pos = cell.pos, normal = cell.normal,
						region = grp.region, grid = grp.grid, kind = kind,
						ui = cell.ui, vi = cell.vi, degree = 0 }
					idOfKey[kk] = id
					ownerOf[cell] = id
					if kind == "tip" then stats.tips += 1
					elseif kind == "junction" then stats.junctions += 1
					else stats.runs += 1 end
				end
			end
		end

		-- Multi-source flood along the spine from every node. A cell is owned by
		-- the node it was reached from, and wherever two owners meet, their
		-- nodes are joined. That gives the arcs of the skeleton without tracing
		-- any of them, and it is correct at a junction where three arcs meet.
		local owner = {}
		local queue = {}
		for kk, id in pairs(idOfKey) do
			owner[kk] = id
			queue[#queue + 1] = kk
		end
		local head = 1
		while head <= #queue do
			local kk = queue[head]; head += 1
			local y = math.floor(kk / 100000)
			local x = kk - y * 100000
			for i = 1, 8 do
				local xx, yy = x + P[i][1], y + P[i][2]
				if xx >= 1 and xx <= W and yy >= 1 and yy <= H and m[yy][xx] then
					local nk = yy * 100000 + xx
					if owner[nk] == nil then
						owner[nk] = owner[kk]
						local cell = at[nk]
						if cell then ownerOf[cell] = owner[kk] end
						queue[#queue + 1] = nk
					elseif owner[nk] ~= owner[kk] then
						join(owner[kk], owner[nk])
					end
				end
			end
		end
	end

	-- ACROSS THE SEAMS, through Severance's gate rather than a copy of it. Two
	-- spine cells on different parts that an agent could step between join their
	-- owners, which is what carries a staircase and a region split over two
	-- slabs.
	local G = math.max(Severance.stepPlane, Severance.stepNormal)
	local plane2 = Severance.stepPlane * Severance.stepPlane
	local hash: { [string]: { any } } = {}
	for _, cell in ipairs(spine) do
		local p = cell.pos
		local kk = ("%d,%d,%d"):format(
			math.floor(p.X / G), math.floor(p.Y / G), math.floor(p.Z / G))
		local t = hash[kk]
		if not t then t = {}; hash[kk] = t end
		t[#t + 1] = cell
	end
	for _, a in ipairs(spine) do
		local p = a.pos
		local up = a.normal or Vector3.yAxis
		local bx, by, bz = math.floor(p.X / G), math.floor(p.Y / G), math.floor(p.Z / G)
		for ox = -1, 1 do
			for oy = -1, 1 do
				for oz = -1, 1 do
					local t = hash[("%d,%d,%d"):format(bx + ox, by + oy, bz + oz)]
					if t then
						for _, b in ipairs(t) do
							if b ~= a and ownerOf[b] ~= ownerOf[a] then
								local dv = b.pos - p
								local dn = dv:Dot(up)
								local flat = dv - up * dn
								if flat:Dot(flat) <= plane2
									and math.abs(dn) <= Severance.stepNormal then
									join(ownerOf[a], ownerOf[b])
								end
							end
						end
					end
				end
			end
		end
	end

	for _, L in ipairs(links) do
		nodes[L.a].degree += 1
		nodes[L.b].degree += 1
	end
	stats.nodes = #nodes
	stats.links = #links
	stats.orphans = 0
	for _, n in ipairs(nodes) do
		if n.degree == 0 then stats.orphans += 1 end
	end
	return { nodes = nodes, links = links, spine = spine, stats = stats }
end

function Skeleton.report(res: any): string
	local s = res.stats
	local lines = {
		("skeleton  %d cells -> %d spine (%.1f%%), %d nodes, %d links")
			:format(s.cells, s.spine, s.cells > 0 and (s.spine / s.cells * 100) or 0,
				s.nodes, s.links),
		("  %d tips, %d junctions, %d along runs, over %d region pieces")
			:format(s.tips, s.junctions, s.runs, s.groups),
	}
	if s.spanning > 0 then
		lines[#lines + 1] = ("  %d regions span more than one grid, thinned per grid")
			:format(s.spanning)
	end
	if s.orphans > 0 then
		lines[#lines + 1] = ("  ! %d nodes with no link at all"):format(s.orphans)
	end
	return table.concat(lines, "\n")
end

return Skeleton
