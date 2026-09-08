--!strict
-- REGION LINKING -- the graph the regions were always implying.
--
-- Contour treats every region in isolation: a set of closed polygons that know
-- nothing about each other. That is enough to draw a floor and useless for
-- anything else, because the two questions we actually want to ask are both
-- about the space BETWEEN regions -- which boundary cells are a doorway and
-- which are just the edge of the world, and whether a crouch region is a
-- shortcut or a pointless detour.
--
-- The adjacency test is NOT new. `NodeWalk.purge` already flood-fills cells
-- with a step-tolerant gate (in-plane <= stepPlane, along-normal <= stepNormal)
-- and that gate is what makes a staircase read as ONE component instead of one
-- per tread. Running the same gate across a region boundary rather than within
-- one is the whole idea here: a link is a place where you could have walked
-- straight on, and the regions split only because the plane bucket changed.
--
--   * portals -- contiguous runs of linked cell pairs between the same two
--                regions, each with a centre, a span and a passability verdict
--   * graph   -- regions as nodes, portals as edges
--   * prune   -- crouch/prone regions the graph proves are redundant

local RegionLink = {}

local DEFAULT = {
	leaf = 0.5,
	stepPlane = 0.75,   -- max in-plane separation, same as purge's link
	stepNormal = 0.6,   -- max offset along the normal: one step
	agentWidth = 2,     -- recorded against the span, but NEVER used to cut a link
	flush = 0.12,       -- |step| below this and the two regions are level
	-- Cells of one portal are grouped by walking cell-to-cell along the seam.
	-- The seam is a row of cells, so the reach only has to clear one leaf plus
	-- the diagonal; 1.6 leaves is the same slack purge uses for its radius scan.
	seamReach = 1.6,
}

local function merged(cfg)
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if c[k] ~= nil then c[k] = v end end end
	return c
end

local function key(x: number, y: number, z: number): number
	return x * 73856093 + y * 19349663 + z * 83492791
end

-- Same spatial hash as purge. It is rebuilt rather than passed in because the
-- caller holds `walk` and `regionOf`, not purge's locals -- and rebuilding is
-- a single pass over cells we are about to scan anyway.
local function spatial(walk, live)
	local CELLSZ = 1.0
	local hash = {}
	for i, cell in ipairs(walk) do
		if live(i) then
			local k = key(
				math.floor(cell.face.X / CELLSZ),
				math.floor(cell.face.Y / CELLSZ),
				math.floor(cell.face.Z / CELLSZ))
			local b = hash[k]; if not b then b = {}; hash[k] = b end
			table.insert(b, i)
		end
	end
	return function(v: Vector3, r: number)
		local out = {}
		for x = math.floor((v.X - r) / CELLSZ), math.floor((v.X + r) / CELLSZ) do
		for y = math.floor((v.Y - r) / CELLSZ), math.floor((v.Y + r) / CELLSZ) do
		for z = math.floor((v.Z - r) / CELLSZ), math.floor((v.Z + r) / CELLSZ) do
			local b = hash[key(x, y, z)]
			if b then for _, i in ipairs(b) do table.insert(out, i) end end
		end end end
		return out
	end
end

-- Every cell pair that straddles a region boundary and passes the step gate.
function RegionLink.seams(walk, regionOf, cfg)
	local c = merged(cfg)
	local near = spatial(walk, function(i) return regionOf[i] ~= nil end)
	local r = math.max(c.stepPlane, c.stepNormal) * 1.05

	local seams = {}   -- "ra|rb" -> { {i, j}, ... }, ra < rb
	local seen = {}
	for i, cell in ipairs(walk) do
		local ra = regionOf[i]
		if ra then
			for _, j in ipairs(near(cell.face, r)) do
				local rb = regionOf[j]
				if rb and rb ~= ra then
					local lo, hi = math.min(i, j), math.max(i, j)
					local pk = lo * 1e7 + hi
					if not seen[pk] then
						local dv = walk[j].face - cell.face
						local dn = dv:Dot(cell.up)
						local dp = (dv - cell.up * dn).Magnitude
						if dp <= c.stepPlane and math.abs(dn) <= c.stepNormal then
							seen[pk] = true
							local a, b = ra, rb
							local ci, cj = i, j
							if a > b then a, b = b, a; ci, cj = j, i end
							local sk = a .. "|" .. b
							local s = seams[sk]
							if not s then s = { a = a, b = b, pairs = {} }; seams[sk] = s end
							table.insert(s.pairs, { ci, cj })
						end
					end
				end
			end
		end
	end
	return seams
end

-- One seam between two regions can be several DOORWAYS -- two rooms joined by
-- a floor on both sides of a pillar are one region pair and two openings. Split
-- each seam into contiguous runs by walking the cells on the low-id side.
local function split(seam, walk, c)
	local cells, index = {}, {}
	for _, p in ipairs(seam.pairs) do
		if not index[p[1]] then index[p[1]] = true; table.insert(cells, p[1]) end
	end
	local reach = c.leaf * c.seamReach

	-- Hash the seam cells before grouping. Scanning every seam cell for every
	-- pop is fine on a 20-cell doorway and quadratic death on a seam of a few
	-- thousand, which case5 has -- and a wedged Studio is not recoverable.
	local G = reach
	local bucket = {}
	for _, i in ipairs(cells) do
		local p = walk[i].face
		local k = key(math.floor(p.X / G), math.floor(p.Y / G), math.floor(p.Z / G))
		local b = bucket[k]; if not b then b = {}; bucket[k] = b end
		table.insert(b, i)
	end

	local group, out = {}, {}
	for _, seed in ipairs(cells) do
		if not group[seed] then
			local id = #out + 1
			group[seed] = id
			local stack, members = { seed }, {}
			while #stack > 0 do
				local k = table.remove(stack) :: number
				table.insert(members, k)
				local p = walk[k].face
				local bx, by, bz = math.floor(p.X / G), math.floor(p.Y / G), math.floor(p.Z / G)
				for dx = -1, 1 do for dy = -1, 1 do for dz = -1, 1 do
					local b = bucket[key(bx + dx, by + dy, bz + dz)]
					if b then
						for _, m in ipairs(b) do
							if not group[m] and (walk[m].face - p).Magnitude <= reach then
								group[m] = id; table.insert(stack, m)
							end
						end
					end
				end end end
			end
			table.insert(out, members)
		end
	end
	return out
end

-- Regions as nodes, portals as edges.
function RegionLink.build(walk, regions, regionOf, cfg)
	local c = merged(cfg)
	local seams = RegionLink.seams(walk, regionOf, c)

	local byId = {}
	for _, r in ipairs(regions) do byId[r.id] = r end

	local portals = {}
	for _, seam in pairs(seams) do
		-- who each side-a cell actually touches on side b
		local partner = {}
		for _, pr in ipairs(seam.pairs) do
			local t = partner[pr[1]]; if not t then t = {}; partner[pr[1]] = t end
			table.insert(t, pr[2])
		end
		for _, members in ipairs(split(seam, walk, c)) do
			local sum = Vector3.zero
			for _, i in ipairs(members) do sum += walk[i].face end
			local centre = sum / #members
			-- Span is DESCRIPTION, not a verdict. A diagonal run of n cells is
			-- not n leaves wide, so measure the real extent -- but never gate on
			-- it: a 1.5-wide seam across open floor is perfectly walkable, and
			-- cutting it for being narrower than the agent box cuts a link that
			-- was legitimate.
			local far = 0
			for _, i in ipairs(members) do
				far = math.max(far, (walk[i].face - centre).Magnitude)
			end
			local span = far * 2 + c.leaf

			-- WALLS DECIDE PASSABILITY, NOT WIDTH. `fits` is the probe's real
			-- body test -- an agent box placed at the cell and checked against
			-- geometry with GetPartsInPart -- so it already accounts for the
			-- railing, doorframe or overhang that a span measurement cannot see.
			-- A portal is passable if a body fits ANYWHERE along the seam, on
			-- both sides of it.
			local fitCells, minClear, drop, np = 0, math.huge, 0, 0
			for _, i in ipairs(members) do
				local ca = walk[i]
				minClear = math.min(minClear, ca.clearance or 0)
				local ok = false
				for _, j in ipairs(partner[i] or {}) do
					local cb = walk[j]
					minClear = math.min(minClear, cb.clearance or 0)
					drop += (cb.face - ca.face):Dot(ca.up); np += 1
					if ca.fits and cb.fits then ok = true end
				end
				if ok then fitCells += 1 end
			end
			local step = (np > 0) and (drop / np) or 0

			table.insert(portals, {
				a = seam.a, b = seam.b,
				cells = members, count = #members,
				centre = centre, span = span, step = step,
				fitCells = fitCells, fitFrac = fitCells / #members,
				minClearance = (minClear == math.huge) and 0 or minClear,
				passable = fitCells > 0,
				narrow = span < c.agentWidth,
			})
		end
	end

	-- How two regions connect, from the asking region's side: the same seam is
	-- a step UP from below and a step DOWN from above, so the sign flips.
	local function describe(step, c2)
		if math.abs(step) <= c2.flush then return "flush" end
		return (step > 0) and "step up" or "step down"
	end

	local adj, degree = {}, {}
	for _, r in ipairs(regions) do adj[r.id] = {}; degree[r.id] = 0 end
	-- EVERY seam is an edge. Passability is an ATTRIBUTE of the link, never a
	-- filter on it -- both filters tried here were wrong in the same direction.
	-- Span cut legitimate links: a 1.5-wide seam across open floor is walkable,
	-- width only matters where there is something beside it to bump into. And
	-- `fits` cut far more: it is the strict agent-box test, true for 5073 of
	-- 56306 cells, so demanding it at the seam itself dropped case3 from 13
	-- passable portals to 4 and left 43 of 50 regions isolated. Whoever plans a
	-- path can weigh `span`, `clearance` and `fitCells`; the graph's job is to
	-- record that the connection is THERE.
	for pi, p in ipairs(portals) do
		if adj[p.a] and adj[p.b] then
			local function link(from, to, sign)
				local s = sign * p.step
				table.insert(adj[from], {
					to = to, portal = pi, step = s, kind = describe(s, c),
					span = p.span, clearance = p.minClearance,
					width = p.fitCells * c.leaf, passable = p.passable,
				})
			end
			link(p.a, p.b, 1)
			link(p.b, p.a, -1)
			degree[p.a] += 1; degree[p.b] += 1
		end
	end

	-- A region should be able to answer "who are my neighbours, and how do I
	-- get to them?" on its own, without the caller holding the graph.
	for _, r in ipairs(regions) do r.neighbours = adj[r.id] end

	-- connected components of the REGION graph
	local comp, ncomp = {}, 0
	for _, r in ipairs(regions) do
		if not comp[r.id] then
			ncomp += 1
			local stack = { r.id }; comp[r.id] = ncomp
			while #stack > 0 do
				local k = table.remove(stack)
				for _, e in ipairs(adj[k]) do
					if not comp[e.to] then comp[e.to] = ncomp; table.insert(stack, e.to) end
				end
			end
		end
	end

	local blocked, narrow, isolated = 0, 0, 0
	for _, p in ipairs(portals) do
		if not p.passable then blocked += 1 end
		if p.narrow then narrow += 1 end
	end
	for _, r in ipairs(regions) do if degree[r.id] == 0 then isolated += 1 end end

	return {
		portals = portals, adj = adj, degree = degree, comp = comp,
		byId = byId, config = c,
		stats = {
			regions = #regions, portals = #portals,
			passable = #portals - blocked, blocked = blocked,
			narrow = narrow,             -- span < agentWidth, kept anyway
			isolated = isolated, components = ncomp,
		},
	}
end

-- Is `id` load-bearing? Drop it and see whether its neighbours still reach each
-- other. A region that is a dead end, or a detour around a link that already
-- exists, is not.
local function loadBearing(g, id): boolean
	local nbrs = {}
	for _, e in ipairs(g.adj[id]) do nbrs[e.to] = true end
	local first = nil
	for k in pairs(nbrs) do first = k; break end
	if first == nil then return false end   -- isolated: nothing depends on it

	local seen = { [first] = true }
	local stack = { first }
	while #stack > 0 do
		local k = table.remove(stack)
		for _, e in ipairs(g.adj[k]) do
			if e.to ~= id and not seen[e.to] then
				seen[e.to] = true; table.insert(stack, e.to)
			end
		end
	end
	for k in pairs(nbrs) do if not seen[k] then return true end end
	return false
end

-- The payoff: a crouch or prone region the graph proves nobody needs. Standing
-- regions are never dropped here however redundant -- they are floor, and floor
-- is the thing we are trying to describe.
function RegionLink.prune(g, cfg)
	local c = merged(cfg)
	local drop, kept, reasons = {}, {}, {}
	for id, r in pairs(g.byId) do
		local low = (r.mode == "crouch" or r.mode == "prone")
		if low and not loadBearing(g, id) then
			drop[id] = true
			reasons[id] = (g.degree[id] == 0) and "isolated"
				or (g.degree[id] == 1) and "dead end"
				or "redundant detour"
		end
	end
	for id in pairs(g.byId) do if not drop[id] then table.insert(kept, id) end end
	local n = 0
	for _ in pairs(drop) do n += 1 end
	return drop, { dropped = n, kept = #kept, reasons = reasons }
end

local ROOT_NAME = "NavGen"
local SECTION = "Portals"

function RegionLink.clear()
	local root = workspace:FindFirstChild(ROOT_NAME)
	local sec = root and root:FindFirstChild(SECTION)
	if sec then local n = #sec:GetChildren(); sec:Destroy(); return n end
	return 0
end

-- A portal is drawn as a bar across its own opening: green passable, red not.
function RegionLink.draw(g, walk, opts)
	opts = opts or {}
	local root = workspace:FindFirstChild(ROOT_NAME)
	if not root then root = Instance.new("Folder"); root.Name = ROOT_NAME; root.Parent = workspace end
	local sec = root:FindFirstChild(SECTION)
	if not sec then sec = Instance.new("Folder"); sec.Name = SECTION; sec.Parent = root end

	local n = 0
	for pi, p in ipairs(g.portals) do
		local up = walk[p.cells[1]].up
		-- the bar lies along the seam: the direction of greatest spread
		local dir = Vector3.zero
		if #p.cells > 1 then
			local best = -1
			for _, i in ipairs(p.cells) do
				local d = walk[i].face - p.centre
				if d.Magnitude > best then best = d.Magnitude; dir = d end
			end
		end
		if dir.Magnitude < 1e-4 then dir = up:Cross(Vector3.new(0, 0, 1)) end
		if dir.Magnitude < 1e-4 then dir = up:Cross(Vector3.new(1, 0, 0)) end

		local bar = Instance.new("Part")
		bar.Anchored = true; bar.CanCollide = false; bar.CanQuery = false; bar.CanTouch = false
		bar.Material = Enum.Material.Neon
		bar.Color = p.passable and Color3.fromRGB(60, 255, 90) or Color3.fromRGB(255, 40, 40)
		bar.Size = Vector3.new(0.35, 0.35, math.max(p.span, 0.4))
		bar.CFrame = CFrame.lookAt(p.centre + up * (opts.lift or 0.9), p.centre + up * (opts.lift or 0.9) + dir.Unit)
		bar.Name = ("p%d_%d-%d_%.2f"):format(pi, p.a, p.b, p.span)
		bar.Parent = sec
		n += 1
	end
	return n
end

function RegionLink.run(walk, regions, regionOf, cfg)
	local g = RegionLink.build(walk, regions, regionOf, cfg)
	local drop, pstats = RegionLink.prune(g, g.config)
	g.drop = drop
	g.stats.prune = pstats
	return g
end

return RegionLink
