--!strict
-- NVGN.NodeGraph -- faces decide where the nodes are, cells decide what reaches
-- what.
--
-- Keeping those two apart is the whole design, and mixing them was the mistake
-- that produced every wrong link we chased. A face is a convex patch of floor,
-- so its centroid is a sensible place to stand and the count of faces is a
-- sensible count of nodes. But two face OUTLINES coming near each other is not
-- the same question as an agent being able to step across, and answering the
-- second with the first linked a stair tread to the crawl space under the
-- staircase while refusing two nodes a clean stud apart.
--
-- So connections come from CELLS, through Severance's own gate, which is the
-- same rule the connectivity report uses. The node graph and the connectivity
-- check can then never disagree about what is reachable.
--
-- THREE THINGS THE GATE ALONE GETS WRONG, and what each one needs:
--
--   * a crate post wedged between two cells half a stud apart. The gate says
--     neighbours. Solid space has to veto it.
--   * a spatial query cannot see the crate post. 114 of case3's 625 parts have
--     CanQuery off, and GetPartsInPart and Raycast both skip those silently. So
--     the veto indexes the part list itself and tests points against oriented
--     boxes. Nothing here queries the world.
--   * a stair riser fills the space between two treads, so a probe at the
--     midpoint of a cell pair is inside solid material on every step ever
--     built. The probe is anchored to whichever cell is HIGHER, where an agent
--     would actually be standing. A step then reads clear and a wall does not,
--     because a wall is tall enough to be in the way from either surface.
--
-- A CULLED FACE STILL CONDUCTS. Dropping a small face as a node must not drop
-- the route through it, so its cells stay in the graph as wire: each connected
-- blob of them joins whatever surviving nodes touch its ends.

local NodeGraph = {}

local Severance = require(script.Parent:WaitForChild("Severance"))

-- A face smaller than this in square studs gets no node of its own. It still
-- conducts. A quarter of case3's faces are under 2.25 square studs, so this is
-- doing real work rather than trimming outliers.
NodeGraph.minArea = 2.0

-- Adjacent nodes closer together than this collapse into one, at the
-- area-weighted centre of the group. ADJACENT ONLY: two nodes a stud apart with
-- a wall between them are not one place, and plain distance clustering cannot
-- tell the difference. Convex merging already absorbs most of what this would
-- catch, so it fires rarely -- 4 nodes into 2 groups on case3.
NodeGraph.clusterGap = 2.0

-- Heights above the standing surface at which the gap between two cells is
-- probed, in studs. Raising these was measured and is worse, not better: at
-- 1.5/2.5 the crawl space under case3's stairs reconnects and at 2.0/3.0 a
-- crate link comes back, because the probes start clearing the obstacle instead
-- of hitting it.
NodeGraph.probeHeights = { 0.4, 1.0, 1.6 }

-- How far outside a face a cell may sit and still be counted as belonging to
-- it, in studs. Simplification moves the boundary a little, so the rim cells of
-- a region land just outside their own polygon. On case3, 8252 cells fall
-- inside a face, 458 within half a stud, and 15 further, the worst at 0.74.
NodeGraph.faceSlack = 0.8

local FIT_NAME = { "prone", "crouch", "stand" }

-- Oriented-box index over the model's parts.
--
-- NOT A SPATIAL QUERY. `GetPartsInPart` and `Raycast` both skip anything with
-- CanQuery off, which on case3 is 114 parts including the crate posts that
-- started this. Indexing the list ourselves is the only way to see them.
local function partIndex(parts: { any }, grid: number): any
	local hash: { [string]: { any } } = {}
	for _, d in ipairs(parts) do
		local cf, s = d.CFrame, d.Size * 0.5
		local ex = Vector3.new(
			math.abs(cf.RightVector.X)*s.X + math.abs(cf.UpVector.X)*s.Y + math.abs(cf.LookVector.X)*s.Z,
			math.abs(cf.RightVector.Y)*s.X + math.abs(cf.UpVector.Y)*s.Y + math.abs(cf.LookVector.Y)*s.Z,
			math.abs(cf.RightVector.Z)*s.X + math.abs(cf.UpVector.Z)*s.Y + math.abs(cf.LookVector.Z)*s.Z)
		local lo, hi = d.Position - ex, d.Position + ex
		for x = math.floor(lo.X/grid), math.floor(hi.X/grid) do
			for y = math.floor(lo.Y/grid), math.floor(hi.Y/grid) do
				for z = math.floor(lo.Z/grid), math.floor(hi.Z/grid) do
					local k = ("%d,%d,%d"):format(x, y, z)
					local t = hash[k]
					if not t then t = {}; hash[k] = t end
					t[#t + 1] = d
				end
			end
		end
	end
	return function(p: Vector3): boolean
		local k = ("%d,%d,%d"):format(
			math.floor(p.X/grid), math.floor(p.Y/grid), math.floor(p.Z/grid))
		for _, d in ipairs(hash[k] or {}) do
			local lp = d.CFrame:PointToObjectSpace(p)
			local s = d.Size * 0.5
			if math.abs(lp.X) <= s.X and math.abs(lp.Y) <= s.Y and math.abs(lp.Z) <= s.Z then
				return true
			end
		end
		return false
	end
end

-- How far outside a convex face a point lies, 0 when inside.
local function outside(t: any, p: Vector3): number
	local v, up = t.verts, t.up
	local worst = 0
	for k = 1, #v do
		local a, b = v[k], v[(k % #v) + 1]
		local e = b - a
		local len = e.Magnitude
		if len > 1e-9 then
			local d = -(e:Cross(p - a):Dot(up)) / len
			if d > worst then worst = d end
		end
	end
	return worst
end

-- Faces to nodes, cells to links.
--
-- `tri` is a Triangulate result. Returns `{ nodes, links, stats }`; a node
-- carries its world centre, its region, its posture, the faces behind it and
-- its degree, and a link carries the two node ids, how many cell openings
-- support it, and whether it only exists through a culled face.
function NodeGraph.build(data: any, tri: any, cfg: any?): any
	local c = cfg or {}
	local minArea = c.minArea or NodeGraph.minArea
	local gap = c.clusterGap or NodeGraph.clusterGap
	local heights = c.probeHeights or NodeGraph.probeHeights
	local faces = tri.tris

	local stats = { faces = #faces, culled = 0, nodes = 0, clustered = 0,
		links = 0, bridged = 0, tested = 0, blocked = 0, conductors = 0,
		orphans = 0, posture = { 0, 0, 0 } }

	-- 1. which faces are too small to deserve a node
	local dead = {}
	for i, t in ipairs(faces) do
		if t.area < minArea then dead[i] = true; stats.culled += 1 end
	end

	-- 2. cluster surviving faces that share an edge and sit close together
	local uf = {}
	local function find(a: number): number
		while uf[a] ~= a do a = uf[a] end
		return a
	end
	for i in ipairs(faces) do uf[i] = i end
	local function vkey(v: Vector3): string
		return ("%.2f,%.2f,%.2f"):format(v.X, v.Y, v.Z)
	end
	local byVert: { [string]: { number } } = {}
	for i, t in ipairs(faces) do
		for _, v in ipairs(t.verts) do
			local k = vkey(v)
			byVert[k] = byVert[k] or {}
			table.insert(byVert[k], i)
		end
	end
	local shares: { [number]: number } = {}
	for _, list in pairs(byVert) do
		for a = 1, #list do
			for b = a + 1, #list do
				local i, j = list[a], list[b]
				if i > j then i, j = j, i end
				local k = i * 1000000 + j
				shares[k] = (shares[k] or 0) + 1
			end
		end
	end
	for k, n in pairs(shares) do
		if n >= 2 then
			local j = k % 1000000
			local i = (k - j) / 1000000
			if not dead[i] and not dead[j]
				and (faces[j].centre - faces[i].centre).Magnitude < gap then
				local ra, rb = find(i), find(j)
				if ra ~= rb then uf[ra] = rb end
			end
		end
	end

	-- 3. one node per surviving group, at its area-weighted centre. Weighting by
	-- area puts it where most of the floor is instead of halfway between two
	-- faces of very different size.
	local nodes, nodeOfFace, indexOf = {}, {}, {}
	for i, t in ipairs(faces) do
		if not dead[i] then
			local root = find(i)
			local id = indexOf[root]
			if not id then
				id = #nodes + 1
				indexOf[root] = id
				nodes[id] = { id = id, region = t.region, up = t.up, area = 0,
					acc = Vector3.zero, faces = {}, fit = 3, degree = 0 }
			end
			local n = nodes[id]
			n.area += t.area
			n.acc += t.centre * t.area
			n.faces[#n.faces + 1] = i
			nodeOfFace[i] = id
		end
	end
	for _, n in ipairs(nodes) do
		n.centre = n.acc / n.area
		n.acc = nil
		if #n.faces > 1 then stats.clustered += 1 end
	end
	stats.nodes = #nodes

	-- 4. every cell to the face it sits in
	local byRegion: { [number]: { number } } = {}
	for i, t in ipairs(faces) do
		byRegion[t.region] = byRegion[t.region] or {}
		table.insert(byRegion[t.region], i)
	end
	local live, faceOf = {}, {}
	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			if cell.region then
				local best, bd = nil, math.huge
				for _, fi in ipairs(byRegion[cell.region] or {}) do
					local d = outside(faces[fi], cell.pos)
					if d < bd then bd, best = d, fi end
					if bd <= 0 then break end
				end
				if best and bd <= NodeGraph.faceSlack then
					faceOf[cell] = best
					live[#live + 1] = cell
				end
			end
		end
	end

	-- 5. cell adjacency, gated by Severance and vetoed by solid space
	local solid = partIndex(data.parts or {}, 4)
	local CG = 2
	local hash: { [string]: { any } } = {}
	for _, cell in ipairs(live) do
		local p = cell.pos
		local k = ("%d,%d,%d"):format(math.floor(p.X/CG), math.floor(p.Y/CG), math.floor(p.Z/CG))
		hash[k] = hash[k] or {}
		table.insert(hash[k], cell)
	end
	local nb: { [any]: { any } } = {}
	for _, a in ipairs(live) do
		local p = a.pos
		local up = a.normal or Vector3.yAxis
		local bx, by, bz = math.floor(p.X/CG), math.floor(p.Y/CG), math.floor(p.Z/CG)
		for ox = -1, 1 do
			for oy = -1, 1 do
				for oz = -1, 1 do
					local t = hash[("%d,%d,%d"):format(bx+ox, by+oy, bz+oz)]
					if t then
						for _, b in ipairs(t) do
							if b ~= a then
								local dv = b.pos - p
								local dn = dv:Dot(up)
								local flat = (dv - up * dn).Magnitude
								if math.abs(dn) <= Severance.stepNormal
									and flat <= Severance.stepPlane then
									stats.tested += 1
									-- ANCHORED TO THE HIGHER CELL. From the
									-- midpoint, a riser reads solid on every
									-- staircase there is.
									local baseY = math.max(a.pos.Y, b.pos.Y)
									local mid = Vector3.new(
										(a.pos.X + b.pos.X) * 0.5, baseY,
										(a.pos.Z + b.pos.Z) * 0.5)
									local ok = true
									for _, h in ipairs(heights) do
										if solid(mid + up * h) then ok = false; break end
									end
									if ok then
										nb[a] = nb[a] or {}
										table.insert(nb[a], b)
									else
										stats.blocked += 1
									end
								end
							end
						end
					end
				end
			end
		end
	end

	-- 6. links: cells of different nodes touching, plus routes through the
	-- cells of culled faces
	local link, lorder = {}, {}
	local function join(a: number, b: number, bridged: boolean)
		if a == b then return end
		local x, y = a, b
		if x > y then x, y = y, x end
		local k = x * 1000000 + y
		local e = link[k]
		if not e then
			e = { a = x, b = y, openings = 0, bridged = bridged }
			link[k] = e
			lorder[#lorder + 1] = k
		end
		e.openings += 1
		if not bridged then e.bridged = false end
	end
	local ownerOf, conductors = {}, {}
	for _, cell in ipairs(live) do
		local id = nodeOfFace[faceOf[cell]]
		if id then ownerOf[cell] = id else conductors[#conductors + 1] = cell end
	end
	stats.conductors = #conductors
	for _, a in ipairs(live) do
		local ia = ownerOf[a]
		if ia then
			for _, b in ipairs(nb[a] or {}) do
				local ib = ownerOf[b]
				if ib and ib ~= ia then join(ia, ib, false) end
			end
		end
	end
	local seen = {}
	for _, s in ipairs(conductors) do
		if not seen[s] then
			local stack, touch = { s }, {}
			seen[s] = true
			while #stack > 0 do
				local cur = table.remove(stack)
				for _, b in ipairs(nb[cur] or {}) do
					local id = ownerOf[b]
					if id then touch[id] = true
					elseif not seen[b] then seen[b] = true; table.insert(stack, b) end
				end
			end
			local list = {}
			for id in pairs(touch) do list[#list + 1] = id end
			for i = 1, #list do
				for j = i + 1, #list do join(list[i], list[j], true) end
			end
		end
	end
	local links = {}
	for _, k in ipairs(lorder) do links[#links + 1] = link[k] end
	stats.links = #links
	for _, L in ipairs(links) do
		if L.bridged then stats.bridged += 1 end
		nodes[L.a].degree += 1
		nodes[L.b].degree += 1
	end

	-- 7. posture: the most restrictive any cell of a node allows. A node covering
	-- a face that is half crouch tunnel must not claim you can walk it upright.
	for _, cell in ipairs(live) do
		local id = ownerOf[cell]
		if id then
			local f = cell.fit or 3
			if f < nodes[id].fit then nodes[id].fit = f end
		end
	end
	for _, n in ipairs(nodes) do
		n.posture = FIT_NAME[n.fit]
		stats.posture[n.fit] += 1
		if n.degree == 0 then stats.orphans += 1 end
	end
	for _, L in ipairs(links) do
		L.fit = math.min(nodes[L.a].fit, nodes[L.b].fit)
		L.posture = FIT_NAME[L.fit]
	end

	return { nodes = nodes, links = links, stats = stats }
end

function NodeGraph.report(res: any): string
	local s = res.stats
	local lines = {
		("nodes     %d faces -> %d nodes (%d culled, %d clustered), %d links")
			:format(s.faces, s.nodes, s.culled, s.clustered, s.links),
		("  %d prone, %d crouch, %d stand; %d links bridge a culled face")
			:format(s.posture[1], s.posture[2], s.posture[3], s.bridged),
		("  %d of %d cell pairs vetoed by solid space (%.1f%%), %d conductor cells")
			:format(s.blocked, s.tested,
				s.tested > 0 and (s.blocked / s.tested * 100) or 0, s.conductors),
	}
	if s.orphans > 0 then
		lines[#lines + 1] = ("  ! %d nodes with no link at all"):format(s.orphans)
	end
	return table.concat(lines, "\n")
end

return NodeGraph
