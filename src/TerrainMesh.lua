--!strict
-- NVGN.TerrainMesh -- terrain gets its own mesh.
--
-- Parts are flat faces, so a region's outline plus CDT describes them. Terrain
-- is a curved surface: traced and CDT'd, a hill came out as a few flat polygons
-- sitting 9 to 32 studs under its ridges. Its nodes (one per 2x2 studs, four
-- casts per voxel) already are the surface, so they are meshed directly:
--
--   1. TRIANGULATE  the node lattice, as a height map. Two nodes join when their
--                   planes agree within joinTol at the midpoint; a lattice
--                   square becomes two triangles when all four sides join, and
--                   single triangles where only some do (the rim of a cliff).
--   2. SIMPLIFY     quadric edge collapse (Garland-Heckbert). Rim edges carry a
--                   side-plane quadric, so the outline keeps its shape; a
--                   collapse that folds or stands a triangle up is refused.
--   3. MERGE        neighbouring triangles into convex polygons -- convex from
--                   above AND in their own plane -- while every vertex stays
--                   within mergeDev of the polygon's plane.
--
-- Settings are Cocosulx's pick on the islands (2026-09-24): 637 polygons, about
-- the density of part floor, 0.34 studs mean off the nodes, none concave.
--
-- Every mesh-connected piece becomes one region, and the terrain cells are
-- re-labelled into the region of the polygon they lie in -- so Severance,
-- GridPortals and Leaps see terrain the way they see a part's floor.

local TerrainMesh = {}

TerrainMesh.joinTol = 1.0      -- studs two nodes' planes may differ at their midpoint
TerrainMesh.simplifyTol = 16   -- quadric error a collapse may reach (sqrt of the cost cap)
TerrainMesh.mergeAngle = 20    -- degrees between two polygons' planes that may merge
TerrainMesh.mergeDev = 1.5     -- studs a merged polygon's vertex may sit off its plane
-- MICRO BUMPS ARE IGNORED. A 0.6 stud bump made 38-56 degree slivers that the
-- angle test refused to merge; within bumpTol of one plane, any angle merges.
TerrainMesh.bumpTol = 0.5
TerrainMesh.boundW = 2         -- weight of the rim's side planes, per stud of rim
TerrainMesh.claimDrop = 2.0    -- studs a cell may sit off the polygon it is claimed into
-- THE RIM IS HELD, NOT WEIGHTED. The side-plane quadric alone let rim edges cut
-- across the ground beside case4's walls. A rim vertex is removed only while
-- every rim node it stood for stays within rimTol (plan) of the new rim edge.
TerrainMesh.rimTol = 0.75       -- studs (plan)
-- ONE HEADROOM PER POLYGON. A polygon's headroom is read over 75% of its area,
-- so ground under case4's outside staircase merged into open ground read as
-- open to a 10 stud body. Nodes are banded by clearance at these heights, and
-- neither a collapse nor a merge joins two bands.
TerrainMesh.headroomBands = { 3, 5, 7.5, 10 }

local function bandOf(clearance: number): number
	local n = 0
	for _, h in ipairs(TerrainMesh.headroomBands) do
		if clearance >= h then n += 1 end
	end
	return n
end

local function planeY(c: any, x: number, z: number): number
	local n, p = c.normal, c.pos
	if math.abs(n.Y) < 1e-3 then return p.Y end
	return p.Y - ((x - p.X) * n.X + (z - p.Z) * n.Z) / n.Y
end

local function heightAt(f: any, p: Vector3): number
	local c, up = f.centre, f.up
	if math.abs(up.Y) < 1e-3 then return c.Y end
	return c.Y - ((p.X - c.X) * up.X + (p.Z - c.Z) * up.Z) / up.Y
end

-- ------------------------------------------------------------ 1. triangulate
-- AN EDGE MAY NOT CROSS A PART. Where a pillar stands on the ground its node is
-- missing, and a rim triangle joined the neighbours diagonally straight through
-- it (case4's wooden pillar). Each edge is swept between its two nodes, a thin
-- blade from edgeLift to edgeTop over the ground, capped under the lower node's
-- own ceiling.
TerrainMesh.edgeLift = 0.5
TerrainMesh.edgeTop = 2.5

function TerrainMesh.triangulate(cells: { any }, step: number, exclude: { Instance }?, stats: any?): { { any } }
	local rp = RaycastParams.new()
	rp.FilterType = Enum.RaycastFilterType.Exclude
	local ex = { workspace.Terrain :: Instance }
	for _, x in ipairs(exclude or {}) do ex[#ex + 1] = x end
	rp.FilterDescendantsInstances = ex
	rp.RespectCanCollide = true
	local blockedCache: { [any]: { [any]: boolean } } = {}
	local nBlocked = 0
	local function open(a: any, b: any): boolean
		local t = blockedCache[a]
		if t and t[b] ~= nil then return not t[b] end
		local top = math.min(TerrainMesh.edgeTop, math.min(a.clearance or math.huge, b.clearance or math.huge) - 0.1)
		local blocked = false
		if top - TerrainMesh.edgeLift >= 0.2 then
			local h = top - TerrainMesh.edgeLift
			local mid = TerrainMesh.edgeLift + h * 0.5
			local pa, pb = a.pos + Vector3.yAxis * mid, b.pos + Vector3.yAxis * mid
			local d = pb - pa
			if d.Magnitude > 1e-3 then
				blocked = workspace:Blockcast(CFrame.new(pa), Vector3.new(0.2, h, 0.2), d, rp) ~= nil
			end
		end
		if blocked then nBlocked += 1 end
		blockedCache[a] = blockedCache[a] or {}; blockedCache[a][b] = blocked
		blockedCache[b] = blockedCache[b] or {}; blockedCache[b][a] = blocked
		return not blocked
	end
	local bins: { [string]: { any } } = {}
	for _, c in ipairs(cells) do
		local k = math.floor(c.pos.X / step) .. ":" .. math.floor(c.pos.Z / step)
		local b = bins[k]
		if not b then b = {}; bins[k] = b end
		b[#b + 1] = c
	end
	local function joins(a: any, b: any): boolean
		local mx, mz = (a.pos.X + b.pos.X) * 0.5, (a.pos.Z + b.pos.Z) * 0.5
		return math.abs(planeY(a, mx, mz) - planeY(b, mx, mz)) <= TerrainMesh.joinTol and open(a, b)
	end
	-- the node of a bin nearest the height A's plane gives there
	local function pick(from: any, bx: number, bz: number): any
		local list = bins[bx .. ":" .. bz]
		if not list then return nil end
		local y = planeY(from, (bx + 0.5) * step, (bz + 0.5) * step)
		local best, bd = nil, math.huge
		for _, c in ipairs(list) do
			local d = math.abs(c.pos.Y - y)
			if d < bd then best, bd = c, d end
		end
		return best
	end
	local tris = {}
	for _, A in ipairs(cells) do
		local bx, bz = math.floor(A.pos.X / step), math.floor(A.pos.Z / step)
		local B, D = pick(A, bx + 1, bz), pick(A, bx, bz + 1)
		local C = pick(A, bx + 1, bz + 1)
		local ab = B and joins(A, B)
		local ad = D and joins(A, D)
		local bc = B and C and joins(B, C)
		local dc = D and C and joins(D, C)
		local ac = C and joins(A, C)
		local bd = B and D and joins(B, D)
		if ab and ad and bc and dc and (ac or bd) then
			-- the shorter valid diagonal
			local useAC = ac and (not bd or (A.pos - C.pos).Magnitude <= (B.pos - D.pos).Magnitude)
			if useAC then
				tris[#tris + 1] = { A, C, B }; tris[#tris + 1] = { A, D, C }
			else
				tris[#tris + 1] = { A, D, B }; tris[#tris + 1] = { B, D, C }
			end
		else
			if ab and bc and ac then tris[#tris + 1] = { A, C, B } end
			if ad and dc and ac then tris[#tris + 1] = { A, D, C } end
			if ab and ad and bd and not (bc and dc) then tris[#tris + 1] = { A, D, B } end
			if bc and dc and bd and not (ab and ad) then tris[#tris + 1] = { B, D, C } end
		end
	end
	if stats then stats.edgesBlocked = nBlocked end
	return tris
end

-- ------------------------------------------------------------ 2. simplify
local function planeQ(n: Vector3, p: Vector3, w: number): { number }
	local a, b, c = n.X, n.Y, n.Z
	local d = -(a * p.X + b * p.Y + c * p.Z)
	return { a*a*w, a*b*w, a*c*w, a*d*w, b*b*w, b*c*w, b*d*w, c*c*w, c*d*w, d*d*w }
end
local function addQ(q: { number }, r: { number })
	for i = 1, 10 do q[i] += r[i] end
end
local function evalQ(q: { number }, p: Vector3): number
	local x, y, z = p.X, p.Y, p.Z
	return q[1]*x*x + 2*q[2]*x*y + 2*q[3]*x*z + 2*q[4]*x + q[5]*y*y + 2*q[6]*y*z + 2*q[7]*y
		+ q[8]*z*z + 2*q[9]*z + q[10]
end

-- Returns vertex positions, the node each vertex is, and the live triangles
-- (vertex ids, wound with the interior on the negative side as elsewhere in
-- NavG). Stats go in `stats`.
function TerrainMesh.simplify(src: { { any } }, stats: any): ({ Vector3 }, { any }, { { number } })
	local vid: { [any]: number } = {}
	local pos: { Vector3 } = {}
	local nodeOf: { any } = {}
	local function V(c: any): number
		local id = vid[c]
		if not id then id = #pos + 1; vid[c] = id; pos[id] = c.pos; nodeOf[id] = c end
		return id
	end
	local tris: { any } = {}
	for _, t in ipairs(src) do
		local a, b, c = V(t[1]), V(t[2]), V(t[3])
		local n = (pos[b] - pos[a]):Cross(pos[c] - pos[a])
		if n.Y < 0 then b, c = c, b end
		tris[#tris + 1] = { a, b, c, alive = true }
	end
	local nV = #pos
	local band = table.create(nV, 0)
	for i = 1, nV do band[i] = bandOf(nodeOf[i].clearance or math.huge) end
	stats.band = band

	local vtris: { { [number]: boolean } } = table.create(nV)
	for i = 1, nV do vtris[i] = {} end
	for ti, t in ipairs(tris) do for k = 1, 3 do vtris[t[k]][ti] = true end end

	local function triNormal(t: any): Vector3
		local a, b, c = pos[t[1]], pos[t[2]], pos[t[3]]
		return (b - a):Cross(c - a)
	end

	local Q: { { number } } = table.create(nV)
	for i = 1, nV do Q[i] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } end
	local edgeTris: { [string]: { number } } = {}
	local function ek(a: number, b: number): string
		if a < b then return a .. ":" .. b end
		return b .. ":" .. a
	end
	for ti, t in ipairs(tris) do
		local n = triNormal(t)
		local area = n.Magnitude
		if area > 1e-9 then
			local q = planeQ(n.Unit, pos[t[1]], area)
			for k = 1, 3 do addQ(Q[t[k]], q) end
		end
		for k = 1, 3 do
			local key = ek(t[k], t[k % 3 + 1])
			local e = edgeTris[key]
			if not e then e = {}; edgeTris[key] = e end
			e[#e + 1] = ti
		end
	end
	-- the rim gets a wall of error so the outline keeps its shape
	for key, ts in pairs(edgeTris) do
		if #ts == 1 then
			local t = tris[ts[1]]
			local sa, sb = key:match("(%d+):(%d+)")
			local a, b = tonumber(sa) :: number, tonumber(sb) :: number
			local n = triNormal(t)
			if n.Magnitude > 1e-9 then
				local e = pos[b] - pos[a]
				local side = e:Cross(n.Unit)
				if side.Magnitude > 1e-9 then
					local q = planeQ(side.Unit, pos[a], TerrainMesh.boundW * e.Magnitude)
					addQ(Q[a], q); addQ(Q[b], q)
				end
			end
		end
	end

	-- min-heap on cost
	local heap: { any }, hn = {}, 0
	local function push(item: any)
		hn += 1; heap[hn] = item
		local i = hn
		while i > 1 do
			local p = i // 2
			if heap[p].cost <= heap[i].cost then break end
			heap[p], heap[i] = heap[i], heap[p]; i = p
		end
	end
	local function pop(): any
		if hn == 0 then return nil end
		local top = heap[1]
		heap[1] = heap[hn]; heap[hn] = nil; hn -= 1
		local i = 1
		while true do
			local l, r = 2 * i, 2 * i + 1
			local m = i
			if l <= hn and heap[l].cost < heap[m].cost then m = l end
			if r <= hn and heap[r].cost < heap[m].cost then m = r end
			if m == i then break end
			heap[m], heap[i] = heap[i], heap[m]; i = m
		end
		return top
	end

	local stamp = table.create(nV, 0)
	local alive = table.create(nV, true)
	-- collapse u into v: v's position under both quadrics
	local function costOf(u: number, v: number): number
		local q = table.clone(Q[u]); addQ(q, Q[v])
		return evalQ(q, pos[v])
	end
	local function schedule(u: number)
		local seenN = {}
		for ti in pairs(vtris[u]) do
			local t = tris[ti]
			for k = 1, 3 do
				local w = t[k]
				if w ~= u and not seenN[w] then
					seenN[w] = true
					push({ cost = costOf(u, w), u = u, v = w, su = stamp[u], sv = stamp[w] })
					push({ cost = costOf(w, u), u = w, v = u, su = stamp[w], sv = stamp[u] })
				end
			end
		end
	end
	for i = 1, nV do schedule(i) end

	local function isRimVert(u: number): boolean
		for ti in pairs(vtris[u]) do
			local t = tris[ti]
			for k = 1, 3 do
				local a, b = t[k], t[k % 3 + 1]
				if a == u or b == u then
					local cnt = 0
					for _, x in ipairs(edgeTris[ek(a, b)] or {}) do if tris[x].alive then cnt += 1 end end
					if cnt == 1 then return true end
				end
			end
		end
		return false
	end

	-- the rim nodes each rim edge stands for, between its two ends
	local rimPts: { [string]: { Vector3 } } = {}
	local function rimEdgesOf(u: number): { number }
		local out = {}
		for ti in pairs(vtris[u]) do
			local t = tris[ti]
			for k = 1, 3 do
				local a, b = t[k], t[k % 3 + 1]
				local w = (a == u) and b or ((b == u) and a or nil)
				if w and not table.find(out, w) then
					local cnt = 0
					for _, x in ipairs(edgeTris[ek(u, w)] or {}) do if tris[x].alive then cnt += 1 end end
					if cnt == 1 then out[#out + 1] = w end
				end
			end
		end
		return out
	end
	local function planDist(p: Vector3, a: Vector3, b: Vector3): number
		local dx, dz = b.X - a.X, b.Z - a.Z
		local L2 = dx * dx + dz * dz
		local t = L2 > 1e-12 and math.clamp(((p.X - a.X) * dx + (p.Z - a.Z) * dz) / L2, 0, 1) or 0
		local ex, ez = p.X - (a.X + dx * t), p.Z - (a.Z + dz * t)
		return math.sqrt(ex * ex + ez * ez)
	end
	-- nil when u -> v would move the rim more than rimTol; else the new rim
	-- edge's key and the nodes it stands for
	local function rimMove(u: number, v: number): (string?, { Vector3 }?)
		local ns = rimEdgesOf(u)
		if #ns ~= 2 or not table.find(ns, v) then return nil, nil end
		local w = (ns[1] == v) and ns[2] or ns[1]
		local pts = {}
		for _, q in ipairs(rimPts[ek(w, u)] or {}) do pts[#pts + 1] = q end
		pts[#pts + 1] = pos[u]
		for _, q in ipairs(rimPts[ek(u, v)] or {}) do pts[#pts + 1] = q end
		for _, q in ipairs(pts) do
			if planDist(q, pos[w], pos[v]) > TerrainMesh.rimTol then return nil, nil end
		end
		return ek(w, v), pts
	end

	local maxCost = TerrainMesh.simplifyTol * TerrainMesh.simplifyTol
	local collapses = 0
	while true do
		local it = pop()
		if not it or it.cost > maxCost then break end
		local u, v = it.u, it.v
		if alive[u] and alive[v] and stamp[u] == it.su and stamp[v] == it.sv then
			local shared, onlyU = {}, {}
			for ti in pairs(vtris[u]) do
				local t = tris[ti]
				if t[1] == v or t[2] == v or t[3] == v then shared[#shared + 1] = ti else onlyU[#onlyU + 1] = ti end
			end
			if #shared > 0 and band[u] == band[v] then
				-- link condition: the common neighbours are exactly the shared
				-- triangles' third vertices, or the collapse pinches the surface
				local nu, nv = {}, {}
				for ti in pairs(vtris[u]) do for k = 1, 3 do nu[tris[ti][k]] = true end end
				for ti in pairs(vtris[v]) do for k = 1, 3 do nv[tris[ti][k]] = true end end
				local common = 0
				for w in pairs(nu) do if w ~= u and w ~= v and nv[w] then common += 1 end end
				local ok = common == #shared
				-- a rim vertex only slides along the rim, and only as far as rimTol
				local rimKey, rimList = nil, nil
				if ok and isRimVert(u) then
					if not isRimVert(v) then
						ok = false
					else
						rimKey, rimList = rimMove(u, v)
						if not rimKey then ok = false end
					end
				end
				-- no folded, degenerate or stood-up triangle
				if ok then
					for _, ti in ipairs(onlyU) do
						local t = tris[ti]
						local before = triNormal(t)
						local p1 = (t[1] == u) and pos[v] or pos[t[1]]
						local p2 = (t[2] == u) and pos[v] or pos[t[2]]
						local p3 = (t[3] == u) and pos[v] or pos[t[3]]
						local after = (p2 - p1):Cross(p3 - p1)
						if after.Magnitude < 1e-6 or before.Magnitude < 1e-9
							or after.Unit:Dot(before.Unit) < 0.5 or after.Unit.Y < 0.05 then
							ok = false
							break
						end
					end
				end
				if ok then
					for _, ti in ipairs(shared) do
						local t = tris[ti]
						t.alive = false
						for k = 1, 3 do vtris[t[k]][ti] = nil end
					end
					for _, ti in ipairs(onlyU) do
						local t = tris[ti]
						for k = 1, 3 do if t[k] == u then t[k] = v end end
						vtris[v][ti] = true
						for k = 1, 3 do
							local key = ek(t[k], t[k % 3 + 1])
							local e = edgeTris[key]
							if not e then e = {}; edgeTris[key] = e end
							if not table.find(e, ti) then e[#e + 1] = ti end
						end
					end
					if rimKey then rimPts[rimKey] = rimList end
					vtris[u] = {}
					alive[u] = false
					addQ(Q[v], Q[u])
					stamp[v] += 1
					collapses += 1
					schedule(v)
				end
			end
		end
	end

	local live = {}
	for _, t in ipairs(tris) do
		if t.alive then live[#live + 1] = { t[1], t[2], t[3] } end
	end
	stats.triangles = #src
	stats.collapses = collapses
	stats.simplified = #live
	return pos, nodeOf, live
end

-- ------------------------------------------------------------ 3. merge
-- CONVEX BOTH WAYS: from above (what the pathfinder's in-polygon tests use) and
-- in the polygon's own plane. From above alone let 24 of 612 through that
-- turned back up to 11.5 degrees on their slope.
function TerrainMesh.merge(pos: { Vector3 }, tris: { { number } }, stats: any, band: { number }?): { any }
	local cosAngle = math.cos(math.rad(TerrainMesh.mergeAngle))
	local polys = {}
	for _, t in ipairs(tris) do
		local n = (pos[t[2]] - pos[t[1]]):Cross(pos[t[3]] - pos[t[1]])
		if n.Magnitude > 1e-9 then
			local b = band and math.min(band[t[1]], band[t[2]], band[t[3]]) or 0
			polys[#polys + 1] = { verts = { t[1], t[2], t[3] }, n = n.Unit, band = b, area = n.Magnitude * 0.5 }
		end
	end
	local function convex(vs: { number }, nrm: Vector3): boolean
		local n = #vs
		for k = 1, n do
			local a, b, c = pos[vs[k]], pos[vs[k % n + 1]], pos[vs[(k + 1) % n + 1]]
			local cr = (b.X - a.X) * (c.Z - b.Z) - (b.Z - a.Z) * (c.X - b.X)
			if cr > 1e-6 then return false end
			if (b - a):Cross(c - b):Dot(nrm) < -1e-6 then return false end
		end
		return true
	end
	-- every vertex within `tol` of one plane of normal n: measured about their
	-- mean offset, so the anchor vertex is not assumed to lie on it
	local function planar(vs: { number }, n: Vector3, anchor: Vector3, tol: number): boolean
		local lo, hi = math.huge, -math.huge
		for _, id in ipairs(vs) do
			local d = (pos[id] - anchor):Dot(n)
			if d < lo then lo = d end
			if d > hi then hi = d end
		end
		return (hi - lo) * 0.5 <= tol
	end
	local merged = 0
	local changed = true
	while changed do
		changed = false
		local owner: { [string]: any } = {}
		for _, p in ipairs(polys) do
			if not p.dead then
				local vs = p.verts
				for k = 1, #vs do owner[vs[k] .. ">" .. vs[k % #vs + 1]] = p end
			end
		end
		-- a polygon changed this pass is left alone until `owner` is rebuilt
		local touched = {}
		for _, p in ipairs(polys) do
			if p.dead or touched[p] then continue end
			local pv = p.verts
			for k = 1, #pv do
				local a, b = pv[k], pv[k % #pv + 1]
				local q = owner[b .. ">" .. a]
				if q and q ~= p and not q.dead and not touched[q] and p.band == q.band then
					-- splice q into p along a->b (q has b->a)
					local qv = q.verts
					local out = {}
					local i = k % #pv + 1 -- b
					repeat out[#out + 1] = pv[i]; i = i % #pv + 1 until pv[i] == b
					local j = (table.find(qv, a) :: number) % #qv + 1
					while qv[j] ~= b do out[#out + 1] = qv[j]; j = j % #qv + 1 end
					-- the plane weighted by area, so a sliver cannot tilt its neighbour
					local nn = (p.n * p.area + q.n * q.area).Unit
					local ok = convex(out, nn)
					if ok then
						if p.n:Dot(q.n) >= cosAngle then
							ok = planar(out, nn, pos[out[1]], TerrainMesh.mergeDev)
						else
							ok = planar(out, nn, pos[out[1]], TerrainMesh.bumpTol)
						end
					end
					if ok then
						p.verts = out; p.n = nn; p.area += q.area; q.dead = true
						touched[p] = true; touched[q] = true
						merged += 1; changed = true
						break
					end
				end
			end
		end
	end
	local out = {}
	for _, p in ipairs(polys) do if not p.dead then out[#out + 1] = p end end
	stats.merges = merged
	-- A STRAIGHT RIM IS ONE EDGE. A vertex no other polygon uses, in line with
	-- its neighbours, is dropped: headroom bands along an eave left a straight
	-- wall's rim in 2 stud pieces. Only unshared vertices, so every shared edge
	-- still matches exactly on both sides.
	local uses: { [number]: number } = {}
	for _, p in ipairs(out) do for _, id in ipairs(p.verts) do uses[id] = (uses[id] or 0) + 1 end end
	local dropped = 0
	for _, p in ipairs(out) do
		local vs = p.verts
		local k = 1
		while #vs > 3 and k <= #vs do
			local n = #vs
			local a, b, c = pos[vs[(k - 2) % n + 1]], pos[vs[k]], pos[vs[k % n + 1]]
			local ac = c - a
			local t = ac:Dot(ac) > 1e-9 and math.clamp((b - a):Dot(ac) / ac:Dot(ac), 0, 1) or 0
			if uses[vs[k]] == 1 and (a + ac * t - b).Magnitude <= 0.05 then
				table.remove(vs, k)
				dropped += 1
			else
				k += 1
			end
		end
	end
	stats.collinearDropped = dropped
	return out
end

-- ------------------------------------------------------------ build
-- Meshes every terrain grid in `data`, appends the polygons to `mesh.tris`
-- (the CDT's record shape) and re-labels the terrain cells into their
-- polygons' regions. `data.terrainPolyOf` is the claim, cell -> polygon index.
function TerrainMesh.build(data: any, mesh: any): any
	local t0 = os.clock()
	local stats: any = { cells = 0, polys = 0, regions = 0, claimed = 0, nearby = 0, unclaimed = 0 }
	local cells = {}
	local step = 2
	for _, g in ipairs(data.grids) do
		if g.terrain then
			for _, c in ipairs(g.cells) do
				if c.region then cells[#cells + 1] = c; step = c.su or step end
			end
		end
	end
	stats.cells = #cells
	mesh.terrain = stats
	if #cells == 0 then return stats end

	local ex = {}
	for _, n in ipairs({ "NVGN_Debug", "NVGN_Path_Normal", "NVGN_Path_Wide", "NVGN_Clips_Normal", "NVGN_Clips_Wide",
		"PathStart_Normal", "PathStart_Wide", "PathEnd_case3", "NVGN_Gaps" }) do
		local x = workspace:FindFirstChild(n)
		if x then ex[#ex + 1] = x end
	end
	local tris = TerrainMesh.triangulate(cells, step, ex, stats)
	local pos, nodeOf, live = TerrainMesh.simplify(tris, stats)
	local polys = TerrainMesh.merge(pos, live, stats, stats.band)
	stats.band = nil

	-- regions: one per mesh-connected piece, numbered past every region in use
	local maxRegion = 0
	for _, g in ipairs(data.grids) do
		for _, c in ipairs(g.cells) do
			if c.region and c.region > maxRegion then maxRegion = c.region end
		end
	end
	for _, f in ipairs(mesh.tris) do
		if f.region > maxRegion then maxRegion = f.region end
	end
	local parent = {}
	local function find(i: number): number
		while parent[i] and parent[i] ~= i do
			parent[i] = parent[parent[i]] or parent[i]
			i = parent[i]
		end
		return i
	end
	local edgeOwner: { [string]: number } = {}
	for i, p in ipairs(polys) do
		parent[i] = i
		local vs = p.verts
		for k = 1, #vs do
			local a, b = vs[k], vs[k % #vs + 1]
			local other = edgeOwner[b .. ">" .. a]
			if other then
				local ra, rb = find(i), find(other)
				if ra ~= rb then parent[math.max(ra, rb)] = math.min(ra, rb) end
			end
			edgeOwner[a .. ">" .. b] = i
		end
	end
	local regionOf: { [number]: number } = {}
	local first = #mesh.tris + 1
	for i, p in ipairs(polys) do
		local root = find(i)
		local r = regionOf[root]
		if not r then
			maxRegion += 1
			r = maxRegion
			regionOf[root] = r
			stats.regions += 1
			data.boundary[r] = { faces = {}, ids = {}, loops = {}, terrain = true }
		end
		local verts = table.create(#p.verts)
		for k, id in ipairs(p.verts) do verts[k] = pos[id] end
		local area, acc = 0, Vector3.zero
		for k = 2, #verts - 1 do
			local a = (verts[k] - verts[1]):Cross(verts[k + 1] - verts[1]).Magnitude * 0.5
			area += a
			acc += (verts[1] + verts[k] + verts[k + 1]) / 3 * a
		end
		local up = p.n.Y < 0 and -p.n or p.n
		mesh.tris[#mesh.tris + 1] = {
			verts = verts, n = #verts, region = r, up = up,
			area = area, centre = area > 1e-9 and acc / area or verts[1], minAngle = 0,
			a = verts[1], b = verts[2], c = verts[3], terrain = true,
		}
	end
	local last = #mesh.tris
	stats.polys = last - first + 1
	mesh.terrainFirst, mesh.terrainLast = first, last

	-- CLAIM: every terrain cell into the polygon over it, from above; the one
	-- whose plane is nearest the cell's height, within claimDrop. A cell the
	-- simplifier left just outside the rim goes to the nearest rim within a node.
	local B = 4
	local buck: { [string]: { number } } = {}
	for i = first, last do
		local f = mesh.tris[i]
		local lo, hi = f.verts[1], f.verts[1]
		for _, v in ipairs(f.verts) do lo = lo:Min(v); hi = hi:Max(v) end
		for x = math.floor(lo.X / B), math.floor(hi.X / B) do
			for z = math.floor(lo.Z / B), math.floor(hi.Z / B) do
				local k = x .. ":" .. z
				local b = buck[k]
				if not b then b = {}; buck[k] = b end
				b[#b + 1] = i
			end
		end
	end
	local function planDist(f: any, p: Vector3): number
		local vs = f.verts
		local n = #vs
		local inside, best = true, math.huge
		for k = 1, n do
			local a, b = vs[k], vs[k % n + 1]
			local dx, dz = b.X - a.X, b.Z - a.Z
			if dx * (p.Z - a.Z) - dz * (p.X - a.X) > 1e-4 then inside = false end
			local L2 = dx * dx + dz * dz
			local t = L2 > 1e-12 and math.clamp(((p.X - a.X) * dx + (p.Z - a.Z) * dz) / L2, 0, 1) or 0
			local ex, ez = p.X - (a.X + dx * t), p.Z - (a.Z + dz * t)
			best = math.min(best, math.sqrt(ex * ex + ez * ez))
		end
		return inside and 0 or best
	end
	local polyOf: { [any]: number } = {}
	local orphan = nil
	for _, c in ipairs(cells) do
		local p = c.pos
		local hit, hd, hp = nil, math.huge, math.huge
		for _, i in ipairs(buck[math.floor(p.X / B) .. ":" .. math.floor(p.Z / B)] or {}) do
			local f = mesh.tris[i]
			local dp = planDist(f, p)
			if dp <= step then
				local dh = math.abs(heightAt(f, p) - p.Y)
				if dh <= TerrainMesh.claimDrop and (dp < hp or (dp == hp and dh < hd)) then
					hit, hd, hp = i, dh, dp
				end
			end
		end
		if hit then
			polyOf[c] = hit
			c.region = mesh.tris[hit].region
			if hp == 0 then stats.claimed += 1 else stats.nearby += 1 end
		else
			-- untraced floor: GridPortals may still bridge across it
			if not orphan then maxRegion += 1; orphan = maxRegion end
			c.region = orphan
			stats.unclaimed += 1
		end
	end
	data.terrainPolyOf = polyOf
	stats.seconds = os.clock() - t0
	return stats
end

function TerrainMesh.report(mesh: any): string
	local s = mesh.terrain
	if not s or s.cells == 0 then return "terrain   none" end
	return ("terrain   %d nodes -> %d tris -> %d after %d collapses -> %d polys (%d merges), %d regions; cells %d inside, %d nearby, %d unclaimed; %d edges through parts refused, %d in-line rim vertices dropped (%.2fs)")
		:format(s.cells, s.triangles, s.simplified, s.collapses, s.polys, s.merges, s.regions,
			s.claimed, s.nearby, s.unclaimed, s.edgesBlocked or 0, s.collinearDropped or 0, s.seconds)
end

return TerrainMesh
