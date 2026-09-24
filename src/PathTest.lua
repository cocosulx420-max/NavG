--!strict
-- NVGN.PathTest -- a throwaway pathfinder for LOOKING AT the portals.
--
-- Not the runtime. It exists so a portal can be judged by what a path through
-- it does rather than by how its bar looks: A* over polygons (the polygons are
-- the nodes, the portals are the edges), then a funnel pulled through the
-- chain of portals, which is how an NPC walks a navmesh.
--
-- An edge-matched seam or bridge has TWO sides -- the stretch of A's edge and
-- the matching stretch of B's -- and the funnel is fed both, so a path steps
-- across the gap between two regions instead of cutting a corner over it.
--
-- `PathTest.watch(result)` keeps a path drawn between two parts named
-- PathStart and PathEnd, re-solving whenever either moves.

local PathTest = {}

PathTest.lift = 0.6         -- studs the drawn path floats above the floor
PathTest.snapHeight = 4.0   -- how far below a marker a polygon may be and still hold it
PathTest.snapReach = 3.0    -- studs off the mesh a point may be and still find its polygon
PathTest.folderName = "NVGN_Path"
PathTest.snapDraw = 1.5    -- studs a drawn sample may move to sit on a polygon
PathTest.barTol = 0.75     -- studs off the final route a portal may be and still pin the rubber band

local function flat(v: Vector3): Vector3 return Vector3.new(v.X, 0, v.Z) end

-- the polygon a point stands on: inside it in plan, and the nearest one below
local function locate(mesh: any, p: Vector3): (number?, number)
	local best, bestDy = nil, math.huge
	for i, f in ipairs(mesh.tris) do
		local v = f.verts
		local n = #v
		local inside = true
		for k = 1, n do
			local a, b = v[k], v[k % n + 1]
			local cross = (b.X - a.X) * (p.Z - a.Z) - (b.Z - a.Z) * (p.X - a.X)
			-- polygons are wound counter-clockwise about +Y seen from above, which in
			-- Roblox's X/Z plane makes the interior the NEGATIVE side of each edge
			if cross > 1e-6 then inside = false; break end
		end
		if inside then
			-- height of the polygon's plane under p
			local c, up = f.centre, f.up
			local h = c.Y
			if math.abs(up.Y) > 1e-3 then
				h = c.Y - ((p.X - c.X) * up.X + (p.Z - c.Z) * up.Z) / up.Y
			end
			local dy = p.Y - h
			if dy > -1.0 and dy < PathTest.snapHeight and dy < bestDy then
				best, bestDy = i, dy
			end
		end
	end
	if best then return best, bestDy end
	-- NOT ON THE MESH, then the NEAREST polygon within reach, the way Detour's
	-- findNearestPoly does. The mesh stops short of walls and rims, so a walking
	-- NPC keeps stepping off it; without this it loses its place and stops.
	local bestD = PathTest.snapReach
	for i, f in ipairs(mesh.tris) do
		local v = f.verts
		local n = #v
		for k = 1, n do
			local a, b = v[k], v[k % n + 1]
			local d = Vector3.new(b.X - a.X, 0, b.Z - a.Z)
			local dd = d:Dot(d)
			local q = Vector3.new(p.X - a.X, 0, p.Z - a.Z)
			local t = dd > 1e-9 and math.clamp(q:Dot(d) / dd, 0, 1) or 0
			local dist = (q - d * t).Magnitude
			if dist < bestD then
				local h = a.Y + (b.Y - a.Y) * t
				local dy = p.Y - h
				if dy > -1.0 and dy < PathTest.snapHeight then best, bestD, bestDy = i, dist, dy end
			end
		end
	end
	return best, (best and bestDy or 0)
end

-- adjacency: every link both ways, with the portal's two sides
local function graph(res: any): { [number]: { any } }
	local adj: { [number]: { any } } = {}
	local function add(from: number, to: number, L: any, reverse: boolean)
		local t = adj[from]
		if not t then t = {}; adj[from] = t end
		t[#t + 1] = { to = to, L = L, reverse = reverse }
	end
	for _, L in ipairs(res.links) do
		add(L.a, L.b, L, false)
		-- a drop or a jump goes one way only
		if not L.oneWay then add(L.b, L.a, L, true) end
	end
	return adj
end

-- CAN THIS PROFILE USE IT. The bake stores measurements (see Agents and
-- Pipeline.measure_); here each NPC profile decides. `banned` is the stuck
-- fallback: polygons or links a follower gave up on, for a while.
local function usable(mesh: any, e: any, prof: any?, banned: any?): boolean
	if banned and (banned[e.L] or banned[e.to]) then return false end
	if not prof then return true end
	local L, T = e.L, mesh.tris[e.to]
	local w = 2 * (prof.radius or 0)
	-- what most of the polygon has (Pipeline.measure_): a tall body may bump a
	-- low strip at its edge and the caller's stuck fallback routes round it
	local head = T.headroomOpen or T.headroom
	if head and head < (prof.prone or 0) then return false end
	-- width by the PORTAL'S clearance (see Pipeline.measure_); a polygon's own
	-- width misleads, a thin triangle along a wall can sit in a wide room
	if L.clear and L.clear < w then return false end
	local rise = L.rise or 0
	if e.reverse then rise = -rise end
	if L.kind == "drop" then
		-- over a rail first: the vault is a jump
		return -rise <= (prof.drop or math.huge) and (L.vault or 0) <= (prof.jump or 0)
	elseif L.kind == "jump" then
		return rise <= (prof.jump or 0) and (L.gap or 0) <= (prof.jumpDistance or 0)
	end
	return math.abs(rise) <= (prof.step or math.huge)
end

-- extra cost for moving somewhere cramped, so an upright route wins when there is one
local function postureCost(T: any, prof: any?): number
	local head = T.headroomOpen or T.headroom
	if not prof or not head then return 1 end
	if head < (prof.crouch or 0) then return 3 end
	if head < (prof.height or 0) then return 1.6 end
	return 1
end

-- WHERE A CROSSING IS SCORED. Not at the gate's centre: a wide gate's centre
-- can sit tens of studs off the way through, and a pinhole gate onto a sliver
-- then looks cheaper than the wide one next to it. Each gate is crossed at the
-- point that makes from -> gate -> goal shortest (sampled along it), and a
-- two-sided gate is crossed bar by bar, so the cost is the length walked.
PathTest.gateSamples = 8
local function bestOn(a: Vector3, b: Vector3, from: Vector3, goal: Vector3): Vector3
	local best, bd = a, math.huge
	for k = 0, PathTest.gateSamples do
		local x = a:Lerp(b, k / PathTest.gateSamples)
		local d = (x - from).Magnitude + (goal - x).Magnitude
		if d < bd then best, bd = x, d end
	end
	return best
end

local function astar(mesh: any, adj: any, s: number, g: number, sp: Vector3, gp: Vector3,
	prof: any?, banned: any?)
	local open, came, gs = { s }, {}, { [s] = 0 }
	local pos = { [s] = sp }
	local fs = { [s] = (gp - sp).Magnitude }
	local closed = {}
	while #open > 0 do
		local bi = 1
		for i = 2, #open do if fs[open[i]] < fs[open[bi]] then bi = i end end
		local cur = table.remove(open, bi)
		if cur == g then
			local chain = {}
			local c = cur
			while came[c] do table.insert(chain, 1, came[c]); c = came[c].from end
			return chain
		end
		closed[cur] = true
		for _, e in ipairs(adj[cur] or {}) do
			if not closed[e.to] and usable(mesh, e, prof, banned) then
				local L = e.L
				local first, second = { L.left, L.right }, (L.bLeft and L.bRight) and { L.bLeft, L.bRight } or nil
				if second and e.reverse then first, second = second, first end
				local x1 = bestOn(first[1], first[2], pos[cur], gp)
				local via, walked = x1, (x1 - pos[cur]).Magnitude
				if second then
					via = bestOn(second[1], second[2], x1, gp)
					walked += (via - x1).Magnitude
				end
				local leap = (L.kind == "jump" or L.kind == "drop") and 2 or 0
				local cost = gs[cur] + walked * postureCost(mesh.tris[e.to], prof) + leap
				if gs[e.to] == nil or cost < gs[e.to] then
					gs[e.to] = cost
					pos[e.to] = via
					fs[e.to] = cost + (gp - via).Magnitude
					came[e.to] = { from = cur, e = e }
					if not table.find(open, e.to) then open[#open + 1] = e.to end
				end
			end
		end
	end
	return nil
end

-- the portals along a chain, each as {left, right} in travel order. A two-sided
-- portal contributes both of its sides.
local function portalsOf(mesh: any, chain: { any }): { { Vector3 } }
	local out = {}
	for _, step in ipairs(chain) do
		local L, from, to = step.e.L, step.from, step.e.to
		local sides = { { L.left, L.right } }
		if L.bLeft and L.bRight then
			sides[2] = { L.bLeft, L.bRight }
			if step.e.reverse then sides = { sides[2], sides[1] } end
		end
		local dir = flat(mesh.tris[to].centre - mesh.tris[from].centre)
		local leftVec = Vector3.yAxis:Cross(dir) -- left of travel
		for _, sd in ipairs(sides) do
			local p, q = sd[1], sd[2]
			if flat(p - q):Dot(leftVec) >= 0 then out[#out + 1] = { p, q } else out[#out + 1] = { q, p } end
		end
	end
	return out
end

-- simple stupid funnel, in plan; heights ride along on the chosen points
local function funnel(sp: Vector3, gp: Vector3, portals: { { Vector3 } }): { Vector3 }
	local P = { { sp, sp } }
	for _, pt in ipairs(portals) do P[#P + 1] = pt end
	P[#P + 1] = { gp, gp }
	local function area2(a: Vector3, b: Vector3, c: Vector3): number
		-- positive when c is to the LEFT of a->b, Y up (left of +X is -Z)
		return (c.X - a.X) * (b.Z - a.Z) - (b.X - a.X) * (c.Z - a.Z)
	end
	local path = { sp }
	local apex, left, right = sp, P[1][1], P[1][2]
	local ai, li, ri = 1, 1, 1
	local i = 2
	local guard = 0
	while i <= #P and guard < 10000 do
		guard += 1
		local pl, pr = P[i][1], P[i][2]
		-- tighten right: the new right point is inside if it is left of apex->right,
		-- and it may not cross over to the left of apex->left
		if area2(apex, right, pr) >= 0 then
			if apex == right or area2(apex, left, pr) < 0 then
				right = pr; ri = i
			else
				path[#path + 1] = left
				apex = left; ai = li
				left, right = apex, apex; li, ri = ai, ai
				i = ai + 1
				continue
			end
		end
		-- tighten left, mirrored
		if area2(apex, left, pl) <= 0 then
			if apex == left or area2(apex, right, pl) > 0 then
				left = pl; li = i
			else
				path[#path + 1] = right
				apex = right; ai = ri
				left, right = apex, apex; li, ri = ai, ai
				i = ai + 1
				continue
			end
		end
		i += 1
	end
	if path[#path] ~= gp then path[#path + 1] = gp end
	return path
end

local function segment(a: Vector3, b: Vector3, colour: Color3, thick: number, parent: Instance)
	local d = b - a
	if d.Magnitude < 1e-3 then return end
	local p = Instance.new("Part")
	p.Anchored = true; p.CanCollide = false; p.CanQuery = false; p.CanTouch = false
	p.Material = Enum.Material.Neon
	p.Color = colour
	p.Size = Vector3.new(thick, thick, d.Magnitude)
	p.CFrame = CFrame.lookAt((a + b) * 0.5, b)
	p.Parent = parent
end

-- STRAIGHTENING ACROSS THE MESH. The funnel is the shortest line through the
-- polygons A* picked, and no shorter: where a better polygon sat next to the
-- chosen one (a wide gate beside a sliver's pinhole) the line still bends
-- through the pinhole. So each stretch is tried as a straight line walked
-- across the mesh itself, polygon to polygon through WALK links only -- the
-- way Detour's raycast smooths a path -- and kept where it holds. Drops and
-- jumps are never shortcut through.
local WALK = { shared = true, seam = true, bridge = true }
local function inPoly(f: any, p: Vector3, tol: number): boolean
	local v = f.verts
	local n = #v
	for k = 1, n do
		local a, b = v[k], v[k % n + 1]
		local cross = (b.X - a.X) * (p.Z - a.Z) - (b.Z - a.Z) * (p.X - a.X)
		local len = math.sqrt((b.X - a.X) ^ 2 + (b.Z - a.Z) ^ 2)
		if cross > tol * math.max(len, 1e-6) then return false end
	end
	return true
end
local function heightOn(f: any, p: Vector3): number
	local c, up = f.centre, f.up
	if math.abs(up.Y) < 1e-3 then return c.Y end
	return c.Y - ((p.X - c.X) * up.X + (p.Z - c.Z) * up.Z) / up.Y
end
-- where segment a->b (plan) crosses segment u->w, as params along each
local function cross2(a: Vector3, b: Vector3, u: Vector3, w: Vector3): (number?, number?)
	local dx, dz = b.X - a.X, b.Z - a.Z
	local ex, ez = w.X - u.X, w.Z - u.Z
	local den = dx * ez - dz * ex
	if math.abs(den) < 1e-9 then return nil, nil end
	local s = ((u.X - a.X) * ez - (u.Z - a.Z) * ex) / den
	local r = ((u.X - a.X) * dz - (u.Z - a.Z) * dx) / den
	return s, r
end
local function walkRay(mesh: any, adj: any, from: number, a: Vector3, b: Vector3, targets: { [number]: boolean },
	prof: any?, banned: any?, visited: { number }, crossed: { any }?): boolean
	local cur, t = from, 0
	for _ = 1, 512 do
		if targets[cur] then return true end
		local v = mesh.tris[cur].verts
		local n = #v
		-- the edge the ray leaves through: first crossing past t, going outward
		local tExit = math.huge
		for k = 1, n do
			local u, w = v[k], v[k % n + 1]
			local sR, rE = cross2(a, b, u, w)
			if sR and sR > t + 1e-6 and rE >= -1e-4 and rE <= 1 + 1e-4 then
				-- outward: the ray heads to the edge's positive side, which is outside
				-- (interior is the negative side, as in locate)
				local outward = ((w.X - u.X) * (b.Z - a.Z) - (w.Z - u.Z) * (b.X - a.X)) > 0
				if outward and sR < tExit then tExit = sR end
			end
		end
		if tExit >= 1 then return false end -- ends in a polygon that is not the target
		local x = a:Lerp(b, tExit)
		local nextPoly, nextT, nextLink = nil, nil, nil
		for _, e in ipairs(adj[cur] or {}) do
			local L = e.L
			if WALK[L.kind] and usable(mesh, e, prof, banned) then
				local near1, near2 = L.left, L.right
				local far1, far2 = L.bLeft, L.bRight
				if e.reverse and far1 and far2 then near1, near2, far1, far2 = far1, far2, near1, near2 end
				local sN, rN = cross2(a, b, near1, near2)
				if sN and math.abs(sN - tExit) < 0.02 and rN >= -0.02 and rN <= 1.02 then
					if far1 and far2 then
						local sF, rF = cross2(a, b, far1, far2)
						if sF and sF >= tExit - 1e-3 and sF < 1 and rF >= -0.02 and rF <= 1.02 then
							nextPoly, nextT = e.to, sF
						end
					else
						nextPoly, nextT = e.to, tExit
					end
					if nextPoly then nextLink = L; break end
				end
			end
		end
		if not nextPoly then return false end
		visited[#visited + 1] = nextPoly
		if crossed then crossed[#crossed + 1] = nextLink end
		cur, t = nextPoly, nextT :: number
	end
	return false
end
local function straighten(mesh: any, adj: any, pts: { Vector3 }, corridor: { number }, prof: any?, banned: any?): ({ Vector3 }, { number }, { any })
	if #pts <= 2 then return pts, {}, {} end
	-- which corridor polygons hold each point
	local holds = {}
	for k, p in ipairs(pts) do
		local t = {}
		for _, i in ipairs(corridor) do
			local f = mesh.tris[i]
			if inPoly(f, p, 0.05) and math.abs(heightOn(f, p) - p.Y) < 1.0 then t[i] = true end
		end
		holds[k] = t
	end
	local out, seen, crossedAll = { pts[1] }, {}, {}
	local i = 1
	while i < #pts do
		local jBest = i + 1
		for j = #pts, i + 2, -1 do
			local ok = false
			local visited, crossed = {}, {}
			for start in pairs(holds[i]) do
				table.clear(visited); table.clear(crossed)
				if walkRay(mesh, adj, start, pts[i], pts[j], holds[j], prof, banned, visited, crossed) then ok = true break end
			end
			if ok then
				jBest = j
				for _, pv in ipairs(visited) do seen[pv] = true end
				for _, L in ipairs(crossed) do crossedAll[#crossedAll + 1] = L end
				break
			end
		end
		out[#out + 1] = pts[jBest]
		i = jBest
	end
	local extra = {}
	for pv in pairs(seen) do extra[#extra + 1] = pv end
	return out, extra, crossedAll
end

-- the point of polygon f nearest p: the plane under p when p is over it in
-- plan, else the nearest point of its rim
local function closestOn(f: any, p: Vector3): Vector3
	local v = f.verts
	local n = #v
	local inside = true
	for k = 1, n do
		local a, b = v[k], v[k % n + 1]
		if (b.X - a.X) * (p.Z - a.Z) - (b.Z - a.Z) * (p.X - a.X) > 1e-6 then inside = false; break end
	end
	if inside then
		local c, up = f.centre, f.up or Vector3.yAxis
		local h = c.Y
		if math.abs(up.Y) > 1e-3 then h = c.Y - ((p.X - c.X) * up.X + (p.Z - c.Z) * up.Z) / up.Y end
		return Vector3.new(p.X, h, p.Z)
	end
	local best, bd = v[1], math.huge
	for k = 1, n do
		local a, b = v[k], v[k % n + 1]
		local d = b - a
		local dd = d:Dot(d)
		local t = dd > 1e-9 and math.clamp((p - a):Dot(d) / dd, 0, 1) or 0
		local q = a + d * t
		local dq = (q - p).Magnitude
		if dq < bd then best, bd = q, dq end
	end
	return best
end

-- every polygon this profile can reach from s
local function reachable(mesh: any, adj: any, s: number, prof: any?, banned: any?): { number }
	local seen, out, q = { [s] = true }, { s }, { s }
	local h = 1
	while h <= #q do
		local cur = q[h]; h += 1
		for _, e in ipairs(adj[cur] or {}) do
			if not seen[e.to] and usable(mesh, e, prof, banned) then
				seen[e.to] = true
				out[#out + 1] = e.to
				q[#q + 1] = e.to
			end
		end
	end
	return out
end

-- opts.nearest: when the goal cannot be reached, go to the reachable point
-- nearest it instead of failing (a follower waits at the door of a room it
-- does not fit in). The path then carries `partial = true`. Off by default:
-- the walk test counts an unreachable goal as a failure.
function PathTest.solve(result: any, sp: Vector3, gp: Vector3, prof: any?, banned: any?, opts: any?): (any, string)
	local mesh, res = result.mesh, result.portals
	result._pathAdj = result._pathAdj or graph(res)
	local s, sdy = locate(mesh, sp)
	local g, gdy = locate(mesh, gp)
	if not s then return nil, "PathStart is not above any polygon" end
	if not g then return nil, "PathEnd is not above any polygon" end
	local sFloor, gFloor = sp - Vector3.yAxis * sdy, gp - Vector3.yAxis * gdy
	local chain = (s == g) and {} or astar(mesh, result._pathAdj, s, g, sp, gp, prof, banned)
	local partial = false
	if not chain and opts and opts.nearest then
		local want = gFloor
		local bestI, bestQ, bd = s, sFloor, math.huge
		for _, i in ipairs(reachable(mesh, result._pathAdj, s, prof, banned)) do
			local q = closestOn(mesh.tris[i], want)
			local d = (q - want).Magnitude
			if d < bd then bestI, bestQ, bd = i, q, d end
		end
		g, gp, gFloor = bestI, bestQ, bestQ
		chain = (s == g) and {} or astar(mesh, result._pathAdj, s, g, sp, gp, prof, banned)
		partial = true
	end
	if not chain then
		return nil, ("no path: polygon f%04d (r%03d) and f%04d (r%03d) are not connected")
			:format(s, mesh.tris[s].region, g, mesh.tris[g].region)
	end
	local pts = funnel(sFloor, gFloor, portalsOf(mesh, chain))
	local corridor = { s }
	for _, st in ipairs(chain) do corridor[#corridor + 1] = st.e.to end
	local extra
	local crossed
	pts, extra, crossed = straighten(mesh, result._pathAdj, pts, corridor, prof, banned)
	local len = 0
	for k = 2, #pts do len += (pts[k] - pts[k - 1]).Magnitude end
	-- THE PORTALS THE FINAL ROUTE CROSSES. straighten cuts across polygons
	-- through other links, so a chain portal can lie well off the route (a 3
	-- stud bridge 9 studs away, case6); reporting and drawing the chain showed
	-- portals the route never touches. Used: every leap on the chain, every
	-- chain portal within barTol of the route, and every link a straightened
	-- stretch crossed.
	local used, isUsed = {}, {}
	local function add(L: any) if not isUsed[L] then isUsed[L] = true; used[#used + 1] = L end end
	local function nearRoute(p: Vector3, q: Vector3): boolean
		for i = 0, 10 do
			local x = p:Lerp(q, i / 10)
			for k = 2, #pts do
				local a, b = pts[k - 1], pts[k]
				local d = flat(b - a)
				local dd = d:Dot(d)
				local t = dd > 1e-9 and math.clamp(flat(x - a):Dot(d) / dd, 0, 1) or 0
				if (flat(a + (b - a) * t) - flat(x)).Magnitude <= PathTest.barTol then return true end
			end
		end
		return false
	end
	for _, st in ipairs(chain) do
		local L = st.e.L
		if L.kind == "jump" or L.kind == "drop" or nearRoute(L.left, L.right) then add(L) end
	end
	for _, L in ipairs(crossed) do add(L) end
	local kinds = {}
	for _, L in ipairs(used) do kinds[L.kind] = (kinds[L.kind] or 0) + 1 end
	local ks = {}
	for k, v in pairs(kinds) do ks[#ks + 1] = k .. " " .. v end
	table.sort(ks)
	return { points = pts, chain = chain, used = used, from = s, to = g, extra = extra, partial = partial },
		(partial and "unreachable, waiting at the nearest point: " or "")
			.. ("%d portals crossed (%s), %d corners, %.1f studs")
				:format(#used, table.concat(ks, ", "), #pts, len)
end

-- opts: colour of the line, folder name, and extra lift -- so several
-- followers can each show their own path without replacing the others'.
function PathTest.draw(result: any, path: any, opts: any?): (Folder, { Vector3 })
	local o = opts or {}
	local name = o.name or PathTest.folderName
	local old = workspace:FindFirstChild(name)
	if old then old:Destroy() end
	local f = Instance.new("Folder")
	f.Name = name
	local up = Vector3.yAxis * (PathTest.lift + (o.lift or 0))
	local pts = path.points
	-- ALONG THE SURFACES: a straight corner-to-corner line cuts through stair
	-- treads and floats over ridges. Sample each stretch and put every sample on
	-- the corridor polygon under it; the route itself is unchanged.
	local mesh = result.mesh
	local corridor = { path.from }
	for _, st in ipairs(path.chain) do corridor[#corridor + 1] = st.e.to end
	-- and the polygons a straightened stretch crossed
	for _, pv in ipairs(path.extra or {}) do corridor[#corridor + 1] = pv end
	-- the corridor polygon under p NEAREST THE ROUTE'S OWN HEIGHT there, and only
	-- within snapDraw of it: a stair climbs over ground the same route crossed,
	-- and across the unmeshed gap between two treads (a bridge) the ground was
	-- the only polygon under the line, which fell through the stairs onto it
	local function surfaceAt(p: Vector3): (Vector3, boolean)
		local best, bd = p, PathTest.snapDraw
		for _, i in ipairs(corridor) do
			local poly = mesh.tris[i]
			local v = poly.verts
			local n = #v
			local inside = true
			for k = 1, n do
				local a2, b2 = v[k], v[k % n + 1]
				if (b2.X - a2.X) * (p.Z - a2.Z) - (b2.Z - a2.Z) * (p.X - a2.X) > 1e-4 then inside = false; break end
			end
			if inside then
				local c, u = poly.centre, poly.up or Vector3.yAxis
				local h = c.Y
				if math.abs(u.Y) > 1e-3 then h = c.Y - ((p.X - c.X) * u.X + (p.Z - c.Z) * u.Z) / u.Y end
				if math.abs(h - p.Y) < bd then best, bd = Vector3.new(p.X, h, p.Z), math.abs(h - p.Y) end
			end
		end
		return best, bd < PathTest.snapDraw
	end
	-- ON THE GEOMETRY, NOT JUST THE POLYGON. Between two treads joined by a
	-- bridge no polygon is under the line, a straight line up a stair cuts every
	-- nosing, and a tiled roof stands proud of its polygon's plane; a short ray
	-- down puts the line back on top. It only ever lifts, by at most snapDraw.
	local rpDraw = RaycastParams.new()
	rpDraw.FilterType = Enum.RaycastFilterType.Exclude
	do
		local ex = {}
		for _, c in ipairs(workspace:GetChildren()) do
			if c.Name:sub(1, 5) == "NVGN_" or c.Name:sub(1, 9) == "PathStart" or c.Name:sub(1, 7) == "PathEnd"
				or c:FindFirstChildOfClass("Humanoid") then ex[#ex + 1] = c end
		end
		rpDraw.FilterDescendantsInstances = ex
	end
	local trail: { Vector3 } = { pts[1] }
	local function walk(a2: Vector3, b2: Vector3)
		local m = math.max(1, math.floor((b2 - a2).Magnitude / 0.5))
		for j = 1, m do
			local q0 = a2:Lerp(b2, j / m)
			local q = surfaceAt(q0)
			-- from well above: overlapping roof tiles put a ray that starts just
			-- over the line INSIDE the next tile, and a ray never sees the part it
			-- starts in; a hit more than snapDraw up is something else's top
			local hit = workspace:Raycast(q0 + Vector3.yAxis * 4, -Vector3.yAxis * (4 + PathTest.snapDraw), rpDraw)
			if hit and hit.Position.Y > q.Y and hit.Position.Y <= q0.Y + PathTest.snapDraw then
				q = Vector3.new(q.X, hit.Position.Y, q.Z)
			end
			trail[#trail + 1] = q
		end
	end

	-- LEAPS ARE DRAWN AS THE BODY GOES. The route is pulled tight across a jump
	-- or a drop, so its straight line ran through the rails and roof edges the
	-- body goes over (case3: one 22 stud stretch crossed three leaps). Where a
	-- stretch passes a leap's takeoff edge, the line walks along the edge to the
	-- gate's middle and takes the path Leaps proved there (`via`, or `over`)
	-- onto the landing. Not shifted to where the route crosses: slid along the
	-- gate it ran through a corner post the check never swept.
	local function flatV(v: Vector3): Vector3 return Vector3.new(v.X, 0, v.Z) end
	local events: { [number]: { any } } = {}
	for _, st in ipairs(path.chain) do
		local L = st.e.L
		if (L.kind == "jump" or L.kind == "drop") and L.bLeft and L.bRight then
			local bestK, bestT, bestS, bd = nil, 0, 0, 1.0
			for k = 2, #pts do
				local a2, b2 = pts[k - 1], pts[k]
				local d = flatV(b2 - a2)
				local dd = d:Dot(d)
				for i = 0, 10 do
					local sBar = i / 10
					local bp = L.left:Lerp(L.right, sBar)
					local t = dd > 1e-9 and math.clamp(flatV(bp - a2):Dot(d) / dd, 0, 1) or 0
					local dist = (flatV(a2 + (b2 - a2) * t) - flatV(bp)).Magnitude
					if dist < bd then bestK, bestT, bestS, bd = k, t, sBar, dist end
				end
			end
			if bestK then
				local ev = events[bestK]
				if not ev then ev = {}; events[bestK] = ev end
				ev[#ev + 1] = { t = bestT, s = bestS, L = L }
			end
		end
	end
	-- after a leap the line goes on from the LANDING: a route corner on the
	-- landing is skipped to, so a point takeoff shared by two stretches is not
	-- walked back to and climbed again
	local k, cur = 2, pts[1]
	while k <= #pts do
		local b2 = pts[k]
		local ev = events[k] or {}
		table.sort(ev, function(x, y) return x.t < y.t end)
		local nextK = k + 1
		for _, e in ipairs(ev) do
			local L = e.L
			local take = (L.left + L.right) * 0.5
			local land = (L.bRight + L.bLeft) * 0.5
			walk(cur, L.left:Lerp(L.right, e.s))
			walk(L.left:Lerp(L.right, e.s), take)
			for _, w in ipairs(L.via or (L.over and { L.over }) or {}) do trail[#trail + 1] = w end
			trail[#trail + 1] = land
			cur = land
			for j = k, #pts do
				if (pts[j] - land).Magnitude < 0.75 then b2 = pts[j]; nextK = j + 1 break end
			end
		end
		if (b2 - cur).Magnitude > 1e-3 then walk(cur, b2) end
		cur = b2
		k = nextK
	end
	-- A STEP IS DRAWN AS A STEP. Where two close samples differ in height the
	-- diagonal between them cut the nosing of a stair or the edge of a roof
	-- tile; go up before crossing, or cross before going down.
	local stepped: { Vector3 } = { trail[1] }
	for k2 = 2, #trail do
		local a2, b2 = trail[k2 - 1], trail[k2]
		local dy = b2.Y - a2.Y
		if math.abs(dy) > 0.3 and flatV(b2 - a2).Magnitude < 0.6 then
			stepped[#stepped + 1] = dy > 0 and Vector3.new(a2.X, b2.Y, a2.Z) or Vector3.new(b2.X, a2.Y, b2.Z)
		end
		stepped[#stepped + 1] = b2
	end
	-- opts.rubberBand: the line pulled tight through the portals instead -- the
	-- start, where the route crosses each portal bar (at the bar's own height),
	-- the goal. Nothing is laid on a floor, so a polygon buried under a roof
	-- cannot drag it into the geometry.
	if o.rubberBand then
		-- IN ORDER ALONG THE ROUTE: each bar is matched only from the previous
		-- crossing on. Matched against the whole route, a 22 stud ridge bar took
		-- its crossing off the stairs 20 studs below it (same plan position), and
		-- the line kinked back to reach it.
		local cursor = 2
		local function nearOnBar(a: Vector3, b: Vector3): (Vector3, number)
			local best, bd, bk = a, math.huge, cursor
			for i = 0, 20 do
				local q = a:Lerp(b, i / 20)
				for k2 = cursor, #pts do
					local p0, p1 = pts[k2 - 1], pts[k2]
					local d = flatV(p1 - p0)
					local dd = d:Dot(d)
					local t = dd > 1e-9 and math.clamp(flatV(q - p0):Dot(d) / dd, 0, 1) or 0
					local dist = (flatV(p0 + (p1 - p0) * t) - flatV(q)).Magnitude
					if dist < bd - 1e-6 or (math.abs(dist - bd) <= 1e-6 and k2 < bk) then best, bd, bk = q, dist, k2 end
				end
			end
			cursor = bk
			return best, bd
		end
		stepped = { pts[1] }
		for _, st in ipairs(path.chain) do
			local L = st.e.L
			local sides = { { L.left, L.right } }
			if L.bLeft and L.bRight then
				sides[2] = { L.bRight, L.bLeft }
				if st.e.reverse then sides = { sides[2], sides[1] } end
			end
			local from = #stepped
			-- ONLY PORTALS THE ROUTE CROSSES. straighten cuts across polygons, so a
			-- portal the search went through can lie well off the final route; the
			-- line bent 9 studs out to touch such a bridge and back (case6). A leap
			-- is always kept: the body has to take it.
			local leap = L.kind == "jump" or L.kind == "drop"
			local cross = {}
			for _, sd in ipairs(sides) do
				local q, d = nearOnBar(sd[1], sd[2])
				if leap or d <= PathTest.barTol then cross[#cross + 1] = q end
			end
			for _, q in ipairs(cross) do stepped[#stepped + 1] = q end
			-- A LEAP GOES OVER WHAT IT CLEARS: up at the edge to the highest point
			-- of the path Leaps proved (a vault's rail), across, down onto the
			-- landing. Across-then-down at the edge's own height ran through the
			-- very rail the body vaults.
			if (L.kind == "jump" or L.kind == "drop") and #stepped == from + 2 then
				local a2, b2 = stepped[from + 1], stepped[from + 2]
				local apex = math.max(a2.Y, b2.Y)
				for _, w in ipairs(L.via or (L.over and { L.over }) or {}) do apex = math.max(apex, w.Y) end
				table.insert(stepped, from + 2, Vector3.new(b2.X, apex, b2.Z))
				table.insert(stepped, from + 2, Vector3.new(a2.X, apex, a2.Z))
			end
		end
		stepped[#stepped + 1] = pts[#pts]
		-- and every other short change of height is a step: across a riser the
		-- two bars sit a fraction apart and the diagonal between them cut the nosing
		local out2: { Vector3 } = { stepped[1] }
		for k2 = 2, #stepped do
			local a2, b2 = stepped[k2 - 1], stepped[k2]
			local dy = b2.Y - a2.Y
			if math.abs(dy) > 0.3 and flatV(b2 - a2).Magnitude > 1e-3 and flatV(b2 - a2).Magnitude < 1.0 then
				out2[#out2 + 1] = dy > 0 and Vector3.new(a2.X, b2.Y, a2.Z) or Vector3.new(b2.X, a2.Y, b2.Z)
			end
			out2[#out2 + 1] = b2
		end
		stepped = out2
	end
	local colour = o.colour or Color3.fromRGB(255, 255, 255)
	for k2 = 2, #stepped do segment(stepped[k2 - 1] + up, stepped[k2] + up, colour, 0.3, f) end
	-- the portals the route crosses, black, so a bad one is visible where it bites
	for _, L in ipairs(path.used or {}) do
		segment(L.left + up * 0.9, L.right + up * 0.9, Color3.fromRGB(10, 10, 10), 0.16, f)
		if L.bLeft then segment(L.bLeft + up * 0.9, L.bRight + up * 0.9, Color3.fromRGB(10, 10, 10), 0.16, f) end
	end
	f.Parent = workspace
	return f, stepped
end

-- THE BODY ALONG THE LINE. The mesh is lenient for a body bigger than the bake
-- knew about (headroomOpen; portal width from the envelope's radius), so a big
-- NPC's route can still pass somewhere it does not fit. This sweeps a box of
-- the body's own size along a drawn line -- `size` is width x height x width,
-- lifted `lift` studs so steps under that do not count -- and returns every
-- stretch where it would touch something: { at, part, from, to }.
-- Each stretch is swept both ways: a cast never sees a part it starts inside.
--
-- THE BODY STANDS ON THE HIGHEST FLOOR UNDER IT. Its footprint reaches past
-- the line, so on a stair it is already over the next tread; swept from the
-- floor under its centre it hit every step block it stood on (case6: 28
-- "clips", all stair steps). Each stretch is swept level, at the highest line
-- point within the body's reach, plus `lift`.
function PathTest.bodyCheck(line: { Vector3 }, size: Vector3, lift: number, rp: RaycastParams): { any }
	local hits = {}
	local box = Vector3.new(size.X, math.max(0.2, size.Y - lift), size.Z)
	local reach = 0.5 * math.sqrt(size.X * size.X + size.Z * size.Z)
	local function flat(v: Vector3): Vector3 return Vector3.new(v.X, 0, v.Z) end
	-- only a floor a step or so up is under the body: anything higher is a wall
	-- or another storey the route crosses later, and counting it lifted the box
	-- 20 studs into the floor above (case6)
	local window = math.max(lift * 1.5, 3)
	local function floorNear(a: Vector3, b: Vector3): number
		local own = math.max(a.Y, b.Y)
		local top = own
		for _, q in ipairs(line) do
			if q.Y > top and q.Y <= own + window then
				local d = flat(b - a)
				local dd = d:Dot(d)
				local t = dd > 1e-9 and math.clamp(flat(q - a):Dot(d) / dd, 0, 1) or 0
				if (flat(a + (b - a) * t) - flat(q)).Magnitude <= reach then top = q.Y end
			end
		end
		return top
	end
	for k = 2, #line do
		local base = floorNear(line[k - 1], line[k])
		local centre = base + lift + box.Y * 0.5
		local a = Vector3.new(line[k - 1].X, centre, line[k - 1].Z)
		local b = Vector3.new(line[k].X, centre, line[k].Z)
		local d = b - a
		if d.Magnitude > 1e-3 then
			local h = workspace:Blockcast(CFrame.new(a), box, d, rp)
			local at = h and (a + d.Unit * h.Distance)
			if not h then
				h = workspace:Blockcast(CFrame.new(b), box, -d, rp)
				at = h and (b - d.Unit * h.Distance)
			end
			if h then hits[#hits + 1] = { at = at, part = h.Instance, from = line[k - 1], to = line[k], box = box } end
		end
	end
	return hits
end

-- marker parts, made if missing, and a watcher that re-solves when they move
function PathTest.watch(result: any, status: StringValue?): () -> ()
	local function marker(name: string, colour: Color3, at: Vector3): BasePart
		local m = workspace:FindFirstChild(name)
		if m and m:IsA("BasePart") then return m end
		local p = Instance.new("Part")
		p.Name = name
		p.Anchored = true; p.CanCollide = false; p.CanQuery = false; p.CanTouch = false
		p.Shape = Enum.PartType.Ball
		p.Size = Vector3.new(1.5, 1.5, 1.5)
		p.Material = Enum.Material.Neon
		p.Color = colour
		p.Position = at
		p.Parent = workspace
		return p
	end
	-- start on the biggest polygon so the first solve lands on floor
	local big = result.mesh.tris[1]
	for _, t in ipairs(result.mesh.tris) do if (t.area or 0) > (big.area or 0) then big = t end end
	local s = marker("PathStart", Color3.fromRGB(40, 255, 60), big.centre + Vector3.new(0, 1, 0))
	local g = marker("PathEnd", Color3.fromRGB(255, 40, 40), big.centre + Vector3.new(6, 1, 0))
	local alive = true
	task.spawn(function()
		local lastS, lastG = nil, nil
		while alive do
			if not s.Parent or not g.Parent then break end
			local ps, pg = s.Position, g.Position
			if ps ~= lastS or pg ~= lastG then
				lastS, lastG = ps, pg
				local ok, err = pcall(function()
					local path, msg = PathTest.solve(result, ps, pg)
					if path then PathTest.draw(result, path) else
						local old = workspace:FindFirstChild(PathTest.folderName)
						if old then old:Destroy() end
					end
					if status then status.Value = msg end
				end)
				if not ok and status then status.Value = "error " .. tostring(err) end
			end
			task.wait(0.25)
		end
	end)
	return function() alive = false end
end

return PathTest
