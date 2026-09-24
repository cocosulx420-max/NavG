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
	if T.headroom and T.headroom < (prof.prone or 0) then return false end
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
	if not prof or not T.headroom then return 1 end
	if T.headroom < (prof.crouch or 0) then return 3 end
	if T.headroom < (prof.height or 0) then return 1.6 end
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
	prof: any?, banned: any?, visited: { number }): boolean
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
		local nextPoly, nextT = nil, nil
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
					if nextPoly then break end
				end
			end
		end
		if not nextPoly then return false end
		visited[#visited + 1] = nextPoly
		cur, t = nextPoly, nextT :: number
	end
	return false
end
local function straighten(mesh: any, adj: any, pts: { Vector3 }, corridor: { number }, prof: any?, banned: any?): ({ Vector3 }, { number })
	if #pts <= 2 then return pts, {} end
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
	local out, seen = { pts[1] }, {}
	local i = 1
	while i < #pts do
		local jBest = i + 1
		for j = #pts, i + 2, -1 do
			local ok = false
			local visited = {}
			for start in pairs(holds[i]) do
				table.clear(visited)
				if walkRay(mesh, adj, start, pts[i], pts[j], holds[j], prof, banned, visited) then ok = true break end
			end
			if ok then
				jBest = j
				for _, pv in ipairs(visited) do seen[pv] = true end
				break
			end
		end
		out[#out + 1] = pts[jBest]
		i = jBest
	end
	local extra = {}
	for pv in pairs(seen) do extra[#extra + 1] = pv end
	return out, extra
end

function PathTest.solve(result: any, sp: Vector3, gp: Vector3, prof: any?, banned: any?): (any, string)
	local mesh, res = result.mesh, result.portals
	result._pathAdj = result._pathAdj or graph(res)
	local s, sdy = locate(mesh, sp)
	local g, gdy = locate(mesh, gp)
	if not s then return nil, "PathStart is not above any polygon" end
	if not g then return nil, "PathEnd is not above any polygon" end
	local sFloor, gFloor = sp - Vector3.yAxis * sdy, gp - Vector3.yAxis * gdy
	local chain = (s == g) and {} or astar(mesh, result._pathAdj, s, g, sp, gp, prof, banned)
	if not chain then
		return nil, ("no path: polygon f%04d (r%03d) and f%04d (r%03d) are not connected")
			:format(s, mesh.tris[s].region, g, mesh.tris[g].region)
	end
	local pts = funnel(sFloor, gFloor, portalsOf(mesh, chain))
	local corridor = { s }
	for _, st in ipairs(chain) do corridor[#corridor + 1] = st.e.to end
	local extra
	pts, extra = straighten(mesh, result._pathAdj, pts, corridor, prof, banned)
	local len = 0
	for k = 2, #pts do len += (pts[k] - pts[k - 1]).Magnitude end
	local kinds = {}
	for _, st in ipairs(chain) do kinds[st.e.L.kind] = (kinds[st.e.L.kind] or 0) + 1 end
	local ks = {}
	for k, v in pairs(kinds) do ks[#ks + 1] = k .. " " .. v end
	table.sort(ks)
	return { points = pts, chain = chain, from = s, to = g, extra = extra },
		("%d polygons, %d portals (%s), %d corners, %.1f studs")
			:format(#chain + 1, #chain, table.concat(ks, ", "), #pts, len)
end

function PathTest.draw(result: any, path: any): Folder
	local old = workspace:FindFirstChild(PathTest.folderName)
	if old then old:Destroy() end
	local f = Instance.new("Folder")
	f.Name = PathTest.folderName
	local up = Vector3.yAxis * PathTest.lift
	local pts = path.points
	-- ALONG THE SURFACES: a straight corner-to-corner line cuts through stair
	-- treads and floats over ridges. Sample each stretch and put every sample on
	-- the corridor polygon under it; the route itself is unchanged.
	local mesh = result.mesh
	local corridor = { path.from }
	for _, st in ipairs(path.chain) do corridor[#corridor + 1] = st.e.to end
	-- and the polygons a straightened stretch crossed
	for _, pv in ipairs(path.extra or {}) do corridor[#corridor + 1] = pv end
	local function surfaceAt(p: Vector3): Vector3
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
				return Vector3.new(p.X, h, p.Z)
			end
		end
		return p
	end
	local trail = { pts[1] }
	for k = 2, #pts do
		local a2, b2 = pts[k - 1], pts[k]
		local m = math.max(1, math.floor((b2 - a2).Magnitude / 0.5))
		for j = 1, m do trail[#trail + 1] = surfaceAt(a2:Lerp(b2, j / m)) end
	end
	for k = 2, #trail do segment(trail[k - 1] + up, trail[k] + up, Color3.fromRGB(255, 255, 255), 0.3, f) end
	-- the portals the path went through, black, so a bad one is visible where it bites
	for _, st in ipairs(path.chain) do
		local L = st.e.L
		segment(L.left + up * 0.9, L.right + up * 0.9, Color3.fromRGB(10, 10, 10), 0.16, f)
		if L.bLeft then segment(L.bLeft + up * 0.9, L.bRight + up * 0.9, Color3.fromRGB(10, 10, 10), 0.16, f) end
	end
	f.Parent = workspace
	return f
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
