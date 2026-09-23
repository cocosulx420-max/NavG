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
		return -rise <= (prof.drop or math.huge)
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
				local via = e.L.centre
				local leap = (e.L.kind == "jump" or e.L.kind == "drop") and 2 or 0
				local cost = gs[cur] + (via - pos[cur]).Magnitude * postureCost(mesh.tris[e.to], prof) + leap
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
	local len = 0
	for k = 2, #pts do len += (pts[k] - pts[k - 1]).Magnitude end
	local kinds = {}
	for _, st in ipairs(chain) do kinds[st.e.L.kind] = (kinds[st.e.L.kind] or 0) + 1 end
	local ks = {}
	for k, v in pairs(kinds) do ks[#ks + 1] = k .. " " .. v end
	table.sort(ks)
	return { points = pts, chain = chain, from = s, to = g },
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
	for k = 2, #pts do segment(pts[k - 1] + up, pts[k] + up, Color3.fromRGB(255, 255, 255), 0.3, f) end
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
