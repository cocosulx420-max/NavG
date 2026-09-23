-- The global grid experiment harness (docs/globalgrid-experiment.md).
--
-- Posted to the resident worker as a command, so `result` is the CURRENT
-- pipeline's held bake -- the thing being compared against -- and ctx.req loads
-- modules from NavGenProj. Builds the experiment area at each cell size, meshes
-- it, measures both, and draws the chosen run into workspace.NVGN_GG.
local result, ctx = ...
local ng = ctx.ng
-- a FRESH loader: the worker's own caches the modules it baked with
local loaded = {}
local function req(inst)
	if loaded[inst] ~= nil then return loaded[inst] end
	local f, err = loadstring("local script,require=... " .. inst.Source, "=" .. inst.Name)
	if not f then error(err) end
	local m = f(inst, req); loaded[inst] = m; return m
end
local GG = req(ng.GlobalGrid)
local PT = req(ng.PathTest)

local BOUNDS = { min = Vector3.new(-608, 30, -480), max = Vector3.new(-512, 100, -384) }
local SIZES = { 0.5, 0.25 }
local DRAW = 0.5
local job = game.ServerScriptService.NVGN_Job
local lastYield = os.clock()
local function onProgress()
	if os.clock() - lastYield > 0.05 then lastYield = os.clock(); task.wait() end
end

local function inArea(p: Vector3): boolean
	return p.X >= BOUNDS.min.X and p.X <= BOUNDS.max.X and p.Z >= BOUNDS.min.Z and p.Z <= BOUNDS.max.Z
		and p.Y >= BOUNDS.min.Y and p.Y <= BOUNDS.max.Y
end

-- random-pair path test over a mesh + links, restricted to a polygon set
local function pathTest(res: any, polys: { number }, n: number): string
	local mesh = res.mesh
	local function inPlan(f, p)
		local v = f.verts
		for k = 1, #v do
			local a, b = v[k], v[k % #v + 1]
			if (b.X - a.X) * (p.Z - a.Z) - (b.Z - a.Z) * (p.X - a.X) > 1e-3 then return false end
		end
		return true
	end
	local rng = Random.new(5)
	local tried, found, off, tot, gap = 0, 0, 0, 0, 0
	res._pathAdj = nil
	for _ = 1, n do
		local a = polys[rng:NextInteger(1, #polys)]
		local b = polys[rng:NextInteger(1, #polys)]
		if a ~= b then
			tried += 1
			local path = PT.solve(res, mesh.tris[a].centre + Vector3.yAxis, mesh.tris[b].centre + Vector3.yAxis)
			if path then
				found += 1
				local corridor = { path.from }
				for _, st in ipairs(path.chain) do corridor[#corridor + 1] = st.e.to end
				local pts = path.points
				for k = 2, #pts do
					local m = math.max(1, math.floor((pts[k] - pts[k - 1]).Magnitude / 0.5))
					for j = 1, m - 1 do
						tot += 1
						local p = pts[k - 1]:Lerp(pts[k], j / m)
						local ok = false
						for _, i in ipairs(corridor) do if inPlan(mesh.tris[i], p) then ok = true; break end end
						if not ok then
							-- inside the gap of a two-sided portal on the path is fine
							for _, st in ipairs(path.chain) do
								local L = st.e.L
								if L.bLeft then
									local c = (L.left + L.right + L.bLeft + L.bRight) * 0.25
									if (Vector3.new(p.X - c.X, 0, p.Z - c.Z)).Magnitude <= L.span * 0.5 + 1 then ok = true; gap += 1; break end
								end
							end
						end
						if not ok then off += 1 end
					end
				end
			end
		end
	end
	return ("%d pairs, %d paths found, %.2f%% of path samples outside the corridor (%d of %d)"):format(tried, found,
		100 * off / math.max(1, tot), off, tot)
end

-- connected pieces by area, walking links only
local function pieces(tris, links, keep)
	local parent = {}
	for i in pairs(keep) do parent[i] = i end
	local function f(x) while parent[x] ~= x do parent[x] = parent[parent[x]]; x = parent[x] end return x end
	for _, L in ipairs(links) do
		if keep[L.a] and keep[L.b] and L.kind ~= "drop" and L.kind ~= "jump" then
			local a, b = f(L.a), f(L.b) if a ~= b then parent[a] = b end
		end
	end
	local area, total = {}, 0
	for i in pairs(keep) do local r = f(i) area[r] = (area[r] or 0) + (tris[i].area or 0); total += tris[i].area or 0 end
	local list = {}
	for _, a in pairs(area) do list[#list + 1] = a end
	table.sort(list, function(x, y) return x > y end)
	local top = {}
	for i = 1, math.min(5, #list) do top[#top + 1] = ("%.0f"):format(list[i]) end
	return ("%d connected pieces over %.0f sq studs; biggest %s"):format(#list, total, table.concat(top, ", "))
end

local out = {}

-- the CURRENT pipeline, clipped to the area
do
	local mesh, links = result.mesh, result.portals.links
	local inside = {}
	local polys = {}
	for i, f in ipairs(mesh.tris) do
		if inArea(f.centre) then inside[i] = true; polys[#polys + 1] = i end
	end
	local kinds, fitted = {}, 0
	for _, L in ipairs(links) do
		if inside[L.a] and inside[L.b] then
			kinds[L.kind] = (kinds[L.kind] or 0) + 1
			if L.kind ~= "shared" and not (L.bLeft and L.bRight) then fitted += 1 end
		end
	end
	local ks = {}
	for k, v in pairs(kinds) do ks[#ks + 1] = k .. " " .. v end
	table.sort(ks)
	local regions = {}
	for _, i in ipairs(polys) do regions[mesh.tris[i].region] = true end
	local nr = 0 for _ in pairs(regions) do nr += 1 end
	out[#out + 1] = ("CURRENT  | %d regions, %d polygons | links %s | %d fitted"):format(nr, #polys, table.concat(ks, ", "), fitted)
	out[#out + 1] = "  path test: " .. pathTest(result, polys, 200)
	-- the experiment has no drops or jumps yet, so also without them
	local walk = {}
	for _, L in ipairs(links) do if L.kind ~= "drop" and L.kind ~= "jump" then walk[#walk + 1] = L end end
	out[#out + 1] = "  path test, walking links only: " .. pathTest({ mesh = mesh, portals = { links = walk } }, polys, 200)
	out[#out + 1] = "  " .. pieces(mesh.tris, links, inside)
end

local keep = nil
for _, size in ipairs(SIZES) do
	job.Value = ("globalgrid %.2f"):format(size)
	local t0 = os.clock()
	local g = GG.build({ bounds = BOUNDS, root = workspace.case6, minCell = size, onProgress = onProgress })
	local m = GG.mesh(g)
	local total = os.clock() - t0
	local complaints = m.mesh.complaints or {}
	local cross, forced = 0, 0
	for _, s in ipairs(complaints) do
		if s:find("not simple") then cross += 1 end
		if s:find("could not be forced") then forced += 1 end
	end
	local outers, holes, orphanHoles = 0, 0, 0
	local perRegion = {}
	for _, L in ipairs(m.loops) do
		if L.kind == "outer" then outers += 1; perRegion[L.region] = (perRegion[L.region] or 0) + 1 end
		if L.kind == "hole" then holes += 1; if not L.parent then orphanHoles += 1 end end
	end
	local multiOuter = 0
	for _, v in pairs(perRegion) do if v > 1 then multiOuter += 1 end end
	local polys = {}
	for i in ipairs(m.mesh.tris) do polys[#polys + 1] = i end
	local res = { mesh = m.mesh, portals = m.portals }
	out[#out + 1] = ("GLOBAL %.2f | %.1fs total"):format(size, total)
	out[#out + 1] = "  " .. GG.report(g)
	out[#out + 1] = "  " .. GG.meshReport(m)
	out[#out + 1] = ("  rings: %d outer, %d holes (%d inside no rim), %d regions with more than one rim | CDT: %d rings cross themselves, %d regions with edges not forced")
		:format(outers, holes, orphanHoles, multiOuter, cross, forced)
	out[#out + 1] = "  path test: " .. pathTest(res, polys, 200)
	local all = {}
	for i in ipairs(m.mesh.tris) do all[i] = true end
	out[#out + 1] = "  " .. pieces(m.mesh.tris, m.portals.links, all)
	if size == DRAW then keep = { g = g, m = m } end
end

-- draw the chosen run
if keep then
	job.Value = "drawing"
	local old = workspace:FindFirstChild("NVGN_GG") if old then old:Destroy() end
	local root = Instance.new("Folder"); root.Name = "NVGN_GG"
	local function seg(a, b, col, thick, parent, name)
		local d = b - a
		if d.Magnitude < 1e-3 then return end
		local p = Instance.new("Part")
		p.Anchored = true; p.CanCollide = false; p.CanQuery = false; p.CanTouch = false; p.CastShadow = false
		p.Material = Enum.Material.Neon; p.Color = col
		p.Size = Vector3.new(thick, thick, d.Magnitude)
		p.CFrame = CFrame.lookAt((a + b) * 0.5, b)
		p.Name = name or "seg"
		p.Parent = parent
	end
	-- cells, by region
	local fc = Instance.new("Folder"); fc.Name = "Cells"; fc.Parent = root
	local n = 0
	for _, cell in ipairs(keep.g.cells) do
		local r = cell.region
		local h = (r * 0.61803398875) % 1
		local p = Instance.new("Part")
		p.Anchored = true; p.CanCollide = false; p.CanQuery = false; p.CanTouch = false; p.CastShadow = false
		p.Material = Enum.Material.SmoothPlastic
		p.Color = Color3.fromHSV(h, 0.55, 0.9)
		p.Size = Vector3.new(math.max(cell.s - 0.06, 0.05), 0.05, math.max(cell.s - 0.06, 0.05))
		local nrm = cell.n
		local right = Vector3.xAxis - nrm * nrm.X
		right = right.Magnitude > 1e-3 and right.Unit or Vector3.xAxis
		p.CFrame = CFrame.fromMatrix(Vector3.new(cell.cx, cell.y + 0.06, cell.cz), right, nrm)
		p.Name = ("r%d_%.2f"):format(r, cell.s)
		p.Parent = fc
		n += 1
		if n % 4000 == 0 then task.wait() end
	end
	-- banded outlines (cyan), cut stretches (white)
	local fo = Instance.new("Folder"); fo.Name = "Outlines"; fo.Parent = root
	for _, L in ipairs(keep.m.loops) do
		local pts = L.pts
		for k = 1, #pts do
			seg(pts[k] + Vector3.new(0, 0.25, 0), pts[k % #pts + 1] + Vector3.new(0, 0.25, 0),
				L.kind == "hole" and Color3.fromRGB(255, 80, 160) or Color3.fromRGB(60, 230, 255), 0.12, fo)
		end
	end
	-- polygon edges (dark), portals (shared blue, tile orange)
	local fp = Instance.new("Folder"); fp.Name = "Polygons"; fp.Parent = root
	for i, f in ipairs(keep.m.mesh.tris) do
		for k = 1, f.n do seg(f.verts[k] + Vector3.new(0, 0.18, 0), f.verts[k % f.n + 1] + Vector3.new(0, 0.18, 0), Color3.fromRGB(60, 60, 60), 0.06, fp) end
	end
	local fl = Instance.new("Folder"); fl.Name = "Portals"; fl.Parent = root
	for i, L in ipairs(keep.m.portals.links) do
		seg(L.left + Vector3.new(0, 0.35, 0), L.right + Vector3.new(0, 0.35, 0),
			L.kind == "tile" and Color3.fromRGB(255, 140, 20) or Color3.fromRGB(40, 110, 255), 0.18, fl, ("p%d_%s"):format(i, L.kind))
	end
	root.Parent = workspace
	out[#out + 1] = ("drew the %.2f run into workspace.NVGN_GG (%d cells)"):format(DRAW, n)
end
return table.concat(out, "\n")
