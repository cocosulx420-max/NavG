--!strict
-- NVGN.FloorLocal — walkable surface extraction driven by SVOLocal.
--
-- Same shape as Floor.extract, but the candidate source is a per-part local-axis
-- tree instead of one world-aligned octree. The principle is unchanged and is
-- the whole reason voxel precision does not set navmesh precision: the SVO only
-- says WHERE to look. Every surface position and normal below comes from a
-- raycast onto the real part.
--
-- SVOLocal nodes are ORIENTED, so a node's "top" is not world-up. Candidates are
-- therefore taken from each node's world-space AABB top, over the 1-stud world
-- lattice, and de-duplicated -- many nodes land on the same column.

local FloorLocal = {}

local UP = Vector3.new(0, 1, 0)

export type Surfel = {
	pos: Vector3,
	normal: Vector3,
	slope: number,
	clearance: number,
	part: BasePart,
}

local DEFAULT = {
	maxSlope = 65,            -- max walkable slope (deg), same as Floor
	clearCap = 20,
	minClearance = 1.5,
	maxGroundFootprint = 400, -- skip the baseplate
	skipNonCollide = true,    -- a character cannot stand on CanCollide=false trim
	skipCharacters = true,
}

local function merged(cfg)
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

local function isCharacter(p: Instance): boolean
	local a = p.Parent
	while a and a ~= workspace do
		if a:IsA("Model") and a:FindFirstChildOfClass("Humanoid") then return true end
		a = a.Parent
	end
	return false
end

-- World-space AABB half-extents of an oriented cube.
local function aabbHalf(cf: CFrame, edge: number): Vector3
	local e = edge * 0.5
	local r, u, l = cf.RightVector, cf.UpVector, cf.LookVector
	return Vector3.new(
		(math.abs(r.X) + math.abs(u.X) + math.abs(l.X)) * e,
		(math.abs(r.Y) + math.abs(u.Y) + math.abs(l.Y)) * e,
		(math.abs(r.Z) + math.abs(u.Z) + math.abs(l.Z)) * e
	)
end

-- trees: { [BasePart] = SVOLocal }, as returned by SVOLocal.fromParts
function FloorLocal.extract(trees, cfg)
	local c = merged(cfg)

	-- Which parts may a character actually stand on?
	local solids: {BasePart} = {}
	for part in pairs(trees) do
		local skip = false
		if c.skipNonCollide and not part.CanCollide then skip = true end
		if c.skipCharacters and isCharacter(part) then skip = true end
		local s = part.Size
		if math.max(s.X, s.Z) > c.maxGroundFootprint then skip = true end
		if not skip then table.insert(solids, part) end
	end

	local rp = RaycastParams.new()
	rp.FilterType = Enum.RaycastFilterType.Include
	rp.FilterDescendantsInstances = solids

	local probe = Instance.new("Part")
	probe.Name = "NVGN_ClearProbe"
	probe.Size = Vector3.new(0.05, math.max(c.minClearance - 0.1, 0.05), 0.05)
	probe.Anchored = true
	probe.CanCollide, probe.CanQuery, probe.CanTouch = false, false, false
	probe.Transparency = 1
	probe.Parent = workspace
	local op = OverlapParams.new()
	op.FilterType = Enum.RaycastFilterType.Include
	op.FilterDescendantsInstances = solids
	op.RespectCanCollide = false

	-- A small cube used to ask "is there real geometry just above this column?",
	-- standing in for the world SVO's isSolid guard.
	local above = Instance.new("Part")
	above.Size = Vector3.new(0.6, 0.6, 0.6)
	above.Anchored = true
	above.CanCollide, above.CanQuery, above.CanTouch = false, false, false
	above.Transparency = 1
	above.Parent = workspace

	-- Gather de-duplicated candidate columns from the node set.
	local seen: { [string]: boolean } = {}
	local cand: { { x: number, z: number, top: number, span: number } } = {}
	for part, tree in pairs(trees) do
		local skip = false
		if c.skipNonCollide and not part.CanCollide then skip = true end
		if c.skipCharacters and isCharacter(part) then skip = true end
		if math.max(part.Size.X, part.Size.Z) > c.maxGroundFootprint then skip = true end
		if not skip then
			tree:forEachNode(function(cf: CFrame, edge: number)
				local h = aabbHalf(cf, edge)
				local p = cf.Position
				local top = p.Y + h.Y
				local x0, x1 = p.X - h.X, p.X + h.X
				local z0, z1 = p.Z - h.Z, p.Z + h.Z
				local i = math.floor(x0)
				while i < x1 do
					local j = math.floor(z0)
					while j < z1 do
						local cx, cz = i + 0.5, j + 0.5
						local key = string.format("%d_%d_%d", i, j, math.floor(top * 2 + 0.5))
						if not seen[key] then
							seen[key] = true
							table.insert(cand, { x = cx, z = cz, top = top, span = edge })
						end
						j += 1
					end
					i += 1
				end
			end)
		end
	end

	local surfels: {Surfel} = {}
	local rejSolidAbove, rejNoHit, rejSlope, rejDup = 0, 0, 0, 0
	-- Several candidates can share a column -- different nodes, different AABB
	-- tops, the same surface underneath. De-duplicate on the RESULT, not on the
	-- candidate, or one walkable cell is reported many times.
	local landed: { [string]: boolean } = {}

	for _, k in ipairs(cand) do
		-- empty directly above? (the SVO's "top face is exposed" guard)
		above.CFrame = CFrame.new(k.x, k.top + 0.5, k.z)
		if #workspace:GetPartsInPart(above, op) > 0 then
			rejSolidAbove += 1
			continue
		end
		local res = workspace:Raycast(
			Vector3.new(k.x, k.top + 1, k.z),
			Vector3.new(0, -(k.span + 2), 0), rp)
		if not res then
			rejNoHit += 1
			continue
		end
		local n = res.Normal
		local slope = math.deg(math.acos(math.clamp(n:Dot(UP), -1, 1)))
		if slope > c.maxSlope then
			rejSlope += 1
			continue
		end
		local rk = string.format("%d_%d_%d",
			math.floor(res.Position.X), math.floor(res.Position.Z),
			math.floor(res.Position.Y * 4 + 0.5))
		if landed[rk] then
			rejDup += 1
			continue
		end
		landed[rk] = true
		-- clearance: precise overlap first (a raycast never hits the part its
		-- origin is inside), then an up-ray to the real ceiling
		local clearance
		probe.CFrame = CFrame.new(res.Position + Vector3.new(0, 0.1 + (c.minClearance - 0.1) * 0.5, 0))
		local blocked = false
		for _, hit in ipairs(workspace:GetPartsInPart(probe, op)) do
			if hit ~= res.Instance then blocked = true break end
		end
		if blocked then
			clearance = 0
		else
			local upRes = workspace:Raycast(res.Position + Vector3.new(0, 0.15, 0), Vector3.new(0, c.clearCap, 0), rp)
			clearance = upRes and upRes.Distance or c.clearCap
		end
		table.insert(surfels, {
			pos = res.Position, normal = n, slope = slope,
			clearance = clearance, part = res.Instance,
		})
	end

	probe:Destroy()
	above:Destroy()

	return {
		surfels = surfels,
		config = c,
		stats = {
			candidates = #cand,
			kept = #surfels,
			rejectedSolidAbove = rejSolidAbove,
			rejectedNoHit = rejNoHit,
			rejectedSlope = rejSlope,
			rejectedDuplicate = rejDup,
			standableParts = #solids,
		},
	}
end

-- ---- drawing ----------------------------------------------------------------

local ROOT_NAME = "NVGN_Debug"
local SECTION = "FloorLocal"

function FloorLocal.clear(name: string?)
	local root = workspace:FindFirstChild(ROOT_NAME)
	local sec = root and root:FindFirstChild(SECTION)
	if not sec then return 0 end
	if name == nil then
		local n = #sec:GetChildren()
		sec:ClearAllChildren()
		return n
	end
	local f = sec:FindFirstChild(name)
	if not f then return 0 end
	local n = #f:GetChildren()
	f:Destroy()
	return n
end

-- One tile per walkable surfel, laid ON the surface and tilted to its normal.
--   colourBy  "slope" (default) or "clearance"
--   lift      studs to raise the tile off the surface (default 0.05)
function FloorLocal.draw(result, opts)
	opts = opts or {}
	local name: string = opts.name or "floor"
	local colourBy: string = opts.colourBy or "slope"
	local lift: number = opts.lift or 0.05
	local size: number = opts.size or 0.92
	local transparency: number = opts.transparency or 0

	FloorLocal.clear(name)
	local root = workspace:FindFirstChild(ROOT_NAME)
	if not root then
		root = Instance.new("Folder")
		root.Name = ROOT_NAME
		root.Parent = workspace
	end
	local sec = root:FindFirstChild(SECTION)
	if not sec then
		sec = Instance.new("Folder")
		sec.Name = SECTION
		sec.Parent = root
	end
	local folder = Instance.new("Folder")
	folder.Name = name

	local maxSlope = result.config.maxSlope
	local cap = result.config.clearCap
	for _, s in ipairs(result.surfels) do
		local p = Instance.new("Part")
		p.Anchored = true
		p.CanCollide, p.CanQuery, p.CanTouch = false, false, false
		p.CastShadow = false
		p.Material = Enum.Material.SmoothPlastic
		p.Size = Vector3.new(size, 0.06, size)
		-- lay the tile in the surface plane
		local up = s.normal
		local fwd = math.abs(up.Y) > 0.99 and Vector3.new(0, 0, 1) or UP:Cross(up).Unit
		local right = up:Cross(fwd).Unit
		p.CFrame = CFrame.fromMatrix(s.pos + up * lift, right, up)
		if colourBy == "clearance" then
			local t = math.clamp(s.clearance / math.max(cap, 1e-6), 0, 1)
			p.Color = s.clearance <= 0 and Color3.fromRGB(210, 40, 40)
				or Color3.fromRGB(math.floor(255 * (1 - t)), math.floor(90 + 140 * t), 90)
		else
			local t = math.clamp(s.slope / math.max(maxSlope, 1e-6), 0, 1)
			p.Color = Color3.fromRGB(math.floor(60 + 190 * t), math.floor(220 - 130 * t), 90)
		end
		p.Transparency = transparency
		p:SetAttribute("slope", s.slope)
		p:SetAttribute("clearance", s.clearance)
		p:SetAttribute("part", s.part.Name)
		p.Parent = folder
	end
	folder.Parent = sec
	return { drawn = #result.surfels, folder = folder }
end

return FloorLocal
