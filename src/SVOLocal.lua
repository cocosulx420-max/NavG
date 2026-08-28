--!strict
local SVOLocal = {}
SVOLocal.__index = SVOLocal

local OFF = {}
for i = 0, 7 do
	OFF[i] = Vector3.new(
		(bit32.band(i,1) ~= 0) and 1 or -1,
		(bit32.band(i,2) ~= 0) and 1 or -1,
		(bit32.band(i,4) ~= 0) and 1 or -1
	)
end

local function worldAABBHalf(cf: CFrame, size: Vector3): Vector3
	local e = size * 0.5
	local r, u, l = cf.RightVector, cf.UpVector, cf.LookVector
	return Vector3.new(
		math.abs(r.X)*e.X + math.abs(u.X)*e.Y + math.abs(l.X)*e.Z,
		math.abs(r.Y)*e.X + math.abs(u.Y)*e.Y + math.abs(l.Y)*e.Z,
		math.abs(r.Z)*e.X + math.abs(u.Z)*e.Y + math.abs(l.Z)*e.Z
	)
end

function SVOLocal.neighbours(part: BasePart, all: {BasePart}, pad: number): {BasePart}
	local ha = worldAABBHalf(part.CFrame, part.Size) + Vector3.new(pad, pad, pad)
	local ca = part.Position
	local out = {}
	for _, other in ipairs(all) do
		if other ~= part then
			local hb = worldAABBHalf(other.CFrame, other.Size)
			local d = other.Position - ca
			if math.abs(d.X) <= ha.X + hb.X
				and math.abs(d.Y) <= ha.Y + hb.Y
				and math.abs(d.Z) <= ha.Z + hb.Z then
				table.insert(out, other)
			end
		end
	end
	return out
end

local function anchor(dim: number, leaf: number): number
	local n = math.max(1, math.ceil(dim / leaf - 1e-9))
	return -n * leaf * 0.5
end

-- Is a part's COLLISION shape identical to its bounding box?
-- CanQuery = false is treated as exact ON PURPOSE: the precise test is
-- GetPartsInPart, which cannot see such a part at all, so the precise path
-- would silently delete it. An over-large box beats a vanished part.
function SVOLocal.boxIsExact(part: BasePart): boolean
	if not part.CanQuery then return true end
	if part:IsA("UnionOperation") then return false end
	if part:IsA("MeshPart") then
		local ok, fid = pcall(function() return (part :: any).CollisionFidelity end)
		return ok and fid == Enum.CollisionFidelity.Box
	end
	local ok, shape = pcall(function() return (part :: any).Shape end)
	return ok and shape == Enum.PartType.Block
end

function SVOLocal.new(part: BasePart, leaf: number)
	local self = setmetatable({}, SVOLocal)
	local s = part.Size
	local origin = Vector3.new(anchor(s.X, leaf), anchor(s.Y, leaf), anchor(s.Z, leaf))
	local maxE = math.max(s.X, s.Y, s.Z)
	local depth = math.max(0, math.ceil(math.log(maxE / leaf) / math.log(2)))
	local edge = leaf * (2 ^ depth)
	while (origin.X + edge < s.X * 0.5 - 1e-9)
		or (origin.Y + edge < s.Y * 0.5 - 1e-9)
		or (origin.Z + edge < s.Z * 0.5 - 1e-9) do
		depth += 1
		edge = leaf * (2 ^ depth)
	end
	self.part = part
	self.cf = part.CFrame
	self.size = s
	self.leaf = leaf
	self.maxDepth = depth
	self.origin = origin
	self.half = edge * 0.5
	self.center = origin + Vector3.new(edge, edge, edge) * 0.5
	self.root = {}
	return self
end

function SVOLocal:_cubeHitsPart(lc: Vector3, h: number): boolean
	local e = self.size * 0.5
	return math.abs(lc.X) < e.X + h - 1e-6
		and math.abs(lc.Y) < e.Y + h - 1e-6
		and math.abs(lc.Z) < e.Z + h - 1e-6
end

function SVOLocal:_cubeInsidePart(lc: Vector3, h: number): boolean
	local e = self.size * 0.5
	return math.abs(lc.X) + h <= e.X + 1e-6
		and math.abs(lc.Y) + h <= e.Y + 1e-6
		and math.abs(lc.Z) + h <= e.Z + 1e-6
end

function SVOLocal:_build(node, lc: Vector3, h: number, depth: number, seamAt, hitsAt, fullAt): boolean
	if not hitsAt(lc, h) then return false end
	local seam = seamAt(lc, h)
	local full = fullAt(lc, h)

	if depth == 0 then
		node.solid = true
		node.seam = seam
		return true
	end
	if full and not seam then
		node.solid = true
		node.seam = false
		return true
	end

	node.children = {}
	local ch = h * 0.5
	local any = false
	for i = 0, 7 do
		local child = {}
		if self:_build(child, lc + OFF[i] * ch, ch, depth - 1, seamAt, hitsAt, fullAt) then
			node.children[i] = child
			any = true
		end
	end
	if not any then
		node.children = nil
		return false
	end

	local allSolid, seamCount = true, 0
	for i = 0, 7 do
		local c = node.children[i]
		if not (c and c.solid) then
			allSolid = false
			break
		end
		if c.seam then
			seamCount += 1
		end
	end
	if allSolid and (seamCount == 0 or seamCount == 8) then
		node.children = nil
		node.solid = true
		node.seam = (seamCount == 8)
	end
	return true
end

function SVOLocal.forPart(part: BasePart, others: {BasePart}, leaf: number, contactPad: number?)
	local self = SVOLocal.new(part, leaf)
	local pad = contactPad or 0.02
	local exact = SVOLocal.boxIsExact(part)
	self.exact = exact

	local probe = Instance.new("Part")
	probe.Anchored = true
	probe.CanCollide, probe.CanQuery, probe.CanTouch = false, false, false
	probe.Transparency = 1
	probe.Parent = workspace

	local shapeProbes = 0
	local hitsAt, fullAt
	if exact then
		hitsAt = function(lc: Vector3, h: number) return self:_cubeHitsPart(lc, h) end
		fullAt = function(lc: Vector3, h: number) return self:_cubeInsidePart(lc, h) end
	else
		local selfOp = OverlapParams.new()
		selfOp.FilterType = Enum.RaycastFilterType.Include
		selfOp.FilterDescendantsInstances = { part }
		selfOp.RespectCanCollide = false
		selfOp.MaxParts = 1
		hitsAt = function(lc: Vector3, h: number): boolean
			if not self:_cubeHitsPart(lc, h) then return false end
			shapeProbes += 1
			local edge = h * 2
			probe.Size = Vector3.new(edge, edge, edge)
			probe.CFrame = self.cf * CFrame.new(lc)
			return #workspace:GetPartsInPart(probe, selfOp) > 0
		end
		-- Overlap is boolean, so containment inside a non-box shape cannot be
		-- proven cheaply. Never claim "full": descend to the leaf and let the
		-- eight-children collapse rebuild the big nodes from below.
		fullAt = function() return false end
	end

	local seamProbes = 0
	local seamAt
	if #others == 0 then
		seamAt = function() return false end
	else
		local op = OverlapParams.new()
		op.FilterType = Enum.RaycastFilterType.Include
		op.FilterDescendantsInstances = others
		op.RespectCanCollide = false
		seamAt = function(lc: Vector3, h: number): boolean
			seamProbes += 1
			local edge = h * 2 + pad
			probe.Size = Vector3.new(edge, edge, edge)
			probe.CFrame = self.cf * CFrame.new(lc)
			return #workspace:GetPartsInPart(probe, op) > 0
		end
	end

	self:_build(self.root, self.center, self.half, self.maxDepth, seamAt, hitsAt, fullAt)
	probe:Destroy()
	self.probeCount = seamProbes + shapeProbes
	self.seamProbes = seamProbes
	self.shapeProbes = shapeProbes
	return self
end

function SVOLocal:forEachNode(fn)
	local function rec(node, lc: Vector3, h: number)
		if node.solid then
			fn(self.cf * CFrame.new(lc), h * 2, node.seam == true)
			return
		end
		local ch = node.children
		if not ch then return end
		local q = h * 0.5
		for i = 0, 7 do
			local c = ch[i]
			if c then rec(c, lc + OFF[i] * q, q) end
		end
	end
	rec(self.root, self.center, self.half)
end

function SVOLocal:stats()
	local nodes, seam, vol, seamVol = 0, 0, 0, 0
	local minE, maxE = math.huge, 0
	self:forEachNode(function(_, edge, isSeam)
		nodes += 1
		vol += edge ^ 3
		if isSeam then
			seam += 1
			seamVol += edge ^ 3
		end
		minE = math.min(minE, edge)
		maxE = math.max(maxE, edge)
	end)
	return {
		nodes = nodes,
		seamNodes = seam,
		volume = vol,
		seamVolume = seamVol,
		trueVolume = self.size.X * self.size.Y * self.size.Z,
		minNode = (minE == math.huge) and 0 or minE,
		maxNode = maxE,
		probes = self.probeCount or 0,
		shapeProbes = self.shapeProbes or 0,
		exact = self.exact,
	}
end

function SVOLocal.fromParts(parts: {BasePart}, leaf: number, contactPad: number?)
	leaf = leaf or 1
	local trees = {}
	local totals = {
		parts = 0, nodes = 0, seamNodes = 0,
		volume = 0, seamVolume = 0, trueVolume = 0, probes = 0,
		shapeProbes = 0, preciseParts = 0,
	}
	for _, part in ipairs(parts) do
		local others = SVOLocal.neighbours(part, parts, leaf)
		local t = SVOLocal.forPart(part, others, leaf, contactPad)
		trees[part] = t
		local s = t:stats()
		totals.parts += 1
		totals.nodes += s.nodes
		totals.seamNodes += s.seamNodes
		totals.volume += s.volume
		totals.seamVolume += s.seamVolume
		totals.trueVolume += s.trueVolume
		totals.probes += s.probes
		totals.shapeProbes += s.shapeProbes
		if not s.exact then totals.preciseParts += 1 end
	end
	return trees, totals
end

return SVOLocal
