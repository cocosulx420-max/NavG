--!strict
-- NVGN.SVOLocal — one octree per part, in THAT PART'S OWN frame.
--
-- Why: a world-aligned lattice staircases every rotated part, because the
-- part's faces cut its cells at an angle. Sampling a part on its own axes puts
-- its rim on whole lattice lines instead, so the part itself is exact.
--
-- The consequence that makes this cheap: a Block part FILLS its own root, so
-- the tree collapses to a single node and never subdivides. Resolution is spent
-- only where ANOTHER part cuts this one. The intersection test therefore drives
-- subdivision; it is not a labelling pass done afterwards.
--
-- The lattice is anchored at the part's own minimum corner in local space, so
-- that corner lands exactly on a lattice boundary (the same anchoring rule the
-- world SVO uses). Only the far faces can fall mid-cell, and only when a
-- dimension is not a whole multiple of leaf — that is the accepted staircase.
--
-- Node classes:
--   "solid" — inside this part, no other part present.
--   "seam"  — inside this part AND touching or overlapping another part.

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

-- World-space AABB half-extents of an OBB.
local function worldAABBHalf(cf: CFrame, size: Vector3): Vector3
	local e = size * 0.5
	local r, u, l = cf.RightVector, cf.UpVector, cf.LookVector
	return Vector3.new(
		math.abs(r.X)*e.X + math.abs(u.X)*e.Y + math.abs(l.X)*e.Z,
		math.abs(r.Y)*e.X + math.abs(u.Y)*e.Y + math.abs(l.Y)*e.Z,
		math.abs(r.Z)*e.X + math.abs(u.Z)*e.Y + math.abs(l.Z)*e.Z
	)
end

-- Broad phase: which of `all` could touch `part` at all? Expanded by `pad` so
-- parts that merely rest against each other still qualify.
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

-- ---- the per-part tree ------------------------------------------------------

-- Anchor one axis so the CELLS THE PART OCCUPIES are centred on the part.
--
-- When a dimension is a whole multiple of leaf this returns exactly -dim/2, so
-- it is identical to anchoring at the part's min corner and the clean cases stay
-- clean. Otherwise it splits the overhang evenly instead of dumping all of it on
-- the plus side. That matters most BELOW leaf size: a 0.38-thick panel in a
-- 1-stud cell used to sit flush with one face and 0.62 studs proud of the other,
-- so the voxels read as a slab beside the geometry rather than on it.
local function anchor(dim: number, leaf: number): number
	local n = math.max(1, math.ceil(dim / leaf - 1e-9))
	return -n * leaf * 0.5
end

function SVOLocal.new(part: BasePart, leaf: number)
	local self = setmetatable({}, SVOLocal)
	local s = part.Size
	local origin = Vector3.new(anchor(s.X, leaf), anchor(s.Y, leaf), anchor(s.Z, leaf))
	local maxE = math.max(s.X, s.Y, s.Z)
	local depth = math.max(0, math.ceil(math.log(maxE / leaf) / math.log(2)))
	local edge = leaf * (2 ^ depth)
	-- re-anchoring can push the far face out, so make sure the root still holds it
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

-- Does the axis-aligned local cube (lc, h) intersect the part's own local box?
-- Exact tangency does NOT count: a cube merely resting on the face is outside.
function SVOLocal:_cubeHitsPart(lc: Vector3, h: number): boolean
	local e = self.size * 0.5
	return math.abs(lc.X) < e.X + h - 1e-6
		and math.abs(lc.Y) < e.Y + h - 1e-6
		and math.abs(lc.Z) < e.Z + h - 1e-6
end

-- Is the cube wholly inside the part's own local box?
function SVOLocal:_cubeInsidePart(lc: Vector3, h: number): boolean
	local e = self.size * 0.5
	return math.abs(lc.X) + h <= e.X + 1e-6
		and math.abs(lc.Y) + h <= e.Y + 1e-6
		and math.abs(lc.Z) + h <= e.Z + 1e-6
end

function SVOLocal:_build(node, lc: Vector3, h: number, depth: number, seamAt): boolean
	if not self:_cubeHitsPart(lc, h) then return false end
	local seam = seamAt(lc, h)
	local full = self:_cubeInsidePart(lc, h)

	if depth == 0 then
		node.solid = true
		node.seam = seam
		return true
	end
	-- Interior of this part with nothing else present: nothing left to resolve.
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
		if self:_build(child, lc + OFF[i] * ch, ch, depth - 1, seamAt) then
			node.children[i] = child
			any = true
		end
	end
	if not any then
		node.children = nil
		return false
	end

	-- Collapse only when all eight children exist AND agree on both flags.
	-- Collapsing on `solid` alone would discard a seam marking.
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

-- Build the tree. `others` are the parts that may cut this one; `contactPad`
-- inflates the overlap probe so parts that merely REST against each other still
-- register as a seam (a leg standing on a plate touches, it does not overlap).
function SVOLocal.forPart(part: BasePart, others: {BasePart}, leaf: number, contactPad: number?)
	local self = SVOLocal.new(part, leaf)
	local pad = contactPad or 0.02

	if #others == 0 then
		-- Nothing can cut it, so no probes are needed -- but still build, or the
		-- root CUBE (which is larger than the part) would be reported as solid.
		self:_build(self.root, self.center, self.half, self.maxDepth, function() return false end)
		self.probeCount = 0
		return self
	end

	local probe = Instance.new("Part")
	probe.Anchored = true
	probe.CanCollide, probe.CanQuery, probe.CanTouch = false, false, false
	probe.Transparency = 1
	probe.Parent = workspace
	local op = OverlapParams.new()
	op.FilterType = Enum.RaycastFilterType.Include
	op.FilterDescendantsInstances = others
	op.RespectCanCollide = false

	local probes = 0
	local function seamAt(lc: Vector3, h: number): boolean
		probes += 1
		local edge = h * 2 + pad
		probe.Size = Vector3.new(edge, edge, edge)
		probe.CFrame = self.cf * CFrame.new(lc)
		return #workspace:GetPartsInPart(probe, op) > 0
	end

	self:_build(self.root, self.center, self.half, self.maxDepth, seamAt)
	probe:Destroy()
	self.probeCount = probes
	return self
end

-- Visit every solid node: fn(worldCFrame, edgeSize, isSeam)
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
	}
end

-- ---- whole-model build ------------------------------------------------------

-- Build one tree per part. Returns { [part] = tree }, and totals.
function SVOLocal.fromParts(parts: {BasePart}, leaf: number, contactPad: number?)
	leaf = leaf or 1
	local trees = {}
	local totals = {
		parts = 0, nodes = 0, seamNodes = 0,
		volume = 0, seamVolume = 0, trueVolume = 0, probes = 0,
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
	end
	return trees, totals
end

return SVOLocal
