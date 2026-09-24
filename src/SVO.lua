--!strict
-- NVGN.SVO — compressing Sparse Voxel Octree
--
-- Node encoding (plain tables, bounds derived during traversal):
--   node.solid == true      -> node is entirely solid (a leaf); no children.
--   node.children ~= nil     -> internal node; children[0..7] are octants (may be nil = empty).
--   neither                  -> empty node.
--
-- Octant index bits: 1=+X, 2=+Y, 4=+Z (relative to node center).
--
-- Build strategy: rasterize each part's oriented bounding box (OBB) into the tree.
-- A node fully inside an OBB collapses to a single solid leaf (the "sparse" win);
-- a node straddling the OBB surface subdivides down to leafSize. This is conservative:
-- any leaf the OBB touches at max depth is marked solid.

local SVO = {}
SVO.__index = SVO

-- ---- geometry helpers -------------------------------------------------------

-- World-space AABB half-extents of an OBB (cf, size).
local function worldAABBHalf(cf: CFrame, size: Vector3): Vector3
	local e = size * 0.5
	local rx, uy, lz = cf.RightVector, cf.UpVector, cf.LookVector
	return Vector3.new(
		math.abs(rx.X)*e.X + math.abs(uy.X)*e.Y + math.abs(lz.X)*e.Z,
		math.abs(rx.Y)*e.X + math.abs(uy.Y)*e.Y + math.abs(lz.Y)*e.Z,
		math.abs(rx.Z)*e.X + math.abs(uy.Z)*e.Y + math.abs(lz.Z)*e.Z
	)
end

-- Build an OBB descriptor from a part.
function SVO.obbFromPart(part: BasePart)
	local cf = part.CFrame
	local s = part.Size * 0.5
	return {
		c = cf.Position,
		ax = { cf.RightVector, cf.UpVector, cf.LookVector },
		e = { s.X, s.Y, s.Z },
	}
end

-- Is a world point inside the OBB?
local function pointInOBB(p: Vector3, o): boolean
	local d = p - o.c
	for i = 1, 3 do
		if math.abs(d:Dot(o.ax[i])) > o.e[i] + 1e-4 then return false end
	end
	return true
end

-- Are all 8 corners of an axis-aligned cube (center nc, half nh) inside the OBB?
-- Sufficient for "cube fully contained" since the OBB is convex.
local function cubeInsideOBB(nc: Vector3, nh: number, o): boolean
	for cx = -1, 1, 2 do
		for cy = -1, 1, 2 do
			for cz = -1, 1, 2 do
				if not pointInOBB(nc + Vector3.new(cx*nh, cy*nh, cz*nh), o) then
					return false
				end
			end
		end
	end
	return true
end

-- Separating Axis Test: does the OBB intersect the axis-aligned cube (center nc, half nh)?
local function obbHitsCube(o, nc: Vector3, nh: number): boolean
	local t = o.c - nc
	local A = o.ax
	local e = o.e
	-- Precompute |dot(worldAxis, obbAxis)| matrix and the absolute t.
	local function test(Lx: number, Ly: number, Lz: number): boolean
		-- returns true if L is a separating axis
		local len2 = Lx*Lx + Ly*Ly + Lz*Lz
		if len2 < 1e-8 then return false end -- degenerate (parallel axes) -> skip
		-- projection radius of the cube (axis-aligned, half nh on each world axis)
		local rCube = nh * (math.abs(Lx) + math.abs(Ly) + math.abs(Lz))
		-- projection radius of the OBB
		local rObb = 0
		for i = 1, 3 do
			local a = A[i]
			rObb = rObb + e[i] * math.abs(a.X*Lx + a.Y*Ly + a.Z*Lz)
		end
		local sep = math.abs(t.X*Lx + t.Y*Ly + t.Z*Lz)
		return sep > rCube + rObb
	end
	-- 3 world axes
	if test(1,0,0) or test(0,1,0) or test(0,0,1) then return false end
	-- 3 OBB axes
	for i = 1, 3 do
		local a = A[i]
		if test(a.X, a.Y, a.Z) then return false end
	end
	-- 9 cross products (worldAxis x obbAxis)
	local world = { Vector3.new(1,0,0), Vector3.new(0,1,0), Vector3.new(0,0,1) }
	for i = 1, 3 do
		for j = 1, 3 do
			local c = world[i]:Cross(A[j])
			if test(c.X, c.Y, c.Z) then return false end
		end
	end
	return true
end

-- ---- tree core --------------------------------------------------------------

local OFF = {} -- octant -> unit offset (filled below)
for i = 0, 7 do
	OFF[i] = Vector3.new(
		(bit32.band(i,1) ~= 0) and 1 or -1,
		(bit32.band(i,2) ~= 0) and 1 or -1,
		(bit32.band(i,4) ~= 0) and 1 or -1
	)
end

local function markSolid(node)
	node.children = nil
	node.solid = true
end

-- collapse an internal node whose 8 octants are all solid
local function tryCollapse(node)
	local ch = node.children
	if not ch then return end
	for i = 0, 7 do
		local c = ch[i]
		if not (c and c.solid) then return end
	end
	node.children = nil
	node.solid = true
end

function SVO.new(center: Vector3, half: number, leaf: number)
	local self = setmetatable({}, SVO)
	self.center = center
	self.half = half                                   -- half the root cube edge
	self.leaf = leaf
	self.maxDepth = math.max(0, math.ceil(math.log(2*half/leaf) / math.log(2)))
	self.root = {}
	return self
end

function SVO:_insert(node, nc: Vector3, nh: number, depth: number, o)
	if node.solid then return end
	if not obbHitsCube(o, nc, nh) then return end
	if cubeInsideOBB(nc, nh, o) then
		markSolid(node)
		return
	end
	if depth == 0 then
		markSolid(node) -- conservative: OBB touches this leaf
		return
	end
	node.children = node.children or {}
	local ch = nh * 0.5
	for i = 0, 7 do
		local cc = nc + OFF[i] * ch
		local child = node.children[i] or {}
		node.children[i] = child
		self:_insert(child, cc, ch, depth - 1, o)
		if not child.solid and not child.children then
			node.children[i] = nil -- prune empties, keep it sparse
		end
	end
	tryCollapse(node)
end

function SVO:insertOBB(o)
	self:_insert(self.root, self.center, self.half, self.maxDepth, o)
end

-- EXACT AXIS-ALIGNED INSERT, for boxes on the tree's own grid (terrain voxels).
-- `_insert` is conservative -- a cell merely TOUCHING an oriented box goes
-- solid, the right call for a tilted part -- but a grid-aligned box then paints
-- the layer of empty cells resting on it: every terrain surface grew a phantom
-- stud and its floor was found in the wrong place, or not at all (23% of the
-- islands' open ground). Here a cell is solid only if it OVERLAPS the box.
local AABB_EPS = 1e-4
function SVO:_insertAABB(node, nc: Vector3, nh: number, depth: number, lo: Vector3, hi: Vector3)
	if node.solid then return end
	local nlo, nhi = nc - Vector3.one * nh, nc + Vector3.one * nh
	if nlo.X >= hi.X - AABB_EPS or nhi.X <= lo.X + AABB_EPS
		or nlo.Y >= hi.Y - AABB_EPS or nhi.Y <= lo.Y + AABB_EPS
		or nlo.Z >= hi.Z - AABB_EPS or nhi.Z <= lo.Z + AABB_EPS then
		return
	end
	if (nlo.X >= lo.X - AABB_EPS and nhi.X <= hi.X + AABB_EPS
		and nlo.Y >= lo.Y - AABB_EPS and nhi.Y <= hi.Y + AABB_EPS
		and nlo.Z >= lo.Z - AABB_EPS and nhi.Z <= hi.Z + AABB_EPS) or depth == 0 then
		markSolid(node)
		return
	end
	node.children = node.children or {}
	local ch = nh * 0.5
	for i = 0, 7 do
		local cc = nc + OFF[i] * ch
		local child = node.children[i] or {}
		node.children[i] = child
		self:_insertAABB(child, cc, ch, depth - 1, lo, hi)
		if not child.solid and not child.children then
			node.children[i] = nil
		end
	end
	tryCollapse(node)
end

function SVO:insertAABB(lo: Vector3, hi: Vector3)
	self:_insertAABB(self.root, self.center, self.half, self.maxDepth, lo, hi)
end

function SVO:insertPart(part: BasePart)
	self:insertOBB(SVO.obbFromPart(part))
end

-- Point query: is world point p inside solid space?
function SVO:isSolid(p: Vector3): boolean
	local node, nc, nh = self.root, self.center, self.half
	while true do
		if node.solid then return true end
		if not node.children then return false end
		local i = 0
		if p.X >= nc.X then i = i + 1 end
		if p.Y >= nc.Y then i = i + 2 end
		if p.Z >= nc.Z then i = i + 4 end
		local child = node.children[i]
		if not child then return false end
		nh = nh * 0.5
		nc = nc + OFF[i] * nh
		node = child
	end
end

-- Visit every solid leaf: fn(centerVec3, half)
function SVO:forEachSolidLeaf(fn)
	local function rec(node, nc: Vector3, nh: number)
		if node.solid then fn(nc, nh); return end
		local ch = node.children
		if not ch then return end
		local h = nh * 0.5
		for i = 0, 7 do
			local c = ch[i]
			if c then rec(c, nc + OFF[i] * h, h) end
		end
	end
	rec(self.root, self.center, self.half)
end

-- Stats for validation.
function SVO:stats()
	local solidLeaves, internal, minLeaf, maxLeaf = 0, 0, math.huge, 0
	local function rec(node, nh)
		if node.solid then
			solidLeaves += 1
			minLeaf = math.min(minLeaf, nh*2)
			maxLeaf = math.max(maxLeaf, nh*2)
			return
		end
		if not node.children then return end
		internal += 1
		for i = 0, 7 do
			local c = node.children[i]
			if c then rec(c, nh*0.5) end
		end
	end
	rec(self.root, self.half)
	return {
		solidLeaves = solidLeaves,
		internalNodes = internal,
		maxDepth = self.maxDepth,
		minLeafSize = (minLeaf == math.huge) and 0 or minLeaf,
		maxSolidNodeSize = maxLeaf,
		leaf = self.leaf,
	}
end

-- ---- precise (real-geometry) insertion --------------------------------------

-- A part whose collision geometry is a true box can use the fast OBB path.
-- Unions, MeshParts, wedges, cylinders, etc. must be voxelized against their
-- REAL collision geometry, or their bounding box fills the octree with phantom
-- solid (e.g. the air inside an arch union).
function SVO.isBlockPart(part: BasePart): boolean
	local ok, shape = pcall(function() return (part :: any).Shape end)
	if ok and shape ~= nil then
		return shape == Enum.PartType.Block
	end
	return false -- no Shape property (Union/MeshPart/WedgePart/...) -> not a box
end

function SVO:_insertPrecise(node, nc: Vector3, nh: number, depth: number, overlaps)
	if node.solid then return end
	if not overlaps(nc, nh) then return end
	if depth == 0 then
		markSolid(node) -- real geometry touches this leaf
		return
	end
	node.children = node.children or {}
	local ch = nh * 0.5
	for i = 0, 7 do
		local cc = nc + OFF[i] * ch
		local child = node.children[i] or {}
		node.children[i] = child
		self:_insertPrecise(child, cc, ch, depth - 1, overlaps)
		if not child.solid and not child.children then
			node.children[i] = nil
		end
	end
	tryCollapse(node)
end

-- Voxelize a part against its real collision geometry using GetPartsInPart.
-- Descends the octree only where the part's geometry actually overlaps a node,
-- so concave shapes (arches, holes) are represented correctly.
-- Probes between yields when a caller opts into progress. Small enough that one
-- chunk is a frame's worth of work, large enough that the yield is not the cost.
SVO.probeBudget = 2000
SVO.maxProbe = 2048 -- studs; Roblox's largest part size

-- WHY THIS ONE CALL CAN WEDGE STUDIO, and what was done about it.
--
-- The descent has to reach leaf size everywhere the geometry is, interior
-- included, so the cost scales with the part's VOLUME. There is no escape from
-- that: SVOLocal settled the same question and wrote it down -- containment
-- inside a non-box shape cannot be proven by a boolean overlap query, so it
-- never claims "full" either -- and the point-probe substitute is ruled out by
-- GetPartsInPart's measured ~46% miss rate on solids probed at their own centre.
--
-- So the work is real and only the SHAPE of it can change. case6 has non-block
-- parts of 146,000 cubic studs and this call, having no yield in it, blocked
-- Studio's main thread for 45 minutes with nothing to show; case3's largest is
-- nowhere near, which is why it never showed up before. With the cheap reject
-- below in place, case6's whole SVO stage measures 53s (2026-09-16).
--
-- `onYield` makes the work divisible: the unit is a probe budget rather than a
-- spatial slice, which needs no geometry to divide and gives smooth progress.
-- Recursion is untouched -- Luau yields across depth without trouble, so the
-- post-order tryCollapse and empty-child pruning below still run exactly as they
-- did. Absent `onYield` the call is synchronous and behaves as it always has,
-- which is what keeps every existing caller safe in a non-yieldable context.
function SVO:insertPartPrecise(part: BasePart, worldRoot: WorldRoot?, onYield: (() -> ())?)
	local root = worldRoot or workspace
	local probe = Instance.new("Part")
	probe.Anchored = true
	probe.CanCollide = false
	probe.CanQuery = false
	probe.CanTouch = false
	probe.Transparency = 1
	probe.Parent = root :: any
	local op = OverlapParams.new()
	op.FilterType = Enum.RaycastFilterType.Include
	op.FilterDescendantsInstances = { part }
	op.RespectCanCollide = false
	-- the probe only ever asks WHETHER anything overlaps, and the filter is one
	-- part, so the rest of the result table is built and thrown away
	op.MaxParts = 1

	-- CHEAP REJECT BEFORE THE EXPENSIVE ONE, the same guard SVOLocal puts in
	-- front of its shape probes. Sound rather than heuristic: a part's Size box
	-- bounds its collision geometry, so no OBB overlap means no geometry and the
	-- probe could only have returned false. The tree is identical; the number of
	-- world queries is not. Without it the descent starts at the world root and
	-- probes all eight children at every level, so each part paid roughly eight
	-- queries a level for ten levels before reaching itself -- on case6 that is
	-- some 400,000 queries across the map that can only ever answer false.
	local obb = SVO.obbFromPart(part)

	local n = 0
	local function overlaps(nc: Vector3, nh: number): boolean
		if not obbHitsCube(obb, nc, nh) then return false end
		-- A PART IS AT MOST 2048 ON A SIDE, so a bigger probe is silently shrunk
		-- to the middle of the cube and misses anything off-centre. A 4096 root
		-- (a 2 km heightmap) dropped every mesh away from the map's centre. Too
		-- big to probe: the box test above decides, and the descent goes on.
		if nh * 2 > SVO.maxProbe then return true end
		if onYield then
			n += 1
			if n >= SVO.probeBudget then n = 0; onYield() end
		end
		probe.Size = Vector3.new(nh * 2, nh * 2, nh * 2)
		probe.CFrame = CFrame.new(nc)
		return #root:GetPartsInPart(probe, op) > 0
	end
	self:_insertPrecise(self.root, self.center, self.half, self.maxDepth, overlaps)
	probe:Destroy()
end

-- ---- convenience build ------------------------------------------------------

-- Build an SVO from a list of parts. margin pads the world bounds.
-- Block parts use fast OBB rasterization; non-block parts (unions/meshes/etc)
-- are voxelized against their real collision geometry.
-- `opts.onProgress(done, total)` is called as each part finishes AND at every
-- probe-budget yield inside a part, and may yield. Supplying it makes this
-- function yield, so call it from a coroutine; omit it and the build is
-- synchronous exactly as before. See insertPartPrecise for why a big map needs
-- this to be divisible at all.
-- opts.boxes: axis-aligned solid boxes { min, max } inserted after the parts
-- (terrain voxels, one vertical run per box). opts.align: snap the root's lower
-- corner to a multiple of this, so boxes on that grid fill whole cells instead
-- of straddling them -- terrain's 4 stud voxels would otherwise shatter into
-- 1 stud leaves.
function SVO.fromParts(parts: {BasePart}, leaf: number, margin: number, opts: any?)
	local boxes = opts and opts.boxes or {}
	assert(#parts > 0 or #boxes > 0, "SVO.fromParts: no parts")
	local onProgress = opts and opts.onProgress
	local lo = Vector3.new(math.huge, math.huge, math.huge)
	local hi = -lo
	for _, part in ipairs(parts) do
		local h = worldAABBHalf(part.CFrame, part.Size)
		lo = lo:Min(part.Position - h)
		hi = hi:Max(part.Position + h)
	end
	for _, b in ipairs(boxes) do
		lo = lo:Min(b.min)
		hi = hi:Max(b.max)
	end
	lo -= Vector3.new(margin, margin, margin)
	hi += Vector3.new(margin, margin, margin)
	local align = opts and opts.align
	if align then
		lo = Vector3.new(math.floor(lo.X / align) * align, math.floor(lo.Y / align) * align, math.floor(lo.Z / align) * align)
	end
	local extent = hi - lo
	local maxE = math.max(extent.X, extent.Y, extent.Z)
	-- root edge = leaf * 2^depth, big enough to hold maxE
	local depth = math.max(0, math.ceil(math.log(maxE/leaf) / math.log(2))) -- luau: log base via division
	local rootEdge = leaf * (2 ^ depth)
	-- aligned: the root's lower corner IS `lo`, so every cell edge lands on the grid
	local center = if align then lo + Vector3.one * (rootEdge * 0.5) else (lo + hi) * 0.5
	local tree = SVO.new(center, rootEdge * 0.5, leaf)
	local onYield = onProgress and function() onProgress(nil, #parts) end
	for i, part in ipairs(parts) do
		if SVO.isBlockPart(part) then
			tree:insertPart(part)                          -- fast OBB path
		else
			tree:insertPartPrecise(part, nil, onYield)     -- real-geometry path
		end
		if onProgress then onProgress(i, #parts) end
	end
	for i, b in ipairs(boxes) do
		tree:insertAABB(b.min, b.max)
		if onProgress and i % 500 == 0 then onProgress(nil, #boxes) end
	end
	return tree
end

return SVO
