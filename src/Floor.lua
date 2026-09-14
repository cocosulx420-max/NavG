--!strict
-- NVGN.Floor — walkable surface extraction
--
-- Produces one surfel per 1-stud walkable cell. The SVO finds candidates (solid
-- voxels with empty space above); each candidate's top face is walked at 1-stud
-- resolution and a raycast onto the real part gives exact height and normal, so
-- ramps are smooth and a collapsed node covering several parts is sampled per
-- cell. Clearance is an overlap probe for embedded origins plus an upward
-- raycast to the ceiling, with terrain handled by its own ray pair. Only
-- walkable surfels are kept.

local SVO = require(script.Parent:WaitForChild("SVO"))

local Floor = {}

export type Surfel = {
	pos: Vector3,       -- exact surface position
	normal: Vector3,    -- surface normal
	slope: number,      -- degrees from world-up
	clearance: number,  -- studs of headroom above (capped; 0 = embedded/dead space)
	part: BasePart,     -- the part under this surfel
	-- A ClipRamp is the smooth surface authored over a staircase. Where one
	-- exists it IS the walkable surface and the steps beneath it are not; the
	-- boundary stage drops the risers wherever a clip surfel covers the cell.
	clip: boolean,
}

-- Clearance is baked, horizontal width is not: width is recoverable from the
-- navmesh boundary edges, vertical headroom is not.

export type Config = {
	leaf: number?, maxSlope: number?, agentHeight: number?,
	clearCap: number?, maxGroundFootprint: number?, minClearance: number?,
	root: Instance?, -- restrict the bake to this subtree (default: whole workspace)
	onProgress: ((number?, number) -> ())?, -- opt-in; makes the voxelization yield
}

local DEFAULT = {
	leaf = 1,                 -- SVO leaf size (studs)
	maxSlope = 65,            -- max walkable slope (deg); Cocosulx-tested
	agentHeight = 5,          -- reference stand height
	clearCap = 20,            -- clearance raycast cap
	maxGroundFootprint = 400, -- parts wider than this (baseplate) are excluded
	minClearance = 1.5,       -- headroom below this is dead space (crawl floor)
}

local UP = Vector3.new(0, 1, 0)

-- Columns between yields in Floor.extract when a caller opts into progress.
-- At the measured ~264us a column this is roughly a frame's worth of work.
Floor.columnBudget = 250

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

local function cellKey(x: number, z: number): string
	return string.format("%d:%d", math.floor(x), math.floor(z))
end
Floor.cellKey = cellKey

-- Default world part filter: collidable, not terrain, not character,
-- not a huge flat ground slab (handled analytically elsewhere).
function Floor.gatherParts(cfg: Config?): {BasePart}
	local c = merged(cfg)
	local out = {}
	-- `root` scopes the bake to one model. A test map and the real map usually
	-- share a place, and baking the whole workspace to look at one of them wastes
	-- the only expensive stage in the pipeline.
	for _, d in ipairs((c.root or workspace):GetDescendants()) do
		if d:IsA("BasePart") and d.CanCollide and d.ClassName ~= "Terrain" and not isCharacter(d) then
			if math.max(d.Size.X, d.Size.Z) <= c.maxGroundFootprint then
				table.insert(out, d)
			end
		end
	end
	return out
end

-- IS THERE ANY TERRAIN OVER THIS BAKE AT ALL?
--
-- Terrain is never walkable and is never in `parts`, but it still blocks headroom,
-- and an overlap query cannot see it -- so every clearance test in the pipeline
-- pays a terrain-only ray pair. That is two of the four rays a surviving column
-- costs here, and two of the five world queries a LocalGrid cell costs. On a map
-- with no terrain over it every one of those provably returns nil.
--
-- `Terrain.MaxExtents` CANNOT answer this. It reports the maximum POSSIBLE region
-- -- (-32000, -32000, -32000) to (32000, 32000, 32000) -- and reads identically on
-- an empty world. `ReadVoxels` at resolution 4 is the native voxel pitch, so an
-- all-Air result is a PROOF and not a sample. Measured on case6: 607926 voxels
-- over the town's bounding box, every one of them Air.
--
-- SCOPED TO THE BAKE ROOT, never global. A place usually holds several test maps
-- side by side, and terrain beside one of them would switch the saving off for
-- every other -- which is exactly the case here: case6 has none over it while the
-- place does have terrain elsewhere.
local TERRAIN_LIMIT = 32000
local VOXEL = 4
function Floor.hasTerrain(cfg: Config?): boolean
	local c = merged(cfg)
	local root = c.root
	-- A whole-workspace bake has no bounds worth taking, so assume the worst and
	-- keep casting: this may only ever REMOVE provably-empty queries.
	if not root then return true end

	local lo = Vector3.new(math.huge, math.huge, math.huge)
	local hi = -lo
	local any = false
	for _, d in ipairs(root:GetDescendants()) do
		if d:IsA("BasePart") then
			local cf, sz = d.CFrame, d.Size * 0.5
			local x, y, z = cf.RightVector, cf.UpVector, cf.LookVector
			local h = Vector3.new(
				math.abs(x.X) * sz.X + math.abs(y.X) * sz.Y + math.abs(z.X) * sz.Z,
				math.abs(x.Y) * sz.X + math.abs(y.Y) * sz.Y + math.abs(z.Y) * sz.Z,
				math.abs(x.Z) * sz.X + math.abs(y.Z) * sz.Y + math.abs(z.Z) * sz.Z)
			lo = lo:Min(d.Position - h); hi = hi:Max(d.Position + h); any = true
		end
	end
	if not any then return false end

	-- Reach as far as any clearance ray could, then snap out to the voxel grid.
	local pad = Vector3.new(c.clearCap, c.clearCap, c.clearCap)
	lo -= pad; hi += pad
	local function clamp(v: Vector3, f): Vector3
		return Vector3.new(
			math.clamp(f(v.X / VOXEL) * VOXEL, -TERRAIN_LIMIT, TERRAIN_LIMIT),
			math.clamp(f(v.Y / VOXEL) * VOXEL, -TERRAIN_LIMIT, TERRAIN_LIMIT),
			math.clamp(f(v.Z / VOXEL) * VOXEL, -TERRAIN_LIMIT, TERRAIN_LIMIT))
	end
	lo = clamp(lo, math.floor); hi = clamp(hi, math.ceil)

	-- Chunked so one call never approaches ReadVoxels' per-call voxel ceiling.
	local T = workspace.Terrain
	local SPAN, SPAN_Y = 128, 256
	for x = lo.X, hi.X - VOXEL, SPAN do
		for z = lo.Z, hi.Z - VOXEL, SPAN do
			for y = lo.Y, hi.Y - VOXEL, SPAN_Y do
				local a = Vector3.new(x, y, z)
				local b = Vector3.new(
					math.min(x + SPAN, hi.X), math.min(y + SPAN_Y, hi.Y), math.min(z + SPAN, hi.Z))
				if b.X > a.X and b.Y > a.Y and b.Z > a.Z then
					local ok, mats = pcall(function()
						return T:ReadVoxels(Region3.new(a, b):ExpandToGrid(VOXEL), VOXEL)
					end)
					-- A refused read is not a proof of emptiness, so it counts as terrain.
					if not ok then return true end
					for i = 1, #mats do
						local mi = mats[i]
						for j = 1, #mi do
							local mj = mi[j]
							for k = 1, #mj do
								if mj[k] ~= Enum.Material.Air then return true end
							end
						end
					end
				end
			end
		end
	end
	return false
end

-- A FILTER LIST IS SCANNED PER QUERY, SO ITS LENGTH IS THE COST OF THE QUERY.
--
-- Measured on case6, one raycast against an Include filter: 0.95us with 1 entry
-- in the list, 1.76us at 100, 5.5us at 500, 169us at 2000, 527us at 18197. Every
-- hot query in this pipeline filtered on the full part list, and that one number
-- was most of the bake -- Floor.extract's own comment already recorded the
-- symptom without naming the cause ("about 264 microseconds" a column, 1,017,217
-- of them, 268 seconds). The geometry in a column is a fraction of a microsecond.
--
-- So filter by the ROOT, which is ONE entry, and re-establish exactness in Lua.
-- `RespectCanCollide` reproduces gatherParts' CanCollide rule for free; what is
-- left is the handful the root admits and the gather rejects -- on case6 exactly
-- 49 of 15821, being 44 character parts and 5 oversize slabs.
--
-- `cast` re-casts from the SAME origin with the rejected instances excluded,
-- rather than nudging the origin past them. Nudging cost 18 of case6's 298549
-- surfels: a decoration lying flush on a floor puts a non-bake surface and a bake
-- surface at the same point, and stepping even 1e-3 past the first skips the
-- second. Excluding by identity cannot skip anything, and the RaycastResult comes
-- back untouched, so `Distance` needs no repair.
--
-- Overlap queries need none of this -- they return a LIST, so exactness is
-- restored by skipping what is not in `set`.
-- ONLY WORTH IT ON A LONG LIST, and the crossover was measured rather than
-- guessed. Under a few hundred entries the Include list is already ~1-5us and the
-- root filter's membership tests and re-casts cost more than they save: case5,
-- 155 parts, went 7.76s -> 17.68s on the root filter, while case6's 15772 parts
-- are the case the whole thing exists for. So a short list keeps the exact filter
-- it always had and `cast` is a plain raycast; `set` is still returned, and the
-- membership guards at the call sites stay in, because with an Include list of
-- `parts` every candidate is in `set` anyway and the lookup is free.
Floor.bigFilter = 1000

function Floor.bakeFilter(parts: {BasePart}, root: Instance?)
	local set: { [Instance]: boolean } = {}
	for _, p in ipairs(parts) do set[p] = true end
	local scope = root or workspace
	local wide = #parts > Floor.bigFilter

	local rp = RaycastParams.new()
	rp.FilterType = Enum.RaycastFilterType.Include
	rp.FilterDescendantsInstances = wide and { scope } or parts
	rp.RespectCanCollide = wide
	local op = OverlapParams.new()
	op.FilterType = Enum.RaycastFilterType.Include
	op.FilterDescendantsInstances = wide and { scope } or parts
	op.RespectCanCollide = wide

	if not wide then
		return { set = set, rp = rp, op = op, wide = false,
			cast = function(origin: Vector3, dir: Vector3)
				return workspace:Raycast(origin, dir, rp)
			end }
	end

	-- Reused across the cold path so a rejected hit costs no allocation.
	local rpEx = RaycastParams.new()
	rpEx.FilterType = Enum.RaycastFilterType.Exclude
	rpEx.RespectCanCollide = true

	local function cast(origin: Vector3, dir: Vector3)
		local res = workspace:Raycast(origin, dir, rp)
		if not res then return nil end
		if set[res.Instance] then return res end
		-- Cold: the nearest thing in front of us is not part of the bake. Exclude it
		-- by IDENTITY and ask again from the same origin, so a bake surface sharing
		-- that exact point is still found. Exclude mode also admits parts outside
		-- the root, which are rejected the same way and converge out.
		local ex = { res.Instance }
		for _ = 1, 32 do
			rpEx.FilterDescendantsInstances = ex
			res = workspace:Raycast(origin, dir, rpEx)
			if not res then return nil end
			if set[res.Instance] then return res end
			ex[#ex + 1] = res.Instance
		end
		return nil
	end

	return { set = set, rp = rp, op = op, cast = cast, wide = true }
end

-- Extract surfels from a prebuilt SVO over `parts`.
function Floor.extract(parts: {BasePart}, tree: any, cfg: Config?)
	local c = merged(cfg)

	-- See Floor.bakeFilter: the filter list length IS the cost of the query.
	local bf = Floor.bakeFilter(parts, c.root)
	local bake, rp, castBake = bf.set, bf.rp, bf.cast

	-- Embedded-origin probe: a thin invisible part spanning [0.1, minClearance]
	-- above each candidate, tested with precise GetPartsInPart (see clearance
	-- note below for why raycasts cannot do this job).
	local probe = Instance.new("Part")
	probe.Name = "NVGN_ClearProbe"
	probe.Size = Vector3.new(0.05, c.minClearance - 0.1, 0.05)
	probe.Anchored = true; probe.CanCollide = false; probe.CanQuery = false; probe.CanTouch = false
	probe.Transparency = 1
	probe.Parent = workspace
	local op = bf.op
	-- nil when the bake root has no terrain over it, which makes every terrain
	-- cast below a no-op rather than a query. See Floor.hasTerrain.
	local rpTerrain = nil
	if Floor.hasTerrain(c) then
		rpTerrain = RaycastParams.new()
		rpTerrain.FilterType = Enum.RaycastFilterType.Include
		rpTerrain.FilterDescendantsInstances = { workspace.Terrain }
	end

	local surfels: {Surfel} = {}
	local index: { [string]: {Surfel} } = {}

	-- SAME OPT-IN YIELD AS THE VOXELIZATION, for the same reason and measured the
	-- same way: this walks one column per stud over every solid leaf's top face
	-- and spends four world queries on each, about 264 microseconds. case3's
	-- thousands of columns are nothing; case6 has 1,017,217 of them, which is 269
	-- seconds in a single call that cannot be interrupted or watched.
	--
	-- Absent `onProgress` nothing changes -- no yield, no counter cost worth
	-- naming, and the surfels come out in the same order either way.
	local onProgress = c.onProgress
	local cols = 0

	tree:forEachSolidLeaf(function(ctr: Vector3, h: number)
		local edge = 2 * h
		local top = ctr.Y + h
		for i = 0, edge - 1 do
			for j = 0, edge - 1 do
				if onProgress then
					cols += 1
					if cols % Floor.columnBudget == 0 then onProgress(cols, nil) end
				end
				local cx = ctr.X - h + 0.5 + i
				local cz = ctr.Z - h + 0.5 + j
				if tree:isSolid(Vector3.new(cx, top + 0.5, cz)) then continue end
				local res = castBake(Vector3.new(cx, top + 1, cz), Vector3.new(0, -(edge + 2), 0))
				if not res then continue end
				local n = res.Normal
				local slope = math.deg(math.acos(math.clamp(n:Dot(UP), -1, 1)))
				local isClip = res.Instance.Name:find("ClipRamp") ~= nil
				if not ((slope <= c.maxSlope) or isClip) then continue end
				-- Clearance. A raycast NEVER hits a part its origin is inside, so
				-- ray logic alone cannot detect an embedded origin (wall flush on
				-- the floor, buried overlap region, curved union). Precise overlap
				-- probe first: any foreign solid crossing [0.1, minClearance] above
				-- the surface means true headroom < minClearance -> clearance 0
				-- (surfel kept, truthful for the global index). Host excluded; a
				-- non-convex host's self-overhang is covered by the SVO empty-above
				-- guard at voxel scale. A clear probe guarantees the up-ray origin
				-- is outside every collider, making its distance exact.
				local clearance
				probe.CFrame = CFrame.new(res.Position + Vector3.new(0, 0.1 + (c.minClearance - 0.1) * 0.5, 0))
				local blocked = false
				for _, hit in ipairs(workspace:GetPartsInPart(probe, op)) do
					if hit ~= res.Instance and bake[hit] then blocked = true; break end
				end
				if blocked then
					clearance = 0
				else
					local upRes = castBake(res.Position + Vector3.new(0, 0.15, 0), Vector3.new(0, c.clearCap, 0))
					clearance = upRes and upRes.Distance or c.clearCap
					-- Terrain is not in `parts` (never walkable) but still blocks
					-- headroom; overlap queries are parts-only, so use a terrain-only
					-- ray pair (down-ray catches embedded origin under a blob).
					if rpTerrain then
						local tUp = workspace:Raycast(res.Position + Vector3.new(0, 0.15, 0), Vector3.new(0, c.clearCap, 0), rpTerrain)
						if tUp then
							clearance = math.min(clearance, tUp.Distance)
						elseif workspace:Raycast(res.Position + Vector3.new(0, c.clearCap, 0), Vector3.new(0, -(c.clearCap - 0.25), 0), rpTerrain) then
							clearance = 0
						end
					end
				end
				local surfel: Surfel = {
					pos = res.Position, normal = n, slope = slope,
					clearance = clearance, part = res.Instance, clip = isClip,
				}
				surfels[#surfels + 1] = surfel
				local key = cellKey(cx, cz)
				local bucket = index[key]
				if not bucket then bucket = {}; index[key] = bucket end
				bucket[#bucket + 1] = surfel
			end
		end
	end)
	probe:Destroy()

	-- Carried so LocalGrid does not have to re-derive it.
	return { surfels = surfels, index = index, config = c, noTerrain = rpTerrain == nil }
end

-- Convenience one-call bake: gather parts, build SVO, extract floor.
-- Returns floorData, tree, parts.
function Floor.build(cfg: Config?)
	local c = merged(cfg)
	local parts = Floor.gatherParts(c)
	-- onProgress is opt-in and makes the voxelization yield; a bake that does not
	-- set it is synchronous as before. See SVO.insertPartPrecise.
	local tree = SVO.fromParts(parts, c.leaf, 2,
		c.onProgress and { onProgress = c.onProgress } or nil)
	local data = Floor.extract(parts, tree, c)
	return data, tree, parts
end

return Floor
