--!strict

local Floor = require(script.Parent:WaitForChild("Floor"))

local LocalGrid = {}

export type Cell = {
	ui: number, vi: number,   -- integer lattice indices in the part's local frame
	pos: Vector3,             -- exact surface position (world)
	normal: Vector3,
	slope: number,            -- degrees from world-up
	clearance: number,        -- studs of vertical headroom (capped)
	cover: Instance?,
	-- Cell edge length in studs. Full cells carry the grid step; cells recovered
	-- by subdividing a dead cell carry step/2. Cells are NO LONGER UNIFORM, so
	-- anything that steps between cells must read this and not grid.step.
	size: number,
	sub: boolean?,            -- true => recovered from a subdivided dead cell
	pui: number?, pvi: number?, -- parent cell's lattice indices (subcells only)
	-- Set by classifyNodes. Bitmasks over DIR8, plus the booleans they imply.
	wallMask: number?,        -- directions with a surface standing above us
	dropMask: number?,        -- directions with nothing to stand on
	wall: boolean?,
	dropoff: boolean?,
	-- Set by Boundary's pass one: this node is where a staircase ended, so an
	-- edge begins here. Not a geometric property of the node on its own.
	edgeCorner: boolean?,
}

export type DeadCell = {
	ui: number, vi: number,
	pos: Vector3,
	killer: Instance?,
	size: number,
	sub: boolean?,
}

export type Grid = {
	part: BasePart,
	fallback: boolean,        -- true => world-aligned (non-block part)
	origin: Vector3?,         -- face corner (world); block grids only
	u: Vector3?, v: Vector3?, -- in-plane unit axes (world); block grids only
	n: Vector3?,              -- surface normal (world); block grids only
	center: Vector3?,         -- centre of the walkable face (world)
	uExt: number?, vExt: number?, -- half-extents along u and v
	step: number,
	cells: {Cell},
	index: { [string]: Cell },-- "ui:vi" -> cell (FULL cells only)
	-- Recovered subcells, keyed on the half-pitch lattice "2*ui+sx:2*vi+sy".
	-- Kept out of `index` so integer-lattice adjacency there stays meaningful.
	subIndex: { [string]: Cell },
	dead: {DeadCell},
	deadIndex: { [string]: DeadCell },
}

export type Config = {
	step: number?, maxSlope: number?, clearCap: number?, minClearance: number?,
	flushTol: number?, probeRadius: number?,
}

local DEFAULT = {
	step = 1,           -- local cell size (studs)
	maxSlope = 65,      -- max walkable slope (deg); Cocosulx-tested
	clearCap = 20,      -- clearance raycast cap
	minClearance = 1.5, -- below this a cell isn't standable floor (crawl minimum)
	-- How far a neighbouring surface may sit from where THIS grid's surface
	-- would continue, and still count as the same floor. Not a step height: a
	-- step up of any size is a wall now, and pathfinding deals with climbing it.
	-- It is only slack for authoring mismatch and raycast noise.
	flushTol = 0.5,
	-- How far from the expected neighbour position a foreign grid's cell may sit
	-- and still count as that neighbour. A neighbour on another part's grid is
	-- on a different lattice at a different angle, so it never lands on our
	-- sample point: the nearest cell of an arbitrarily placed lattice of pitch
	-- `step` can be up to step*sqrt(2)/2 ~= 0.707 away. Anything below that and
	-- a foreign floor reads as air, which turns every part join into a dropoff.
	probeRadius = 0.75,
}

local UP = Vector3.new(0, 1, 0)

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

local function isBlock(p: BasePart): boolean
	return p:IsA("Part") and p.Shape == Enum.PartType.Block
end

local function isClip(p: Instance): boolean
	return p.Name:find("ClipRamp") ~= nil
end

-- Group walkable surfels by the part beneath them.
local function groupByPart(surfels: {any}): { [BasePart]: {any} }
	local byPart: { [BasePart]: {any} } = {}
	for _, s in ipairs(surfels) do
		local b = byPart[s.part]
		if not b then b = {}; byPart[s.part] = b end
		b[#b + 1] = s
	end
	return byPart
end

-- Average the surfel normals to get the true walkable-face direction.
local function avgNormal(surfels: {any}): Vector3
	local s = Vector3.zero
	for _, sf in ipairs(surfels) do s += sf.normal end
	return (s.Magnitude > 1e-4) and s.Unit or Vector3.yAxis
end

local function topFace(part: BasePart, surfaceN: Vector3)
	local cf = part.CFrame
	local sz = part.Size
	local axes = {
		{ dir = cf.RightVector, ext = sz.X * 0.5 },
		{ dir = cf.UpVector,    ext = sz.Y * 0.5 },
		{ dir = cf.RightVector:Cross(cf.UpVector), ext = sz.Z * 0.5 }, -- local Z basis
	}
	local bi, best = 2, -math.huge
	for i, a in ipairs(axes) do
		local d = math.abs(a.dir:Dot(surfaceN))
		if d > best then best = d; bi = i end
	end
	local a = axes[bi]
	local n = (a.dir:Dot(surfaceN) >= 0) and a.dir or -a.dir
	local plane = {}
	for i, ax in ipairs(axes) do
		if i ~= bi then plane[#plane + 1] = ax end
	end
	return n, a.ext, plane[1], plane[2]
end

local function buildBlockGrid(part: BasePart, surfels: {any}, c: any, filterAll: RaycastParams, probe: BasePart, op: OverlapParams, rpTerrain: RaycastParams): Grid
	local n, nExt, ua, va = topFace(part, avgNormal(surfels))
	local u, uExt = ua.dir, ua.ext
	local v, vExt = va.dir, va.ext
	local surfaceCenter = part.Position + n * nExt
	local corner = surfaceCenter - u * uExt - v * vExt

	local rpPart = RaycastParams.new()
	rpPart.FilterType = Enum.RaycastFilterType.Include
	rpPart.FilterDescendantsInstances = { part }

	local grid: Grid = {
		part = part, fallback = false, origin = corner,
		u = u, v = v, n = n, step = c.step, cells = {}, index = {}, subIndex = {},
		dead = {}, deadIndex = {},
	    center = surfaceCenter, uExt = uExt, vExt = vExt,
	}

	local step = c.step
	local nu = math.max(1, math.floor(2 * uExt / step + 1e-6))
	local nv = math.max(1, math.floor(2 * vExt / step + 1e-6))
	local castH = 2 -- studs above the surface to start the (downward-along-normal) ray

	-- Evaluate ONE sample point on the face. `sp` is the sample's centre in the
	-- part's face plane. Returns "miss" (no surface / too steep, nothing to
	-- record), "dead" (something is there we cannot stand in) or "live", plus
	-- the resolved surface data.
	--
	-- Split out of the main loop so the subdivision pass re-tests a subcell with
	-- BYTE-FOR-BYTE the same rules as a full cell. If the two ever diverge, a
	-- recovered subcell stops being comparable to the cells around it.
	local function evalSample(sp: Vector3)
		local res = workspace:Raycast(sp + n * castH, -n * (castH + 0.5), rpPart)
		if not res then return "miss" end
		local slope = math.deg(math.acos(math.clamp(res.Normal:Dot(UP), -1, 1)))
		if not ((slope <= c.maxSlope) or isClip(part)) then return "miss" end
		probe.CFrame = CFrame.new(res.Position + UP * (0.1 + (c.minClearance - 0.1) * 0.5))
		for _, hit in ipairs(workspace:GetPartsInPart(probe, op)) do
			if hit ~= part then return "dead", res, slope, nil, hit end
		end
		local upRes = workspace:Raycast(res.Position + Vector3.new(0, 0.15, 0), UP * c.clearCap, filterAll)
		local clearance = upRes and upRes.Distance or c.clearCap
		local cover: Instance? = upRes and upRes.Instance or nil
		local tUp = workspace:Raycast(res.Position + Vector3.new(0, 0.15, 0), UP * c.clearCap, rpTerrain)
		if tUp then
			if tUp.Distance < clearance then
				clearance = tUp.Distance
				cover = workspace.Terrain
			end
		elseif workspace:Raycast(res.Position + UP * c.clearCap, -UP * (c.clearCap - 0.25), rpTerrain) then
			clearance = 0
			cover = workspace.Terrain
		end
		if clearance < c.minClearance then return "dead", res, slope, clearance, cover end
		return "live", res, slope, clearance, cover
	end

	-- Pass one: the full-pitch lattice, exactly as before.
	for iu = 0, nu - 1 do
		for iv = 0, nv - 1 do
			local sp = corner + u * ((iu + 0.5) * step) + v * ((iv + 0.5) * step)
			local status, res, slope, clearance, inst = evalSample(sp)
			if status == "miss" then continue end
			if status == "dead" then
				local d: DeadCell = {
					ui = iu, vi = iv, pos = res.Position, killer = inst, size = step,
				}
				grid.dead[#grid.dead + 1] = d
				grid.deadIndex[string.format("%d:%d", iu, iv)] = d
				continue
			end
			local cell: Cell = {
				ui = iu, vi = iv, pos = res.Position, normal = res.Normal,
				slope = slope, clearance = clearance, cover = inst, size = step,
			}
			grid.cells[#grid.cells + 1] = cell
			grid.index[string.format("%d:%d", iu, iv)] = cell
		end
	end

	-- Pass two: SUBDIVIDE EVERY DEAD CELL ONCE.
	--
	-- A dead cell means "the sample point at this cell's centre is inside
	-- something". At 1-stud pitch that verdict is all-or-nothing, which is
	-- exactly where the staircase comes from: a cell 10% buried in a skewed wall
	-- and a cell 90% buried are both simply gone, so the boundary can only ever
	-- land on the lattice.
	--
	-- Splitting each dead cell into four half-pitch samples and re-testing gives
	-- the boundary a second, finer place to land. The partially-buried cell
	-- recovers the quadrants that are actually clear; the fully-buried one
	-- recovers nothing and stays gone. The result is still a staircase -- it is
	-- a staircase at half the amplitude, with intermediate treads that say
	-- whether a step is one edge continuing or a genuinely new one.
	--
	-- The parent dead cell is REPLACED by its four children, so a dead cell in
	-- the output always describes solid at its own recorded size.
	local parents = grid.dead
	grid.dead, grid.deadIndex = {}, {}
	local half = step * 0.5
	for _, d in ipairs(parents) do
		for sx = 0, 1 do
			for sy = 0, 1 do
				local hu, hv = 2 * d.ui + sx, 2 * d.vi + sy
				local sp = corner
					+ u * ((d.ui + 0.25 + 0.5 * sx) * step)
					+ v * ((d.vi + 0.25 + 0.5 * sy) * step)
				local status, res, slope, clearance, inst = evalSample(sp)
				if status == "miss" then continue end
				local k = string.format("%d:%d", hu, hv)
				if status == "dead" then
					local sd: DeadCell = {
						ui = hu, vi = hv, pos = res.Position, killer = inst,
						size = half, sub = true,
					}
					grid.dead[#grid.dead + 1] = sd
					grid.deadIndex[k] = sd
					continue
				end
				local cell: Cell = {
					ui = hu, vi = hv, pos = res.Position, normal = res.Normal,
					slope = slope, clearance = clearance, cover = inst,
					size = half, sub = true, pui = d.ui, pvi = d.vi,
				}
				grid.cells[#grid.cells + 1] = cell
				grid.subIndex[k] = cell
			end
		end
	end

	return grid
end

local function buildFallbackGrid(part: BasePart, surfels: {any}, c: any): Grid
	local grid: Grid = {
		part = part, fallback = true, step = c.step, cells = {}, index = {},
		subIndex = {}, dead = {}, deadIndex = {},
	}
	for _, s in ipairs(surfels) do
		if s.clearance < c.minClearance then continue end
		local iu = math.floor(s.pos.X / c.step)
		local iv = math.floor(s.pos.Z / c.step)
		local cell: Cell = {
			ui = iu, vi = iv, pos = s.pos, normal = s.normal,
			slope = s.slope, clearance = s.clearance, cover = s.cover,
			size = c.step,
		}
		grid.cells[#grid.cells + 1] = cell
		grid.index[string.format("%d:%d", iu, iv)] = cell
	end
	return grid
end

-- The 8 local directions, starting east and going counter-clockwise.
local DIR8 = {
	{ 1, 0 }, { 1, 1 }, { 0, 1 }, { -1, 1 },
	{ -1, 0 }, { -1, -1 }, { 0, -1 }, { 1, -1 },
}

-- World XZ bucket, 1 stud, holding every cell and every dead cell so a
-- neighbour can be found without knowing which grid owns it.
local function buildWorldIndex(grids: any)
	local live: {[string]: {any}} = {}
	local dead: {[string]: {any}} = {}
	local function push(t, pos, v)
		local k = math.floor(pos.X) .. ":" .. math.floor(pos.Z)
		local b = t[k]
		if not b then b = {}; t[k] = b end
		b[#b + 1] = v
	end
	for part, g in pairs(grids) do
		for _, cell in ipairs(g.cells) do push(live, cell.pos, { cell = cell, part = part }) end
		for _, d in ipairs(g.dead) do push(dead, d.pos, { dead = d, part = part }) end
	end
	-- NOTE: the bucket is 1 stud regardless of cell size, and a half-pitch cell
	-- is strictly smaller, so a subcell never spans more buckets than a full one
	-- and the existing 3x3 bucket sweep still covers every candidate.
	return live, dead
end

-- Where the neighbour in local direction d would be, in world space. Block
-- grids step along their own face axes; fallback grids are world-aligned.
local function neighbourPos(g: Grid, cell: Cell, d: {number}): Vector3
	-- Step by THIS CELL's own pitch. A recovered subcell is half-pitch, and
	-- stepping a full stud from it would skip straight over its neighbour and
	-- read open floor as a dropoff.
	local sp = cell.size or g.step
	if not g.fallback and g.u and g.v then
		return cell.pos + g.u * (d[1] * sp) + g.v * (d[2] * sp)
	end
	return cell.pos + Vector3.new(d[1] * sp, 0, d[2] * sp)
end

-- Mark every cell with the directions in which it has a wall and the
-- directions in which it has air.
--
--   wall    -- something stands above us there (a surface higher than stepTol,
--             or a cell killed by cover overhead)
--   dropoff -- nothing to stand on there: no surface within a step, in any grid
--
-- A cell can be both: a ledge running along the foot of a wall is the ordinary
-- case. Neither means the floor simply continues, whether or not it continues
-- onto a different part.
function LocalGrid.classifyNodes(data: any, cfg: Config?)
	local c = merged(cfg)
	if data.config then
		c.step = data.config.step or c.step
		c.flushTol = (cfg and cfg.flushTol) or data.config.flushTol or c.flushTol
	end
	local live, dead = buildWorldIndex(data.grids)
	-- Match radius is PER PAIR, not per grid. probeRadius exists because a
	-- neighbouring grid's lattice never lands on our sample point, so the nearest
	-- cell can sit up to pitch*sqrt(2)/2 away. With mixed pitches the relevant
	-- pitch is the COARSER of the two: sizing off a half-pitch subcell alone
	-- would shrink the window below what a full neighbour needs and turn every
	-- sub-to-full join into a false dropoff.
	local function matchR2(a: number, b: number): number
		local r = c.probeRadius * math.max(a, b)
		return r * r
	end
	local tol = c.flushTol
	local nWall, nDrop, nBoth = 0, 0, 0

	for _, g in pairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			local wallMask, dropMask = 0, 0
			for bit, d in ipairs(DIR8) do
				local p = neighbourPos(g, cell, d)
				local bx, bz = math.floor(p.X), math.floor(p.Z)
				-- BELOW OUTRANKS ABOVE, and getting that backwards is what marked
				-- the rim of every raised platform as a wall. "Any surface higher
				-- than stepTol is a wall" cannot tell a riser you would bump into
				-- from a balcony three storeys up: measured, the surface found
				-- above those rim nodes was 9, 13, even 18 studs overhead, with
				-- open air the whole way down.
				--
				-- Live floor BELOW is the giveaway. If floor is visible down there
				-- the space is open and you would fall through it -- a dropoff, no
				-- matter what is overhead. A wall standing at that spot would have
				-- killed that floor, so it would not be live. So: floor first,
				-- then below, then above.
				local csize = cell.size or c.step
				local floor, above, below = false, false, false
				for ox = -1, 1 do
					for oz = -1, 1 do
						for _, e in ipairs(live[(bx + ox) .. ":" .. (bz + oz)] or {}) do
							local q = e.cell.pos
							local dx, dz = q.X - p.X, q.Z - p.Z
							if dx * dx + dz * dz <= matchR2(csize, e.cell.size or c.step) then
								-- MEASURED AGAINST WHERE THIS SURFACE WOULD CONTINUE,
								-- not against our own height. `p` lies on this grid's
								-- own plane, so on a tilted slab the next cell along
								-- is at dy = 0 however steep the slab is. Comparing to
								-- cell.pos.Y instead would make a 65-degree ramp's own
								-- cells read as walls and dropoffs the moment the
								-- tolerance dropped below its per-cell rise.
								local dy = q.Y - p.Y
								if math.abs(dy) <= tol then
									floor = true
								elseif dy > tol then
									above = true
								else
									below = true
								end
							end
						end
					end
				end
				if below then above = false end
				if not floor then
					-- a cell killed by something overhead is that something's wall
					if not above and not below then
						for ox = -1, 1 do
							for oz = -1, 1 do
								for _, e in ipairs(dead[(bx + ox) .. ":" .. (bz + oz)] or {}) do
									local q = e.dead.pos
									local dx, dz = q.X - p.X, q.Z - p.Z
									if dx * dx + dz * dz <= matchR2(csize, e.dead.size or c.step)
										and e.dead.killer
										and math.abs(q.Y - p.Y) <= tol then
										above = true
									end
								end
							end
						end
					end
					local m = bit32.lshift(1, bit - 1)
					if above then wallMask = bit32.bor(wallMask, m) else dropMask = bit32.bor(dropMask, m) end
				end
			end
			cell.wallMask, cell.dropMask = wallMask, dropMask
			cell.wall, cell.dropoff = wallMask ~= 0, dropMask ~= 0
			if cell.wall then nWall += 1 end
			if cell.dropoff then nDrop += 1 end
			if cell.wall and cell.dropoff then nBoth += 1 end
		end
	end

	data.stats.wallNodes, data.stats.dropNodes, data.stats.bothNodes = nWall, nDrop, nBoth
	return data
end

-- Build per-part local grids from an existing floor extraction.
function LocalGrid.fromFloor(floorData: any, parts: {BasePart}, cfg: Config?)
	local c = merged(cfg)
	local filterAll = RaycastParams.new()
	filterAll.FilterType = Enum.RaycastFilterType.Include
	filterAll.FilterDescendantsInstances = parts

	local probe = Instance.new("Part")
	probe.Name = "NVGN_ClearProbe"
	probe.Size = Vector3.new(0.05, c.minClearance - 0.1, 0.05)
	probe.Anchored = true; probe.CanCollide = false; probe.CanQuery = false; probe.CanTouch = false
	probe.Transparency = 1
	probe.Parent = workspace
	local op = OverlapParams.new()
	op.FilterType = Enum.RaycastFilterType.Include
	op.FilterDescendantsInstances = parts
	local rpTerrain = RaycastParams.new()
	rpTerrain.FilterType = Enum.RaycastFilterType.Include
	rpTerrain.FilterDescendantsInstances = { workspace.Terrain }

	local byPart = groupByPart(floorData.surfels)
	local grids: { [BasePart]: Grid } = {}
	local nBlock, nFallback, nCells, nDead = 0, 0, 0, 0
	local nSubCells, nSubDead = 0, 0
	for part, sfs in pairs(byPart) do
		local g: Grid
		if isBlock(part) then
			g = buildBlockGrid(part, sfs, c, filterAll, probe, op, rpTerrain)
			nBlock += 1
		else
			g = buildFallbackGrid(part, sfs, c)
			nFallback += 1
		end
		grids[part] = g
		nCells += #g.cells
		nDead += #g.dead
		for _, cell in ipairs(g.cells) do if cell.sub then nSubCells += 1 end end
		for _, d in ipairs(g.dead) do if d.sub then nSubDead += 1 end end
	end
	probe:Destroy()

	local data = {
		grids = grids, config = c,
		stats = {
			parts = nBlock + nFallback, block = nBlock, fallback = nFallback,
			cells = nCells, dead = nDead,
			-- Recovered by subdivision: cells that were solid at full pitch and
			-- turned out to be standable at half. `subDead` is the other half of
			-- the split -- quadrants that stayed solid.
			subCells = nSubCells, subDead = nSubDead,
		},
	}
	LocalGrid.classifyNodes(data, cfg)
	return data
end

-- Debug viz for classifyNodes: red = wall, blue = dropoff, purple = both,
-- grey = interior. Tiles are oriented to their grid like the main viz.
-- `opts.interior = false` omits the 91% of tiles that are neither wall nor
-- dropoff. Nothing here is transparent: a transparent part costs alpha blending
-- and a depth sort, and 46k of them is what made this viz lag when the opaque
-- one at the same tile count did not.
function LocalGrid.visualizeClasses(data: any, opts: any?, parent: Instance?)
	if typeof(opts) == "Instance" then parent = opts :: Instance; opts = nil end
	local o = opts or {}
	local showInterior = o.interior ~= false
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then
		dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root
	end
	local old = dbg:FindFirstChild("NodeClasses")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "NodeClasses"; folder.Parent = dbg

	local WALL = Color3.fromRGB(255, 70, 70)
	local DROP = Color3.fromRGB(70, 160, 255)
	local BOTH = Color3.fromRGB(220, 90, 255)
	local PLAIN = Color3.fromRGB(58, 58, 66)
	local step = data.config.step
	for _, g in pairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			local col, w = PLAIN, 0.55
			if cell.wall and cell.dropoff then col, w = BOTH, 0.9
			elseif cell.wall then col, w = WALL, 0.9
			elseif cell.dropoff then col, w = DROP, 0.9 end
			-- A corner keeps its class hue and goes darker, so it reads as "this
			-- kind of node, where a staircase ended" rather than as a new class.
			-- Only appears after a bake, since Boundary sets it.
			if cell.edgeCorner then
				col = Color3.new(col.R * 0.28, col.G * 0.28, col.B * 0.28)
			end
			if col == PLAIN and not showInterior then continue end
			local csize = cell.size or step
			local dot = Instance.new("Part")
			dot.Anchored = true; dot.CanCollide = false; dot.CanQuery = false; dot.CanTouch = false
			dot.Material = Enum.Material.SmoothPlastic
			dot.Color = col
			dot.Size = Vector3.new(w * csize, 0.08, w * csize)
			if not g.fallback and g.n and g.u then
				dot.CFrame = CFrame.fromMatrix(cell.pos + Vector3.new(0, 0.12, 0), g.u, g.n)
			else
				dot.CFrame = CFrame.new(cell.pos + Vector3.new(0, 0.12, 0))
			end
			dot.Name = string.format("w%d_d%d", cell.wallMask or 0, cell.dropMask or 0)
			dot.Parent = folder
		end
	end
	return folder
end

-- Convenience one-call bake: Floor.build + local grids.
-- Returns localData, floorData, tree, parts.
function LocalGrid.build(cfg: Config?)
	local floorData, tree, parts = Floor.build(cfg)
	local data = LocalGrid.fromFloor(floorData, parts, cfg)
	return data, floorData, tree, parts
end

function LocalGrid.visualize(data: any, parent: Instance?)
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then
		dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root
	end
	local old = dbg:FindFirstChild("LocalGrid")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "LocalGrid"; folder.Parent = dbg

	local step = data.config.step
	local i = 0
	for part, g in pairs(data.grids) do
		i += 1
		local hue = (i * 0.61803398875) % 1
		local sat = g.fallback and 0.3 or 0.9
		local pf = Instance.new("Folder"); pf.Name = part.Name; pf.Parent = folder
		for _, cell in ipairs(g.cells) do
			local dot = Instance.new("Part")
			dot.Anchored = true; dot.CanCollide = false; dot.CanQuery = false; dot.CanTouch = false
			local w, v
			if cell.clearance >= 4 then
				w, v = 0.9, 1
			elseif cell.clearance >= 3 then
				w, v = 0.7, 0.55
			else
				w, v = 0.55, 0.28
			end
			dot.Size = Vector3.new(w * (cell.size or step), 0.1, w * (cell.size or step))
			dot.Color = Color3.fromHSV(hue, sat, v)
			-- matte, so the neon Boundary edges pop over the grid layer
			dot.Material = Enum.Material.SmoothPlastic
			if not g.fallback and g.n then
				dot.CFrame = CFrame.fromMatrix(cell.pos, g.u, g.n)
			else
				dot.CFrame = CFrame.new(cell.pos)
			end
			dot.Name = string.format("c%.1f", cell.clearance)
			dot.Parent = pf
		end
	end
	return folder
end

return LocalGrid
