--!strict
-- NVGN.GlobalGrid -- one world-aligned, adaptive, tiled grid of walkable floor.
--
-- THE EXPERIMENT (docs/globalgrid-experiment.md). The per-surface pipeline
-- builds a grid per part face, splits the floor into thousands of surface
-- regions, meshes each apart and then stitches them back together -- and that
-- stitching is where every messy portal came from. Here the floor lives on ONE
-- lattice, so there are no seams between grids; cells are joined by "can I step
-- there", so a staircase is one region; and the map is cut into fixed tiles, so
-- one tile can be rebuilt on its own when something is destroyed.
--
-- ADAPTIVE FROM THE TOP. Each tile is a quadtree from `maxCell` down to
-- `minCell`. The SVO decides where to look: a square is accepted whole when
-- every 1-stud SVO column under it -- and a ring of columns around it -- has the
-- same layer structure, since then nothing can hide inside it and no edge can
-- sit on its border. Only where the SVO shows something happening is the square
-- split, and below one voxel (1 stud) the SVO can no longer see, so those cells
-- are decided exactly: a ray down for the surface, and LocalGrid's narrow-phase
-- solid tests (Solid) for anything standing on it.
--
-- Output, per build: cells (with surface point, normal, slope, headroom, part),
-- step-aware regions per tile, and each region's boundary as loops of lattice
-- segments -- `bound` where the floor really ends, `cut` where a tile border
-- runs through connected floor.

local GlobalGrid = {}

local Agents = require(script.Parent:WaitForChild("Agents"))
local SVO = require(script.Parent:WaitForChild("SVO"))
local Solid = require(script.Parent:WaitForChild("Solid"))

GlobalGrid.config = {
	tile = 32,        -- studs; one tile is the unit of rebuilding
	maxCell = 16,     -- largest cell
	minCell = 0.5,    -- smallest cell; 0.25 for the fine run
	leaf = 1,         -- SVO voxel
	probeUp = 0.5,    -- a surface ray starts this far above the SVO's solid top
	probeDown = 2.2,  -- and reaches this far below it (the SVO over-claims by < 1 leaf)
	knee = 0.5,       -- height of the wall test between two fine cells
	chest = 1.4,      -- and the second one
	bandLo = 0.02,    -- the kill-test box starts this far above the surface
	killShrink = 0.5, -- share of a cell's width the kill test covers, centred
	-- narrowest floor kept, as the current pipeline's minWidth (Cocosulx: keep
	-- strips 3 cells thick at 0.5)
	minWidth = 1.5,
	-- how outlines are straightened: "simplify" (the current pipeline's tuned
	-- simplifier) or "band" (the rubber band)
	outline = "simplify",
	-- studs; outline edges longer than this get evenly spaced points before
	-- triangulation, against fans of thin polygons (0 to disable)
	splitEdges = 8,
	-- merge polygons across tile borders where the result stays convex
	mergeTiles = true,
}

local UP = Vector3.yAxis

local function isCharacter(p: Instance): boolean
	local a = p.Parent
	while a and a ~= workspace do
		if a:IsA("Model") and a:FindFirstChildOfClass("Humanoid") then return true end
		a = a.Parent
	end
	return false
end

local function aabbOf(p: BasePart): (Vector3, Vector3)
	local cf, s = p.CFrame, p.Size * 0.5
	local x, y, z = cf.RightVector, cf.UpVector, cf.LookVector
	local h = Vector3.new(
		math.abs(x.X) * s.X + math.abs(y.X) * s.Y + math.abs(z.X) * s.Z,
		math.abs(x.Y) * s.X + math.abs(y.Y) * s.Y + math.abs(z.Y) * s.Z,
		math.abs(x.Z) * s.X + math.abs(y.Z) * s.Y + math.abs(z.Z) * s.Z)
	return p.Position - h, p.Position + h
end

-- height of a cell's surface plane under (x, z)
local function heightAt(cell: any, x: number, z: number): number
	local n = cell.n
	if math.abs(n.Y) < 1e-3 then return cell.y end
	return cell.y - ((x - cell.cx) * n.X + (z - cell.cz) * n.Z) / n.Y
end
GlobalGrid.heightAt = heightAt

function GlobalGrid.build(cfg: any): any
	local c = table.clone(GlobalGrid.config)
	for k, v in pairs(cfg or {}) do c[k] = v end
	local env = Agents.envelope()
	local prof = Agents.get("default")
	local bmin: Vector3, bmax: Vector3 = c.bounds.min, c.bounds.max
	local T, MIN, LEAF = c.tile, c.minCell, c.leaf
	local stats = { parts = 0, tiles = 0, cells = 0, bigCells = 0, fineCells = 0, rays = 0,
		killRays = 0, killed = 0, noSurface = 0, steep = 0, links = 0, wallCut = 0,
		regions = 0, boundSegs = 0, cutSegs = 0, t = {} }
	local tick0 = os.clock()
	local lastYield = os.clock()
	local function yield()
		if c.onProgress and os.clock() - lastYield > 0.05 then
			lastYield = os.clock()
			c.onProgress()
		end
	end

	-- 1. parts touching the area, and a world-aligned SVO over them
	local parts = {}
	local lo4, hi4 = bmin - Vector3.new(4, 4, 4), bmax + Vector3.new(4, 4, 4)
	for _, d in ipairs((c.root or workspace):GetDescendants()) do
		if d:IsA("BasePart") and d.CanCollide and not isCharacter(d) then
			local a, b = aabbOf(d)
			if a.X <= hi4.X and b.X >= lo4.X and a.Y <= hi4.Y and b.Y >= lo4.Y and a.Z <= hi4.Z and b.Z >= lo4.Z then
				parts[#parts + 1] = d
			end
		end
	end
	stats.parts = #parts
	local partSet = {}
	for _, p in ipairs(parts) do partSet[p] = true end
	local tree = SVO.fromParts(parts, LEAF, 2, { align = true, onProgress = c.onProgress and function() yield() end })
	stats.t.svo = os.clock() - tick0

	-- A SHORT EXCLUDE LIST, never an include list of every part: the filter list
	-- is scanned per query (1 entry 0.95us, 18197 entries 527us).
	local ex = {}
	for _, n in ipairs({ "NVGN_Debug", "NVGN_Path", "NVGN_GG", "NVGN_Grid", "PathStart", "PathEnd", "NVGN_Follower" }) do
		local x = workspace:FindFirstChild(n)
		if x then ex[#ex + 1] = x end
	end
	for _, d in ipairs((c.root or workspace):GetDescendants()) do
		if d:IsA("Humanoid") and d.Parent then ex[#ex + 1] = d.Parent end
	end
	local rp = RaycastParams.new()
	rp.FilterType = Enum.RaycastFilterType.Exclude
	rp.FilterDescendantsInstances = ex
	rp.IgnoreWater = true
	local op = OverlapParams.new()
	op.FilterType = Enum.RaycastFilterType.Exclude
	op.FilterDescendantsInstances = ex
	local probe = Instance.new("Part")
	probe.Anchored = true; probe.CanCollide = false; probe.CanQuery = false; probe.CanTouch = false

	-- a ray that only counts collidable parts of the bake: decoration that
	-- cannot be stood on or bumped into is flown straight through
	local function cast(o: Vector3, d: Vector3): RaycastResult?
		local len = d.Magnitude
		if len < 1e-6 then return nil end
		local dir = d / len
		local from, left = o, len
		for _ = 1, 6 do
			local hit = workspace:Raycast(from, dir * left, rp)
			if not hit or partSet[hit.Instance] then return hit end
			local went = (hit.Position - from).Magnitude + 0.01
			left -= went
			if left <= 0 then return nil end
			from = hit.Position + dir * 0.01
		end
		return nil
	end

	-- 2. walkable layers of every 1-stud column: a solid top with at least the
	-- smallest profile's crawl height of air above it
	local t1 = os.clock()
	local yLo, yHi = bmin.Y - 2, bmax.Y + env.height + 2
	local colCache: { [string]: { any } } = {}
	local function layersOf(ix: number, iz: number): { any }
		local k = ix .. ":" .. iz
		local L = colCache[k]
		if L then return L end
		local runs = tree:columnRuns(ix + LEAF * 0.5, iz + LEAF * 0.5, yLo, yHi)
		L = {}
		for i, r in ipairs(runs) do
			local top = r[2]
			local nextStart = runs[i + 1] and runs[i + 1][1] or math.huge
			local head = nextStart - top
			if head >= env.prone and top > yLo + 0.5 and top < yHi - 0.5 then
				local cls = (head >= prof.height) and 3 or (head >= prof.crouch) and 2 or 1
				L[#L + 1] = { top = top, bottom = r[1], head = head, cls = cls }
			end
		end
		colCache[k] = L
		return L
	end
	local function signature(ix: number, iz: number): string
		local L = layersOf(ix, iz)
		local s = {}
		for _, l in ipairs(L) do s[#s + 1] = l.top .. "/" .. l.cls end
		return table.concat(s, ",")
	end

	-- 3. the quadtree, per tile
	local cells: { any } = {}
	local slots: { [string]: { any } } = {} -- minCell slot -> cells covering it
	local function slotKey(x: number, z: number): string
		return math.floor(x / MIN + 1e-6) .. ":" .. math.floor(z / MIN + 1e-6)
	end
	local function register(cell: any)
		cells[#cells + 1] = cell
		local n = math.max(1, math.floor(cell.s / MIN + 0.5))
		for a = 0, n - 1 do
			for b = 0, n - 1 do
				local k = slotKey(cell.x0 + (a + 0.5) * MIN, cell.z0 + (b + 0.5) * MIN)
				local l = slots[k]
				if not l then l = {}; slots[k] = l end
				l[#l + 1] = cell
			end
		end
	end

	-- the exact surface under (x, z) inside one SVO solid run. ANYWHERE in the
	-- run, not just near its top: at 1-stud voxels neighbouring stair treads blur
	-- into one lump (case6's left staircase: a run 48-51 over a tread at 49), and
	-- only accepting hits near the lump's top threw every other tread away. The
	-- ray finds the highest real surface in the run, which is the one underfoot.
	local function surface(x: number, z: number, layer: any): RaycastResult?
		stats.rays += 1
		local o = Vector3.new(x, layer.top + c.probeUp, z)
		local depth = (layer.top - layer.bottom) + c.probeUp + 0.3
		local hit = cast(o, -UP * depth)
		if hit and hit.Position.Y >= layer.bottom - 0.3 then return hit end
		return nil
	end

	-- is anything solid standing in the box over a fine cell? LocalGrid's narrow
	-- phase: exact for blocks and wedges, GetPartsInPart as a positive for meshes
	local function killed(cell: any): boolean
		-- THE MIDDLE OF THE CELL, not all of it. A world-aligned cell does not
		-- line up with a stair tread, so the one straddling a tread's edge
		-- overlaps the next riser; testing its whole footprint killed a row of
		-- cells between every pair of treads and cut every staircase apart. The
		-- outline's band already allows a cell of slack at an edge.
		local s = cell.s * c.killShrink
		local cf = CFrame.fromMatrix(Vector3.new(cell.cx, cell.y + c.bandLo + env.prone * 0.5, cell.cz),
			Vector3.xAxis, UP, Vector3.zAxis)
		local size = Vector3.new(s, env.prone - c.bandLo, s)
		local probed = false
		for _, cand in ipairs(workspace:GetPartBoundsInBox(cf, size, op)) do
			-- A STEP IS NOT AN OBSTACLE. On a grid that joins cells by "can I step
			-- there", a part whose top is within a step above this cell is the
			-- next tread, not a wall -- and treating it as one killed the cell
			-- straddling every tread edge and cut each staircase apart.
			local candTop = cand.Position.Y + Solid.supportHalf(cand, UP)
			if candTop <= cell.y + env.step + 1e-3 then continue end
			if cand ~= cell.part and partSet[cand] then
				if Solid.isBlock(cand) then
					if Solid.boxOverlap(cf, size, cand.CFrame, cand.Size) then return true end
				elseif Solid.isWedge(cand) then
					if Solid.wedgeOverlap(cf, size, cand) then return true end
				elseif not probed then
					probed = true
					probe.Size = size
					probe.CFrame = cf
					for _, h in ipairs(workspace:GetPartsInPart(probe, op)) do
						if h ~= cell.part and partSet[h] then return true end
					end
				end
			end
		end
		return false
	end

	local maxSlopeCos = math.cos(math.rad(env.maxSlope))
	local function makeCell(tile: any, x0: number, z0: number, s: number, layer: any, fine: boolean): any?
		local cx, cz = x0 + s * 0.5, z0 + s * 0.5
		local hit = surface(cx, cz, layer)
		if not hit then stats.noSurface += 1; return nil end
		if hit.Normal.Y < maxSlopeCos then stats.steep += 1; return nil end
		local cell = { x0 = x0, z0 = z0, s = s, cx = cx, cz = cz, y = hit.Position.Y, n = hit.Normal,
			part = hit.Instance, head = layer.head, cls = layer.cls, tile = tile, fine = fine,
			slope = math.deg(math.acos(math.clamp(hit.Normal.Y, -1, 1))) }
		if fine then
			stats.killRays += 1
			if killed(cell) then stats.killed += 1; return nil end
			-- exact headroom: the SVO over-claims solid, so its headroom is a floor on the truth
			local upHit = cast(hit.Position + UP * 0.05, UP * math.min(cell.head + 1, 40))
			if upHit then cell.head = math.min(cell.head + 1, upHit.Distance + 0.05) end
			if cell.head < env.prone then stats.killed += 1; return nil end
			cell.cls = (cell.head >= prof.height) and 3 or (cell.head >= prof.crouch) and 2 or 1
		end
		return cell
	end

	local function square(tile: any, x0: number, z0: number, s: number)
		yield()
		if s > LEAF + 1e-6 then
			-- uniform over the square and one ring of columns around it?
			local sig = nil
			local uniform = true
			local ix0, iz0 = math.floor(x0 / LEAF), math.floor(z0 / LEAF)
			local n = math.floor(s / LEAF + 0.5)
			for a = -1, n do
				for b = -1, n do
					local sg = signature(ix0 + a, iz0 + b)
					if sig == nil then sig = sg elseif sg ~= sig then uniform = false; break end
				end
				if not uniform then break end
			end
			if uniform then
				if sig == "" then return end -- no floor here at all
				for _, layer in ipairs(layersOf(ix0, iz0)) do
					local cell = makeCell(tile, x0, z0, s, layer, false)
					if cell then register(cell); stats.bigCells += 1 end
				end
				return
			end
			local h = s * 0.5
			square(tile, x0, z0, h); square(tile, x0 + h, z0, h)
			square(tile, x0, z0 + h, h); square(tile, x0 + h, z0 + h, h)
			return
		end
		-- one voxel column: below this the SVO cannot see, so go fine and exact
		local L = layersOf(math.floor(x0 / LEAF), math.floor(z0 / LEAF))
		if #L == 0 then return end
		local n = math.floor(s / MIN + 0.5)
		for a = 0, n - 1 do
			for b = 0, n - 1 do
				for _, layer in ipairs(L) do
					local cell = makeCell(tile, x0 + a * MIN, z0 + b * MIN, MIN, layer, true)
					if cell then register(cell); stats.fineCells += 1 end
				end
			end
		end
	end

	local tiles = {}
	for tx = math.floor(bmin.X / T), math.floor((bmax.X - 1e-6) / T) do
		for tz = math.floor(bmin.Z / T), math.floor((bmax.Z - 1e-6) / T) do
			local tile = { tx = tx, tz = tz, x0 = tx * T, z0 = tz * T, id = #tiles + 1 }
			tiles[#tiles + 1] = tile
			stats.tiles += 1
			local M = c.maxCell
			for a = 0, T / M - 1 do
				for b = 0, T / M - 1 do
					square(tile, tile.x0 + a * M, tile.z0 + b * M, M)
				end
			end
		end
	end
	probe:Destroy()
	stats.t.cells = os.clock() - t1

	-- 3b. TOO NARROW TO STAND ON. The same rule as LocalGrid.pruneNarrow: a
	-- cell stays only if a footprint `minWidth` across fits over it -- but on one
	-- lattice the footprint may span a STEP, so a staircase survives while a
	-- handrail or a wall cap standing higher than a step above everything around
	-- it does not. Iterated, since a strip can prop up its neighbour for a pass.
	local tp = os.clock()
	local k = math.ceil(c.minWidth / MIN - 1e-6)
	stats.narrow = 0
	if k > 1 then
		for _ = 1, 4 do
			local function floorNear(sx: number, sz: number, h: number): boolean
				for _, o in ipairs(slots[slotKey(sx, sz)] or {}) do
					if not o.dead and math.abs(heightAt(o, sx, sz) - h) <= env.step then return true end
				end
				return false
			end
			local killedNow = {}
			for _, cell in ipairs(cells) do
				if cell.dead or cell.s >= c.minWidth - 1e-6 then continue end
				local ok = false
				for a = 0, k - 1 do
					for b = 0, k - 1 do
						local fits = true
						for i = 0, k - 1 do
							for j = 0, k - 1 do
								local sx = cell.x0 + (i - a + 0.5) * MIN
								local sz = cell.z0 + (j - b + 0.5) * MIN
								if not floorNear(sx, sz, heightAt(cell, sx, sz)) then fits = false; break end
							end
							if not fits then break end
						end
						if fits then ok = true; break end
					end
					if ok then break end
				end
				if not ok then killedNow[#killedNow + 1] = cell end
				yield()
			end
			for _, cell in ipairs(killedNow) do cell.dead = true end
			stats.narrow += #killedNow
			if #killedNow == 0 then break end
		end
		local keep = {}
		for _, cell in ipairs(cells) do if not cell.dead then keep[#keep + 1] = cell end end
		cells = keep
		for key, l in pairs(slots) do
			local kl = {}
			for _, o in ipairs(l) do if not o.dead then kl[#kl + 1] = o end end
			slots[key] = kl
		end
	end
	stats.t.narrow = os.clock() - tp
	stats.cells = #cells

	-- 4. step-aware connections, per unit edge segment. Each side of each cell is
	-- walked in minCell steps; across each step, the cell on the far side whose
	-- surface meets this one's within a step is the neighbour.
	local t2 = os.clock()
	local function across(cell: any, px: number, pz: number, h: number): any?
		local best, bd = nil, math.huge
		for _, o in ipairs(slots[slotKey(px, pz)] or {}) do
			if o ~= cell then
				local d = math.abs(heightAt(o, px, pz) - h)
				if d < bd then best, bd = o, d end
			end
		end
		if best and bd <= env.step then return best end
		return nil
	end
	-- a thin wall exactly on the line between two fine cells passes both kill
	-- tests (touching counts as apart), so fine neighbours also need a clear line
	local function wallBetween(a: any, b: any, px: number, pz: number, hA: number, hB: number, nx: number, nz: number): boolean
		-- measured from the HIGHER surface: from the lower one a knee-high ray
		-- hits the riser of every stair step and calls it a wall
		local top = math.max(hA, hB)
		for _, hgt in ipairs({ c.knee, c.chest }) do
			local o = Vector3.new(px - nx * 0.2, top + hgt, pz - nz * 0.2)
			local e = Vector3.new(px + nx * 0.2, top + hgt, pz + nz * 0.2)
			if cast(o, e - o) then return true end
		end
		return false
	end
	-- the four sides, each travelled with the cell on the LEFT (Y up: left of +X is -Z)
	local segs: { any } = {}
	local parent: { [any]: any } = {}
	local function find(x: any): any
		while parent[x] ~= x do parent[x] = parent[parent[x]]; x = parent[x] end
		return x
	end
	for _, cell in ipairs(cells) do parent[cell] = cell end
	local function union(a: any, b: any)
		local ra, rb = find(a), find(b)
		if ra ~= rb then parent[ra] = rb end
	end
	-- TWO PASSES, SO BOTH SIDES AGREE. Each unit segment first picks its
	-- candidate across; a connection counts only when the two cells pick EACH
	-- OTHER, and the wall test runs once per pair and segment. Deciding from one
	-- side at a time let A join B while B, looking back, picked a different layer
	-- or tripped the wall test -- and a boundary one side has and the other lacks
	-- can never close into a loop (1397 open loops at 0.25).
	local units = {}
	local pick: { [string]: any } = {}
	for ci, cell in ipairs(cells) do cell.idx = ci end
	for _, cell in ipairs(cells) do
		local x0, z0, s = cell.x0, cell.z0, cell.s
		local sides = {
			{ Vector3.new(x0, 0, z0 + s), Vector3.new(1, 0, 0), 0, 1 },     -- max Z, travel +X
			{ Vector3.new(x0 + s, 0, z0 + s), Vector3.new(0, 0, -1), 1, 0 }, -- max X, travel -Z
			{ Vector3.new(x0 + s, 0, z0), Vector3.new(-1, 0, 0), 0, -1 },   -- min Z, travel -X
			{ Vector3.new(x0, 0, z0), Vector3.new(0, 0, 1), -1, 0 },        -- min X, travel +Z
		}
		local n = math.max(1, math.floor(s / MIN + 0.5))
		for _, sd in ipairs(sides) do
			local st, dir, nx, nz = sd[1], sd[2], sd[3], sd[4]
			for k = 0, n - 1 do
				local ax, az = st.X + dir.X * k * MIN, st.Z + dir.Z * k * MIN
				local bx, bz = ax + dir.X * MIN, az + dir.Z * MIN
				local mx, mz = (ax + bx) * 0.5, (az + bz) * 0.5
				local h = heightAt(cell, mx, mz)
				local o = across(cell, mx + nx * MIN * 0.5, mz + nz * MIN * 0.5, h)
				local u = { cell = cell, ax = ax, az = az, bx = bx, bz = bz, mx = mx, mz = mz, nx = nx, nz = nz, h = h, o = o }
				units[#units + 1] = u
				pick[cell.idx .. "@" .. ("%.3f:%.3f"):format(mx, mz)] = o
			end
		end
		yield()
	end
	local wallMemo: { [string]: boolean } = {}
	for _, u in ipairs(units) do
		local cell, o = u.cell, u.o
		local key = ("%.3f:%.3f"):format(u.mx, u.mz)
		if o and pick[o.idx .. "@" .. key] ~= cell then o = nil end -- not mutual
		if o and (cell.fine or o.fine) then
			local lo, hi = math.min(cell.idx, o.idx), math.max(cell.idx, o.idx)
			local wk = lo .. ":" .. hi .. "@" .. key
			local w = wallMemo[wk]
			if w == nil then
				w = wallBetween(cell, o, u.mx, u.mz, u.h, heightAt(o, u.mx, u.mz), u.nx, u.nz)
				wallMemo[wk] = w
				if w then stats.wallCut += 1 end
			end
			if w then o = nil end
		end
		local kind = nil
		if not o then
			kind = "bound"
		elseif o.tile ~= cell.tile then
			kind = "cut"
		else
			union(cell, o)
			stats.links += 1
			u.linked = o
			cell.nb = cell.nb or {}
			cell.nb[#cell.nb + 1] = o
		end
		if kind then
			segs[#segs + 1] = { cell = cell, kind = kind,
				a = Vector3.new(u.ax, heightAt(cell, u.ax, u.az), u.az),
				b = Vector3.new(u.bx, heightAt(cell, u.bx, u.bz), u.bz),
				other = o }
		end
	end
	stats.t.links = os.clock() - t2

	-- 5. regions, numbered in cell order so a rebuild numbers them the same way
	local regionOf: { [any]: number } = {}
	local nRegion = 0
	for _, cell in ipairs(cells) do
		local r = find(cell)
		if not regionOf[r] then nRegion += 1; regionOf[r] = nRegion end
		cell.region = regionOf[r]
	end
	stats.regions = nRegion

	-- 5b. LAYERS. Joining by "can I step there" makes a staircase and the floor
	-- it climbs over ONE region -- and seen from above that region then sits on
	-- top of itself, which the mesher (a 2D triangulation) cannot represent: its
	-- outline overlapped itself and its polygons slanted through the arch. So each
	-- region is split into layers that never overlap in plan, grown outward along
	-- the links and refusing any cell whose footprint the layer already covers.
	-- Where two layers meet along a link, the edge is exact (it is a cell edge),
	-- is kept as it is like a tile cut, and becomes a portal.
	local layerOf: { [any]: number } = {}
	local nLayer = 0
	local function slotsOf(cell: any): { string }
		local out = {}
		local n = math.max(1, math.floor(cell.s / MIN + 0.5))
		for a = 0, n - 1 do
			for b = 0, n - 1 do out[#out + 1] = slotKey(cell.x0 + (a + 0.5) * MIN, cell.z0 + (b + 0.5) * MIN) end
		end
		return out
	end
	for _, seed in ipairs(cells) do
		if layerOf[seed] then continue end
		nLayer += 1
		local claimed: { [string]: boolean } = {}
		for _, k in ipairs(slotsOf(seed)) do claimed[k] = true end
		layerOf[seed] = nLayer
		local queue, qi = { seed }, 1
		while qi <= #queue do
			local cc = queue[qi]; qi += 1
			for _, o in ipairs(cc.nb or {}) do
				if not layerOf[o] and o.region == cc.region then
					local ks = slotsOf(o)
					local free = true
					for _, k in ipairs(ks) do if claimed[k] then free = false; break end end
					if free then
						for _, k in ipairs(ks) do claimed[k] = true end
						layerOf[o] = nLayer
						queue[#queue + 1] = o
					end
				end
			end
		end
	end
	for _, cell in ipairs(cells) do
		cell.conn = cell.region -- the connected floor it belongs to
		cell.region = layerOf[cell] -- the layer, which is what gets meshed
	end
	stats.layers = nLayer
	-- linked units whose two cells ended up in different layers are cuts
	for _, u in ipairs(units) do
		local o = u.linked
		if o and layerOf[o] ~= layerOf[u.cell] then
			segs[#segs + 1] = { cell = u.cell, kind = "cut",
				a = Vector3.new(u.ax, heightAt(u.cell, u.ax, u.az), u.az),
				b = Vector3.new(u.bx, heightAt(u.cell, u.bx, u.bz), u.bz),
				other = o }
			stats.layerCuts = (stats.layerCuts or 0) + 1
		end
	end

	-- 6. boundary loops per region, chained end to start. At a pinch -- two cells
	-- of one region touching only at a corner -- take the LEFTMOST turn, which
	-- keeps each loop simple instead of figure-eighting through the pinch.
	local t3 = os.clock()
	local function pk(v: Vector3): string return ("%.3f:%.3f"):format(v.X, v.Z) end
	local byRegion: { [number]: { any } } = {}
	for _, sg in ipairs(segs) do
		if sg.kind == "bound" then stats.boundSegs += 1 else stats.cutSegs += 1 end
		local r = sg.cell.region
		local l = byRegion[r]
		if not l then l = {}; byRegion[r] = l end
		l[#l + 1] = sg
	end
	local loops = {}
	for r = 1, nLayer do
		local list = byRegion[r]
		if not list then continue end
		local starts: { [string]: { any } } = {}
		for _, sg in ipairs(list) do
			local k = pk(sg.a)
			local l = starts[k]
			if not l then l = {}; starts[k] = l end
			l[#l + 1] = sg
		end
		local used = {}
		for _, first in ipairs(list) do
			if used[first] then continue end
			local chain = {}
			local sg = first
			local guard = 0
			while sg and not used[sg] and guard < 1e6 do
				guard += 1
				used[sg] = true
				chain[#chain + 1] = sg
				local cands = starts[pk(sg.b)]
				local nextSg, bestTurn = nil, -math.huge
				if cands then
					local din = (sg.b - sg.a) * Vector3.new(1, 0, 1)
					for _, cnd in ipairs(cands) do
						if not used[cnd] then
							local dout = (cnd.b - cnd.a) * Vector3.new(1, 0, 1)
							-- signed turn, LEFT positive: with Y up, X x (-Z) = +Y, so a left
							-- turn has a positive cross product
							local cr = din:Cross(dout).Y
							local dt = din:Dot(dout)
							local turn = math.atan2(cr, dt)
							if turn > bestTurn then bestTurn, nextSg = turn, cnd end
						end
					end
				end
				sg = nextSg
			end
			-- merge collinear runs of one kind
			local pts, kinds = {}, {}
			for i, s2 in ipairs(chain) do
				local prev = chain[(i - 2) % #chain + 1]
				local d1 = (s2.b - s2.a) * Vector3.new(1, 0, 1)
				local d0 = (prev.b - prev.a) * Vector3.new(1, 0, 1)
				local straight = d0.Unit:Dot(d1.Unit) > 0.999 and prev.kind == s2.kind
				if not straight or #chain == 1 then
					pts[#pts + 1] = s2.a
					kinds[#kinds + 1] = s2.kind -- kind of the run STARTING here
				end
			end
			local closed = pk(chain[#chain].b) == pk(chain[1].a)
			-- THE DENSE RING the old outline simplifier was tuned on: one node per
			-- cell edge, the edge's midpoint set half a cell into the floor
			-- (Pipeline's polyline recipe). Cut stretches keep their exact lattice
			-- points instead, so tile borders and layer seams stay on their lines.
			local dense, dkinds = {}, {}
			for ci2, s2 in ipairs(chain) do
				if s2.kind == "cut" then
					dense[#dense + 1] = s2.a
					dkinds[#dkinds + 1] = "cut"
					-- where a cut stretch hands over to floor edge, its exact end
					-- point starts the bound stretch, so the two meet on the line
					local nxt = chain[ci2 % #chain + 1]
					if nxt.kind ~= "cut" then
						dense[#dense + 1] = s2.b
						dkinds[#dkinds + 1] = "bound"
					end
				else
					local d = (s2.b - s2.a) * Vector3.new(1, 0, 1)
					local left = d.Magnitude > 1e-9 and Vector3.new(d.Z, 0, -d.X).Unit or Vector3.zero
					dense[#dense + 1] = (s2.a + s2.b) * 0.5 + left * (MIN * 0.5)
					dkinds[#dkinds + 1] = "bound"
				end
			end
			loops[#loops + 1] = { region = r, pts = pts, kinds = kinds, closed = closed,
				tile = chain[1].cell.tile, raw = #chain, dense = dense, denseKinds = dkinds }
		end
	end
	stats.loops = #loops
	stats.t.loops = os.clock() - t3
	stats.t.total = os.clock() - tick0

	return { cells = cells, slots = slots, tiles = tiles, loops = loops, segs = segs,
		config = c, stats = stats, tree = tree, slotKey = slotKey }
end

-- POLYGONS AND PORTALS from a built grid.
--
-- Each region's loops are rubber-banded, classified and handed to the existing
-- CDT. Every cell then belongs to a polygon of its own region -- the one holding
-- its centre, or the nearest for the few rim cells the band pulled inside -- so
-- membership is known, not claimed. Portals are of two kinds only:
--   shared  an edge two polygons of one region both have (exact floats)
--   tile    the overlap of two polygons' edges on one tile border line, where
--           the grid said the cells on either side connect
-- There is nothing to fit and nothing to fall back to.
function GlobalGrid.mesh(g: any): any
	local RubberBand = require(script.Parent:WaitForChild("RubberBand"))
	local Rings = require(script.Parent:WaitForChild("Rings"))
	local CDT = require(script.Parent:WaitForChild("CDT"))
	local c = g.config
	local env = Agents.envelope()
	local t0 = os.clock()
	local stats = { loops = 0, rawCorners = 0, corners = 0, polys = 0, shared = 0, tile = 0, layer = 0,
		cellsInside = 0, cellsNearest = 0, cellsLost = 0, t = {} }

	-- THE CURRENT PIPELINE'S OUTLINE SIMPLIFIER, per stretch between exact
	-- points: fit, merge, dejog, collapse bevels, and clean corners on whole
	-- rings -- with the same tuned settings (Pipeline.OVERRIDES) and the same
	-- ray validator. Cut stretches are left exactly as they are, and a ring the
	-- simplifier would shrink by more than keepArea falls back to the band.
	local PathSimplify = require(script.Parent:WaitForChild("PathSimplify"))
	local vrp = RaycastParams.new()
	vrp.FilterType = Enum.RaycastFilterType.Exclude
	do
		local ex = {}
		for _, nm in ipairs({ "NVGN_Debug", "NVGN_Path", "NVGN_GG", "NVGN_Grid", "PathStart", "PathEnd", "NVGN_Follower" }) do
			local x = workspace:FindFirstChild(nm)
			if x then ex[#ex + 1] = x end
		end
		vrp.FilterDescendantsInstances = ex
	end
	local LIFT, RISE, DROP = UP * 0.35, UP * 0.8, -UP * 1.8
	local function clear(a: Vector3, b: Vector3): boolean
		local d = b - a
		if d.Magnitude < 1e-4 then return true end
		return workspace:Raycast(a + LIFT, d, vrp) == nil
	end
	-- the simplifier works on flattened points; rays need the real heights back
	local curFlat: { Vector3 }?, cur3: { Vector3 }? = nil, nil
	local function real(v: Vector3): Vector3
		if not curFlat or not cur3 then return v end
		local best, bd = v, math.huge
		for m, f in ipairs(curFlat) do
			local d = (f.X - v.X) ^ 2 + (f.Z - v.Z) ^ 2
			if d < bd then bd, best = d, cur3[m] end
		end
		return Vector3.new(v.X, best.Y, v.Z)
	end
	local function validate(p: Vector3, q: Vector3, r: Vector3?): boolean
		p, q = real(p), real(q)
		if r then r = real(r) end
		if r == nil then return clear(p, q) end
		if workspace:Raycast(p + RISE, DROP, vrp) == nil then return false end
		return clear(q, p) and clear(p, r)
	end
	local base = { up = UP, validate = validate, mergeAngle = 30, mergeMin = 0.55, mergeMax = 1.2,
		inwardMax = 1.0 }
	local run2
	-- IN PLAN. A layer can climb a staircase, and measured in 3D every tread's
	-- rise read as a deviation the simplifier had to keep. Simplify the flattened
	-- outline, then give each point back the height of the nearest real node.
	local function run(poly3: { Vector3 }, closed: boolean): { Vector3 }
		local poly = table.create(#poly3)
		for k, v in ipairs(poly3) do poly[k] = Vector3.new(v.X, 0, v.Z) end
		curFlat, cur3 = poly, poly3
		local q = run2(poly, closed)
		curFlat, cur3 = nil, nil
		for k, v in ipairs(q) do
			local best, bd = poly3[1], math.huge
			for m, f in ipairs(poly) do
				local d = (f - v).Magnitude
				if d < bd then bd, best = d, poly3[m] end
			end
			q[k] = Vector3.new(v.X, best.Y, v.Z)
		end
		if not closed and #q >= 2 then q[1], q[#q] = poly3[1], poly3[#poly3] end
		return q
	end
	run2 = function(poly: { Vector3 }, closed: boolean): { Vector3 }
		local o = table.clone(base)
		o.closed = closed
		local p, i = PathSimplify.simplify(poly, o)
		p, i = PathSimplify.merge(p, i, poly, o)
		p, i = PathSimplify.dejog(p, i, poly, o)
		local q = PathSimplify.collapseBevels(p, o)
		if closed then
			local idx = {}
			for k = 1, #q do idx[k] = k end
			q = PathSimplify.cleanCorners(q, idx, o)
		elseif #q >= 2 then
			q[1], q[#q] = poly[1], poly[#poly] -- the ends are pinned to the exact points
		end
		return q
	end
	local function area(pts: { Vector3 }): number
		local a = 0
		for k = 1, #pts do local p, q = pts[k], pts[k % #pts + 1]; a += p.X * q.Z - q.X * p.Z end
		return math.abs(a) * 0.5
	end
	local function simplifyRing(L: any): ({ Vector3 }?, { string }?)
		local P, K = L.dense, L.denseKinds
		local n = #P
		if n < 4 then return nil, nil end
		local anyCut = false
		for k = 1, n do if K[k] == "cut" then anyCut = true; break end end
		local out, outK = {}, {}
		if not anyCut then
			out = run(P, true)
			for k = 1, #out do outK[k] = "bound" end
		else
			-- start at the first point of a cut run
			local start = 1
			for k = 1, n do
				if K[k] == "cut" and K[(k - 2) % n + 1] ~= "cut" then start = k; break end
			end
			local k, walked = start, 0
			while walked < n do
				if K[k] == "cut" then
					out[#out + 1] = P[k]; outK[#outK + 1] = "cut"
					k = k % n + 1; walked += 1
				else
					-- a bound stretch: its first node is the exact end of the cut
					-- before it, and it runs to the exact start of the next cut
					local poly = {}
					while K[k] ~= "cut" and walked < n do
						poly[#poly + 1] = P[k]
						k = k % n + 1; walked += 1
					end
					poly[#poly + 1] = P[k]
					local q = (#poly >= 3) and run(poly, false) or poly
					for m = 1, #q - 1 do out[#out + 1] = q[m]; outK[#outK + 1] = "bound" end
				end
			end
		end
		-- drop the in-between points of every straight cut stretch
		do
			local keepP, keepK = {}, {}
			local m = #out
			for k = 1, m do
				local pv, nx = out[(k - 2) % m + 1], out[k % m + 1]
				local straight = false
				if outK[k] == "cut" and outK[(k - 2) % m + 1] == "cut" then
					local d1 = Vector3.new(out[k].X - pv.X, 0, out[k].Z - pv.Z)
					local d2 = Vector3.new(nx.X - out[k].X, 0, nx.Z - out[k].Z)
					straight = d1.Magnitude > 1e-6 and d2.Magnitude > 1e-6 and d1.Unit:Dot(d2.Unit) > 0.9999
				end
				if not straight then keepP[#keepP + 1] = out[k]; keepK[#keepK + 1] = outK[k] end
			end
			out, outK = keepP, keepK
		end
		if #out < 3 or area(out) < 0.7 * area(P) then return nil, nil end
		return out, outK
	end

	local loops = {}
	for li, L in ipairs(g.loops) do
		if L.closed and #L.pts >= 3 then
			local pts, kinds
			if c.outline == "band" then
				pts, kinds = RubberBand.pull(L.pts, L.kinds, c.minCell)
			else
				pts, kinds = simplifyRing(L)
				if not pts then
					stats.bandFallback = (stats.bandFallback or 0) + 1
					pts, kinds = RubberBand.pull(L.pts, L.kinds, c.minCell)
				end
			end
			-- LONG EDGES ARE SPLIT. A Delaunay triangulation of an outline with few
			-- corners along long straight edges fans every far corner out of one
			-- vertex; a point every `splitEdges` studs gives it somewhere else to
			-- go. The points are exactly on the edge, and merging removes the ones
			-- that end up in the middle of a polygon's side.
			if c.splitEdges and c.splitEdges > 0 then
				local sp, sk = {}, {}
				local n = #pts
				local E = c.splitEdges
				for k = 1, n do
					local a, b = pts[k], pts[k % n + 1]
					sp[#sp + 1] = a; sk[#sk + 1] = kinds[k]
					if kinds[k] == "cut" then
						-- cut edges lie on an axis line: split at fixed WORLD positions,
						-- so the polygons on both sides of a border get the same corners
						-- and can be merged across it
						local axis = (math.abs(a.X - b.X) < 1e-4) and "Z" or "X"
						local lo, hi = math.min(a[axis], b[axis]), math.max(a[axis], b[axis])
						local ts = {}
						for w = math.floor(lo / E) + 1, math.ceil(hi / E) - 1 do
							local t = (w * E - a[axis]) / (b[axis] - a[axis])
							if t > 1e-4 and t < 1 - 1e-4 then ts[#ts + 1] = t end
						end
						table.sort(ts)
						for _, t in ipairs(ts) do sp[#sp + 1] = a:Lerp(b, t); sk[#sk + 1] = "cut" end
					else
						local len = Vector3.new(b.X - a.X, 0, b.Z - a.Z).Magnitude
						local pieces = math.floor(len / E)
						for j = 1, pieces do
							sp[#sp + 1] = a:Lerp(b, j / (pieces + 1)); sk[#sk + 1] = kinds[k]
						end
					end
				end
				pts, kinds = sp, sk
			end
			stats.rawCorners += #L.pts
			stats.corners += #pts
			loops[#loops + 1] = { region = L.region, index = li, pts = pts, kinds = kinds, up = UP, closed = true,
				tile = L.tile }
		end
	end
	stats.loops = #loops
	stats.t.band = os.clock() - t0

	-- ONE OUTER RIM PER REGION, as CDT requires. Where a region touches itself
	-- only at a corner the trace rightly comes back as two rims; each becomes
	-- its own sub-region and takes the holes that lie inside it. A corner is a
	-- zero-width contact, so nothing walkable is lost by the split.
	local function signedArea(pts: { Vector3 }): number
		local a = 0
		for k = 1, #pts do
			local p, q = pts[k], pts[k % #pts + 1]
			a += p.X * q.Z - q.X * p.Z
		end
		return -a * 0.5 -- region-on-the-left with Y up is positive
	end
	local function inRing(pts: { Vector3 }, x: number, z: number): boolean
		local inside, n = false, #pts
		for k = 1, n do
			local a, b = pts[k], pts[k % n + 1]
			if (a.Z > z) ~= (b.Z > z) and x < (b.X - a.X) * (z - a.Z) / (b.Z - a.Z) + a.X then inside = not inside end
		end
		return inside
	end
	local byReg = {}
	for _, L in ipairs(loops) do
		local l = byReg[L.region]
		if not l then l = {}; byReg[L.region] = l end
		l[#l + 1] = L
		L.sa = signedArea(L.pts)
	end
	local nextId = 0
	for r in pairs(byReg) do if r > nextId then nextId = r end end
	local subOf = {} -- original region -> list of sub-region ids
	stats.splitRegions = 0
	for r, list in pairs(byReg) do
		local outers = {}
		for _, L in ipairs(list) do if L.sa > 0 then outers[#outers + 1] = L end end
		subOf[r] = { r }
		if #outers > 1 then
			stats.splitRegions += 1
			for k = 2, #outers do
				nextId += 1
				outers[k].region = nextId
				table.insert(subOf[r], nextId)
			end
			for _, L in ipairs(list) do
				if L.sa <= 0 then
					local p = L.pts[1]
					for _, O in ipairs(outers) do
						if inRing(O.pts, p.X + 1e-3, p.Z + 1e-3) then L.region = O.region; break end
					end
				end
			end
		end
	end
	local t1 = os.clock()
	stats.rings = Rings.classify(loops)
	-- the outlines are already simplified, and the split points are exactly on
	-- their edges, so CDT's own collinear straightening must not undo the split
	local savedCol = CDT.collinear
	if c.splitEdges and c.splitEdges > 0 then CDT.collinear = 0 end
	local ok, mesh = pcall(CDT.build, loops, nil)
	CDT.collinear = savedCol
	if not ok then error(mesh) end
	do
		local tileOfR = {}
		for _, cell in ipairs(g.cells) do tileOfR[cell.region] = cell.tile end
		for r, subs in pairs(subOf) do for _, sr in ipairs(subs) do tileOfR[sr] = tileOfR[r] end end
		for _, f in ipairs(mesh.tris) do f.tile = tileOfR[f.region] end
	end
	stats.t.cdt = os.clock() - t1

	-- MERGE ACROSS TILE BORDERS. Tiles stay the unit of rebuilding, but where two
	-- polygons meet across a border on an identical edge and together are still
	-- convex, they become one -- so a border only shows where it has to. Greedy,
	-- longest shared edge first, to a fixed point.
	stats.tileMerges = 0
	if c.mergeTiles then
		local tris = mesh.tris
		local function k2(v: Vector3): string return ("%.3f,%.3f"):format(v.X, v.Z) end
		local function convex(v: { Vector3 }): boolean
			local n = #v
			for q = 1, n do
				local a, b, cc = v[q], v[q % n + 1], v[(q + 1) % n + 1]
				-- Y up, region on the left: a right turn is a reflex corner
				local cr = (b.X - a.X) * (cc.Z - b.Z) - (b.Z - a.Z) * (cc.X - b.X)
				if cr > 1e-6 then return false end
			end
			return true
		end
		for _ = 1, 50 do
			local edgeOwner = {}
			for fi, f in ipairs(tris) do
				if f then
					for q = 1, #f.verts do
						local a, b = f.verts[q], f.verts[q % #f.verts + 1]
						edgeOwner[k2(a) .. ">" .. k2(b)] = { fi, q }
					end
				end
			end
			local cand = {}
			for fi, f in ipairs(tris) do
				if f then
					for q = 1, #f.verts do
						local a, b = f.verts[q], f.verts[q % #f.verts + 1]
						local o = edgeOwner[k2(b) .. ">" .. k2(a)]
						if o and o[1] > fi then
							local g2 = tris[o[1]]
							if g2 and g2.tile ~= f.tile then
								cand[#cand + 1] = { fi, q, o[1], o[2], Vector3.new(b.X - a.X, 0, b.Z - a.Z).Magnitude }
							end
						end
					end
				end
			end
			table.sort(cand, function(x, y) return x[5] > y[5] end)
			local did = 0
			for _, cd in ipairs(cand) do
				local A, B = tris[cd[1]], tris[cd[3]]
				if A and B and A.verts[cd[2]] and B.verts[cd[4]] then
					-- SPLICE ALONG THE WHOLE SHARED RUN. Border points sit at fixed world
					-- positions, so two polygons across a border share several edges in
					-- a row; splicing along just one leaves the rest as a spur that runs
					-- out and back, and the shape is no longer simple.
					local av, bv = A.verts, B.verts
					local na, nb = #av, #bv
					local bEdge = {}
					for q = 1, nb do bEdge[k2(bv[q]) .. ">" .. k2(bv[q % nb + 1])] = q end
					local sharedA = {}
					local count = 0
					for q = 1, na do
						if bEdge[k2(av[q % na + 1]) .. ">" .. k2(av[q])] then sharedA[q] = true; count += 1 end
					end
					local clean = nil
					if count > 0 and count < na then
						-- the run must be contiguous: exactly one shared edge whose
						-- predecessor is not shared
						local starts, s0 = 0, nil
						for q = 1, na do
							if sharedA[q] and not sharedA[(q - 2) % na + 1] then starts += 1; s0 = q end
						end
						if starts == 1 and s0 then
							local runEnd = (s0 + count - 1) % na + 1 -- vertex index after the run
							local merged = {}
							-- A's own part: from the run's end vertex round to its start vertex
							local q = runEnd
							for _ = 1, na do
								merged[#merged + 1] = av[q]
								if q == s0 then break end
								q = q % na + 1
							end
							-- B's own part, strictly between av[s0] and av[runEnd]
							local jb = bEdge[k2(av[s0 % na + 1]) .. ">" .. k2(av[s0])] -- B edge matching A's first shared edge
							-- B's run ends at the vertex equal to av[s0]; walk B forward from there
							local startB = nil
							for w = 1, nb do if k2(bv[w]) == k2(av[s0]) then startB = w; break end end
							if startB and jb then
								local w = startB % nb + 1
								for _ = 1, nb do
									if k2(bv[w]) == k2(av[runEnd]) then break end
									merged[#merged + 1] = bv[w]
									w = w % nb + 1
								end
								clean = {}
								for z = 1, #merged do
									if k2(merged[z]) ~= k2(merged[z % #merged + 1]) then clean[#clean + 1] = merged[z] end
								end
							end
						end
					end
					if clean then
						if #clean >= 3 and convex(clean) then
							local ctr = Vector3.zero
							for _, w in ipairs(clean) do ctr += w end
							A.verts = clean
							A.n = #clean
							A.centre = ctr / #clean
							A.area = (A.area or 0) + (B.area or 0)
							A.regions = A.regions or { [A.region] = true }
							for r2 in pairs(B.regions or { [B.region] = true }) do A.regions[r2] = true end
							tris[cd[3]] = false
							did += 1
							stats.tileMerges += 1
						end
					end
				end
			end
			if did == 0 then break end
		end
		local keep = {}
		for _, f in ipairs(tris) do if f then keep[#keep + 1] = f end end
		mesh.tris = keep
	end
	stats.polys = #mesh.tris

	-- which tile a region lives in, and a region's polygons
	local tileOfRegion, polysOf = {}, {}
	for _, cell in ipairs(g.cells) do tileOfRegion[cell.region] = cell.tile end
	for r, subs in pairs(subOf) do for _, sr in ipairs(subs) do tileOfRegion[sr] = tileOfRegion[r] end end
	for i, f in ipairs(mesh.tris) do
		for r2 in pairs(f.regions or { [f.region] = true }) do
			local l = polysOf[r2]
			if not l then l = {}; polysOf[r2] = l end
			l[#l + 1] = i
		end
		f.tile = f.tile or tileOfRegion[f.region]
	end

	-- membership
	local t2 = os.clock()
	local function inside(f: any, x: number, z: number): boolean
		local v = f.verts
		local n = #v
		for k = 1, n do
			local a, b = v[k], v[k % n + 1]
			if (b.X - a.X) * (z - a.Z) - (b.Z - a.Z) * (x - a.X) > 1e-4 then return false end
		end
		return true
	end
	local function distTo(f: any, x: number, z: number): number
		local v = f.verts
		local best = math.huge
		for k = 1, #v do
			local a, b = v[k], v[k % #v + 1]
			local dx, dz = b.X - a.X, b.Z - a.Z
			local dd = dx * dx + dz * dz
			local t = dd > 1e-12 and math.clamp(((x - a.X) * dx + (z - a.Z) * dz) / dd, 0, 1) or 0
			local ex, ez = a.X + dx * t - x, a.Z + dz * t - z
			best = math.min(best, math.sqrt(ex * ex + ez * ez))
		end
		return best
	end
	local polyOf = {}
	for _, cell in ipairs(g.cells) do
		local list = {}
		for _, sr in ipairs(subOf[cell.region] or { cell.region }) do
			for _, i in ipairs(polysOf[sr] or {}) do list[#list + 1] = i end
		end
		if #list == 0 then stats.cellsLost += 1; continue end
		local found = nil
		for _, i in ipairs(list) do
			if inside(mesh.tris[i], cell.cx, cell.cz) then found = i; break end
		end
		if found then
			stats.cellsInside += 1
		else
			local bd = math.huge
			for _, i in ipairs(list) do
				local d = distTo(mesh.tris[i], cell.cx, cell.cz)
				if d < bd then bd, found = d, i end
			end
			stats.cellsNearest += 1
		end
		polyOf[cell] = found
	end
	stats.t.member = os.clock() - t2

	-- portals
	local t3 = os.clock()
	local links = {}
	local function vk(v: Vector3): string return ("%.3f,%.3f,%.3f"):format(v.X, v.Y, v.Z) end
	local seen = {}
	for i, f in ipairs(mesh.tris) do
		for k = 1, f.n do
			local A, B = f.verts[k], f.verts[k % f.n + 1]
			local ka, kb = vk(A), vk(B)
			local key = (ka < kb) and (ka .. "|" .. kb) or (kb .. "|" .. ka)
			local prev = seen[key]
			if prev and prev.poly ~= i then
				links[#links + 1] = { kind = "shared", a = prev.poly, b = i, left = prev.a, right = prev.b,
					centre = (prev.a + prev.b) * 0.5, span = (prev.b - prev.a).Magnitude, rise = 0, drop = 0 }
				stats.shared += 1
			else
				seen[key] = { poly = i, a = A, b = B }
			end
		end
	end
	-- CUT EDGES: tile borders and layer seams. Both are stretches the rubber band
	-- kept exactly, lying on lattice lines, and the grid already proved the cells
	-- either side connect. A polygon edge that lies on one of its own loop's cut
	-- stretches is matched to the overlapping cut edge of the polygon across it.
	-- consecutive collinear cut pieces are joined into one run first: a polygon
	-- edge along a border spans many half-stud pieces of the dense ring
	local cutsOf: { [number]: { any } } = {}
	for _, L in ipairs(loops) do
		local n = #L.pts
		local l = cutsOf[L.region]
		if not l then l = {}; cutsOf[L.region] = l end
		local cur = nil
		for k = 1, n do
			if L.kinds and L.kinds[k] == "cut" then
				local a, b = L.pts[k], L.pts[k % n + 1]
				local d = Vector3.new(b.X - a.X, 0, b.Z - a.Z)
				if cur and d.Magnitude > 1e-6 then
					local cd = Vector3.new(cur[2].X - cur[1].X, 0, cur[2].Z - cur[1].Z)
					if cd.Magnitude > 1e-6 and cd.Unit:Dot(d.Unit) > 0.9999 and (cur[2] - a).Magnitude < 1e-4 then
						cur[2] = b
						continue
					end
				end
				cur = { a, b }
				l[#l + 1] = cur
			else
				cur = nil
			end
		end
	end
	local function onCut(region: number, a: Vector3, b: Vector3): boolean
		for _, cs in ipairs(cutsOf[region] or {}) do
			local p, q = cs[1], cs[2]
			local d = Vector3.new(q.X - p.X, 0, q.Z - p.Z)
			local len = d.Magnitude
			if len > 1e-6 then
				local u = d / len
				local function near(v: Vector3): boolean
					local w = Vector3.new(v.X - p.X, 0, v.Z - p.Z)
					local t = w:Dot(u)
					return t >= -1e-3 and t <= len + 1e-3 and (w - u * t).Magnitude <= 1e-3
				end
				if near(a) and near(b) then return true end
			end
		end
		return false
	end
	local onLine: { [string]: { any } } = {}
	for i, f in ipairs(mesh.tris) do
		for k = 1, f.n do
			local A, B = f.verts[k], f.verts[k % f.n + 1]
			local key, along
			if math.abs(A.X - B.X) < 1e-4 then key, along = ("x:%.3f"):format(A.X), "Z"
			elseif math.abs(A.Z - B.Z) < 1e-4 then key, along = ("z:%.3f"):format(A.Z), "X" end
			local cutHere = false
			if key then
				for r2 in pairs(f.regions or { [f.region] = true }) do
					if onCut(r2, A, B) then cutHere = true; break end
				end
			end
			if cutHere then
				local l = onLine[key]
				if not l then l = {}; onLine[key] = l end
				l[#l + 1] = { poly = i, a = A, b = B, along = along }
			end
		end
	end
	for _, list in pairs(onLine) do
		for x = 1, #list do
			for y = x + 1, #list do
				local E, F = list[x], list[y]
				local P, Q = mesh.tris[E.poly], mesh.tris[F.poly]
				if P.region ~= Q.region then
					local ax = E.along
					local lo = math.max(math.min(E.a[ax], E.b[ax]), math.min(F.a[ax], F.b[ax]))
					local hi = math.min(math.max(E.a[ax], E.b[ax]), math.max(F.a[ax], F.b[ax]))
					if hi - lo > 1e-3 then
						local function at(S: any, t: number): Vector3
							local s0, s1 = S.a[ax], S.b[ax]
							local u = (s1 ~= s0) and (t - s0) / (s1 - s0) or 0
							return S.a:Lerp(S.b, u)
						end
						local pl, pr = at(E, lo), at(E, hi)
						local ql, qr = at(F, lo), at(F, hi)
						local mid = ((pl + pr) * 0.5).Y - ((ql + qr) * 0.5).Y
						if math.abs(mid) <= env.step then
							local kind = (P.tile ~= Q.tile) and "tile" or "layer"
							links[#links + 1] = { kind = kind, a = E.poly, b = F.poly, left = pl, right = pr,
								bLeft = qr, bRight = ql, centre = (pl + pr) * 0.5, span = hi - lo,
								rise = -mid, drop = mid, edge = true }
							stats[kind] = (stats[kind] or 0) + 1
						end
					end
				end
			end
		end
	end
	stats.t.portals = os.clock() - t3
	stats.t.total = os.clock() - t0
	return { mesh = mesh, portals = { links = links, polyOf = polyOf }, loops = loops, stats = stats }
end

function GlobalGrid.meshReport(m: any): string
	local s = m.stats
	return ("mesh  %d loops, %d grid corners -> %d banded corners -> %d polygons | portals: %d shared, %d tile-border, %d layer-seam | cells: %d inside a polygon, %d nearest, %d with no polygon | band %.2fs, cdt %.2fs, membership %.2fs, portals %.2fs")
		:format(s.loops, s.rawCorners, s.corners, s.polys, s.shared, s.tile, s.layer, s.cellsInside, s.cellsNearest,
			s.cellsLost, s.t.band or 0, s.t.cdt or 0, s.t.member or 0, s.t.portals or 0)
end

function GlobalGrid.report(g: any): string
	local s = g.stats
	local open = 0
	for _, L in ipairs(g.loops) do if not L.closed then open += 1 end end
	return ("globalgrid  minCell %.2f | %d parts, %d tiles | %d cells (%d big, %d fine), %d rays, %d killed, %d no surface, %d too steep | %d links, %d cut by a wall | %d regions -> %d layers, %d loops (%d open), %d bound + %d cut segments | %d too narrow | svo %.1fs, cells %.1fs, narrow %.1fs, links %.1fs, loops %.1fs, total %.1fs")
		:format(g.config.minCell, s.parts, s.tiles, s.cells, s.bigCells, s.fineCells, s.rays, s.killed,
			s.noSurface, s.steep, s.links, s.wallCut, s.regions, s.layers or s.regions, s.loops, open, s.boundSegs, s.cutSegs,
			s.narrow or 0, s.t.svo or 0, s.t.cells or 0, s.t.narrow or 0, s.t.links or 0, s.t.loops or 0, s.t.total or 0)
end

return GlobalGrid
