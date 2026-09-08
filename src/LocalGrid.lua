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
	-- Set by classifyNodes. Bitmasks over DIR8, plus the booleans they imply.
	wallMask: number?,        -- directions with a surface standing above us
	dropMask: number?,        -- directions with nothing to stand on
	wall: boolean?,
	dropoff: boolean?,
	fit: number?,             -- 1 prone, 2 crouch, 3 stand (from clearance)
	region: number?,          -- set by LocalGrid.regions; 1 = largest
	edgeMask: number?,        -- directions where the floor continues into ANOTHER region
	regionEdge: boolean?,

}

export type DeadCell = {
	ui: number, vi: number,
	pos: Vector3,
	killer: Instance?,
}

export type Grid = {
	part: BasePart,
	fallback: boolean,        -- true => world-aligned (degenerate frame only)
	origin: Vector3?,         -- face corner (world); block grids only
	u: Vector3?, v: Vector3?, -- in-plane unit axes (world); block grids only
	n: Vector3?,              -- surface normal (world); block grids only
	center: Vector3?,         -- centre of the walkable face (world)
	uExt: number?, vExt: number?, -- half-extents along u and v
	step: number,
	cells: {Cell},
	narrow: {Cell}?,          -- cells dropped by pruneNarrow (kept for debugging)
	index: { [string]: Cell },-- "ui:vi" -> cell
	dead: {DeadCell},
	deadIndex: { [string]: DeadCell },
}

export type Config = {
	step: number?, maxSlope: number?, clearCap: number?, minClearance: number?,
	flushTol: number?, probeRadius: number?, minWidth: number?, regionAngle: number?,
	bandHeight: number?, standHeight: number?, crouchHeight: number?,
}

local DEFAULT = {
	step = 0.5,         -- local cell size (studs)
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
	-- Narrowest walkable strip an agent can stand on (studs). A handrail, a
	-- stair stringer's top edge and a window ledge all pass the slope and
	-- clearance tests -- at 0 to 40 degrees with open sky above they are not
	-- measurably different from a sliver of real floor -- and width is the only
	-- fact that separates them. Default is the Roblox character's shoulder
	-- width; set it to 0 to keep every surface.
	minWidth = 2,
	-- How far two adjacent cells surface normals may diverge and still belong to
	-- the same region. Regions are meant to be surfaces one plane can describe,
	-- so a ramp meeting a floor is a seam even though you can walk straight
	-- across it -- that join is a region LINK, not a merge.
	regionAngle = 15,
	-- Tallest rise one region may cover. A ramp or a roof plane is a single
	-- surface geometrically, but a region that climbs 15 studs is not a place --
	-- it is a route between places, and everything downstream that treats a
	-- region as roughly one altitude gets it wrong. Regions that span more than
	-- this are cut into bands of it; flat ground is untouched, since its span is
	-- zero. Roughly a storey, and the same reference height Floor uses.
	bandHeight = 5,
	-- Headroom a posture needs. Below crouchHeight a cell is prone-only, and
	-- below minClearance it is not floor at all. These are postures, not
	-- preferences: a crouch tunnel and the room it opens into are different
	-- places to move through even where the floor runs straight between them,
	-- so they are never the same region.
	standHeight = 5,
	crouchHeight = 3,
}

local UP = Vector3.new(0, 1, 0)

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

local FIT_NAME = { "prone", "crouch", "stand" }
LocalGrid.FIT_NAME = FIT_NAME

local function fitOf(clearance: number, c: any): number
	if clearance >= c.standHeight then return 3 end
	if clearance >= c.crouchHeight then return 2 end
	return 1
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

-- Orientation from the part's own CFrame, plane from the surfels.
--
-- The CFrame axis most aligned with the average surfel normal names the face we
-- are standing on, and the other two axes are the in-plane pair. But the plane
-- ITSELF -- normal and offset -- comes from the surfels, never from the box: a
-- union's or mesh's bounding box is negotiated geometry and its face is not
-- where the real surface is. Projecting the CFrame's in-plane axes onto the
-- surfel plane keeps the lattice running along the part's own edges without
-- pinning it to a box that may be tilted off that surface.
--
-- Every BasePart has a CFrame, so this works for unions and meshes too. It used
-- to be gated on `p:IsA("Part") and p.Shape == Block`, which sent every union
-- and every mesh to a world-axis lattice that ignored their orientation
-- entirely -- the tiles staircased across the part's edges instead of running
-- along them.
--
-- Returns nil if the part has no usable in-plane axis (a degenerate Size).
local function surfaceFrame(part: BasePart, surfels: {any}, step: number)
	local n = avgNormal(surfels)
	local cf = part.CFrame
	local sz = part.Size
	local axes = {
		{ dir = cf.RightVector, ext = sz.X * 0.5 },
		{ dir = cf.UpVector,    ext = sz.Y * 0.5 },
		{ dir = cf.RightVector:Cross(cf.UpVector), ext = sz.Z * 0.5 }, -- local Z basis
	}
	local bi, best = 2, -math.huge
	for i, a in ipairs(axes) do
		local d = math.abs(a.dir:Dot(n))
		if d > best then best = d; bi = i end
	end
	local plane = {}
	for i, ax in ipairs(axes) do
		if i ~= bi then plane[#plane + 1] = ax end
	end

	-- Project an in-plane axis onto the surfel plane, then force the second
	-- orthogonal to it so the lattice stays square on a face the box is tilted
	-- against. The axis we drop is the one nearest the normal, so at least one
	-- of the remaining two always projects to something well conditioned.
	local u = plane[1].dir - n * plane[1].dir:Dot(n)
	local uExt, vExt = plane[1].ext, plane[2].ext
	if u.Magnitude < 1e-3 then
		u = plane[2].dir - n * plane[2].dir:Dot(n)
		uExt, vExt = plane[2].ext, plane[1].ext
		if u.Magnitude < 1e-3 then return nil end
	end
	u = u.Unit
	local v = n:Cross(u)
	if v.Magnitude < 1e-3 then return nil end
	v = v.Unit

	-- Offset: slide the part centre along n until the plane passes through the
	-- surfels' centroid. On a block this lands on the box face; on a union it
	-- lands on the surface the rays actually found.
	local ctr = Vector3.zero
	for _, s in ipairs(surfels) do ctr += s.pos end
	ctr /= #surfels
	local center = part.Position + n * (ctr - part.Position):Dot(n)

	-- How far the real surface strays from that plane, and how far the surfels
	-- reach in-plane. The box half-extents are a lower bound only: a projected
	-- face is wider than the box side it came from, and a union's surface can
	-- sit outside its own box axes. Grow to whatever the surfels need.
	local dev, uMax, vMax = 0, 0, 0
	for _, s in ipairs(surfels) do
		local r = s.pos - center
		dev = math.max(dev, math.abs(r:Dot(n)))
		uMax = math.max(uMax, math.abs(r:Dot(u)))
		vMax = math.max(vMax, math.abs(r:Dot(v)))
	end
	-- Grow to the outermost surfel, and NO further. An earlier version added a
	-- whole `step` of margin here, which was wrong twice over: rounding the cell
	-- count up already leaves up to half a step of slack on each side, and on a
	-- part thinner than the step the margin dominated its real size -- a 0.5-stud
	-- tread was given a 4-cell lattice whose sample points landed on and past its
	-- edges, so whether a row survived came down to float luck. That is what made
	-- identical stair treads come out one row deep in some places and two in
	-- others.
	uExt = math.max(uExt, uMax)
	vExt = math.max(vExt, vMax)
	-- Cap the stray: it only sizes the probe ray, and a wild surfel should not
	-- turn that into an arbitrarily long cast.
	dev = math.min(dev, 32)

	return n, u, v, uExt, vExt, center, dev
end

local function buildGrid(part: BasePart, surfels: {any}, c: any, filterAll: RaycastParams, probe: BasePart, op: OverlapParams, rpTerrain: RaycastParams): Grid?
	local n, u, v, uExt, vExt, surfaceCenter, dev = surfaceFrame(part, surfels, c.step)
	if not n then return nil end

	-- Centre the lattice on the face it describes. `nu` cells of `step` rarely
	-- divide the extent exactly, and anchoring at a corner dumps the whole
	-- remainder on the +u/+v edge, so the tiles sit visibly off to one side of
	-- the part. Rounding the count UP and splitting the slack puts half on each
	-- side: the lattice is symmetric about surfaceCenter, and the part's centre
	-- lands on a cell centre for an odd count and a cell corner for an even one.
	-- The extra ring costs nothing -- those cells miss the part-filtered ray and
	-- are dropped before they reach the grid.
	local step = c.step
	local nu = math.max(1, math.ceil(2 * uExt / step - 1e-6))
	local nv = math.max(1, math.ceil(2 * vExt / step - 1e-6))
	local corner = surfaceCenter - u * (nu * step * 0.5) - v * (nv * step * 0.5)

	local rpPart = RaycastParams.new()
	rpPart.FilterType = Enum.RaycastFilterType.Include
	rpPart.FilterDescendantsInstances = { part }

	local grid: Grid = {
		part = part, fallback = false, origin = corner,
		u = u, v = v, n = n, step = c.step, cells = {}, index = {},
		dead = {}, deadIndex = {},
		center = surfaceCenter, uExt = nu * step * 0.5, vExt = nv * step * 0.5,
	}

	local function kill(iu: number, iv: number, pos: Vector3, killer: Instance?)
		local d: DeadCell = { ui = iu, vi = iv, pos = pos, killer = killer }
		grid.dead[#grid.dead + 1] = d
		grid.deadIndex[string.format("%d:%d", iu, iv)] = d
	end

	-- Start above the highest stray and reach past the lowest one; on a block
	-- dev is ~0 and this is the old 2 / 2.5.
	local castH = 2 + dev
	local castLen = castH + dev + 0.5

	for iu = 0, nu - 1 do
		for iv = 0, nv - 1 do
			local p = corner + u * ((iu + 0.5) * step) + v * ((iv + 0.5) * step)
			local res = workspace:Raycast(p + n * castH, -n * castLen, rpPart)
			if not res then continue end
			local slope = math.deg(math.acos(math.clamp(res.Normal:Dot(UP), -1, 1)))
			if not ((slope <= c.maxSlope) or isClip(part)) then continue end
			probe.CFrame = CFrame.new(res.Position + UP * (0.1 + (c.minClearance - 0.1) * 0.5))
			local killer: Instance? = nil
			for _, hit in ipairs(workspace:GetPartsInPart(probe, op)) do
				if hit ~= part then killer = hit; break end
			end
			if killer then
				kill(iu, iv, res.Position, killer)
				continue
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
			if clearance < c.minClearance then
				kill(iu, iv, res.Position, cover)
				continue
			end
			local cell: Cell = {
				ui = iu, vi = iv, pos = res.Position, normal = res.Normal,
				slope = slope, clearance = clearance, cover = cover,
			}
			grid.cells[#grid.cells + 1] = cell
			grid.index[string.format("%d:%d", iu, iv)] = cell
		end
	end
	return grid
end

local function buildFallbackGrid(part: BasePart, surfels: {any}, c: any): Grid
	local grid: Grid = {
		part = part, fallback = true, step = c.step, cells = {}, index = {},
		dead = {}, deadIndex = {},
	}
	for _, s in ipairs(surfels) do
		if s.clearance < c.minClearance then continue end
		local iu = math.floor(s.pos.X / c.step)
		local iv = math.floor(s.pos.Z / c.step)
		local cell: Cell = {
			ui = iu, vi = iv, pos = s.pos, normal = s.normal,
			slope = s.slope, clearance = s.clearance, cover = s.cover,
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
	return live, dead
end

-- Where the neighbour in local direction d would be, in world space. Block
-- grids step along their own face axes; fallback grids are world-aligned.
-- Only a part with a degenerate Size ends up world-aligned now.
local function neighbourPos(g: Grid, cell: Cell, d: {number}): Vector3
	if not g.fallback and g.u and g.v then
		return cell.pos + g.u * (d[1] * g.step) + g.v * (d[2] * g.step)
	end
	return cell.pos + Vector3.new(d[1] * g.step, 0, d[2] * g.step)
end

-- Drop cells on surfaces too narrow to stand on.
--
-- Slope and clearance cannot catch a handrail. The top of a 1-stud rail sits at
-- 0 degrees with open sky above it, which on those two measurements is exactly
-- what a strip of real floor looks like. Width is the fact that separates them
-- and nothing was measuring it, so case3 grew cells along stair stringers,
-- ledges and window trim -- 25 grids and 113 cells, 4.3% of the bake.
--
-- The test is whether the agent's square footprint fits SOMEWHERE that covers
-- the cell. Asking for a covering footprint rather than a centred one is what
-- keeps the edge cells of a wide floor -- their footprint just sits further in.
-- It is evaluated in world space against the shared index, the same way a
-- neighbour lookup is, so a deck planked out of 1-stud parts is one wide
-- surface rather than a row of rails, and a ledge flush with a floor is part of
-- that floor while a rail three studs above it is not.
function LocalGrid.pruneNarrow(data: any, cfg: Config?)
	local c = merged(cfg)
	if data.config then
		c.step = data.config.step or c.step
		c.minWidth = (cfg and cfg.minWidth) or data.config.minWidth or c.minWidth
		c.flushTol = (cfg and cfg.flushTol) or data.config.flushTol or c.flushTol
	end
	data.stats.narrow = 0
	data.stats.narrowPasses = 0
	local k = math.ceil(c.minWidth / c.step) -- cells spanning one agent width
	if k <= 1 then return data end

	local r2 = (c.probeRadius * c.step) ^ 2
	local tol = c.flushTol
	local step = c.step
	local half = math.floor((k - 1) / 2)

	-- ITERATED to a fixed point. One pass is not enough because a footprint may
	-- be completed by cells that are themselves about to be pruned: where a rail
	-- runs within flushTol of a post cap or a second rail, each props the other
	-- up and both survive a single pass. Re-running against only the survivors
	-- removes whatever was standing on doomed ground. It terminates because
	-- every pass either removes cells or stops, and it cannot eat real floor:
	-- a cell in open floor keeps a valid footprint no matter how many times it
	-- is asked, so a wide surface is a fixed point from the first pass.
	local totalNarrow, passes = 0, 0
	while passes < 8 do
		passes += 1
		local live = buildWorldIndex(data.grids)

		-- Windows overlap heavily, so the same world point is probed many times.
		-- Quantising to an eighth of a stud is far finer than probeRadius, so two
		-- points that share a key would have answered the same anyway.
		local memo: { [string]: boolean } = {}
		local function floorAt(p: Vector3): boolean
			local key = math.round(p.X * 8) .. ":" .. math.round(p.Y * 8) .. ":" .. math.round(p.Z * 8)
			local m = memo[key]
			if m ~= nil then return m end
			local found = false
			local bx, bz = math.floor(p.X), math.floor(p.Z)
			for ox = -1, 1 do
				for oz = -1, 1 do
					for _, e in ipairs(live[(bx + ox) .. ":" .. (bz + oz)] or {}) do
						local q = e.cell.pos
						local dx, dz = q.X - p.X, q.Z - p.Z
						if dx * dx + dz * dz <= r2 and math.abs(q.Y - p.Y) <= tol then
							found = true
							break
						end
					end
					if found then break end
				end
				if found then break end
			end
			memo[key] = found
			return found
		end

		-- Is the whole k x k footprint anchored at `origin` walkable?
		local function footFits(u: Vector3, v: Vector3, origin: Vector3): boolean
			for a = 0, k - 1 do
				for b = 0, k - 1 do
					if not floorAt(origin + u * (a * step) + v * (b * step)) then
						return false
					end
				end
			end
			return true
		end

		-- A cell is standable if the agent's footprint fits ANYWHERE covering it.
		--
		-- Two 1-D runs through the cell -- an earlier version of this test -- ask
		-- a weaker question, and handrails exploited the gap: a rail has a long
		-- run along its length, and where it meets a newel post or dies into a
		-- wall the crosswise run leaks onto that neighbour and reaches width. So
		-- the middle of every rail was pruned and its ends survived. A filled
		-- square closes that, because a post cap cannot complete one.
		--
		-- Asking whether a covering footprint EXISTS, rather than whether the one
		-- centred here fits, is what keeps the edge cells of a wide floor: their
		-- footprint simply sits further in. The centred window is tried first
		-- because that is the answer for open floor.
		local function standable(u: Vector3, v: Vector3, cell: Cell): boolean
			if footFits(u, v, cell.pos - u * (half * step) - v * (half * step)) then
				return true
			end
			for a = 0, k - 1 do
				for b = 0, k - 1 do
					if (a ~= half or b ~= half)
						and footFits(u, v, cell.pos - u * (a * step) - v * (b * step)) then
						return true
					end
				end
			end
			return false
		end

		local nNarrow = 0
		for _, g in pairs(data.grids) do
			local u = g.u or Vector3.xAxis
			local v = g.v or Vector3.zAxis
			local keep = {}
			local narrow = g.narrow or {}
			for _, cell in ipairs(g.cells) do
				if standable(u, v, cell) then
					keep[#keep + 1] = cell
				else
					narrow[#narrow + 1] = cell
					g.index[string.format("%d:%d", cell.ui, cell.vi)] = nil
					nNarrow += 1
				end
			end
			g.cells = keep
			g.narrow = narrow
		end
		totalNarrow += nNarrow
		if nNarrow == 0 then break end
	end

	data.stats.cells -= totalNarrow
	data.stats.narrow = totalNarrow
	data.stats.narrowPasses = passes
	return data
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
	local r2 = (c.probeRadius * c.step) ^ 2
	local tol = c.flushTol
	local nWall, nDrop, nBoth, nEdge = 0, 0, 0, 0

	for _, g in pairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			local wallMask, dropMask, edgeMask = 0, 0, 0
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
				local floor, above, below = false, false, false
				for ox = -1, 1 do
					for oz = -1, 1 do
						for _, e in ipairs(live[(bx + ox) .. ":" .. (bz + oz)] or {}) do
							local q = e.cell.pos
							local dx, dz = q.X - p.X, q.Z - p.Z
							if dx * dx + dz * dz <= r2 then
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
									-- The floor continues here, so this is not a
									-- wall or a dropoff -- but if it continues
									-- into a DIFFERENT region it is still an edge
									-- of this one. A ramp running into a floor and
									-- a band seam on a long slope both look like
									-- open ground to the wall/drop tests, and both
									-- are boundaries that have to be crossed
									-- deliberately.
									if cell.region and e.cell.region
										and e.cell.region ~= cell.region then
										edgeMask = bit32.bor(edgeMask, bit32.lshift(1, bit - 1))
									end
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
									if dx * dx + dz * dz <= r2 and e.dead.killer
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
			cell.wallMask, cell.dropMask, cell.edgeMask = wallMask, dropMask, edgeMask
			cell.wall, cell.dropoff = wallMask ~= 0, dropMask ~= 0
			cell.regionEdge = edgeMask ~= 0
			if cell.wall then nWall += 1 end
			if cell.dropoff then nDrop += 1 end
			if cell.wall and cell.dropoff then nBoth += 1 end
			if cell.regionEdge then nEdge += 1 end
		end
	end

	data.stats.wallNodes, data.stats.dropNodes, data.stats.bothNodes = nWall, nDrop, nBoth
	data.stats.regionEdgeNodes = nEdge
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
	for part, sfs in pairs(byPart) do
		local g: Grid? = buildGrid(part, sfs, c, filterAll, probe, op, rpTerrain)
		if g then
			nBlock += 1
		else
			g = buildFallbackGrid(part, sfs, c)
			nFallback += 1
		end
		g = g :: Grid
		grids[part] = g
		nCells += #g.cells
		nDead += #g.dead
	end
	probe:Destroy()

	local nFit = { 0, 0, 0 }
	for _, g in pairs(grids) do
		for _, cell in ipairs(g.cells) do
			cell.fit = fitOf(cell.clearance, c)
			nFit[cell.fit] += 1
		end
	end

	local data = {
		grids = grids, config = c,
		stats = { parts = nBlock + nFallback, framed = nBlock, block = nBlock, fallback = nFallback, cells = nCells, dead = nDead,
			prone = nFit[1], crouch = nFit[2], stand = nFit[3] },
	}
	LocalGrid.pruneNarrow(data, cfg)
	-- regions BEFORE classification: classifyNodes walks every neighbour anyway,
	-- so it can mark the seams between regions in the same pass instead of
	-- paying for a third walk of its own.
	LocalGrid.regions(data, cfg)
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
			if col == PLAIN and not showInterior then continue end
			local dot = Instance.new("Part")
			dot.Anchored = true; dot.CanCollide = false; dot.CanQuery = false; dot.CanTouch = false
			dot.Material = Enum.Material.SmoothPlastic
			dot.Color = col
			dot.Size = Vector3.new(w * step, 0.08, w * step)
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

-- Group cells into regions: connected runs of surface that share a plane.
--
-- Adjacency is the same neighbour lookup classifyNodes uses, so a region flows
-- across part boundaries -- a plaza paved from forty slabs is one region, not
-- forty. Two conditions join a pair: the neighbour has to sit on this cells own
-- plane within flushTol (a step up is a different region, which is what keeps
-- stair treads apart), and the normals have to agree within regionAngle (so a
-- ramp is not swallowed by the floor it runs into).
--
-- Ids are assigned largest-first, so region 1 is the main floor on any map and
-- the numbering is stable enough to compare between bakes.
function LocalGrid.regions(data: any, cfg: Config?)
	local c = merged(cfg)
	if data.config then
		c.step = data.config.step or c.step
		c.flushTol = (cfg and cfg.flushTol) or data.config.flushTol or c.flushTol
		c.regionAngle = (cfg and cfg.regionAngle) or data.config.regionAngle or c.regionAngle
		c.bandHeight = (cfg and cfg.bandHeight) or data.config.bandHeight or c.bandHeight
	end
	local live = buildWorldIndex(data.grids)
	local r2 = (c.probeRadius * c.step) ^ 2
	local tol = c.flushTol
	local cosTol = math.cos(math.rad(c.regionAngle))

	local up: { [any]: any } = {}
	local function find(x)
		local r = x
		while up[r] do r = up[r] end
		while up[x] do up[x], x = r, up[x] end
		return r
	end
	local function union(a, b)
		local ra, rb = find(a), find(b)
		if ra ~= rb then up[ra] = rb end
	end

	local gridOf: { [any]: any } = {}
	for _, g in pairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			gridOf[cell] = g
			for _, d in ipairs(DIR8) do
				local p = neighbourPos(g, cell, d)
				local bx, bz = math.floor(p.X), math.floor(p.Z)
				for ox = -1, 1 do
					for oz = -1, 1 do
						for _, e in ipairs(live[(bx + ox) .. ":" .. (bz + oz)] or {}) do
							local q = e.cell
							local dx, dz = q.pos.X - p.X, q.pos.Z - p.Z
							if dx * dx + dz * dz <= r2
								and math.abs(q.pos.Y - p.Y) <= tol
								and cell.normal:Dot(q.normal) >= cosTol
								and q.fit == cell.fit then
								union(cell, q)
							end
						end
					end
				end
			end
		end
	end

	local members: { [any]: {Cell} } = {}
	for _, g in pairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			local r = find(cell)
			local m = members[r]
			if not m then m = {}; members[r] = m end
			m[#m + 1] = cell
		end
	end
	local groups = {}
	for _, m in pairs(members) do groups[#groups + 1] = m end

	-- Cut regions that climb too far into bands of bandHeight.
	--
	-- Banding alone would leave a band in several disconnected pieces (think of
	-- a ramp that switches back through the same band twice), so each spanning
	-- region is re-connected inside its bands rather than just sliced. Only
	-- regions that actually span are reprocessed -- flat ground, which is nearly
	-- all of a map, never enters this path.
	local band = c.bandHeight
	if band and band > 0 then
		local out = {}
		for _, m in ipairs(groups) do
			local lo, hi = math.huge, -math.huge
			for _, cell in ipairs(m) do
				lo = math.min(lo, cell.pos.Y); hi = math.max(hi, cell.pos.Y)
			end
			if hi - lo <= band then
				out[#out + 1] = m
			else
				local mine, bandOf = {}, {}
				for _, cell in ipairs(m) do
					mine[cell] = true
					bandOf[cell] = math.floor((cell.pos.Y - lo) / band + 1e-6)
				end
				local bup: { [any]: any } = {}
				local function bfind(x)
					local r = x
					while bup[r] do r = bup[r] end
					while bup[x] do bup[x], x = r, bup[x] end
					return r
				end
				for _, cell in ipairs(m) do
					local g = gridOf[cell]
					for _, d in ipairs(DIR8) do
						local p = neighbourPos(g, cell, d)
						local bx, bz = math.floor(p.X), math.floor(p.Z)
						for ox = -1, 1 do
							for oz = -1, 1 do
								for _, e in ipairs(live[(bx + ox) .. ":" .. (bz + oz)] or {}) do
									local q = e.cell
									if mine[q] and bandOf[q] == bandOf[cell]
										and q.fit == cell.fit then
										local dx, dz = q.pos.X - p.X, q.pos.Z - p.Z
										if dx * dx + dz * dz <= r2
											and math.abs(q.pos.Y - p.Y) <= tol then
											local ra, rb = bfind(cell), bfind(q)
											if ra ~= rb then bup[ra] = rb end
										end
									end
								end
							end
						end
					end
				end
				local parts: { [any]: {Cell} } = {}
				for _, cell in ipairs(m) do
					local r = bfind(cell)
					local t = parts[r]
					if not t then t = {}; parts[r] = t end
					t[#t + 1] = cell
				end
				for _, t in pairs(parts) do out[#out + 1] = t end
			end
		end
		groups = out
	end

	table.sort(groups, function(a, b) return #a > #b end)
	local sizes = {}
	for i, m in ipairs(groups) do
		sizes[i] = #m
		for _, cell in ipairs(m) do cell.region = i end
	end

	data.regions = groups
	data.stats.regions = #groups
	data.stats.regionSizes = sizes
	data.stats.largestRegion = sizes[1] or 0
	-- a region too small to stand a footprint in is a fragment, worth counting
	local frag = 0
	for _, n in ipairs(sizes) do
		if n * c.step * c.step < c.minWidth * c.minWidth then frag += 1 end
	end
	data.stats.regionFragments = frag
	return data
end

-- Debug viz. Border cells -- the ones classifyNodes marked wall or dropoff --
-- go in a `Border` folder per part and interior cells in `Interior`, so either
-- layer can be hidden on its own. A run never spans the two, so the split is
-- exact rather than approximate at the seam.
--
-- Cells are merged into runs along the grid's u axis: at step 0.5 a
-- part-per-cell draw is 200k Parts for case5, and a Part costs ~4KB however
-- invisible it is. Colour is per-grid hue with a clearance band for value, so
-- open floor collapses into a handful of long strips and only genuinely broken
-- ground stays granular. Pass `opts.merge = false` for one Part per cell when
-- you need to select or rename an individual node.
function LocalGrid.visualize(data: any, opts: any?, parent: Instance?)
	if typeof(opts) == "Instance" then parent = opts :: Instance; opts = nil end
	local o = opts or {}
	local merge = o.merge ~= false
	-- Colour and foldering follow regions once they exist; by = "part" restores
	-- the per-part hue, which is what you want when the question is which PART a
	-- cell came from rather than what it connects to.
	local byRegion = (o.by ~= "part") and data.stats.regions ~= nil

	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then
		dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root
	end
	local old = dbg:FindFirstChild("LocalGrid")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "LocalGrid"; folder.Parent = dbg

	local step = data.config.step
	-- band: shrink factor and brightness by headroom. Also the merge key -- two
	-- cells only join if they would have been drawn the same.
	-- Posture drives the tile's size and brightness: full and bright for stand,
	-- smaller and dimmer for crouch, smallest and darkest for prone.
	local function band(cell: Cell): (number, number, number)
		local f = cell.fit or 3
		if f >= 3 then return 3, 0.9, 1 end
		if f == 2 then return 2, 0.7, 0.55 end
		return 1, 0.55, 0.28
	end
	-- A cell is border if it has a wall or a dropoff in any of the 8
	-- directions, or if the floor continues there into another region.
	-- classifyNodes has already worked all three out. Cells carry no masks when
	-- the caller skipped classification, and then everything reads as interior.
	local function isBorder(cell: Cell): boolean
		return cell.wall == true or cell.dropoff == true or cell.regionEdge == true
	end

	local function hueOf(n: number): number
		return (n * 0.61803398875) % 1
	end

	local groups: { [string]: Folder } = {}
	local function groupFolder(name: string): Folder
		local f = groups[name]
		if not f then
			f = Instance.new("Folder"); f.Name = name; f.Parent = folder
			groups[name] = f
		end
		return f
	end

	local i, drawn, nBorder = 0, 0, 0
	for part, g in pairs(data.grids) do
		i += 1
		local partHue = hueOf(i)
		local sat = g.fallback and 0.3 or 0.9
		local partName = part.Name
		local layers = {}
		local function layer(owner: string, name: string): Folder
			local key = owner .. "/" .. name
			local f = layers[key]
			if not f then
				local pf = groupFolder(owner)
				local sub = pf:FindFirstChild(name)
				if not sub then
					sub = Instance.new("Folder"); sub.Name = name; sub.Parent = pf
				end
				f = sub :: Folder
				layers[key] = f
			end
			return f
		end
		local oriented = (not g.fallback) and g.n ~= nil

		-- a run is `len` cells along +u starting at `first`, all in one band and
		-- all on the same side of the border/interior split
		local function emit(first: Cell, last: Cell, len: number, w: number, v: number)
			local dot = Instance.new("Part")
			dot.Anchored = true; dot.CanCollide = false; dot.CanQuery = false; dot.CanTouch = false
			-- along the run: full length less one inter-tile gap, so the seams
			-- read the same as they do between unmerged tiles
			local along = len * step - (1 - w) * step
			dot.Size = Vector3.new(along, 0.1, w * step)
			local hue = byRegion and hueOf(first.region or 0) or partHue
			dot.Color = Color3.fromHSV(hue, sat, v)
			-- matte interior so the neon Boundary edges pop over the grid layer;
			-- border nodes get diamond plate, which reads as a distinct surface
			-- at a glance without spending a colour channel that hue and the
			-- clearance band already use.
			dot.Material = isBorder(first) and Enum.Material.DiamondPlate
				or Enum.Material.SmoothPlastic
			local mid = first.pos:Lerp(last.pos, 0.5)
			if oriented then
				dot.CFrame = CFrame.fromMatrix(mid, g.u, g.n)
			else
				dot.CFrame = CFrame.new(mid)
			end
			dot.Name = (len == 1)
				and string.format("c%.1f", first.clearance)
				or string.format("c%.1f_x%d", first.clearance, len)
			local owner = byRegion
				and string.format("r%03d_%s", first.region or 0, FIT_NAME[first.fit or 3])
				or partName
			dot.Parent = layer(owner, isBorder(first) and "Border" or "Interior")
			drawn += 1
			if isBorder(first) then nBorder += len end
		end

		if not merge then
			for _, cell in ipairs(g.cells) do
				local _, w, v = band(cell)
				emit(cell, cell, 1, w, v)
			end
			continue
		end

		-- bucket by row, then walk each row in ui order joining consecutive
		-- cells of the same band. A hole in the row breaks the run, so pruned
		-- and dead cells still show as gaps.
		local rows: { [number]: {Cell} } = {}
		for _, cell in ipairs(g.cells) do
			local r = rows[cell.vi]
			if not r then r = {}; rows[cell.vi] = r end
			r[#r + 1] = cell
		end
		for _, r in pairs(rows) do
			table.sort(r, function(a, b) return a.ui < b.ui end)
			local first, last, len, bi, w, v = nil, nil, 0, nil, 0, 0
			for _, cell in ipairs(r) do
				local cb, cw, cv = band(cell)
				if first and cb == bi and cell.ui == last.ui + 1
					and isBorder(cell) == isBorder(first)
					and cell.region == first.region then
					last, len = cell, len + 1
				else
					if first then emit(first, last, len, w, v) end
					first, last, len, bi, w, v = cell, cell, 1, cb, cw, cv
				end
			end
			if first then emit(first, last, len, w, v) end
		end
	end
	data.stats.vizParts = drawn
	data.stats.vizBorderCells = nBorder
	return folder
end

return LocalGrid
