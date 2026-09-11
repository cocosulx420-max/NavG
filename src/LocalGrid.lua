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
	line: number?,            -- set by LocalGrid.contours: the fitted line this
	                          -- border cell belongs to, unique across the bake
	loop: number?,            -- which boundary loop of its region
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
	connectivity: number?, regionPlanarity: number?, faceAngle: number?,
	minEdge: number?, chordSlack: number?, contourTol: number?, contourWindow: number?,
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
	--
	-- 0.3, not 0.5. The slack is measured from where THIS surface's plane would
	-- continue, and on a 45 degree ramp that point has already dropped 0.354 studs
	-- per 0.5 stud step. Half a stud of slack is then nearly a whole step height,
	-- and a clipramp edge merges with whatever lies 0.34 below it instead of
	-- reading as a border.
	flushTol = 0.3,
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
	regionAngle = 10,
	-- How far a cell's normal may sit from its REGION's normal, as opposed to
	-- from its neighbour's. regionAngle alone is a pairwise test and pairwise
	-- tests chain: on a hill every adjacent pair agrees to a degree or two and
	-- the whole slope becomes one region, which Contour then flattens onto a
	-- single plane. It fails quietly rather than visibly: 26k cells spread over
	-- 20 degrees and 11 studs of relief, contoured as one plane.
	--
	-- Compared against an ANCHOR normal per region, never a running mean. A
	-- running mean drifts along a curve and swallows the whole thing, the same
	-- trap the Contour segmentation avoids.
	regionPlanarity = 5,
	-- How far two surfels' normals may diverge and still be treated as the same
	-- FACE of a part. One grid per part is wrong for anything presenting more
	-- than one walkable face: averaging the normals gives a plane matching
	-- neither, and every stage downstream inherits the tilt.
	--
	-- A box carrying one flat face and one steep face averages to a frame tilted
	-- between them, which puts a Y component into the in-plane axis v. The
	-- footprint test walks along v, so the plane climbs away from a surface with
	-- no relief at all and most of every footprint fails as "too narrow".
	faceAngle = 15,
	-- Tallest rise one region may cover, or 0 to never cut on height. OFF by
	-- default: a ramp or a roof plane is one surface, and slicing it at an
	-- arbitrary altitude splits something that is genuinely continuous and puts
	-- a seam in the middle of a clear run. Set it only if a downstream stage
	-- really does need a region to be roughly one altitude. Flat ground never
	-- reaches this code either way, since its span is zero.
	bandHeight = 0,
	-- Headroom a posture needs. Below crouchHeight a cell is prone-only, and
	-- below minClearance it is not floor at all. These are postures, not
	-- preferences: a crouch tunnel and the room it opens into are different
	-- places to move through even where the floor runs straight between them,
	-- so they are never the same region.
	standHeight = 5,
	crouchHeight = 3,
	-- 4 or 8. See DIR4 above; 4 is what the boundary tracing needs.
	connectivity = 4,
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

-- Split one part's surfels into faces: runs of surfels whose normals agree.
--
-- Compared against a cluster's ANCHOR normal rather than a running mean, for the
-- same reason the region grouping is: a mean drifts across a curve and swallows
-- everything. A part with a single walkable face -- almost all of them -- yields
-- exactly one cluster and behaves as it always did.
local function splitFaces(sfs: {any}, cosFace: number)
	local clusters = {}
	for _, sf in ipairs(sfs) do
		local hit = nil
		for _, cl in ipairs(clusters) do
			if sf.normal:Dot(cl.anchor) >= cosFace then hit = cl; break end
		end
		if not hit then
			hit = { anchor = sf.normal, list = {} }
			clusters[#clusters + 1] = hit
		end
		hit.list[#hit.list + 1] = sf
	end
	return clusters
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
-- Every BasePart has a CFrame, so this works for unions and meshes too. A
-- world-axis lattice would ignore their orientation and staircase the tiles
-- across the part's edges instead of running along them.
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
	-- Grow to the outermost surfel, and NO further. Rounding the cell count up
	-- already leaves up to half a step of slack on each side, and on a part
	-- thinner than the step any added margin dominates its real size: the sample
	-- points land on and past the part's edges, and whether a row survives comes
	-- down to float luck.
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
	local cosFace = math.cos(math.rad(c.faceAngle))

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

	-- Start above the highest stray and reach past the lowest one. On a block
	-- dev is ~0.
	local castH = 2 + dev
	local castLen = castH + dev + 0.5

	for iu = 0, nu - 1 do
		for iv = 0, nv - 1 do
			local p = corner + u * ((iu + 0.5) * step) + v * ((iv + 0.5) * step)
			local res = workspace:Raycast(p + n * castH, -n * castLen, rpPart)
			if not res then continue end
			local slope = math.deg(math.acos(math.clamp(res.Normal:Dot(UP), -1, 1)))
			if not ((slope <= c.maxSlope) or isClip(part)) then continue end
			-- This grid describes ONE face. Its lattice spans the part, so the ray
			-- also lands on the part's other faces; without this those cells would
			-- be built twice, once per face grid, and the duplicates would union
			-- into regions of doubled size sitting on the same surface. A cell
			-- more than faceAngle off this face belongs to another face, and that
			-- face has a grid of its own to claim it.
			if res.Normal:Dot(n) < cosFace then continue end
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

-- 4-CONNECTED is the default, and it is not a simplification. 8-connectivity
-- adds the inner corners of every staircase and makes the tangent fit worse,
-- linearity 0.959 against 0.973, because those corner cells sit off the line the
-- rest of the stretch defines: a boundary loop is built from cell FACES and a
-- diagonal contact has no face to contribute. 8-conn is still the right rule for
-- erosion and for corner-to-corner pinch detection, which is why pruneNarrow, a
-- filled-square test, ignores this setting.
local DIR4 = {
	{ 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 },
}

local function dirsFor(c: any)
	return (c.connectivity == 8) and DIR8 or DIR4
end

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
	for _, g in pairs(grids) do
		for _, cell in ipairs(g.cells) do push(live, cell.pos, { cell = cell, part = g.part }) end
		for _, d in ipairs(g.dead) do push(dead, d.pos, { dead = d, part = g.part }) end
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
-- what a strip of real floor looks like. Width is the only fact that separates
-- them, and without this pass the bake grows cells along stair stringers, ledges
-- and window trim.
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
		-- A FILLED SQUARE, not two 1-D runs through the cell. Two runs ask a
		-- weaker question that a handrail passes: a rail has a long run along its
		-- length, and where it meets a newel post or dies into a wall the
		-- crosswise run leaks onto that neighbour and reaches width, so the middle
		-- of the rail prunes and its ends survive. A post cap cannot complete a
		-- filled square.
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
-- Heights above the neighbour slot at which the SVO is asked whether space is
-- solid, AS MULTIPLES OF THE OCTREE'S LEAF.
--
-- THE PROBE MUST CLEAR THE FLOOR'S OWN LEAF. The octree is conservative: a leaf
-- is marked solid when geometry merely touches it, so the leaf holding the
-- floor's top face reads solid all the way to its ceiling, which can be a full
-- leaf above the surface. A probe below that height is asking whether the floor
-- exists, not whether a wall does, and it answers yes at the rim of every
-- platform on the map.
--
-- Measured: at 0.6 studs against a 1 stud leaf, 7 of 14 hand-checked dropoffs
-- came back solid while a physics probe at the same point found nothing. At 1.5
-- leaves all but one cleared.
--
-- World up, not the surface normal. The question is whether something STANDS
-- there, and things stand along world Y whatever the ramp underneath is doing.
local WALL_PROBE_LEAVES = { 1.5, 2.5 }

function LocalGrid.classifyNodes(data: any, cfg: Config?)
	local c = merged(cfg)
	if data.config then
		c.step = data.config.step or c.step
		c.flushTol = (cfg and cfg.flushTol) or data.config.flushTol or c.flushTol
	end
	local live, dead = buildWorldIndex(data.grids)
	local r2 = (c.probeRadius * c.step) ^ 2
	local tol = c.flushTol
	local dirs = dirsFor(c)
	local nWall, nDrop, nBoth, nEdge, nSvo = 0, 0, 0, 0, 0
	local svo = data.svo

	for _, g in pairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			local wallMask, dropMask, edgeMask = 0, 0, 0
			for bit, d in ipairs(dirs) do
				local p = neighbourPos(g, cell, d)
				local bx, bz = math.floor(p.X), math.floor(p.Z)
				-- BELOW OUTRANKS ABOVE. "Any surface higher than stepTol is a
				-- wall" cannot tell a riser you would bump into from a balcony
				-- three storeys up, so on its own it marks the rim of every
				-- raised platform as a wall.
				--
				-- Live floor BELOW is the giveaway. If floor is visible down there
				-- the space is open and you would fall through it -- a dropoff, no
				-- matter what is overhead. A wall standing at that spot would have
				-- killed that floor, so it would not be live. Order: floor first,
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
					-- NOTHING IN THE CELL GRIDS KNEW, SO ASK SOLID SPACE ITSELF.
					--
					-- Everything above infers a wall from OTHER walkable floor
					-- higher up, or from a dead cell recording what killed it. A
					-- dead cell only exists where this part's own grid sampled
					-- that slot, so where a floor part ends exactly at a wall
					-- part the slot is outside the grid entirely and the code
					-- falls through to dropoff. That is the common case, not a
					-- corner one: 34% of case3's dropoff faces had a wall
					-- standing against them.
					--
					-- The SVO does not care which grid a point falls in, and it
					-- is conservative -- a leaf geometry merely touches is
					-- marked solid -- so it errs toward calling things walls,
					-- which is the safe direction for the offset.
					if not above and not below and svo then
						local leaf = svo.leaf or 1
						for _, mult in ipairs(WALL_PROBE_LEAVES) do
							local h = mult * leaf
							if svo:isSolid(p + Vector3.yAxis * h) then
								above = true
								nSvo += 1
								break
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
	-- how many wall directions ONLY the SVO found, so the fix stays measurable
	data.stats.svoWalls = nSvo
	return data
end

-- Build per-part local grids from an existing floor extraction.
function LocalGrid.fromFloor(floorData: any, parts: {BasePart}, cfg: Config?, tree: any?)
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
	-- An ARRAY now, not a map keyed by part: a part can own several grids, one
	-- per walkable face. Every grid still carries its own `part`.
	local grids: {Grid} = {}
	local cosFace = math.cos(math.rad(c.faceAngle))
	local nBlock, nFallback, nCells, nDead, nParts, nFaces = 0, 0, 0, 0, 0, 0
	-- DETERMINISTIC PART ORDER -- the seed of the whole bake.
	--
	-- `byPart` is keyed by BasePart, and Luau hashes an instance key by pointer,
	-- so `pairs` walked the parts in a different order every Studio run. That
	-- order fixes the grid array, which fixes the order `regions` unions cells
	-- in, which fixes which side is larger at each merge, which fixes the anchor
	-- each new member is judged against. Without a fixed order the same code on
	-- the same map produces different regions, not merely different numbering.
	--
	-- Order by each part's lexicographically smallest surfel instead. A surfel
	-- belongs to exactly one part, so the key is unique across groups and comes
	-- from the geometry rather than from allocation.
	local ordered = {}
	for part, sfs in pairs(byPart) do
		local best = nil
		for _, s in ipairs(sfs) do
			local p = s.pos
			if not best
				or p.X < best.X
				or (p.X == best.X and (p.Z < best.Z or (p.Z == best.Z and p.Y < best.Y))) then
				best = p
			end
		end
		ordered[#ordered + 1] = { part = part, sfs = sfs, key = best }
	end
	table.sort(ordered, function(a, b)
		if a.key.X ~= b.key.X then return a.key.X < b.key.X end
		if a.key.Z ~= b.key.Z then return a.key.Z < b.key.Z end
		return a.key.Y < b.key.Y
	end)
	for _, entry in ipairs(ordered) do
		local part, sfs = entry.part, entry.sfs
		nParts += 1
		local faces = splitFaces(sfs, cosFace)
		nFaces += #faces
		for _, face in ipairs(faces) do
			local g: Grid? = buildGrid(part, face.list, c, filterAll, probe, op, rpTerrain)
			if g then
				nBlock += 1
			else
				g = buildFallbackGrid(part, face.list, c)
				nFallback += 1
			end
			g = g :: Grid
			grids[#grids + 1] = g
			nCells += #g.cells
			nDead += #g.dead
		end
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
		-- kept so later stages can raycast against the same set the bake used
		grids = grids, parts = parts, config = c,
		-- the global solid-space octree, kept so classifyNodes can ask it
		-- whether a wall stands beside a cell. Floor builds it and it used to
		-- be discarded here.
		svo = tree,
		stats = { parts = nParts, grids = nBlock + nFallback, faces = nFaces,
			framed = nBlock, block = nBlock, fallback = nFallback, cells = nCells, dead = nDead,
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
	local data = LocalGrid.fromFloor(floorData, parts, cfg, tree)
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
	local cosPlanar = math.cos(math.rad(c.regionPlanarity))
	local dirs = dirsFor(c)

	local up: { [any]: any } = {}
	-- each root carries the normal of its anchor cell and its size; the anchor
	-- is what new members are judged against, and the larger side keeps its own
	-- anchor on a merge so the reference does not wander
	local anchorN: { [any]: Vector3 } = {}
	local size: { [any]: number } = {}
	local function find(x)
		local r = x
		while up[r] do r = up[r] end
		while up[x] do up[x], x = r, up[x] end
		return r
	end
	local function union(a, b)
		local ra, rb = find(a), find(b)
		if ra == rb then return end
		local na = anchorN[ra] or a.normal
		local nb = anchorN[rb] or b.normal
		-- the two surfaces must be the same surface, not merely locally parallel
		if na:Dot(nb) < cosPlanar then return end
		if (size[ra] or 1) < (size[rb] or 1) then ra, rb = rb, ra; na, nb = nb, na end
		up[rb] = ra
		anchorN[ra] = na
		size[ra] = (size[ra] or 1) + (size[rb] or 1)
	end

	local gridOf: { [any]: any } = {}
	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			gridOf[cell] = g
			for _, d in ipairs(dirs) do
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

	-- `members` is keyed by the root CELL, a table, so `pairs` over it follows
	-- the same pointer hash and shuffled the groups even when the partition was
	-- identical. Collect in first-seen order over the grids, which are now in a
	-- fixed order, so the same partition always comes out in the same order.
	local members: { [any]: {Cell} } = {}
	local groups = {}
	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			local r = find(cell)
			local m = members[r]
			if not m then m = {}; members[r] = m; groups[#groups + 1] = m end
			m[#m + 1] = cell
		end
	end

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
					for _, d in ipairs(dirs) do
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
				-- first-seen collection, for the same reason as above
				local parts: { [any]: {Cell} } = {}
				for _, cell in ipairs(m) do
					local r = bfind(cell)
					local t = parts[r]
					if not t then t = {}; parts[r] = t; out[#out + 1] = t end
					t[#t + 1] = cell
				end
			end
		end
		groups = out
	end

	-- SIZE ALONE IS NOT A TOTAL ORDER. case5 has fifteen regions of exactly 144
	-- cells, and `table.sort` is not stable, so ties alone reshuffled the ids
	-- between bakes even when every region was identical. Break them on the
	-- group's lexicographically smallest cell, the way Contour.lattice picks its
	-- anchor: a property of the geometry, not of the iteration.
	local anchorKey: { [any]: Vector3 } = {}
	for _, m in ipairs(groups) do
		local best = nil
		for _, cell in ipairs(m) do
			local p = cell.pos
			if not best
				or p.X < best.X
				or (p.X == best.X and (p.Z < best.Z or (p.Z == best.Z and p.Y < best.Y))) then
				best = p
			end
		end
		anchorKey[m] = best
	end
	table.sort(groups, function(a, b)
		if #a ~= #b then return #a > #b end
		local ka, kb = anchorKey[a], anchorKey[b]
		if ka.X ~= kb.X then return ka.X < kb.X end
		if ka.Z ~= kb.Z then return ka.Z < kb.Z end
		return ka.Y < kb.Y
	end)
	local sizes = {}
	for i, m in ipairs(groups) do
		sizes[i] = #m
		for _, cell in ipairs(m) do cell.region = i end
	end

	-- Recount postures over the SURVIVING cells: fit is assigned before
	-- pruneNarrow runs, so the counts taken there include cells the width test
	-- went on to remove.
	local nFit = { 0, 0, 0 }
	for _, g in pairs(data.grids) do
		for _, cell in ipairs(g.cells) do nFit[cell.fit or 3] += 1 end
	end
	data.stats.prone, data.stats.crouch, data.stats.stand = nFit[1], nFit[2], nFit[3]

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

-- Hand each region to Contour: cells -> closed loops -> a tangent per border
-- cell -> lines.
--
-- Contour wants ONE lattice per region, and a LocalGrid region does not have
-- one: it spans parts, each with its own origin and its own in-plane rotation.
-- The anchor cell frame is used for the whole region, so cells from a part
-- rotated against it land off-lattice and can share a slot. `collapsed` counts
-- exactly that, per region, and is the number to watch before trusting any of
-- the line output.
function LocalGrid.contours(data: any, cfg: Config?)
	local Contour = require(script.Parent:WaitForChild("Contour"))
	local c = merged(cfg)
	local step = data.config.step

	-- ONE frame per region, from the region's OWN mean normal.
	--
	-- Contour takes its lattice axes from whichever cell it anchors on -- the
	-- lexicographically smallest, i.e. a corner -- and then rebuilds every line
	-- endpoint as origin + u*i + v*j. If those axes are tilted against the
	-- surface the rebuild drifts LINEARLY with distance from that corner, and a
	-- line sinks into the floor at the far end while sitting correctly at the
	-- near one.
	--
	-- Cells within a region may differ by up to regionPlanarity, so no single
	-- cell's frame speaks for the region. Averaging the normals does: the plane
	-- then sits through the middle of the deviation instead of being pinned to
	-- whatever tilt the corner cell happened to have.
	local sumN: { [number]: Vector3 } = {}
	local axisU: { [number]: Vector3 } = {}
	local cellsOf: { [number]: {any} } = {}
	for _, g in pairs(data.grids) do
		local u = g.u or Vector3.xAxis
		for _, cell in ipairs(g.cells) do
			local r = cell.region
			if r then
				sumN[r] = (sumN[r] or Vector3.zero) + cell.normal
				-- first grid to reach the region supplies the in-plane axis, so
				-- this depends on the grid order fromFloor fixed
				axisU[r] = axisU[r] or u
				local t = cellsOf[r]
				if not t then t = {}; cellsOf[r] = t end
				t[#t + 1] = cell
			end
		end
	end

	local byRegion: { [number]: {any} } = {}
	for r, cells in pairs(cellsOf) do
		local n = sumN[r]
		n = (n.Magnitude > 1e-4) and n.Unit or Vector3.yAxis
		local u = axisU[r] - n * axisU[r]:Dot(n)
		if u.Magnitude < 1e-3 then
			u = Vector3.xAxis - n * Vector3.xAxis:Dot(n)
			if u.Magnitude < 1e-3 then u = Vector3.zAxis - n * Vector3.zAxis:Dot(n) end
		end
		u = u.Unit
		local t = {}
		for _, cell in ipairs(cells) do
			-- the cell rides along on the entry, so the lines Contour returns can
			-- be attributed back to the nodes they were fitted from
			t[#t + 1] = { cf = CFrame.fromMatrix(cell.pos, u, n), cell = cell }
		end
		byRegion[r] = t
	end

	-- one filter for the whole bake: the line validator casts against exactly the
	-- parts the floor was built from, so scenery that was never walkable cannot
	-- veto a line
	local rayFilter = nil
	if data.parts then
		rayFilter = RaycastParams.new()
		rayFilter.FilterType = Enum.RaycastFilterType.Include
		rayFilter.FilterDescendantsInstances = data.parts
	end

	local out, nCells, nSlots, nLines, nLoops, nFailed = {}, 0, 0, 0, 0, 0
	local worstCollapse, worstAt = 0, 0
	for r, parts in pairs(byRegion) do
		-- pass the contour knobs through, so a caller can A/B the fitting stage
		-- (minEdge = 0 disables the dissolve) without editing Contour itself
		local ok, res = pcall(Contour.run, parts, {
			leaf = step,
			rayFilter = rayFilter,
			minEdge = c.minEdge,
			chordSlack = c.chordSlack,
			tol = c.contourTol,
			window = c.contourWindow,
		})
		if ok and typeof(res) == "table" then
			local slots = 0
			for _ in pairs(res.lattice.occ) do slots += 1 end
			res.stats.slots = slots
			res.stats.collapsed = #parts - slots
			local frac = res.stats.collapsed / math.max(1, #parts)
			if frac > worstCollapse then worstCollapse = frac; worstAt = r end
			-- stamp the line back onto the cells. Line ids are made unique across
			-- the whole bake rather than per region, so a colour identifies one
			-- line on the map instead of one line within some region.
			local L = res.lattice
			for li, seq in ipairs(res.lines) do
				for _, k in ipairs(seq) do
					local e = L.partAt[k]
					if e and e.cell then e.cell.line = nLines + li end
				end
			end
			for lo, seq in ipairs(res.loops) do
				for _, k in ipairs(seq) do
					local e = L.partAt[k]
					if e and e.cell then e.cell.loop = nLoops + lo end
				end
			end

			out[r] = res
			nCells += #parts; nSlots += slots
			nLines += res.stats.lines; nLoops += res.stats.loops
		else
			out[r] = { failed = tostring(res) }
			nFailed += 1
		end
	end

	data.contours = out
	data.stats.contourCells = nCells
	data.stats.contourSlots = nSlots
	data.stats.contourCollapsed = nCells - nSlots
	data.stats.contourLines = nLines
	data.stats.contourLoops = nLoops
	data.stats.contourFailed = nFailed
	data.stats.worstCollapse = worstCollapse
	data.stats.worstCollapseRegion = worstAt
	return data
end

-- Draw the cells that did NOT survive, so a gap in the walkable surface says
-- what removed it instead of just being absent.
--
--   red    -- killed by cover: something overhangs within minClearance, so the
--             floor is there but the headroom is not. This is what puts holes
--             in an otherwise clean border run.
--   orange -- pruned as too narrow to stand on (rails, ledges, stringers).
--
-- Nothing is drawn where the surface simply ends; open air is not a dead cell,
-- and a border with nothing beyond it is an ordinary dropoff.
function LocalGrid.drawDead(data: any, opts: any?, parent: Instance?)
	if typeof(opts) == "Instance" then parent = opts :: Instance; opts = nil end
	local o = opts or {}
	local lift = o.lift or 0.05
	local showNarrow = o.narrow ~= false
	local showCover = o.cover ~= false

	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then
		dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root
	end
	local old = dbg:FindFirstChild("Dead")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "Dead"; folder.Parent = dbg
	local coverF = Instance.new("Folder"); coverF.Name = "Cover"; coverF.Parent = folder
	local narrowF = Instance.new("Folder"); narrowF.Name = "Narrow"; narrowF.Parent = folder

	local step = data.config.step
	local nCover, nNarrow = 0, 0

	local function tile(pos: Vector3, up: Vector3, u: Vector3?, col: Color3, name: string, into: Folder)
		local d = Instance.new("Part")
		d.Anchored = true; d.CanCollide = false; d.CanQuery = false; d.CanTouch = false
		d.Size = Vector3.new(0.8 * step, 0.1, 0.8 * step)
		d.Color = col
		d.Material = Enum.Material.Neon
		if u then
			d.CFrame = CFrame.fromMatrix(pos + up * lift, u, up)
		else
			d.CFrame = CFrame.new(pos + up * lift)
		end
		d.Name = name
		d.Parent = into
	end

	for _, g in pairs(data.grids) do
		local up = g.n or Vector3.yAxis
		local u = g.u
		if showCover then
			for _, d in ipairs(g.dead) do
				local who = d.killer and d.killer.Name or "nil"
				tile(d.pos, up, u, Color3.fromRGB(255, 40, 40), "cover_" .. who, coverF)
				nCover += 1
			end
		end
		if showNarrow then
			for _, cell in ipairs(g.narrow or {}) do
				tile(cell.pos, up, u, Color3.fromRGB(255, 150, 0), "narrow", narrowF)
				nNarrow += 1
			end
		end
	end

	data.stats.deadDrawn = nCover
	data.stats.narrowDrawn = nNarrow
	return folder, nCover, nNarrow
end

-- Draw the neighbour graph: one segment per node-to-neighbour link.
--
-- `dirs` is 8 or 4 independently of the bake's own connectivity, because this
-- answers a different question -- what a node can actually see around it --
-- from the one classifyNodes and regions ask. Each undirected link is drawn
-- once: only the four forward directions emit, and the backward four would
-- duplicate them.
--
-- A link is drawn only where a real cell sits at the neighbour position, so a
-- gap in the graph is a gap in the floor, and a link that jumps a wall is
-- visible as a segment crossing it.
function LocalGrid.drawNeighbours(data: any, opts: any?, parent: Instance?)
	if typeof(opts) == "Instance" then parent = opts :: Instance; opts = nil end
	local o = opts or {}
	local lift = o.lift or 0.35
	local dirs = (o.dirs == 4) and DIR4 or DIR8
	local thick = o.thickness or 0.06
	-- On a map the size of case5 a link per cell per direction is three quarters
	-- of a million Parts. `borderOnly` keeps the links that carry the question --
	-- what a node ON AN EDGE can reach -- and drops the interior lattice, which
	-- is a uniform grid everywhere and shows nothing by being drawn.
	local borderOnly = o.borderOnly == true

	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then
		dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root
	end
	local old = dbg:FindFirstChild("Neighbours")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "Neighbours"; folder.Parent = dbg

	local c = data.config
	local live = buildWorldIndex(data.grids)
	local r2 = (c.probeRadius * c.step) ^ 2
	local tol = c.flushTol

	-- forward half of the ring only, so each link is emitted once
	local half = {}
	for i = 1, #dirs // 2 do half[i] = dirs[i] end

	local n, crossRegion = 0, 0
	for _, g in pairs(data.grids) do
		local up = g.n or Vector3.yAxis
		for _, cell in ipairs(g.cells) do
			if borderOnly and not (cell.wall or cell.dropoff or cell.regionEdge) then
				continue
			end
			for _, d in ipairs(half) do
				local p = neighbourPos(g, cell, d)
				local bx, bz = math.floor(p.X), math.floor(p.Z)
				local best, bestD2 = nil, math.huge
				for ox = -1, 1 do
					for oz = -1, 1 do
						for _, e in ipairs(live[(bx + ox) .. ":" .. (bz + oz)] or {}) do
							local q = e.cell
							if q ~= cell then
								local dx, dz = q.pos.X - p.X, q.pos.Z - p.Z
								local dd = dx * dx + dz * dz
								if dd <= r2 and math.abs(q.pos.Y - p.Y) <= tol and dd < bestD2 then
									best, bestD2 = q, dd
								end
							end
						end
					end
				end
				if best then
					local a = cell.pos + up * lift
					local b = best.pos + up * lift
					local v = b - a
					local len = v.Magnitude
					if len > 1e-3 then
						local seg = Instance.new("Part")
						seg.Anchored = true; seg.CanCollide = false
						seg.CanQuery = false; seg.CanTouch = false
						seg.Size = Vector3.new(thick, thick, len)
						seg.CFrame = CFrame.lookAt(a + v * 0.5, b)
						seg.Material = Enum.Material.Neon
						-- a link that leaves the region is the interesting one
						if best.region ~= cell.region then
							seg.Color = Color3.fromRGB(255, 60, 60)
							seg.Name = "link_cross"
							crossRegion += 1
						else
							seg.Color = Color3.fromHSV(((cell.region or 0) * 0.61803398875) % 1, 0.5, 1)
							seg.Name = "link"
						end
						seg.Parent = folder
						n += 1
					end
				end
			end
		end
	end
	data.stats.neighbourLinks = n
	data.stats.neighbourCrossRegion = crossRegion
	return folder, n
end

-- Stand a pillar on every line endpoint: the corners of the fitted boundary.
--
-- Endpoints are shared -- where two lines meet, both name the same point -- so
-- they are deduplicated on a quantised position and the count reported is
-- distinct corners, not line ends. A corner where three or more lines meet is
-- one pillar, which is what makes a junction countable.
--
-- Pillars rise from the same lift as the lines so a corner reads as the post
-- holding up the outline rather than as something floating beside it.
function LocalGrid.drawCorners(data: any, opts: any?, parent: Instance?)
	if typeof(opts) == "Instance" then parent = opts :: Instance; opts = nil end
	local o = opts or {}
	local lift = o.lift or 1
	local height = o.height or 4
	local thick = o.thickness or 0.15
	local col = o.color or Color3.fromRGB(255, 25, 25)

	local only = nil
	if o.only and typeof(o.only) == "table" then
		only = {}
		for k, v in pairs(o.only) do
			if typeof(k) == "number" and typeof(v) == "number" then only[v] = true
			else only[k] = v and true or nil end
		end
	end

	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then
		dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root
	end
	local old = dbg:FindFirstChild("Corners")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "Corners"; folder.Parent = dbg

	local n = 0
	for r, res in pairs(data.contours or {}) do
		if res.edges and (not only or only[r]) then
			local rf = Instance.new("Folder"); rf.Name = string.format("r%03d", r); rf.Parent = folder
			local up = (res.lattice and res.lattice.up) or Vector3.yAxis
			local seen, hits = {}, {}
			for _, e in ipairs(res.edges) do
				for _, pt in ipairs({ e.a, e.b }) do
					local key = string.format("%d:%d:%d",
						math.round(pt.X * 8), math.round(pt.Y * 8), math.round(pt.Z * 8))
					if not seen[key] then
						seen[key] = pt
						hits[key] = 1
					else
						hits[key] += 1
					end
				end
			end
			for key, pt in pairs(seen) do
				local post = Instance.new("Part")
				post.Anchored = true; post.CanCollide = false
				post.CanQuery = false; post.CanTouch = false
				post.Size = Vector3.new(thick, height, thick)
				-- oriented to the surface, so a corner on a ramp or a roof stands
				-- off its face instead of leaning through it
				post.CFrame = CFrame.fromMatrix(pt + up * (lift + height * 0.5), Vector3.xAxis, up)
				post.Color = col
				post.Material = Enum.Material.Neon
				post.Name = string.format("corner_x%d", hits[key])
				post.Parent = rf
				n += 1
			end
		end
	end
	data.stats.corners = n
	return folder, n
end

-- Draw the fitted boundary as neon segments, one colour per region.
--
-- `lift` raises them off the surface along its OWN normal rather than along
-- world up, so the outline of a ramp or a roof stands off its face by the same
-- amount as a floor's does instead of shearing across it.
function LocalGrid.drawContours(data: any, opts: any?, parent: Instance?)
	if typeof(opts) == "Instance" then parent = opts :: Instance; opts = nil end
	local o = opts or {}
	local lift = o.lift or 1
	-- `only` restricts the draw to a set of region ids, given as a list or as a
	-- map of id -> true. Useful for inspecting one region without the rest of
	-- the map on top of it.
	local only = nil
	if o.only then
		only = {}
		if typeof(o.only) == "table" then
			for k, v in pairs(o.only) do
				if typeof(k) == "number" and typeof(v) == "number" then only[v] = true
				else only[k] = v and true or nil end
			end
		end
	end
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then
		dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root
	end
	local old = dbg:FindFirstChild("Contours")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "Contours"; folder.Parent = dbg

	local n = 0
	for r, res in pairs(data.contours or {}) do
		if res.edges and (not only or only[r]) then
			local rf = Instance.new("Folder"); rf.Name = string.format("r%03d", r); rf.Parent = folder
			local col = Color3.fromHSV((r * 0.61803398875) % 1, 0.9, 1)
			local off = (res.lattice and res.lattice.up or Vector3.yAxis) * lift
			for _, e in ipairs(res.edges) do
				local a, b = e.a + off, e.b + off
				local d = b - a
				local len = d.Magnitude
				if len > 1e-3 then
					local seg = Instance.new("Part")
					seg.Anchored = true; seg.CanCollide = false
					seg.CanQuery = false; seg.CanTouch = false
					seg.Size = Vector3.new(0.12, 0.12, len)
					seg.CFrame = CFrame.lookAt(a + d * 0.5, b)
					seg.Color = col
					seg.Material = Enum.Material.Neon
					seg.Name = string.format("e%d_%.1f", e.id or 0, len)
					seg.Parent = rf
					n += 1
				end
			end
		end
	end
	return folder, n
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
	local byLine = o.by == "line"
	local byRegion = (not byLine) and (o.by ~= "part") and data.stats.regions ~= nil
	-- Skip the interior entirely rather than building it and deleting it after.
	-- The interior is roughly nine tenths of the cells, so filtering here rather
	-- than afterwards is what keeps a border-only draw inside a single call.
	local borderOnly = o.borderOnly == true

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
	for _, g in pairs(data.grids) do
		i += 1
		local partHue = hueOf(i)
		local sat = g.fallback and 0.3 or 0.9
		local partName = g.part.Name
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
			if byLine then
				-- a cell Contour never put on a line is not part of the boundary
				-- description, so it recedes rather than competing for attention
				if first.line then
					dot.Color = Color3.fromHSV(hueOf(first.line), 0.95, 1)
				else
					dot.Color = Color3.fromRGB(70, 70, 78)
				end
			else
				local hue = byRegion and hueOf(first.region or 0) or partHue
				dot.Color = Color3.fromHSV(hue, sat, v)
			end
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
			local owner, layerName
			if byLine then
				owner = string.format("r%03d_%s", first.region or 0, FIT_NAME[first.fit or 3])
				layerName = first.line and "Line" or "Unassigned"
				dot.Name = first.line and ("l" .. first.line) or dot.Name
			else
				owner = byRegion and string.format("r%03d_%s", first.region or 0, FIT_NAME[first.fit or 3]) or partName
				layerName = isBorder(first) and "Border" or "Interior"
			end
			dot.Parent = layer(owner, layerName)
			drawn += 1
			if isBorder(first) then nBorder += len end
		end

		if not merge then
			for _, cell in ipairs(g.cells) do
				if not (borderOnly and not isBorder(cell)) then
					local _, w, v = band(cell)
					emit(cell, cell, 1, w, v)
				end
			end
			continue
		end

		-- bucket by row, then walk each row in ui order joining consecutive
		-- cells of the same band. A hole in the row breaks the run, so pruned
		-- and dead cells still show as gaps.
		local rows: { [number]: {Cell} } = {}
		for _, cell in ipairs(g.cells) do
			if not (borderOnly and not isBorder(cell)) then
				local r = rows[cell.vi]
				if not r then r = {}; rows[cell.vi] = r end
				r[#r + 1] = cell
			end
		end
		for _, r in pairs(rows) do
			table.sort(r, function(a, b) return a.ui < b.ui end)
			local first, last, len, bi, w, v = nil, nil, 0, nil, 0, 0
			for _, cell in ipairs(r) do
				local cb, cw, cv = band(cell)
				if first and cb == bi and cell.ui == last.ui + 1
					and isBorder(cell) == isBorder(first)
					and cell.region == first.region
					and cell.line == first.line then
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
