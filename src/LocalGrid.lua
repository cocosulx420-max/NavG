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
	fpos: Vector3?,           -- centre of the footprint the kill test judged
	fu: number?,              -- that footprint's size along the grid's u
	fv: number?,              -- and along v; both <= step, clipped to the part
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

-- Parts between yields in fromFloor when a caller opts into progress.
LocalGrid.partBudget = 25

-- Lattice positions between yields INSIDE buildGrid, same opt-in.
--
-- partBudget alone is not enough on a dense map. A part is not a unit of work:
-- case6's cost per cell runs 40x case5's because the kill test probes whatever
-- geometry is packed around it, so one large face in a crowded street can hold
-- a minute of work with no yield point in it. buildGrid is 78% of fromFloor and
-- was the only stage still able to wedge Studio by itself.
--
-- SAFE TO PAUSE HERE, despite the warning on partBudget, because the grid is
-- local to buildGrid until it is returned: fromFloor appends it to `grids` only
-- after the call, and `data` does not exist yet, so nothing can observe a
-- half-built lattice.
--
-- IDENTICAL OUTPUT IS THE WHOLE REQUIREMENT. A chunk is a contiguous run of
-- lattice positions in the EXISTING scan order, so cells append in exactly the
-- order they did before and every cell's own answer depends only on its own
-- position -- no neighbour is read here. Yielding cannot reorder or change
-- anything; it only decides where the pauses fall.
LocalGrid.cellBudget = 4000

-- Widest collapsed node, in CELLS, and a power of two so the descent bottoms out
-- at exactly one cell.
--
-- 16. It was 4 for most of this module's life, and the reason was sound at the
-- time: 8 and 16 covered 200114 of case5's 200498 slots, so they lost floor, and
-- 4 was the largest node that reproduced it exactly.
--
-- THAT LOSS WAS A BUG, NOT A PROPERTY OF BIG NODES. `pruneNarrow` compared a
-- probe point against a covering node's CENTRE height, which on a slope is not
-- the surface height there, so the wider the node the more of case5's ClipRamp
-- it pruned as narrow. Fixed; every K now reproduces the floor exactly, measured
-- as a slot-for-slot diff rather than as an area.
--
-- On case5, bake against the uniform lattice: K=1 31.1s / 200498 cells,
-- K=4 8.1s / 45035, K=16 7.2s / 38159. On case3 the difference between 4 and 16
-- is inside the noise -- it is a small dense map and finds only 19 nodes of 8
-- cells -- so 16 costs nothing there and pays on open ground.
LocalGrid.maxNodeCells = 16

-- Collapse faces that are NOT blocks. ON, and it is the difference between the
-- adaptive grid earning its keep on a dense map and doing nothing there.
--
-- `collapseOK` was restricted to block faces because a block hands it two things
-- for free: `supportHalf` is the real edge of the floor, so the extent test is
-- exact, and the face is flat, so one centre raycast speaks for every cell in
-- the node. On a mesh or a union `supportHalf` is the BOUNDING BOX -- it claims
-- floor across the opening of an arch -- and the surface can curve or step.
--
-- Both are MEASURED instead: `faceIsWhole` fires one down-ray per lattice cell
-- across the node and its halo and demands every one land on this part, agree on
-- normal within `faceAngle`, and be coplanar within `flushTol`. Affordable only
-- because those rays are cached and are the same rays `emit` is about to fire,
-- so a REFUSED collapse costs almost nothing.
--
-- case3 is the map that shows it, being two thirds mesh and union floor:
-- 1.30s / 7961 cells with this off, 1.07s / 4826 with it on -- 20% off the bake
-- where blocks-only bought 2%. case5, with only 1260 mesh cells, moves 3%.
-- Both reproduce the uniform lattice slot for slot, gaining nothing and losing
-- nothing, with no posture relabelled.
--
-- THE ONE THING TO KNOW: this is a SAMPLE at cell resolution where the block
-- path has a PROOF. Nothing wider than a cell can hide, but a hole NARROWER than
-- a cell -- grating, a slotted mesh floor, a lattice railing underfoot -- would
-- be missed and a node would span it. Neither test map has such geometry, so
-- the zeros above are evidence and not proof. Bake a perforated map against
-- `collapseShaped = false` before trusting it on one.
LocalGrid.collapseShaped = true

-- the four corners of a node, as (u, v) signs
local CORNERS = { { 1, 1 }, { 1, -1 }, { -1, 1 }, { -1, -1 } }

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

-- Exact box-vs-box overlap by separating axis.
--
-- Rays sample points and a wall is not a point: a 0.375 stud slab can cut the
-- side of a 0.5 stud tile without passing through any of its five sample
-- columns, which is how 194 case3 nodes kept standing inside block walls. Most
-- walls ARE blocks, so for those the question has an exact answer and there is
-- no reason to sample it. Anything that is not a block -- mesh, union, wedge --
-- has no box to test and stays with the ray samples, where a bounds test would
-- condemn the floor under every archway.
local function boxOverlap(cfA: CFrame, sA: Vector3, cfB: CFrame, sB: Vector3): boolean
	local hA, hB = sA * 0.5, sB * 0.5
	local A = { cfA.RightVector, cfA.UpVector, cfA.LookVector }
	local B = { cfB.RightVector, cfB.UpVector, cfB.LookVector }
	local t = cfB.Position - cfA.Position
	local function separated(ax: Vector3): boolean
		local rA = math.abs(A[1]:Dot(ax)) * hA.X + math.abs(A[2]:Dot(ax)) * hA.Y + math.abs(A[3]:Dot(ax)) * hA.Z
		local rB = math.abs(B[1]:Dot(ax)) * hB.X + math.abs(B[2]:Dot(ax)) * hB.Y + math.abs(B[3]:Dot(ax)) * hB.Z
		-- MINUS the tolerance, so boxes that merely touch count as SEPARATED.
		-- Abutting is the normal case for floor against wall and for one stair
		-- plank against the next, and with the tile clipped to its part those
		-- contacts are exact: with the sign the other way every stair tread in
		-- case3 died on face-to-face contact alone.
		return math.abs(t:Dot(ax)) > rA + rB - 1e-4
	end
	for _, ax in ipairs(A) do if separated(ax) then return false end end
	for _, ax in ipairs(B) do if separated(ax) then return false end end
	for _, a in ipairs(A) do
		for _, b in ipairs(B) do
			local x = a:Cross(b)
			if x.Magnitude > 1e-6 and separated(x.Unit) then return false end
		end
	end
	return true
end

local function isBlock(p: BasePart): boolean
	return p:IsA("Part") and (p :: Part).Shape == Enum.PartType.Block
end

-- How far the part itself reaches from its centre along `dir` -- the support of
-- its box. Exact for a block, and for anything else the bounding box, which as
-- a CLIP errs towards a smaller tile and so towards keeping a node.
local function supportHalf(part: BasePart, dir: Vector3): number
	local cf, s = part.CFrame, part.Size
	return 0.5 * (math.abs(dir:Dot(cf.RightVector)) * s.X
		+ math.abs(dir:Dot(cf.UpVector)) * s.Y
		+ math.abs(dir:Dot(cf.LookVector)) * s.Z)
end

-- A WEDGE IS HALF A BOX, and case3 has 48 of them holding up its ramps. Tested
-- as "not a block" they fell through to the mesh path and nodes stood inside
-- them; tested as their full box they would condemn the floor under the open
-- half. Both are avoidable: a wedge is a triangular prism, so it has an exact
-- answer too.
--
-- Which half is solid was measured, not assumed -- a ray along the part's local
-- X spans the prism, so it hits exactly where the cross-section is solid. The
-- answer is `y * hz <= z * hy`: the triangle (-hy,-hz), (-hy,+hz), (+hy,+hz),
-- extruded along X. `cf.LookVector` is local -Z, hence `az`.
local function wedgePoints(part: BasePart): ({Vector3}, {Vector3})
	local cf, sz = part.CFrame, part.Size
	local hx, hy, hz = sz.X * 0.5, sz.Y * 0.5, sz.Z * 0.5
	local ax, ay, az = cf.RightVector, cf.UpVector, -cf.LookVector
	local verts = table.create(6)
	for _, x in ipairs({ -hx, hx }) do
		verts[#verts + 1] = cf:PointToWorldSpace(Vector3.new(x, -hy, -hz))
		verts[#verts + 1] = cf:PointToWorldSpace(Vector3.new(x, -hy, hz))
		verts[#verts + 1] = cf:PointToWorldSpace(Vector3.new(x, hy, hz))
	end
	local slopeN = (ay * hz - az * hy)
	local slopeE = (ay * hy + az * hz)
	local dirs = { ax, ay, az,
		slopeN.Magnitude > 1e-6 and slopeN.Unit or ay,
		slopeE.Magnitude > 1e-6 and slopeE.Unit or az }
	return verts, dirs
end

-- Separating axis over two vertex sets. Touching counts as separated, for the
-- same reason it does in boxOverlap: floor meets wall flush everywhere.
local function hullsApart(a: {Vector3}, b: {Vector3}, axes: {Vector3}): boolean
	for _, ax in ipairs(axes) do
		local aLo, aHi = math.huge, -math.huge
		for _, p in ipairs(a) do
			local d = p:Dot(ax)
			if d < aLo then aLo = d end
			if d > aHi then aHi = d end
		end
		local bLo, bHi = math.huge, -math.huge
		for _, p in ipairs(b) do
			local d = p:Dot(ax)
			if d < bLo then bLo = d end
			if d > bHi then bHi = d end
		end
		if aLo > bHi - 1e-4 or bLo > aHi - 1e-4 then return true end
	end
	return false
end

local function boxPoints(cf: CFrame, size: Vector3): ({Vector3}, {Vector3})
	local h = size * 0.5
	local verts = table.create(8)
	for _, x in ipairs({ -h.X, h.X }) do
		for _, y in ipairs({ -h.Y, h.Y }) do
			for _, z in ipairs({ -h.Z, h.Z }) do
				verts[#verts + 1] = cf:PointToWorldSpace(Vector3.new(x, y, z))
			end
		end
	end
	return verts, { cf.RightVector, cf.UpVector, cf.LookVector }
end

local function wedgeOverlap(tileCF: CFrame, tileSize: Vector3, wedge: BasePart): boolean
	local bv, bd = boxPoints(tileCF, tileSize)
	local wv, wd = wedgePoints(wedge)
	local axes = {}
	for _, d in ipairs(bd) do axes[#axes + 1] = d end
	for _, d in ipairs(wd) do axes[#axes + 1] = d end
	for _, p in ipairs(bd) do
		for _, q in ipairs(wd) do
			local x = p:Cross(q)
			if x.Magnitude > 1e-6 then axes[#axes + 1] = x.Unit end
		end
	end
	return not hullsApart(bv, wv, axes)
end

local function isWedge(p: BasePart): boolean
	return p:IsA("Part") and (p :: Part).Shape == Enum.PartType.Wedge
end

local function buildGrid(part: BasePart, surfels: {any}, c: any, filterAll: RaycastParams, probe: BasePart, op: OverlapParams, rpTerrain: RaycastParams?): Grid?
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

	-- reused by the headroom narrow phase, one candidate at a time
	local rpOne = RaycastParams.new()
	rpOne.FilterType = Enum.RaycastFilterType.Include

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

	-- ONE DOWN-RAY PER LATTICE CELL, CACHED. `emit` needs it for every cell it
	-- builds and `faceIsWhole` needs it for every position of a node and its
	-- halo, and those are the same rays: a 4-cell node probes what its 2-cell
	-- children probe, which is what their cells probe. Without the cache the
	-- shaped path pays for the same position up to three times and then `emit`
	-- pays a fourth. `workspace:Raycast` is deterministic, so caching cannot
	-- change a verdict -- `false` is stored for a miss so it is not re-cast.
	local surf: { [string]: any } = {}
	local function surfaceAt(iu: number, iv: number)
		local key = iu .. ":" .. iv
		local hit = surf[key]
		if hit == nil then
			local q = corner + u * ((iu + 0.5) * step) + v * ((iv + 0.5) * step)
			hit = workspace:Raycast(q + n * castH, -n * castLen, rpPart) or false
			surf[key] = hit
		end
		return hit or nil
	end

	-- See LocalGrid.cellBudget. Counts positions VISITED, not cells kept: the
	-- expensive part of a rejected position is the raycast it already paid for,
	-- so counting survivors would leave a face that rejects everything with no
	-- yield at all.
	local onProgress = c.onProgress
	local visited = 0

	-- Build ONE node covering cells [iu, iu+k-1] x [iv, iv+k-1]. k == 1 is the
	-- ordinary cell and every test below runs exactly as it always has; k > 1 is
	-- a collapsed interior node, reached only once `collapseOK` has PROVED that
	-- nothing disturbs it, so the same tests all return the same answers over
	-- the whole span.
	local function emit(iu: number, iv: number, k: number)
		if onProgress then
			visited += k * k
			if visited % LocalGrid.cellBudget < k * k then onProgress(visited, nil) end
		end
		local span = k * step
		local p = corner + u * ((iu + k * 0.5) * step) + v * ((iv + k * 0.5) * step)
		local res
		if k == 1 then
			res = surfaceAt(iu, iv)
		else
			res = workspace:Raycast(p + n * castH, -n * castLen, rpPart)
		end
		if not res then return end
		local slope = math.deg(math.acos(math.clamp(res.Normal:Dot(UP), -1, 1)))
		if not ((slope <= c.maxSlope) or isClip(part)) then return end
		-- This grid describes ONE face. Its lattice spans the part, so the ray
		-- also lands on the part's other faces; without this those cells would
		-- be built twice, once per face grid, and the duplicates would union
		-- into regions of doubled size sitting on the same surface. A cell
		-- more than faceAngle off this face belongs to another face, and that
		-- face has a grid of its own to claim it.
		if res.Normal:Dot(n) < cosFace then return end
		-- HEADROOM. Both of the probes this replaces could miss the solid
		-- standing ON the cell, and case3 shipped `stand` nodes buried in
		-- geometry because of it:
		--
		--   * a 0.05 needle through GetPartsInPart, which does not reliably
		--     report a solid it sits inside -- it returned nothing for a
		--     wooden pillar a 0.6 bounds box finds every time;
		--   * an up-ray from 0.15 above the surface, which returns nothing
		--     when that origin is INSIDE the cover. A pillar standing on
		--     this floor and a slab whose underside IS the cell's plane both
		--     swallow the origin, so the ray flew on and reported the next
		--     thing up: 7.35 and 15.40 studs of "headroom" under solid.
		--
		-- Broad phase by bounds, because GetPartBoundsInBox is the one query
		-- that sees every solid. Narrow phase per candidate, because bounds
		-- alone would condemn the floor under an archway:
		--
		--   underside found  -> blocks only if it is inside the band
		--   none, but the candidate occupies this column from above
		--                    -> no underside above the cell means the cell is
		--                       INSIDE it
		--   neither          -> bounds overlap with nothing over the cell
		-- THE WHOLE TILE, NOT ITS CENTRE COLUMN. A node is a place to stand,
		-- so a wall that cuts any part of it deletes it. One centre sample
		-- left 956 of case3's 9224 nodes standing in geometry -- every cell a
		-- wall clipped without covering its middle.
		-- THE BAND STARTS AT THE SURFACE, not at 0.1 above it. The 0.1
		-- was a toe gap inherited from the needle probe, meant to keep the
		-- probe off the floor it stands on -- but the grid's own part is
		-- excluded by name anyway, so all the gap bought was a blind
		-- sliver. A skirting plate 0.375 thick at the base of a wall,
		-- 19 studs of it, rose 0.078 studs through the tread and sat
		-- entirely inside that sliver: the nodes were visibly buried in it
		-- and every probe reported clear.
		local bandLo = 0.02
		local bandH = c.minClearance - bandLo
		-- CLIP THE TILE TO THE FACE IT SITS ON. The lattice rounds the cell
		-- count up, so a tile at the rim overhangs the part by up to half a
		-- step, and judging a node on that overhang judges it on floor it
		-- does not own. A 1.75 stud stair tread is four rows deep and the
		-- last row pokes 0.125 studs into the NEXT step: tested unclipped,
		-- every tread lost its back row and pruneNarrow then took the whole
		-- tread for being under minWidth. The stairs disappeared.
		--
		-- Clip to the PART, not to `uExt`/`vExt`: those are the lattice's
		-- extents, already padded up to a whole number of steps (a 1.75 stud
		-- tread reports 2.0), so clipping to them left the same 0.125 studs
		-- of overhang and the same dead treads.
		local du = (p - surfaceCenter):Dot(u)
		local dv = (p - surfaceCenter):Dot(v)
		local uLim = math.min(uExt, supportHalf(part, u))
		local vLim = math.min(vExt, supportHalf(part, v))
		local uLo = math.max(du - span * 0.5, -uLim)
		local uHi = math.min(du + span * 0.5, uLim)
		local vLo = math.max(dv - span * 0.5, -vLim)
		local vHi = math.min(dv + span * 0.5, vLim)
		local uW = math.max(uHi - uLo, 1e-3)
		local vW = math.max(vHi - vLo, 1e-3)
		local tileCtr = res.Position
			+ u * ((uLo + uHi) * 0.5 - du)
			+ v * ((vLo + vHi) * 0.5 - dv)
		local hu, hv = uW * 0.49, vW * 0.49
		local cols = {
			tileCtr,
			tileCtr + u * hu + v * hv,
			tileCtr + u * hu - v * hv,
			tileCtr - u * hu + v * hv,
			tileCtr - u * hu - v * hv,
		}
		local tileCF = CFrame.fromMatrix(tileCtr + n * (bandLo + bandH * 0.5), u, n)
		local tileSize = Vector3.new(uW, bandH, vW)
		local killer: Instance? = nil
		local probed, volHit = false, nil :: Instance?
		for _, cand in ipairs(workspace:GetPartBoundsInBox(
			CFrame.new(tileCtr + UP * (bandLo + bandH * 0.5)),
			Vector3.new(uW, bandH, vW), op)) do
			if cand ~= part then
				if isBlock(cand) then
					if boxOverlap(tileCF, tileSize, cand.CFrame, cand.Size) then
						killer = cand
						break
					end
					continue
				end
				if isWedge(cand) then
					if wedgeOverlap(tileCF, tileSize, cand) then
						killer = cand
						break
					end
					continue
				end
				-- A mesh or a union has no box to test, so take the one
				-- true positive that is cheap: GetPartsInPart misses solids
				-- (that is what put nodes inside a pillar to begin with) but
				-- it never invents one, so a hit here is a kill and a miss
				-- falls through to the samples. Worth the call: it catches
				-- the flare of a pillar base that all five columns thread.
				if not probed then
					probed = true
					probe.Size = tileSize
					probe.CFrame = tileCF
					for _, h in ipairs(workspace:GetPartsInPart(probe, op)) do
						if h ~= part then volHit = h; break end
					end
				end
				if volHit then killer = volHit; break end
				rpOne.FilterDescendantsInstances = { cand }
				-- SWEEP THE TILE HORIZONTALLY. A vertical sample can only
				-- miss a vertical wall, and a mesh panel 0.1 studs thick
				-- threads all five columns: 72 case3 nodes stood inside
				-- mesh walls that way. A ray ACROSS the tile crosses such a
				-- panel whatever its thickness.
				--
				-- Fired from just outside each edge, and a hit counts only
				-- if it lands strictly INSIDE the tile -- a wall flush with
				-- the edge is what every floor does where it meets a wall,
				-- and killing those is erosion, which is off.
				local swept = false
				for _, h in ipairs({ bandLo + 0.05, bandH * 0.5, bandH - 0.05 }) do
					local base = tileCtr + n * h
					for _, a in ipairs({ { u, uW }, { v, vW } }) do
						local axis, w = a[1], a[2]
						for _, sgn in ipairs({ 1, -1 }) do
							local from = base - axis * sgn * (w * 0.5 + 0.05)
							local hit = workspace:Raycast(from, axis * sgn * (w + 0.1), rpOne)
							if hit then
								local into = hit.Distance - 0.05
								if into > 1e-3 and into < w - 1e-3 then
									swept = true
									break
								end
							end
						end
						if swept then break end
					end
					if swept then break end
				end
				if swept then killer = cand; break end
				for _, p0 in ipairs(cols) do
					local under = workspace:Raycast(p0 + UP * bandLo, UP * c.clearCap, rpOne)
					local blocks
					if under then
						blocks = under.Distance + bandLo < c.minClearance
					else
						blocks = workspace:Raycast(p0 + UP * c.clearCap,
							-UP * (c.clearCap - 0.05), rpOne) ~= nil
					end
					if blocks then killer = cand; break end
				end
				if killer then break end
			end
		end
		if killer then
			kill(iu, iv, res.Position, killer)
			return
		end
		local upRes = workspace:Raycast(res.Position + Vector3.new(0, 0.15, 0), UP * c.clearCap, filterAll)
		local clearance = upRes and upRes.Distance or c.clearCap
		local cover: Instance? = upRes and upRes.Instance or nil
		if rpTerrain then
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
		end
		if clearance < c.minClearance then
			kill(iu, iv, res.Position, cover)
			return
		end
		local cell: Cell = {
			ui = iu, vi = iv, pos = res.Position, normal = res.Normal,
			slope = slope, clearance = clearance, cover = cover,
			-- The FOOTPRINT the kill test judged: the tile clipped to the
			-- part. Kept so the drawing can show what was actually tested.
			-- Without it the viz draws a full step tile for a rim cell and
			-- the node appears to stand inside the wall it merely abuts,
			-- which reads exactly like a bug in the kill test.
			fpos = tileCtr, fu = uW, fv = vW,
			-- How wide this node is. Absent on a plain cell would mean every
			-- consumer has to know the grid's step, so it is always written;
			-- `cellCovers` and the neighbour lookups key off it being > step.
			su = span, sv = span,
		}
		grid.cells[#grid.cells + 1] = cell
		grid.index[string.format("%d:%d", iu, iv)] = cell
	end

	-- ---- adaptive descent -------------------------------------------------
	--
	-- buildGrid is 78% of fromFloor and its cost is one batch of world queries
	-- per lattice position, so a large flat face pays thousands of them to get
	-- the same answer every time. Roughly nine tenths of a face is interior.
	--
	-- COLLAPSE IS PROVED, NOT SAMPLED. `GetPartBoundsInBox` is the one query the
	-- kill test already trusts to see every solid (see the HEADROOM note above),
	-- so running it over the WHOLE node's band and finding nothing but this part
	-- proves that no geometry disturbs any cell inside it -- the same verdict
	-- the per-cell tests would have reached, in one query instead of k*k. No
	-- corner sampling is involved, and none would have been sound.
	--
	-- BLOCK FACES ONLY, and `isBlock` rather than `SVO.isBlockPart`: the face has
	-- to BE the lattice rectangle for the extent test to be exact, and planar for
	-- one centre raycast to speak for the whole node. A cylinder, a wedge, a
	-- union or a mesh satisfies neither -- its floor is an arbitrary subset of
	-- its bounding rectangle and its surface can curve -- so those keep the
	-- uniform path.
	local shaped = LocalGrid.collapseShaped and not isBlock(part)
	local canCollapse = isBlock(part) or shaped
	local uLimAll = math.min(uExt, supportHalf(part, u))
	local vLimAll = math.min(vExt, supportHalf(part, v))
	local bandLoN = 0.02
	local bandHN = c.minClearance - bandLoN

	-- World-axis half extents of an oriented box, so a bounds query over a node
	-- on a tilted face cannot under-cover it.
	local function aabbHalf(cf: CFrame, size: Vector3): Vector3
		local h = size * 0.5
		local x, y, z = cf.RightVector, cf.UpVector, cf.LookVector
		return Vector3.new(
			math.abs(x.X) * h.X + math.abs(y.X) * h.Y + math.abs(z.X) * h.Z,
			math.abs(x.Y) * h.X + math.abs(y.Y) * h.Y + math.abs(z.Y) * h.Z,
			math.abs(x.Z) * h.X + math.abs(y.Z) * h.Y + math.abs(z.Z) * h.Z)
	end
	-- A BOUNDING BOX IS NOT GEOMETRY, and this test used to treat it as one:
	-- anything `GetPartBoundsInBox` returned refused the collapse. That is a
	-- broadphase query -- it reports every part whose AXIS-ALIGNED BOUNDS meet
	-- the region -- so a mesh or a union vetoed nodes from studs away, its box
	-- being mostly air: an archway, a railing, the flare of a lamp post. Those
	-- nodes then split all the way to single cells and the per-cell tests they
	-- split into accepted every one of them, because the per-cell path has had a
	-- narrow phase since the kill test was fixed. The subdivision bought nothing
	-- but time and a denser lattice than the floor deserves.
	--
	-- So the broad phase stays a bounds query -- it is the only one that sees
	-- every solid -- and each candidate is then asked for real, by the same rules
	-- the kill test uses:
	--
	--   block, wedge -> exact, by separating axis
	--   mesh, union  -> one volume probe, then RAYS across the box on the cell
	--                   lattice in all three axes. A ray hits real collision
	--                   geometry, which is the whole point of the change, and a
	--                   lattice at the cell pitch resolves what the per-cell
	--                   tests would have resolved -- so a node collapses only
	--                   where the cells inside it would have lived anyway.
	--
	-- ONE RAY LATTICE FOR ALL THE MESHES AT ONCE, not one per candidate: the
	-- question is whether ANY of them is in the box, so they go into the filter
	-- together and a crowded node costs the same as a lonely one.
	--
	-- Refusing is still the cheap direction -- it only subdivides -- so anything
	-- the rays cannot answer keeps refusing.
	local function raysHitBox(filter: {BasePart}, cf: CFrame, size: Vector3): boolean
		rpOne.FilterDescendantsInstances = filter
		local axis = { cf.RightVector, cf.UpVector, cf.LookVector }
		local ext = { size.X, size.Y, size.Z }
		local eps = 0.02
		for i = 1, 3 do
			local j, k = i % 3 + 1, (i + 1) % 3 + 1
			local len = ext[i]
			local nj = math.max(1, math.ceil(ext[j] / step - 1e-6))
			local nk = math.max(1, math.ceil(ext[k] / step - 1e-6))
			for a = 0, nj - 1 do
				for b = 0, nk - 1 do
					local mid = cf.Position
						+ axis[j] * (((a + 0.5) / nj - 0.5) * ext[j])
						+ axis[k] * (((b + 0.5) / nk - 0.5) * ext[k])
					local from = mid - axis[i] * (len * 0.5 + eps)
					if workspace:Raycast(from, axis[i] * (len + eps * 2), rpOne) then
						return true
					end
				end
			end
		end
		return false
	end

	local function clearBox(cf: CFrame, size: Vector3): boolean
		local meshes: {BasePart}? = nil
		local half = aabbHalf(cf, size)
		for _, cand in ipairs(workspace:GetPartBoundsInBox(CFrame.new(cf.Position), half * 2, op)) do
			if cand ~= part then
				if isBlock(cand) then
					if boxOverlap(cf, size, cand.CFrame, cand.Size) then return false end
				elseif isWedge(cand) then
					if wedgeOverlap(cf, size, cand) then return false end
				else
					local m = meshes
					if not m then m = {}; meshes = m end
					m[#m + 1] = cand
				end
			end
		end
		if not meshes then return true end
		-- The cheap true positive first, exactly as the kill test takes it:
		-- GetPartsInPart misses solids, so it cannot clear the box, but it never
		-- invents one, so a hit ends the question without casting anything.
		-- Blocks and wedges are already answered exactly above, so a report of
		-- one here is noise -- flush contact -- and is ignored.
		probe.Size = size
		probe.CFrame = cf
		for _, hit in ipairs(workspace:GetPartsInPart(probe, op)) do
			if hit ~= part and not isBlock(hit) and not isWedge(hit) then return false end
		end
		return not raysHitBox(meshes, cf, size)
	end

	-- Is the node and its halo one unbroken, flat piece of THIS face? Every
	-- lattice cell has to answer yes, which is what replaces the two guarantees
	-- a block's shape would have given. Runs LAST in `collapseOK`, after the
	-- cheap box tests, because it is the expensive one -- and mostly free when it
	-- fails, since the rays it casts are the ones `emit` is about to cast anyway.
	local function faceIsWhole(iu: number, iv: number, k: number, p: Vector3): boolean
		for a = -1, k do
			for b = -1, k do
				local res = surfaceAt(iu + a, iv + b)
				if not res then return false end
				if res.Normal:Dot(n) < cosFace then return false end
				if math.abs((res.Position - p):Dot(n)) > c.flushTol then return false end
				local sl = math.deg(math.acos(math.clamp(res.Normal:Dot(UP), -1, 1)))
				if not ((sl <= c.maxSlope) or isClip(part)) then return false end
			end
		end
		return true
	end

	local function collapseOK(iu: number, iv: number, k: number): boolean
		if not canCollapse then return false end
		if iu + k > nu or iv + k > nv then return false end
		local span = k * step
		local h = span * 0.5
		-- EVERY TEST BELOW IS RUN OVER A ONE-CELL HALO, not over the node itself.
		--
		-- A coarse node emits ONE face per direction, spanning its whole edge, so
		-- an edge that is half live floor and half killed strip has no right
		-- answer: suppressing the face punches a hole in the boundary, emitting it
		-- walls off floor that is there. The node has to be STRICTLY interior.
		--
		-- Proving the halo is clear and on the face proves every cell abutting the
		-- node is live floor, so no edge of it can ever be exposed and it never
		-- emits a face at all. That is what keeps the 0.5 ring at the border and
		-- leaves Boundary's corner invariant untouched.
		local ho = h + step
		local p = corner + u * ((iu + k * 0.5) * step) + v * ((iv + k * 0.5) * step)
		-- 1. the node AND its halo entirely over the part's own face. The lattice
		--    rounds the cell count up, so a node at the rim would otherwise be
		--    judged on floor the part does not own -- the trap the per-cell tile
		--    clip already guards.
		local du = (p - surfaceCenter):Dot(u)
		local dv = (p - surfaceCenter):Dot(v)
		if math.abs(du) + ho > uLimAll or math.abs(dv) + ho > vLimAll then return false end
		-- 2. nothing standing in the headroom band over the node or its halo.
		--    Tested in the GRID'S OWN FRAME, not as a world box: on a tilted face
		--    the world box that contains this one is half again as wide, and every
		--    stud of that surplus is a chance to meet something the node does not
		--    touch. The bounds query inside still runs on the world box.
		if not clearBox(CFrame.fromMatrix(p + n * (bandLoN + bandHN * 0.5), u, n),
			Vector3.new(ho * 2, bandHN, ho * 2)) then
			return false
		end
		-- 3. the node has to AGREE WITH ITS CENTRE ON POSTURE. Not be open to the
		--    sky -- that is what it used to ask, and it is the wrong question
		--    anywhere with a ceiling.
		--
		--    `col` was standHeight flat. A crouch space HAS less than standHeight
		--    of headroom and a prone space less than crouchHeight, by definition,
		--    so the test could not be passed on that floor at all and every one of
		--    those cells stayed 0.5 studs across: on case5 all 4474 prone cells
		--    were k1, on case3 all 183 prone and all 283 crouch. A low room is not
		--    a ragged one, and only its BORDER -- where the cover starts or stops
		--    -- has any reason to be fine.
		--
		--    What the collapse actually owes is the claim `emit` makes: one
		--    `clearance` for the node, read off ONE cast at its centre, and the
		--    `fit` derived from it. So measure that clearance here, and ask the
		--    box for the FLOOR OF THE CENTRE'S OWN BAND -- standHeight, crouch-
		--    Height or minClearance -- rather than always the highest of the
		--    three. No cell in the node can then be a worse posture than the
		--    centre's.
		--
		--    THE CORNERS ARE SAMPLED TOO, for the other half of the claim. The box
		--    is a lower bound and cannot prove a cell is not BETTER than the
		--    centre, so a node straddling the lip of an overhang would collapse
		--    and label open floor `crouch`. Four casts at the node's own corners,
		--    all required to land in the same band, is what keeps the fine ring at
		--    the edge of the cover and nowhere else. Sampling, not proof -- the
		--    same standard the per-cell path holds itself to.
		--
		--    THE COLUMN FOLLOWS THE FACE. It used to be a world-axis box with a
		--    flat bottom at the node centre's height, which is right only on level
		--    floor: over a tilted face the floor rises across the footprint, so the
		--    uphill end of that box sat UNDERGROUND. On case5's 26.5 degree
		--    ClipRamp the bottom dug 0.33 / 0.45 / 0.67 / 1.12 / 2.01 studs under
		--    the floor for a node of 1 / 2 / 4 / 8 / 16 cells, and what it found
		--    down there was the staircase the clip ramp is laid over -- a tread
		--    0.71 studs BELOW the walking surface, with five clear studs over the
		--    node's head. Every column refusal on every ramp was a stair block,
		--    and the two 45 degree ramps, where the wedge is as deep as the node is
		--    wide, collapsed nothing at all: 3480 cells that are one flat plane.
		--
		--    Aligned to `n` the bottom face lies ON the floor everywhere, so the
		--    box can only ever hold space above it.
		--
		--    SCALED BY 1/|n.Y|, because a horizontal ceiling `v` studs up sits
		--    `v / |n.Y|` away along the normal: at standHeight the box would reach
		--    only standHeight * |n.Y| vertically -- 3.54 studs at 45 degrees -- and
		--    a node could claim `stand` over a cell that is really crouch. Free on
		--    case5, which reproduces the same node counts either way.
		--
		--    Still an approximation: a true vertical prism leans in-plane as it
		--    rises and this box does not, so an overhang above a big node's uphill
		--    edge on a steep face can be missed. Exact would be one vertical box
		--    per strip across the gradient, k boxes instead of one.
		local function clearanceAt(q: Vector3): number
			local from = q + UP * 0.15
			local hit = workspace:Raycast(from, UP * c.clearCap, filterAll)
			local d = hit and hit.Distance or c.clearCap
			if rpTerrain then
				local t = workspace:Raycast(from, UP * c.clearCap, rpTerrain)
				if t and t.Distance < d then d = t.Distance end
			end
			return d
		end
		local fit = fitOf(clearanceAt(p), c)
		local hc = h * 0.98
		for _, sgn in ipairs(CORNERS) do
			if fitOf(clearanceAt(p + u * (hc * sgn[1]) + v * (hc * sgn[2])), c) ~= fit then
				return false
			end
		end
		local need = (fit >= 3 and c.standHeight) or (fit == 2 and c.crouchHeight) or c.minClearance
		local col = need / math.max(math.abs(n.Y), 0.2)
		if not clearBox(CFrame.fromMatrix(p + n * (bandLoN + col * 0.5), u, n),
			Vector3.new(ho * 2, col, ho * 2)) then
			return false
		end
		-- 4. terrain is invisible to a bounds query, so a hit forces the exact
		--    path rather than being silently ignored.
		if rpTerrain and workspace:Raycast(p + n * 0.15, UP * col, rpTerrain) then return false end
		-- 5. and on a face whose shape proves nothing, the surface itself.
		if shaped and not faceIsWhole(iu, iv, k, p) then return false end
		return true
	end

	local function descend(iu: number, iv: number, k: number)
		if k == 1 then
			if iu < nu and iv < nv then emit(iu, iv, 1) end
			return
		end
		if collapseOK(iu, iv, k) then
			emit(iu, iv, k)
			return
		end
		local q = k // 2
		descend(iu, iv, q)
		descend(iu + q, iv, q)
		descend(iu, iv + q, q)
		descend(iu + q, iv + q, q)
	end

	local K = LocalGrid.maxNodeCells
	for iu = 0, nu - 1, K do
		for iv = 0, nv - 1, K do
			descend(iu, iv, K)
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
	-- A CELL GOES IN EVERY BUCKET IT COVERS, not just the one holding its centre.
	--
	-- Every lookup scans the 3x3 block of 1-stud buckets around the probe point,
	-- which finds a 0.5 cell because its centre is never more than a stud away.
	-- A coarse cell's centre can be four studs from its own edge, so bucketing it
	-- by centre alone hides it from every cell it actually abuts -- and each one
	-- then reports no neighbour and rings the node with a boundary it should not
	-- have. Identical for a uniform cell, whose footprint spans one bucket.
	local function pushCell(g: any, cell: any, v: any)
		local su, sv = cell.su, cell.sv
		if not su or not sv or (su <= g.step and sv <= g.step) then
			push(live, cell.pos, v)
			return
		end
		local hu, hv = su * 0.5, sv * 0.5
		local gu = g.u or Vector3.xAxis
		local gv = g.v or Vector3.zAxis
		local ax = math.abs(gu.X) * hu + math.abs(gv.X) * hv
		local az = math.abs(gu.Z) * hu + math.abs(gv.Z) * hv
		local p = cell.pos
		for bx = math.floor(p.X - ax), math.floor(p.X + ax) do
			for bz = math.floor(p.Z - az), math.floor(p.Z + az) do
				local k = bx .. ":" .. bz
				local b = live[k]
				if not b then b = {}; live[k] = b end
				b[#b + 1] = v
			end
		end
	end

	for _, g in pairs(grids) do
		-- `g` as well as `g.part`: a cell's footprint is measured in its own
		-- grid's in-plane axes, and a coarse cell cannot be tested without them.
		for _, cell in ipairs(g.cells) do pushCell(g, cell, { cell = cell, part = g.part, g = g }) end
		-- dead cells are always one step across: a node only collapses where
		-- nothing kills it, so no coarse cell is ever killed.
		for _, d in ipairs(g.dead) do push(dead, d.pos, { dead = d, part = g.part, g = g }) end
	end
	return live, dead
end

-- Does `p` fall inside this cell's own footprint?
--
-- THE NEIGHBOUR LOOKUPS ARE CENTRE-PROXIMITY TESTS: they ask whether some cell's
-- CENTRE sits within probeRadius of where a neighbour should be. That is exact
-- while every cell is one `step` square, and wrong the moment one is not -- a
-- 0.5 cell beside a 4-stud cell looks 0.5 studs inward and finds the big cell's
-- centre 2 studs away, so it reports no neighbour and emits a boundary face into
-- the middle of solid floor.
--
-- Deliberately answers FALSE for any cell that is still one step across, so on a
-- uniform lattice this is dead code and the verdicts are bit-identical. A
-- uniform cell's footprint lies inside the proximity radius anyway (half-diagonal
-- 0.354 against a 0.375 reach), so it could never have added a match.
local function cellCovers(g: any, cell: any, p: Vector3): boolean
	local su, sv = cell.su, cell.sv
	if not su or not sv then return false end
	if su <= g.step and sv <= g.step then return false end
	local d = p - cell.pos
	local du, dv
	if not g.fallback and g.u and g.v then
		du, dv = d:Dot(g.u), d:Dot(g.v)
	else
		du, dv = d.X, d.Z
	end
	return math.abs(du) <= su * 0.5 and math.abs(dv) <= sv * 0.5
end
LocalGrid.cellCovers = cellCovers

-- Where the neighbour in local direction d would be, in world space. Block
-- grids step along their own face axes; fallback grids are world-aligned.
-- Only a part with a degenerate Size ends up world-aligned now.
-- Steps from THIS cell's own edge, not by a fixed `step`: half of this cell plus
-- half of a minimum-size one lands in the middle of whatever abuts it. For a
-- uniform cell (su == sv == step) that is `d * step` exactly as before.
local function neighbourPos(g: Grid, cell: Cell, d: {number}): Vector3
	local ou = d[1] * ((cell.su or g.step) + g.step) * 0.5
	local ov = d[2] * ((cell.sv or g.step) + g.step) * 0.5
	if not g.fallback and g.u and g.v then
		return cell.pos + g.u * ou + g.v * ov
	end
	return cell.pos + Vector3.new(ou, 0, ov)
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
						-- A COARSE NODE IS A PLANE, NOT A POINT. `cellCovers` proves
						-- p is inside the node's footprint, but the flush test then
						-- compared p against the node's CENTRE height, which on a
						-- slope is not the surface height at p: half of a 2 stud node
						-- on case5's 26.5 degree ClipRamp is 0.5 studs of rise against
						-- a 0.3 flushTol, so the node refused to hold up the very
						-- cells it abuts and the ramp lost its outer two columns to
						-- the narrow prune. Evaluate the node's own plane at p
						-- instead. Dead code on a uniform lattice, where `cellCovers`
						-- is always false and this is the centre it always was.
						-- The centre-proximity test stays FIRST and unchanged: it is
						-- the answer for every one-step cell, which is nearly all of
						-- them, and `cellCovers` is only asked when it has failed.
						if dx * dx + dz * dz <= r2 and math.abs(q.Y - p.Y) <= tol then
							found = true
							break
						end
						if cellCovers(e.g, e.cell, p) then
							local nrm = e.cell.normal
							local qy = q.Y
							if nrm and math.abs(nrm.Y) > 1e-3 then
								qy -= ((p.X - q.X) * nrm.X + (p.Z - q.Z) * nrm.Z) / nrm.Y
							end
							if math.abs(qy - p.Y) <= tol then
								found = true
								break
							end
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
		-- SAMPLE ON THE FINE LATTICE, WHATEVER SIZE THIS CELL IS.
		--
		-- The footprint is walked in `step` strides from the cell's position, which
		-- for a one-step cell lands every probe exactly on a neighbouring cell's
		-- CENTRE. A coarse cell's centre sits on a lattice VERTEX instead, so every
		-- probe lands on a vertex too -- 0.354 studs from the nearest centre against
		-- a 0.375 reach. Six percent of margin, which case5's rotated grids exceed
		-- routinely: 15344 cells pruned against 560, and the cascade ran the pass
		-- limit out at 8 instead of 2.
		--
		-- Shifting the anchor to the centre of the cell's first sub-cell puts the
		-- probes back on centres. Zero shift for a one-step cell, so unchanged.
		local function standable(u: Vector3, v: Vector3, cell: Cell): boolean
			-- A NODE AN AGENT WIDE IS STANDABLE BY CONSTRUCTION. `collapseOK`
			-- already proved the node and a one-cell halo are one uninterrupted
			-- piece of this part's face, so a footprint fits inside it and no
			-- probing can say otherwise. Without this the window is sized to the
			-- AGENT but anchored on the node's first sub-cell, so for a node wider
			-- than the agent it never reaches the node's own middle and judges it
			-- on ground outside its corner: at maxNodeCells 8 that deleted six
			-- whole 4-stud nodes, 384 cells, and broke a loop.
			if cell.su and cell.su >= c.minWidth and cell.sv and cell.sv >= c.minWidth then
				return true
			end
			local bu = cell.su and (cell.su - step) * 0.5 or 0
			local bv = cell.sv and (cell.sv - step) * 0.5 or 0
			local base = cell.pos - u * bu - v * bv
			if footFits(u, v, base - u * (half * step) - v * (half * step)) then
				return true
			end
			for a = 0, k - 1 do
				for b = 0, k - 1 do
					if (a ~= half or b ~= half)
						and footFits(u, v, base - u * (a * step) - v * (b * step)) then
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

-- How far OUT to look, in neighbour steps.
--
-- One step lands in the slot the floor would have continued into, which is
-- where a flush wall stands. It is not where every wall stands: case3 has a
-- pillar set back about a stud from the floor edge, so the one step probe found
-- empty air in the gap and called a blocked edge a dropoff. Two steps finds it.
--
-- Not further. Past a stud or so the thing is no longer against this edge, and
-- the gap between is ground an agent could legitimately be standing on.
local WALL_PROBE_STEPS = { 1, 2 }

-- A SURFACE THIS FAR ABOVE THE NEIGHBOURING SLOT IS A STEP, NOT A WALL.
--
-- A stair riser is solid, so the octree calls it a wall and it is not wrong: you
-- would walk into it. But you would also walk UP it, and a wall you can climb
-- must not make the floor stand back from it -- erode a node off the back of
-- every tread and the treads survive while the LINKS between them do not. Both
-- of case3's flights came apart that way, into pieces of 109, 52, 27 and 23
-- cells where they had been whole.
--
-- 1.5 studs, matching the step gate the connectivity check uses and under the
-- 2.0 a Roblox humanoid climbs by default. case3's treads rise 1.37 to 1.38, so
-- 59 of flight A's 112 wall directions are risers by this test and 53 are real
-- masonry.
local STEP_UP = 1.5

-- Exported so FaceKind splits step from drop on the SAME number this gate
-- uses. Two copies of it would drift, and the difference between them is the
-- difference between a two-way link and a one-way fall.
LocalGrid.STEP_UP = STEP_UP

-- THE BOX THE WALL TEST ACTUALLY OCCUPIES, once the octree says something might
-- be there. Reaching from the floor's own edge outward, ankle to head.
--
-- The octree is CONSERVATIVE -- a leaf geometry merely touches is marked solid
-- -- and its leaves are a stud across while a cell is half that. So a wall
-- running along one axis makes the leaf beside it solid in the OTHER axis too,
-- and the probe for a free direction lands inside it. Measured on case3: 124 of
-- 1033 wall directions, 12%, had no geometry in them at all, and they cluster
-- where a run meets a corner.
--
-- Conservatism is only safe in one direction. "Might be solid" has to be
-- checked; "definitely empty" can be trusted, which is why the octree still
-- runs first and the box is only tested where it said yes. 1033 checks cost
-- 0.01s.
local PROBE_DEPTH = 1.0   -- studs outward from the floor edge; catches a set-back pillar
local PROBE_HEIGHT = 2.0  -- ankle to head
local PROBE_RISE = 1.2    -- centre height above the slot

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
	local nWall, nDrop, nBoth, nEdge, nSvo, nVeto = 0, 0, 0, 0, 0, 0
	local nProbed, nRefused = 0, 0
	local svo = data.svo

	-- Included, not excluded: the probe can only ever hit the parts this bake
	-- was built from, so a debug drawing in the workspace cannot register as a
	-- wall however it is parented.
	local op = OverlapParams.new()
	op.FilterType = Enum.RaycastFilterType.Include
	op.FilterDescendantsInstances = data.parts or {}
	op.MaxParts = 1
	-- The same filter WITHOUT the cap, for the bounds query below. MaxParts = 1
	-- would hand back whichever candidate the broadphase happened to return
	-- first, so a mesh standing in front of a wall would hide the wall.
	local opAll = OverlapParams.new()
	opAll.FilterType = Enum.RaycastFilterType.Include
	opAll.FilterDescendantsInstances = data.parts or {}
	local box = Instance.new("Part")
	box.Name = "NVGN_WallProbe"
	box.Size = Vector3.new(c.step * 0.9, PROBE_HEIGHT, PROBE_DEPTH)
	box.Anchored = true
	box.CanCollide = false; box.CanQuery = false; box.CanTouch = false
	box.Transparency = 1
	box.Parent = workspace

	for _, g in pairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			local wallMask, dropMask, edgeMask, stepMask = 0, 0, 0, 0
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
				-- the LOWEST floor found above this slot, for the step test
				local upMin = math.huge
				for ox = -1, 1 do
					for oz = -1, 1 do
						for _, e in ipairs(live[(bx + ox) .. ":" .. (bz + oz)] or {}) do
							local q = e.cell.pos
							local dx, dz = q.X - p.X, q.Z - p.Z
							if dx * dx + dz * dz <= r2 or cellCovers(e.g, e.cell, p) then
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
									if dy < upMin then upMin = dy end
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
					-- SOLID SPACE DECIDES, BOTH WAYS.
					--
					-- The tests above only ever INFER a wall: from other
					-- walkable floor higher up, or from a dead cell recording
					-- what killed it. Both inferences were measured wrong on
					-- case3, in opposite directions.
					--
					-- Too few: where a floor part ends exactly at a wall part
					-- the neighbouring slot is outside that grid, so there is
					-- no live cell and no dead cell, and the code fell through
					-- to dropoff. 34% of dropoff faces had a wall against them.
					--
					-- Too many: a platform rim with a balcony somewhere above
					-- has "a surface higher than stepTol" and was called a wall
					-- with nothing beside it at all, and a union lying flush at
					-- floor level leaves a killed cell that read as masonry
					-- overhead. Hand-checked, a physics probe found NOTHING at
					-- any height beside several of them.
					--
					-- So ask the octree first, then put a box where it said
					-- yes and see whether anything is actually in it.
					local solid = false
					if svo then
						local leaf = svo.leaf or 1
						local out = p - cell.pos
						local maybe = false
						for _, k in ipairs(WALL_PROBE_STEPS) do
							local q = cell.pos + out * k
							for _, mult in ipairs(WALL_PROBE_LEAVES) do
								if svo:isSolid(q + Vector3.yAxis * mult * leaf) then
									maybe = true
									break
								end
							end
							if maybe then break end
						end
						-- THE EXACT TEST NO LONGER WAITS FOR THE OCTREE. Gating it on
						-- `maybe` meant a wall the SVO missed was never looked for at
						-- all: 67 plain blocks sat within 0.9 studs of a face labelled
						-- dropoff on case3, every one of them a shape we can test
						-- exactly. GetPartBoundsInBox is a broadphase query and SAT is
						-- arithmetic, so running it always costs little.
						--
						-- `maybe` still gates the MESH fallback, which is the expensive
						-- and unreliable half.
						do
							-- a box from the floor's edge outward, along this
							-- direction only, so a wall on the OTHER axis of a
							-- corner cannot register here
							nProbed += 1
							local dir = out.Unit
							local mid = cell.pos + dir * (c.step * 0.5 + PROBE_DEPTH * 0.5)
								+ Vector3.yAxis * PROBE_RISE
							local probeCF = CFrame.lookAt(mid, mid + dir)
							local probeSize = box.Size
							box.CFrame = probeCF
							-- GETPARTSINPART WAS VETOING REAL WALLS. It was the final
							-- arbiter here and it misses solids outright -- measured on
							-- case3 at 31% of probes refused, and every refusal turns a
							-- wall into a dropoff. Downstream that is a boundary edge
							-- labelled as open ground with masonry 0.5 studs behind it.
							--
							-- Same remedy as the kill test in buildGrid: GetPartBoundsInBox
							-- sees EVERY solid, and the shapes we can test exactly are then
							-- tested exactly rather than trusted on their bounding box.
							-- A mesh is neither, so it still falls through to the physics
							-- probe -- unreliable, but better than a bounding box, which
							-- calls the empty middle of an archway solid.
							local meshSeen = false
							for _, cand in ipairs(workspace:GetPartBoundsInBox(probeCF, probeSize, opAll)) do
								if isBlock(cand) then
									if boxOverlap(probeCF, probeSize, cand.CFrame, cand.Size) then
										solid = true
										break
									end
								elseif isWedge(cand) then
									if wedgeOverlap(probeCF, probeSize, cand) then
										solid = true
										break
									end
								else
									meshSeen = true
								end
							end
							if not solid and meshSeen and maybe then
								solid = #workspace:GetPartsInPart(box, op) > 0
							end
							if not solid then nRefused += 1 end
						end
						if solid ~= above then
							if solid then nSvo += 1 else nVeto += 1 end
						end
						above = solid
					end
					local m = bit32.lshift(1, bit - 1)
					-- Recorded whatever the wall verdict comes out as. A riser is
					-- BOTH a wall and a step: you walk into it and you walk up it,
					-- so the boundary is real and the erosion must ignore it.
					if upMin <= STEP_UP then stepMask = bit32.bor(stepMask, m) end
					if above then wallMask = bit32.bor(wallMask, m) else dropMask = bit32.bor(dropMask, m) end
				end
			end
			cell.wallMask, cell.dropMask, cell.edgeMask = wallMask, dropMask, edgeMask
			cell.stepMask = stepMask
			cell.wall, cell.dropoff = wallMask ~= 0, dropMask ~= 0
			cell.regionEdge = edgeMask ~= 0
			if cell.wall then nWall += 1 end
			if cell.dropoff then nDrop += 1 end
			if cell.wall and cell.dropoff then nBoth += 1 end
			if cell.regionEdge then nEdge += 1 end
		end
	end

	data.stats.wallNodes, data.stats.dropNodes, data.stats.bothNodes = nWall, nDrop, nBoth
	box:Destroy()
	data.stats.regionEdgeNodes = nEdge
	data.stats.wallProbed, data.stats.wallRefused = nProbed, nRefused
	-- how many wall directions ONLY the SVO found, so the fix stays measurable
	data.stats.svoWalls = nSvo
	-- and how many inferred walls it refused, which is the other half of the fix
	data.stats.svoVetoed = nVeto
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
	-- NIL WHEN THE BAKE ROOT HAS NO TERRAIN OVER IT, which turns every terrain cast
	-- in `emit`, `clearanceAt` and collapse test 4 into a no-op instead of a query.
	-- Terrain is invisible to an overlap query, so those casts are the only way to
	-- see it -- and on a map without any they provably return nil every time. Two
	-- of the five world queries an ordinary cell costs, and half of the clearance
	-- rays a collapse attempt costs. `Floor.extract` has already answered this, so
	-- read its verdict rather than paying for the voxel scan twice.
	local rpTerrain = nil
	local noTerrain = floorData.noTerrain
	if noTerrain == nil then noTerrain = not Floor.hasTerrain(c) end
	if not noTerrain then
		rpTerrain = RaycastParams.new()
		rpTerrain.FilterType = Enum.RaycastFilterType.Include
		rpTerrain.FilterDescendantsInstances = { workspace.Terrain }
	end

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
	-- THE SAME OPT-IN YIELD THE VOXELIZATION AND THE FLOOR EXTRACT CARRY, at the
	-- one point that is safe to pause: between parts, before any face of the next
	-- one is built. Yielding inside buildGrid would suspend a half-built lattice;
	-- here every grid in `grids` is complete and the counters agree with it.
	--
	-- Off unless asked for, so ordinary bakes are untouched -- and the order above
	-- is the seed of the whole bake, so nothing here may reorder anything.
	local onProgress = c.onProgress

	for idx, entry in ipairs(ordered) do
		if onProgress and idx % LocalGrid.partBudget == 0 then
			onProgress(idx, #ordered)
		end
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
							if (dx * dx + dz * dz <= r2 or cellCovers(e.g, q, p))
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
										if (dx * dx + dz * dz <= r2 or cellCovers(e.g, q, p))
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
	-- EROSION IS NOT DESTRUCTIVE -- it clears `cell.region` and leaves the cell
	-- in the grid -- so a draw that walks `g.cells` shows the UN-eroded mask
	-- whatever the bake did, which is a picture of a floor the trace will not
	-- use. Follow the erosion by default; `showEroded` puts the removed cells
	-- back in, which is how you see what the radius cost.
	local showEroded = o.showEroded == true
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
			-- A SINGLE CELL IS DRAWN AT ITS TESTED FOOTPRINT, not at a full
			-- step. A rim cell's tile is clipped to the part it stands on, and
			-- drawing the unclipped square put nodes visibly inside walls they
			-- only abut -- indistinguishable, by eye, from the kill test
			-- failing. Runs keep the step, since a run is interior by
			-- construction and its ends are the only cells that could differ.
			if len == 1 and first.fu then
				dot.Size = Vector3.new(w * first.fu, 0.1, w * first.fv)
			end
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
			if len == 1 and first.fpos then mid = first.fpos end
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

		local function shown(cell: Cell): boolean
			if borderOnly and not isBorder(cell) then return false end
			if cell.eroded and not showEroded then return false end
			return true
		end

		if not merge then
			for _, cell in ipairs(g.cells) do
				if shown(cell) then
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
			if shown(cell) then
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
