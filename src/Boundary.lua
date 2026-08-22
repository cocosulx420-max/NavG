--!strict
-- NavGen.Boundary — clean outlines from the local grids, by DESIGN.md's method.
--
-- The input is LocalGrid's per-part, part-aligned cell masks. The method is
-- DESIGN.md steps 3-5, 7 and 8: chain the boundary edges of a mask, grow a
-- best-fit line along the boundary cells while the MAXIMUM residual stays under
-- a cell, and take corners as the places where that fit failed.
--
-- THIS MODULE NEVER ASKS A PART ANYTHING. No CFrames, no face planes, no sizes,
-- no raycasts, no overlap queries. It reads cell centres and nothing else. That
-- is DESIGN.md's central rule and it is not an aesthetic one: a Union or a
-- MeshPart has no readable planar face, and interpenetrating parts report faces
-- that are not surfaces. An earlier version of this module took its lines from
-- the side planes of whichever part killed each cell. It was more accurate on
-- SmallMap -- and it is exactly the approach DESIGN.md records as having given
-- this project hell on real maps, so it is gone.
--
-- WHY THE FIT IS CHEAP HERE, WHICH IS THE POINT OF THE LOCAL GRIDS. On a
-- world-aligned raster every rotated part staircases, so the fit is doing heavy
-- reconstruction everywhere and its tolerance is load-bearing everywhere. On a
-- part-aligned grid the host's own rim already lies along whole lattice lines,
-- so the fit reproduces it with zero residual and has nothing to undo. What is
-- left for the fit is the genuinely hard case: the footprint of some OTHER part
-- crossing this one's lattice at an angle. Same method as DESIGN.md, applied to
-- a fraction of the edges, which is why the tolerances stop fighting.
--
-- SAFETY. The only morphology is erosion. Lines are biased inward (step 5) and
-- polygons are fitted to cell centres, so the boundary sits at or inside the
-- walkable cells and never outside them. Erosion can remove walkable ground but
-- can never invent connectivity, so no amount of it welds two rooms through a
-- wall. Nothing here dilates.

local Boundary = {}

export type Config = {
	-- Fallback grids only. A world-aligned grid can hold two surfaces at once
	-- (a mesh with a ledge), so its trace needs to know what counts as a cliff.
	-- A block grid is a single plane and never consults this.
	stepTol: number?,

	-- STEP 4. The maximum perpendicular distance a boundary node may sit from
	-- its segment's line. A whole cell is too loose in practice: it lets a line
	-- run a stud clear of nodes it claims, which reads as the line ignoring
	-- them. Half a cell puts 223 -> 49 nodes over half a stud from the boundary
	-- for 21 more segments; tightening further buys nothing.
	fitTol: number?,

	-- STEP 8. Runs shorter than this are absorbed, and runs whose directions
	-- agree to within collinearDeg are merged -- both BEFORE corners are
	-- intersected, because short runs are what make an intersection unstable.
	minSegLen: number?,
	-- Two runs whose directions agree to within this are one run. Deliberately
	-- generous: Cocosulx would rather carry fewer, straighter edges than have the
	-- boundary kink by 14 degrees to chase a node.
	collinearDeg: number?,

	-- STEP 7. An acute corner throws two lines' intersection arbitrarily far
	-- out -- the classic miter spike. Past this distance from the corner it
	-- replaces, bevel across instead.
	miterLimit: number?,

	-- A run shorter than `cornerSpan` sitting between two runs that turn through
	-- more than `cornerDeg` is a chamfer across a corner, and is dropped so the
	-- two meet properly.
	cornerSpan: number?,
	cornerDeg: number?,


	-- How long a stretch of foreign-typed nodes a line may ignore before that
	-- stretch counts as a boundary of its own. This is the doorway guard: below
	-- it a speck is invisible, above it a wall cannot leap the opening.
	maxGap: number?,
}

local DEFAULT = {
	stepTol = 2.2,
	fitTol = 0.5,
	minSegLen = 1.0,
	collinearDeg = 20,
	miterLimit = 6.0,
	cornerSpan = 4.0,
	cornerDeg = 45,
	maxGap = 3.0,
	mergeNeedsFit = false,
	insetAll = true,
}

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

--------------------------------------------------------------------------
-- 2D helpers, in the grid's own face coordinates
--------------------------------------------------------------------------

type P2 = { x: number, z: number }

local function sub(a: P2, b: P2): P2 return { x = a.x - b.x, z = a.z - b.z } end
local function dot(a: P2, b: P2): number return a.x * b.x + a.z * b.z end
local function len(a: P2): number return math.sqrt(a.x * a.x + a.z * a.z) end

-- Interior lies to the LEFT of travel, so this is the outward side.
local function outwardOf(d: P2): P2
	local m = len(d)
	if m < 1e-9 then return { x = 0, z = 0 } end
	return { x = d.z / m, z = -d.x / m }
end

-- Total least squares: the principal axis of the point set. NOT ordinary least
-- squares -- a boundary run can be near-vertical in the grid's frame, where
-- fitting one coordinate as a function of the other blows up.
--
-- Takes an explicit list of point INDICES rather than a range: the run that
-- straddles the loop's arbitrary start point wraps, and a range cannot express
-- that.
local function fitLine(pts: {P2}, idx: {number}): (P2, P2)
	local n = #idx
	local sx, sz = 0, 0
	for _, i in ipairs(idx) do sx += pts[i].x; sz += pts[i].z end
	local cx, cz = sx / n, sz / n
	local sxx, szz, sxz = 0, 0, 0
	for _, i in ipairs(idx) do
		local dx, dz = pts[i].x - cx, pts[i].z - cz
		sxx += dx * dx; szz += dz * dz; sxz += dx * dz
	end
	-- larger eigenvalue of the 2x2 covariance, closed form
	local tr, det = sxx + szz, sxx * szz - sxz * sxz
	local disc = math.max(tr * tr * 0.25 - det, 0)
	local lam = tr * 0.5 + math.sqrt(disc)
	local dx, dz
	if math.abs(sxz) > 1e-12 then
		dx, dz = lam - szz, sxz
	elseif sxx >= szz then
		dx, dz = 1, 0
	else
		dx, dz = 0, 1
	end
	-- ORIENT ALONG TRAVEL. A principal axis has no sign -- PCA is just as happy
	-- to hand back the reverse direction -- but everything downstream reads the
	-- sign as meaning something. outwardOf() assumes interior-on-the-left, which
	-- is only true if dir runs the way the loop was walked; a flipped segment
	-- gets its "outward" normal pointing INTO the floor, and the inward bias
	-- then pushes the line the wrong way, straight through the wall.
	local sxs = pts[idx[n]].x - pts[idx[1]].x
	local szs = pts[idx[n]].z - pts[idx[1]].z
	if dx * sxs + dz * szs < 0 then dx, dz = -dx, -dz end
	local m = math.sqrt(dx * dx + dz * dz)
	if m < 1e-12 then dx, dz, m = 1, 0, 1 end
	return { x = cx, z = cz }, { x = dx / m, z = dz / m }
end

local function maxResidual(pts: {P2}, idx: {number}, c: P2, d: P2): number
	local nx, nz = -d.z, d.x
	local worst = 0
	for _, i in ipairs(idx) do
		local r = math.abs((pts[i].x - c.x) * nx + (pts[i].z - c.z) * nz)
		if r > worst then worst = r end
	end
	return worst
end

--------------------------------------------------------------------------
-- STEP 3 — trace the contour of a cell mask
--
-- Boundary EDGES are chained rather than boundary cells walked: the boundary of
-- a set of cells is a closed loop by construction, so there is no open end to
-- chase and no tolerance involved. The ordered cells the fit needs fall out of
-- the edge order.
--
-- `cliff(a, b)` reports that two lattice-adjacent cells are not continuous
-- ground even though both are in the mask. On a block grid it is never true;
-- on a fallback grid it is what stops a polygon spanning a mesh's ledge.
--------------------------------------------------------------------------

local DIRS = { { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 } }

-- cardinal bits (E, N, W, S) and diagonal bits (NE, NW, SW, SE) of LocalGrid's
-- 8-direction masks
local CARDINALS = 1 + 4 + 16 + 64
local DIAGONALS = 2 + 8 + 32 + 128

-- lattice corners of cell (u,v)'s edge in direction di, oriented so the loop
-- runs with the interior on its left
local function cornersOf(u: number, v: number, di: number)
	if di == 1 then return { u + 1, v }, { u + 1, v + 1 }
	elseif di == 2 then return { u + 1, v + 1 }, { u, v + 1 }
	elseif di == 3 then return { u, v + 1 }, { u, v }
	else return { u, v }, { u + 1, v } end
end

local function traceMask(cells: {any}, index: {[string]: any}, cliff: ((any, any) -> boolean)?)
	local segs: {any} = {}
	local byStart: {[string]: {any}} = {}
	for _, cell in ipairs(cells) do
		for di, d in ipairs(DIRS) do
			local nu, nv = cell.ui + d[1], cell.vi + d[2]
			local nb = index[nu .. ":" .. nv]
			if not nb or (cliff and cliff(cell, nb)) then
				local a, b = cornersOf(cell.ui, cell.vi, di)
				local s = { a = a, b = b, cell = cell, nu = nu, nv = nv, dir = d }
				segs[#segs + 1] = s
				local k = a[1] .. "," .. a[2]
				local bucket = byStart[k]
				if not bucket then bucket = {}; byStart[k] = bucket end
				bucket[#bucket + 1] = s
			end
		end
	end

	-- CHAINING AT A JUNCTION is not a free choice. A cliff between two cells
	-- that are both in the mask emits an edge from EACH side, on the same
	-- lattice edge in opposite directions, because the upper rim and the lower
	-- rim are two different boundaries that coincide in plan. The outline
	-- therefore has zero-width slits whose ends are vertices with four edges
	-- leaving them, and taking whichever candidate came first there splices one
	-- rim onto the other and drops degenerate slivers out of the walk.
	--
	-- Standard face traversal of an embedded planar graph fixes it: arriving
	-- along an edge, leave on the next edge CLOCKWISE from it. With the interior
	-- kept on the left, that walks each rim whole and turns a slit around at its
	-- tip.
	local function angle(s: any): number
		return math.atan2(s.b[2] - s.a[2], s.b[1] - s.a[1])
	end
	local TAU = math.pi * 2
	local used: {[any]: boolean} = {}
	local loops: {{any}} = {}
	for _, s0 in ipairs(segs) do
		if not used[s0] then
			local loop, cur = {}, s0
			while cur and not used[cur] do
				used[cur] = true
				loop[#loop + 1] = cur
				local cands = byStart[cur.b[1] .. "," .. cur.b[2]]
				local nxt, bestTurn = nil, math.huge
				if cands then
					local back = angle(cur) + math.pi
					for _, s in ipairs(cands) do
						if not used[s] then
							local turn = (back - angle(s)) % TAU
							-- straight back is the full turn, taken only when
							-- nothing else is left: the tip of a slit
							if turn <= 1e-9 then turn = TAU end
							if turn < bestTurn then nxt, bestTurn = s, turn end
						end
					end
				end
				cur = nxt
			end
			if #loop >= 4 then loops[#loops + 1] = loop end
		end
	end
	return loops
end

--------------------------------------------------------------------------
-- STEP 4 — greedy line fit. THIS IS WHERE THE STAIRCASE DIES.
--
-- Walk the loop maintaining a best-fit line through the cells accepted so far.
-- While the MAXIMUM perpendicular residual stays under a cell, keep extending.
-- When it exceeds, close the segment and start fresh from that cell.
--
-- The cells of a foreign part's footprint crossing this grid at an angle all
-- sit within a cell of one straight line at the true angle, so they collapse
-- into a single segment and the fit recovers the angle without being told it.
--
-- CORNERS ARE WHERE THE FIT FAILS. They are a byproduct, not a prerequisite --
-- which is the piece that killed every attempt that tried to detect them
-- against real geometry instead.
--------------------------------------------------------------------------

local function segmentLoop(pts: {P2}, c: any, cls: {string}, stats: any)
	local n = #pts
	local segs: {any} = {}
	if n < 2 then return segs end

	-- A LINE OWNS ONE NODE TYPE AND IGNORES THE REST.
	--
	-- The residual is measured against the run's OWN nodes only. A foreign node
	-- is invisible to the fit -- not stepped over, not fitted, just not its
	-- business -- so a single seam node sitting in the middle of a long wall no
	-- longer closes the wall. That is what makes the classification's grit
	-- harmless instead of fatal: 79% of class runs on this map are one or two
	-- nodes, and previously every one of them forced a break that step 8 could
	-- never undo.
	--
	-- But a line may not EXPAND INTO foreign nodes either. If it could, a wall
	-- run would happily leap the seam nodes across a doorway and seal it, which
	-- is the one thing that invents connectivity. So a foreign stretch longer
	-- than `maxGap` ends the run: a speck is ignored, a real stretch is a
	-- boundary of its own.
	-- A RUN OF ONE NODE HAS NO DIRECTION OF ITS OWN.
	--
	-- fitLine returns an arbitrary axis when the covariance is degenerate, which
	-- is exactly what one node gives it. The neighbouring line is then crossed
	-- against a line pointing nowhere in particular and the corner lands wherever
	-- that happens to meet.
	--
	-- It is not a rare shape. Every ClipRamp on this map has a single-node seam
	-- run at EACH end of its bottom edge -- the bottom edge itself is one clean
	-- 25-node run fitting to 0.00 -- so both corners where the ramp foot meets
	-- the ground were being placed by an arbitrary line.
	--
	-- Deleting the run was tried and cost far more than it gained. The node is
	-- real; only the direction is missing, and the ring already knows it: take
	-- the travel from the node before to the node after.
	local function fitAndPush(idx: {number}, T: string)
		if #idx < 1 then return end
		if #idx == 1 then
			local i = idx[1]
			local d = sub(pts[i % n + 1], pts[(i - 2) % n + 1])
			local m = len(d)
			stats.singletonRuns += 1
			segs[#segs + 1] = {
				idx = idx, cen = pts[i], class = T,
				dir = (m > 1e-9) and { x = d.x / m, z = d.z / m } or { x = 1, z = 0 },
			}
			return
		end
		local cen, dir = fitLine(pts, idx)
		segs[#segs + 1] = { idx = idx, cen = cen, dir = dir, class = T }
	end

	local T = cls[1]
	local cur = { 1 }
	local gapAt: number? = nil

	for i = 2, n do
		if cls[i] ~= T then
			-- foreign node: ignore it, unless the stretch has become real
			gapAt = gapAt or i
			if len(sub(pts[i], pts[gapAt])) > c.maxGap then
				fitAndPush(cur, T)
				T = cls[gapAt]
				cur = {}
				for k = gapAt, i do
					if cls[k] == T then cur[#cur + 1] = k end
				end
				if #cur == 0 then cur = { i }; T = cls[i] end
				gapAt = nil
			end
			continue
		end
		gapAt = nil

		-- A reversal still ends a run, and the residual cannot see it: round the
		-- end of a strip narrower than fitTol and every returning node is within
		-- tolerance of the line fitted to the side just left.
		local reversed = false
		if #cur >= 2 then
			local travel = sub(pts[i], pts[cur[#cur]])
			local run = sub(pts[cur[#cur]], pts[cur[1]])
			if dot(travel, run) < 0 then reversed = true end
		end

		local trial = table.clone(cur)
		trial[#trial + 1] = i
		local cen, dir = fitLine(pts, trial)
		-- MAXIMUM residual, never the average: an average lets a shallow corner
		-- hide inside a long run, which is precisely the corner worth keeping.
		local fails = reversed or maxResidual(pts, trial, cen, dir) > c.fitTol

		if fails then
			-- A BREAK MUST EARN ITSELF -- BUT THE FIT COMES FIRST.
			--
			-- The residual is the promise this stage makes: a line follows the
			-- nodes it claims, to within fitTol. So a failure always breaks. The
			-- earlier version treated "don't make a pointless corner" as able to
			-- override that, and carried on accepting nodes however far off the
			-- line they fell. Worse, it measured the turn between the CURRENT fit
			-- and the trial fit -- adding one node to a long run barely rotates
			-- it, so the turn was always about zero, always under the threshold,
			-- and the break was refused every time. Runs swallowed whole rings
			-- and lines were drawn straight across surfaces with no nodes near
			-- them: worst case, a line 51 studs from a node it claimed.
			--
			-- Not making a pointless corner is step 8's job, where two finished
			-- runs can be compared properly and merged only if the merged line
			-- still fits. It cannot be done by refusing to break, because by then
			-- there is no second run to compare against.
			fitAndPush(cur, T)
			cur = { cur[#cur], i }
		else
			cur = trial
		end
	end
	fitAndPush(cur, T)
	return segs
end

local function spanLength(pts: {P2}, idx: {number}): number
	return len(sub(pts[idx[#idx]], pts[idx[1]]))
end

-- THE FALLBACK FOR A RING THE FIT CANNOT RESOLVE.
--
-- fitTol is one cell, deliberately: DESIGN.md's rule is that anything the mask
-- can express as straight IS straight. The consequence is that a strip two
-- cells wide can never be segmented, because its cell centres form a rectangle
-- one cell deep and a one-cell-deep rectangle is within tolerance of a line.
-- That is not a bug in the fit, it is the tolerance meaning what it says -- and
-- this map's stair steps are 35x2, so it is 22 rings.
--
-- The raw lattice outline is the safe answer but a terrible one: 70 vertices
-- for a rectangle. When the ring's cells are exactly the border of their own
-- bounding box, though, the shape IS that box, and the box is four vertices.
-- Note what this does NOT do: it reads cell indices only. No part is consulted,
-- so it holds for a Union or a MeshPart exactly as it does for a Block.
local function boxIfExact(pts: {P2}, step: number): {P2}?
	local lu, hu, lv, hv = math.huge, -math.huge, math.huge, -math.huge
	for _, p in ipairs(pts) do
		lu = math.min(lu, p.x); hu = math.max(hu, p.x)
		lv = math.min(lv, p.z); hv = math.max(hv, p.z)
	end
	local w = math.floor((hu - lu) / step + 0.5) + 1
	local h = math.floor((hv - lv) / step + 0.5) + 1
	if w < 1 or h < 1 then return nil end
	-- how many cells the border of a w x h box holds
	local border = (w == 1 or h == 1) and (w * h) or (2 * w + 2 * h - 4)
	if #pts ~= border then return nil end
	local seen: {[string]: boolean} = {}
	for _, p in ipairs(pts) do
		local iu = math.floor((p.x - lu) / step + 0.5)
		local iv = math.floor((p.z - lv) / step + 0.5)
		-- every point must actually lie ON the border, or the counts agreeing was
		-- a coincidence and the shape is something else
		if iu ~= 0 and iu ~= w - 1 and iv ~= 0 and iv ~= h - 1 then return nil end
		local k = iu .. ":" .. iv
		if seen[k] then return nil end
		seen[k] = true
	end
	return {
		{ x = lu, z = lv }, { x = hu, z = lv }, { x = hu, z = hv }, { x = lu, z = hv },
	}
end

--------------------------------------------------------------------------
-- STEP 8 — clean up, and all of it BEFORE any corner is intersected
--------------------------------------------------------------------------

local function mergeSegments(pts: {P2}, segs: {any}, c: any, stats: any)
	local cosLim = math.cos(math.rad(c.collinearDeg))

	local function refit(a: any, b: any, force: boolean?): any?
		-- never across a class change: the merged run would need one push for
		-- what is wall and another for what is ledge
		if a.class ~= b.class then return nil end
		-- never across a reversal, for the same reason step 4 breaks on one: the
		-- two sides of a thin strip are within fitTol of each other, so the
		-- residual test alone would happily weld them into one segment
		if dot(a.dir, b.dir) < 0 then return nil end
		local idx = table.clone(a.idx)
		local seen: {[number]: boolean} = {}
		for _, i in ipairs(idx) do seen[i] = true end
		for _, i in ipairs(b.idx) do
			if not seen[i] then idx[#idx + 1] = i; seen[i] = true end
		end
		local cen, dir = fitLine(pts, idx)
		-- A merge demanded because a run is too short to be an edge, or because
		-- the turn between two runs is not a real corner, happens whether or not
		-- the combined residual is inside fitTol. The alternative is keeping a
		-- stub that no downstream stage can use: its line is near parallel to its
		-- neighbour's, so the corner between them is unstable by construction.
		if (not force or c.mergeNeedsFit) and maxResidual(pts, idx, cen, dir) > c.fitTol then return nil end
		return { idx = idx, cen = cen, dir = dir, class = a.class }
	end

	-- SEAM. The walk's start point on a closed loop is arbitrary, so a straight
	-- run that happens to straddle it is always split in two -- a spurious
	-- corner at exactly the place where nothing happened.
	if #segs >= 2 then
		local first, last = segs[1], segs[#segs]
		if dot(first.dir, last.dir) >= cosLim then
			local m = refit(last, first)
			if m then
				segs[1] = m
				segs[#segs] = nil
			end
		end
	end

	-- SLIDE THE BREAK POINT BEFORE JUDGING THE CORNER.
	--
	-- The greedy fit is order-dependent. It walks forward and locks a run's end
	-- at the first node whose residual fails, then leaves the next run to pick up
	-- whatever is left -- including a node that really belonged to the run
	-- behind. That inherited node tilts the next line, and the tilt forces
	-- another break, so one misplaced boundary produces two segments with a
	-- meaningless angle between them.
	--
	-- Measured: on SmallMap's ground slab, a 50.9-stud run stops one node short
	-- and the following stretch comes out as two edges 11.8 degrees apart,
	-- covering nodes that lie along a single straight line. Map-wide, 101 of 532
	-- corners turn by less than 20 degrees.
	--
	-- So before deciding whether a corner is real, try moving the boundary
	-- between each pair of runs a couple of nodes either way and keep whichever
	-- placement fits both runs best. The merge below then sees the arrangement
	-- the fit would have found had it not been greedy.
	local function contiguous(idx: {number}): boolean
		for i = 2, #idx do
			if idx[i] ~= idx[i - 1] + 1 then return false end
		end
		return true
	end
	local slid = true
	while slid do
		slid = false
		for k = 1, #segs do
			local a = segs[k]
			local b = segs[k % #segs + 1]
			if a ~= b and a.class == b.class and contiguous(a.idx) and contiguous(b.idx)
				-- adjacent runs SHARE their boundary node: the fit restarts with
				-- `cur = { cur[#cur], i }`, so a's last index is b's first
				and a.idx[#a.idx] == b.idx[1] and #a.idx >= 2 and #b.idx >= 2
			then
				local lo, hi = a.idx[1], b.idx[#b.idx]
				local function scoreAt(cut: number): number?
					-- the cut node belongs to BOTH runs, as it does in the fit
					if cut - lo + 1 < 2 or hi - cut + 1 < 2 then return nil end
					local ia, ib = {}, {}
					for i = lo, cut do ia[#ia + 1] = i end
					for i = cut, hi do ib[#ib + 1] = i end
					local ca, da = fitLine(pts, ia)
					local cb, db = fitLine(pts, ib)
					return math.max(maxResidual(pts, ia, ca, da), maxResidual(pts, ib, cb, db))
				end
				local cut0 = a.idx[#a.idx]
				local best, bestCut = scoreAt(cut0), cut0
				if best then
					for d = -2, 2 do
						if d ~= 0 then
							local sc = scoreAt(cut0 + d)
							-- a clear improvement only; ties keep the fit's own choice
							if sc and sc < best - 1e-6 then best, bestCut = sc, cut0 + d end
						end
					end
					if bestCut ~= cut0 then
						local ia, ib = {}, {}
						for i = lo, bestCut do ia[#ia + 1] = i end
						for i = bestCut, hi do ib[#ib + 1] = i end
						local ca, da = fitLine(pts, ia)
						local cb, db = fitLine(pts, ib)
						segs[k] = { idx = ia, cen = ca, dir = da, class = a.class }
						segs[k % #segs + 1] = { idx = ib, cen = cb, dir = db, class = b.class }
						stats.breaksSlid += 1
						slid = true
						break
					end
				end
			end
		end
	end

	-- A CHAMFER ACROSS A CORNER IS NOT AN EDGE.
	--
	-- Where two walls meet at an angle, the nodes in the crook belong to neither
	-- line, so the fit gives them a run of their own -- a short segment cutting
	-- the corner off. Cocosulx marked eight of them: a 90-degree turn arriving as
	-- 33 degrees, a 3.6-stud chamfer, then 65 degrees. The corner is one corner
	-- and should be one vertex.
	--
	-- So a short run whose neighbours turn through a real corner between them is
	-- dropped, and those neighbours are left to meet. Both lines are already
	-- inset off their nodes, so their crossing falls on the inside of the crook:
	-- this removes ground rather than inventing it.
	local cosCorner = math.cos(math.rad(c.cornerDeg))
	local cut = true
	while cut and #segs > 3 do
		cut = false
		for k = 1, #segs do
			local prev = segs[(k - 2) % #segs + 1]
			local cur = segs[k]
			local nxt = segs[k % #segs + 1]
			if prev ~= cur and nxt ~= cur and prev ~= nxt
				and spanLength(pts, cur.idx) < c.cornerSpan
				and dot(prev.dir, nxt.dir) < cosCorner
			then
				table.remove(segs, k)
				stats.chamfersCut += 1
				cut = true
				break
			end
		end
	end

	-- Near-collinear and too-short runs, absorbed until nothing changes.
	--
	-- This is also the rule that fixes a staircase built out of SEPARATE PARTS.
	-- A flight here is nine stacked blocks whose ends are flush, so the foot of
	-- the flight is one straight edge crossed by eight part seams. Each seam
	-- puts a one-cell jog in the mask, the fit closes a run at each, and without
	-- this merge the result is eight spurious corners along a straight line.
	local changed = true
	while changed and #segs > 3 do
		changed = false
		for k = 1, #segs do
			local a, b = segs[k], segs[(k % #segs) + 1]
			if a == b then break end
			-- SIGNED, not abs. Directions are oriented along the walk, so an abs
			-- test calls a 180-degree reversal "collinear" -- and the two long
			-- sides of a thin wall's ring are exactly that, sitting within fitTol
			-- of each other because the wall is thinner than the tolerance. They
			-- would merge and the ring would collapse.
			local nearly = dot(a.dir, b.dir) >= cosLim
			local tiny = spanLength(pts, a.idx) < c.minSegLen
				or spanLength(pts, b.idx) < c.minSegLen
			if nearly or tiny then
				local m = refit(a, b, true)
				if m then
					segs[k] = m
					table.remove(segs, (k % #segs) + 1)
					changed = true
					break
				end
			end
		end
	end
	return segs
end

local function signed2(pts: {P2}): number
	local a2 = 0
	for i = 1, #pts do
		local p, q = pts[i], pts[i % #pts + 1]
		a2 += p.x * q.z - q.x * p.z
	end
	return a2 * 0.5
end

--------------------------------------------------------------------------
-- One grid -> its rings, in world space
--------------------------------------------------------------------------

--------------------------------------------------------------------------
-- READ THE CLASS OFF THE NODE
--
-- LocalGrid already marks every node with the directions in which it has a wall
-- and the directions in which it has air, over 8 local directions, using a
-- world-space probe that copes with a neighbour living on another part's grid.
-- This module used to re-derive that itself, with its own step tolerance -- and
-- once the step-up allowance was removed from LocalGrid the two disagreed about
-- what a step even is. There is one answer and it belongs to the grid.
--
-- The three classes the fit cares about:
--   WALL -- masonry an agent must be stood off from
--   DROP -- a ledge; walking its lip is legitimate and eroding every one of
--           them removed 12.4% of SmallMap's cells at its narrowest places
--   SEAM -- neither: the floor continues onto another part's grid. 40% of all
--           boundary edges here. Not a boundary at all.
--------------------------------------------------------------------------

local SEAM, WALL, DROP = "seam", "wall", "drop"

-- the four cardinal directions as bit positions in LocalGrid's DIR8
local DIR_BIT: {[string]: number} = {
	["1:0"] = 1, ["0:1"] = 3, ["-1:0"] = 5, ["0:-1"] = 7,
}

local function classOf(g: any, s: any): string
	local cell = s.cell
	local bitIdx = DIR_BIT[s.dir[1] .. ":" .. s.dir[2]]
	if not bitIdx or not cell.wallMask then
		-- grid was not classified; fall back to what this stage can see alone
		return g.deadIndex[s.nu .. ":" .. s.nv] and WALL or DROP
	end
	local m = bit32.lshift(1, bitIdx - 1)
	if bit32.band(cell.wallMask, m) ~= 0 then return WALL end
	if bit32.band(cell.dropMask, m) ~= 0 then return DROP end
	-- neither: LocalGrid found floor continuing there, on some other grid
	return SEAM
end

-- The ring's NODES, before anything is fitted to them: ordered boundary cell
-- centres, their class, and their integer lattice coordinates.
--
-- Extracted so the fit is not the only thing that can read a ring. LineFit
-- consumes `lat` directly — integer cells are its whole contract, and it must
-- see the same ring the fit sees, apex nodes and all, or it is being measured
-- against different ground.
local function ringNodesOf(g: any, c: any, stats: any)
	local isBlock = not g.fallback and g.n ~= nil
	local step = g.step

	local cliff = nil
	if not isBlock then
		cliff = function(a: any, b: any): boolean
			return math.abs(a.pos.Y - b.pos.Y) > c.stepTol
		end
	end

	local out: {any} = {}
	for _, loop in ipairs(traceMask(g.cells, g.index, cliff)) do
		-- Ordered boundary cell CENTRES. A corner cell contributes two edges and
		-- the duplicate carries no information, so collapse it.
		local pts: {P2} = {}
		local cls: {string} = {}
		local owner: {any} = {}
		local lat: {{number}} = {}
		local lastCell, lastCls = nil, nil
		local function pushNode(cell: any, k: string)
			pts[#pts + 1] = { x = (cell.ui + 0.5) * step, z = (cell.vi + 0.5) * step }
			cls[#cls + 1] = k
			owner[#owner + 1] = cell
			lat[#lat + 1] = { cell.ui, cell.vi }
		end
		for _, s in ipairs(loop) do
			local k = classOf(g, s)
			-- Collapse the duplicate a corner cell contributes, but ONLY while the
			-- class holds: a cell with a wall on one side and a seam on the other
			-- has to appear twice or one of the two runs loses its start point.
			if s.cell ~= lastCell or k ~= lastCls then
				-- THE APEX OF A DIAGONAL TURN IS NOT ON THE RING. Where solid
				-- touches the floor only at its corner, the cell at the inside of
				-- the turn has all four cardinals clear -- only a diagonal blocked
				-- -- so a 4-connected trace walks straight past it. Measured, 357
				-- of 6046 boundary nodes are in that position, and they include
				-- corners marked by hand. The ring has to go through the turn, not
				-- around it, or those corners cannot be found by any rule.
				if lastCell then
					local du = s.cell.ui - lastCell.ui
					local dv = s.cell.vi - lastCell.vi
					if math.abs(du) == 1 and math.abs(dv) == 1 then
						local a = g.index[lastCell.ui .. ":" .. (lastCell.vi + dv)]
						local b = g.index[(lastCell.ui + du) .. ":" .. lastCell.vi]
						local function pureDiagonal(cand): boolean
							if not cand then return false end
							local blk = bit32.bor(cand.wallMask or 0, cand.dropMask or 0)
							return bit32.band(blk, CARDINALS) == 0
								and bit32.band(blk, DIAGONALS) ~= 0
						end
						local tip = pureDiagonal(a) and a or (pureDiagonal(b) and b or nil)
						if tip and tip ~= s.cell and tip ~= lastCell then
							pushNode(tip, k)
							stats.apexNodes += 1
						end
					end
				end
				pushNode(s.cell, k)
				lastCell, lastCls = s.cell, k
			end
			stats.edges += 1
			stats[k] += 1
		end
		if #pts >= 3 then
			out[#out + 1] = { pts = pts, cls = cls, lat = lat, owner = owner }
		end
	end
	return out
end

-- Every ring of every grid, as integer lattice cells plus the world position of
-- each node. This is the fit's input, handed over unfitted.
function Boundary.ringCells(localData: any, cfg: Config?)
	local c = merged(cfg)
	local stats = { edges = 0, seam = 0, wall = 0, drop = 0, apexNodes = 0 }
	local out: {any} = {}
	for part, g in pairs(localData.grids) do
		local step = g.step
		local isBlock = not g.fallback and g.n ~= nil
		local toWorld
		if isBlock then
			toWorld = function(p: P2): Vector3 return g.origin + g.u * p.x + g.v * p.z end
		else
			toWorld = function(p: P2): Vector3
				local iu, iv = math.floor(p.x / step), math.floor(p.z / step)
				local bestY = nil
				for du = -1, 1 do
					for dv = -1, 1 do
						local cell = g.index[(iu + du) .. ":" .. (iv + dv)]
						if cell and (not bestY or cell.pos.Y > bestY) then bestY = cell.pos.Y end
					end
				end
				return Vector3.new(p.x, bestY or 0, p.z)
			end
		end
		for _, r in ipairs(ringNodesOf(g, c, stats)) do
			local world = table.create(#r.pts)
			for i, p in ipairs(r.pts) do world[i] = toWorld(p) end
			out[#out + 1] = {
				part = part, fallback = g.fallback,
				lat = r.lat, cls = r.cls, world = world,
			}
		end
	end
	return { rings = out, stats = stats, config = c }
end

local function ringsOfGrid(g: any, c: any, stats: any)
	local isBlock = not g.fallback and g.n ~= nil
	local step = g.step

	local toWorld
	if isBlock then
		-- A point in the host's face coordinates is a point ON the host's face
		-- plane, so heights are exact by construction: nothing is sampled from a
		-- neighbouring cell and a vertex cannot pick up a height from the wrong
		-- side of a cliff.
		toWorld = function(p: P2): Vector3
			return g.origin + g.u * p.x + g.v * p.z
		end
	else
		-- Fallback grid: coordinates are world XZ and the surface is not one
		-- plane, so a point takes the height of the highest cell touching it.
		toWorld = function(p: P2): Vector3
			local iu, iv = math.floor(p.x / step), math.floor(p.z / step)
			local bestY = nil
			for du = -1, 1 do
				for dv = -1, 1 do
					local cell = g.index[(iu + du) .. ":" .. (iv + dv)]
					if cell and (not bestY or cell.pos.Y > bestY) then bestY = cell.pos.Y end
				end
			end
			return Vector3.new(p.x, bestY or 0, p.z)
		end
	end

	-- is a point in the grid's own face coordinates on a live cell?
	local function inMask(a: number, b: number): boolean
		return g.index[math.floor(a / step) .. ":" .. math.floor(b / step)] ~= nil
	end

	-- A ring is built in the grid's 2D face coordinates and only converted to
	-- world once outer-vs-hole is known, because a ring that could not be fitted
	-- is treated differently depending on which it is.
	local rings: {any} = {}
	for _, node in ipairs(ringNodesOf(g, c, stats)) do
		local pts, cls = node.pts, node.cls

		-- A RING TOO THIN TO SURVIVE THE FIT. DESIGN.md step 8 warns about this
		-- and the old implementation hit it on 165 of 169 holes: the two long
		-- sides of a strip narrower than fitTol sit within the tolerance of each
		-- other, so the fit walks straight round the end without ever failing and
		-- the whole ring collapses to one or two segments.
		--
		-- It is not a corner case here. This map's stair steps are 35x2 cell
		-- strips -- 36 of them, 3.6% of every walkable cell on the map -- and
		-- silently dropping them is the one operation in this module that can
		-- make real ground disappear. So a ring that cannot be fitted keeps its
		-- raw lattice outline instead. Jagged, but present.
		local rawSegs = segmentLoop(pts, c, cls, stats)
		local segs = mergeSegments(pts, rawSegs, c, stats)
		stats.rawSegments += #segs
		if c.debugPart and g.part == c.debugPart then
			local function snap(list)
				local out = {}
				for _, sg in ipairs(list) do
					out[#out + 1] = {
						first = sg.idx[1], last = sg.idx[#sg.idx], n = #sg.idx,
						class = sg.class, cen = sg.cen, dir = sg.dir,
						res = maxResidual(pts, sg.idx, sg.cen, sg.dir),
					}
				end
				return out
			end
			stats.debug = stats.debug or {}
			stats.debug[#stats.debug + 1] = {
				points = #pts, cls = cls, pts = pts,
				afterFit = snap(rawSegs), afterMerge = snap(segs),
			}
		end
		if #segs < 3 then
			local box = boxIfExact(pts, step)
			if box then
				stats.boxRings += 1
				rings[#rings + 1] = { pts2 = box, area = signed2(box) }
			else
				stats.rawRings += 1
				rings[#rings + 1] = { pts2 = pts, area = signed2(pts), raw = true }
			end
			continue
		end

		----------------------------------------------------------------
		-- A LINE SITS BALANCED AMONG ITS NODES.
		--
		-- The fit is total least squares, so the line it returns already
		-- minimises the summed squared distance to its own nodes: it runs down
		-- the middle of a staircased run rather than favouring either side. That
		-- is the property to keep.
		--
		-- What used to happen next destroyed it. The line was translated outward
		-- until it touched the OUTERMOST node -- nominally "bias inward", to
		-- guarantee no node lay outside the polygon -- which is precisely leaning
		-- hard toward a few nodes and ignoring the rest. On a staircase it parked
		-- the line on the outer corners of the steps instead of through them.
		--
		-- So the line is left where the fit put it, through the centroid. The
		-- boundary then runs through the middle of the boundary nodes, which
		-- gives up about half a cell of ground: erosion, and the safe direction.
		local lines: {any} = {}
		for _, sg in ipairs(segs) do
			local nrm = outwardOf(sg.dir)
			local cval = dot(sg.cen, nrm)
			-- A WALL LINE PULLS BACK OFF THE WALL.
			--
			-- Balanced is right for a ledge, where the boundary is the edge of the
			-- floor and there is nothing to clip. It is wrong against masonry: a
			-- line through the centroid leaves half its nodes on the far side, so
			-- the boundary cuts into the wall -- 66 of 521 edges ran outside the
			-- walkable cells, one of them 29% of its length. Cocosulx would rather
			-- the boundary sat back from a wall than traced it exactly.
			--
			-- So a wall line is moved in until no node of its own run lies outside
			-- it. Erosion, and the safe direction.
			if sg.class == WALL or c.insetAll then
				local inner = math.huge
				for _, i in ipairs(sg.idx) do
					local d = dot(pts[i], nrm)
					if d < inner then inner = d end
				end
				if inner < cval then
					stats.wallsInset += 1
					cval = inner
				end
			end
			-- WORST DISTANCE FROM A LINE TO A NODE IT CLAIMS. fitTol is the
			-- promise; anything far above it means a line is being drawn through
			-- nodes it does not follow.
			local res = maxResidual(pts, sg.idx, sg.cen, sg.dir)
			if res > stats.worstFit then stats.worstFit = res end
			if res > c.fitTol * 1.5 then stats.linesOffNodes += 1 end
			-- Balance is structural, not hoped for: the fit runs through the
			-- centroid, so the signed offsets of its own nodes sum to zero. Kept
			-- as a measured invariant because the previous version's outward
			-- translation broke it silently.
			if #sg.idx >= 2 then
				local acc = 0
				for _, i in ipairs(sg.idx) do acc += dot(pts[i], nrm) - cval end
				local mean = math.abs(acc / #sg.idx)
				if mean > stats.worstLean then stats.worstLean = mean end
			end
			lines[#lines + 1] = {
				n = nrm, c = cval, cen = sg.cen, dir = sg.dir,
				anchor = pts[sg.idx[#sg.idx]], class = sg.class,
			}
		end

		----------------------------------------------------------------
		-- STEP 7 — corners are the intersections of adjacent lines
		----------------------------------------------------------------
		local verts: {P2} = {}
		local vcls: {string} = {}
		local vhow: {string} = {}

		local anchorNow: P2 = { x = 0, z = 0 }
		local function footOn(L: any, p: P2): P2
			local d = L.c - dot(p, L.n)
			return { x = p.x + L.n.x * d, z = p.z + L.n.z * d }
		end
		-- THE LAST RESORT IS THE CELL CENTRE ITSELF.
		--
		-- Refusing an off-floor intersection is not enough on its own: the bevel
		-- that replaces it projects the anchor onto each line, and those feet can
		-- sit off the floor too. Fixing only the intersection moved 123 edges to
		-- 125. The anchor is a boundary CELL CENTRE, so it is on a live cell by
		-- construction -- fall all the way back to it and the polygon cannot
		-- leave the mask at a corner at all.
		local how = "cross"
		local function put(p: P2, k: string)
			if not inMask(p.x, p.z) then
				stats.cornersClamped += 1
				p = anchorNow
			end
			-- A vertex is never allowed to go missing quietly. Appending nil here
			-- is a no-op on the array while the class still lands, which desyncs
			-- the two and deletes a corner without a trace -- that is exactly how
			-- three runs' worth of the Union's crescent disappeared.
			if not p then
				stats.cornersLost += 1
				return
			end
			verts[#verts + 1] = p
			vcls[#vcls + 1] = k
			vhow[#vhow + 1] = how
		end
		local nL = #lines
		for i = 1, nL do
			local l1, l2 = lines[i], lines[(i % nL) + 1]
			-- the edge LEAVING this corner runs along l2, so it carries l2's class
			local leaving = l2.class
			local det = l1.n.x * l2.n.z - l1.n.z * l2.n.x
			local anchor = l1.anchor
			-- AFTER `anchor` exists. Assigned before it, this read the outer
			-- scope, found nothing, and left the clamp fallback nil for every
			-- corner in the bake.
			anchorNow = anchor
			if math.abs(det) < 1e-6 then
				-- near-parallel: fall back to the foot of the anchor on l1
				stats.unstableCorners += 1
				how = "parallel"
				put(footOn(l1, anchor), leaving)
			else
				local px = (l1.c * l2.n.z - l2.c * l1.n.z) / det
				local pz = (l1.n.x * l2.c - l2.n.x * l1.c) / det
				-- A CORNER MAY NOT LAND OUTSIDE THE FLOOR. The miter limit only
				-- catches an overshoot longer than miterLimit studs, so a two-stud
				-- one that sits entirely off the walkable cells sailed through:
				-- measured, 115 of 123 edges that left the mask did so at an end,
				-- and only 8 had the fitted line itself wandering. Erosion is the
				-- safe direction, so an intersection that is not on a live cell is
				-- refused and bevelled across instead. Cell lookup only.
				local outside = not inMask(px, pz)
				if outside then stats.cornersOffMask += 1 end
				if outside or len(sub({ x = px, z = pz }, anchor)) > c.miterLimit then
					-- MITER LIMIT. An acute corner throws the intersection
					-- arbitrarily far out; bevel across it instead.
					-- A BEVEL OF NO WIDTH IS A TWITCH, NOT A CORNER.
					--
					-- Both feet are the SAME anchor projected onto two lines, so
					-- when those lines are nearly parallel the two land almost on
					-- top of each other: a fraction of a stud of edge, with the
					-- boundary either side heading the same way. 30 of 628 edges
					-- on SmallMap, one of them 0.18 studs between neighbours 3.4
					-- degrees apart. There is no corner being described there.
					stats.bevels += 1
					how = outside and "bevel-offmask" or "bevel-miter"
					local f1, f2 = footOn(l1, anchor), footOn(l2, anchor)
					if len(sub(f2, f1)) < c.minSegLen then
						stats.bevelsCollapsed += 1
						put({ x = (f1.x + f2.x) * 0.5, z = (f1.z + f2.z) * 0.5 }, leaving)
					else
						put(f1, leaving)
						put(f2, leaving)
					end
				else
					how = "cross"
					put({ x = px, z = pz }, leaving)
				end
			end
		end
		if #verts < 3 then
			local box = boxIfExact(pts, step)
			if box then
				stats.boxRings += 1
				rings[#rings + 1] = { pts2 = box, area = signed2(box) }
			else
				stats.rawRings += 1
				rings[#rings + 1] = { pts2 = pts, area = signed2(pts), raw = true }
			end
			continue
		end
		if c.debugPart and g.part == c.debugPart and stats.debug then
			local last = stats.debug[#stats.debug]
			last.how, last.faceVerts = vhow, verts
		end
		rings[#rings + 1] = { pts2 = verts, cls = vcls, area = signed2(verts) }
	end

	-- Outer is the largest by magnitude. Magnitude, never the sign: this project
	-- has been bitten by a handedness assumption before.
	local outer = rings[1]
	for _, r in ipairs(rings) do
		if math.abs(r.area) > math.abs(outer.area) then outer = r end
	end

	for _, r in ipairs(rings) do
		-- An unfitted OUTER ring stands as it is: cell centres lie inside the
		-- walkable cells, so the polygon is conservative already. An unfitted
		-- HOLE is the opposite -- cell centres sit half a cell INTO the obstacle
		-- the hole represents, so the hole would come out too small and hand
		-- back ground that is not there. Push it out by half a cell. An obstacle
		-- that is slightly too big is safe; one that is too small is not.
		if r.raw and r ~= outer then
			local cx, cz = 0, 0
			for _, p in ipairs(r.pts2) do cx += p.x; cz += p.z end
			cx, cz = cx / #r.pts2, cz / #r.pts2
			for _, p in ipairs(r.pts2) do
				local dx, dz = p.x - cx, p.z - cz
				local m = math.sqrt(dx * dx + dz * dz)
				if m > 1e-6 then
					p.x += dx / m * step * 0.5
					p.z += dz / m * step * 0.5
				end
			end
		end
		local world = table.create(#r.pts2)
		for i, p in ipairs(r.pts2) do world[i] = toWorld(p) end
		r.verts = world

		r.outer = (r == outer)
		stats.segments += #world
	end
	return rings
end

--------------------------------------------------------------------------
-- Boundary.fromLocal — the entry point
--------------------------------------------------------------------------

function Boundary.fromLocal(localData: any, cfg: Config?)
	local c = merged(cfg)
	local t0 = os.clock()

	local regions: {any} = {}
	local stats = {
		parts = 0, block = 0, fallback = 0,
		regions = 0, holes = 0, verts = 0, emptyGrids = 0,
		rawSegments = 0, segments = 0, bevels = 0, unstableCorners = 0,
		-- rings the fit could not resolve, kept as their raw lattice outline
		rawRings = 0, boxRings = 0,
		-- boundary edges by class; seam edges are joins, not boundaries
		edges = 0, seam = 0, wall = 0, drop = 0,
		-- corners refused because the intersection landed off the walkable cells
		cornersOffMask = 0, cornersClamped = 0, cornersLost = 0, bevelsCollapsed = 0, breaksSlid = 0,
		wallsInset = 0, singletonRuns = 0, chamfersCut = 0,
		apexNodes = 0,
		-- worst mean signed distance from a line to its own nodes; 0 = balanced
		worstLean = 0, worstFit = 0, linesOffNodes = 0,
	}

	for part, g in pairs(localData.grids) do
		stats.parts += 1
		if g.fallback then stats.fallback += 1 else stats.block += 1 end
		local rings = ringsOfGrid(g, c, stats)
		if #rings == 0 then
			stats.emptyGrids += 1
			continue
		end
		local outer = rings[1]
		for _, r in ipairs(rings) do
			if r.outer then outer = r end
		end
		local region = {
			part = part,
			fallback = g.fallback,
			verts = outer.verts,
			cls = outer.cls,
			area = math.abs(outer.area),
			cells = #g.cells,
			dead = #g.dead,
			holes = {},
		}
		for _, r in ipairs(rings) do
			if r ~= outer then
				region.holes[#region.holes + 1] = { verts = r.verts, cls = r.cls, area = math.abs(r.area) }
			end
		end
		stats.verts += #region.verts
		for _, h in ipairs(region.holes) do stats.verts += #h.verts end
		stats.regions += 1
		stats.holes += #region.holes
		regions[#regions + 1] = region
	end

	stats.seconds = os.clock() - t0
	return { regions = regions, stats = stats, config = c }
end

--------------------------------------------------------------------------

function Boundary.visualize(res: any, parent: Instance?)
	local root = parent or workspace
	local old = root:FindFirstChild("NavGen_Boundary")
	if old then old:Destroy() end
	local folder = Instance.new("Folder")
	folder.Name = "NavGen_Boundary"
	folder.Parent = root

	local function bar(a: Vector3, b: Vector3, colour: Color3, thick: number, into: Instance)
		local d = b - a
		if d.Magnitude < 1e-4 then return end
		local p = Instance.new("Part")
		p.Anchored = true; p.CanCollide = false; p.CanQuery = false; p.CanTouch = false
		p.Material = Enum.Material.Neon
		p.Color = colour
		p.Size = Vector3.new(thick, thick, d.Magnitude)
		p.CFrame = CFrame.lookAt(a + d * 0.5, b) + Vector3.new(0, 0.08, 0)
		p.Parent = into
	end

	-- COLOURED BY CLASS, because the class is the thing worth looking at: it
	-- decides what gets offset and what becomes a join. Red is wall, blue is
	-- dropoff, green is seam -- and a green run means two polygons meet there,
	-- so a green ring around a flat floor is not a boundary at all. Grey is a
	-- ring that fell back to its raw outline, where no class was resolved.
	local COL = {
		wall = Color3.fromRGB(255, 80, 80),
		drop = Color3.fromRGB(80, 170, 255),
		seam = Color3.fromRGB(90, 255, 120),
	}
	local GREY = Color3.fromRGB(150, 150, 150)
	for ri, r in ipairs(res.regions) do
		local sub = Instance.new("Folder")
		sub.Name = string.format("R%d_%s_c%d_h%d%s", ri, r.part.Name, r.cells, #r.holes,
			r.fallback and "_FALLBACK" or "")
		sub.Parent = folder
		local function drawRing(v: {Vector3}, cls: {string}?)
			for i = 1, #v do
				local k = cls and cls[i]
				bar(v[i], v[i % #v + 1], (k and COL[k]) or GREY, 0.18, sub)
			end
		end
		drawRing(r.verts, r.cls)
		for _, hr in ipairs(r.holes) do drawRing(hr.verts, hr.cls) end
	end
	return folder
end

return Boundary
