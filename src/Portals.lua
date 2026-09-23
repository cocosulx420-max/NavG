--!strict
-- NVGN.Portals -- the links that turn 93 convex faces into one navmesh.
--
-- CDT emits polygons that know their own corners and nothing else. That is a
-- pile of faces, not a mesh: a navmesh is polygons PLUS the gate between each
-- pair of neighbours, and everything downstream -- a search, a funnel, a reach
-- test -- walks the gates rather than the faces. Recast stores a link per
-- neighbour on every polygon for exactly this reason.
--
-- TWO SOURCES, BECAUSE THE MESH IS BUILT PER REGION.
--
--   inside a region   the shared polygon edge, exact. Each region is
--                     triangulated on its own, so two adjacent polygons hold
--                     the same corner Vector3s bit for bit and the portal IS
--                     that edge. Nothing is approximated here.
--
--   between regions   Severance's cross-region cell pairs. Different regions
--                     never share a vertex -- separate meshes, separate bases --
--                     so the position key finds nothing across a region
--                     boundary and something else has to say a staircase is
--                     connected. Severance already tested every cell pair in
--                     the map against a gate built for that question (0.75 in
--                     plane, 1.5 along the normal, the latter measured to clear
--                     case3's 1.38 stud treads) and now keeps the ones that
--                     passed.
--
-- WALL AND DROP CLASSIFICATION IS NOT USED, deliberately. EdgeKind labels the
-- boundary and is wired up, but a portal needs neither of its answers: inside a
-- region the shared edge is proof of adjacency on its own, and across one
-- Severance has already decided reachability with a gate it validated. The
-- classification also has a measured defect (213 of 1369 `wall` samples have
-- nothing in the probe box) which there is no reason to inherit here.
--
-- TWO RULES CARRIED OVER FROM RegionLink, which did this shape on the old
-- pipeline and learnt both the hard way:
--
--   * SPAN IS DESCRIPTION, NEVER A VERDICT. A 1.5 stud seam across open floor
--     is perfectly walkable. Cutting links for being narrower than an agent cut
--     legitimate ones, because width only matters where there is something
--     beside you to bump into -- which is a body test, not a measurement.
--   * EVERY SEAM IS AN EDGE. Passability is an attribute of a link and never a
--     filter on it. A link nothing can use is still information; a link that
--     was silently dropped is a map with a missing door and no way to tell.

local Portals = {}

local Rings = require(script.Parent:WaitForChild("Rings"))
local Agents = require(script.Parent:WaitForChild("Agents"))

-- How far outside a polygon a cell may sit and still be claimed by it, as a
-- fraction of the grid step.
--
-- NOT ZERO, AND THE REASON IS STRUCTURAL. `polyline` insets every raw boundary
-- node half a step into the region, so the polygon outline runs through the
-- CENTRES of the outermost cells rather than past them. A strict inside test is
-- therefore a coin flip on exactly the cells that matter most here -- the ones
-- near a region boundary are the ones Severance pairs. Half a step puts the
-- acceptance boundary back where the floor actually ends.
Portals.claimSlack = 0.5

-- Reach for grouping seam pairs into separate doorways, in grid steps. One row
-- of cells has to stay connected through its own diagonal, so this clears
-- sqrt(2); much more and two openings either side of a pillar merge into one
-- portal that runs straight through the pillar.
Portals.groupReach = 1.6

-- Quantisation for the shared-edge key. The corners being matched are the same
-- floats, not nearby ones -- the two polygons were cut from one triangulation --
-- so this only has to survive the lift through `W()`, which is arithmetic on
-- identical inputs.
Portals.snap = 1e-3

-- How many studs of untraced floor a bridge link may cross. `nil` takes
-- `traceMinWidth`, which is the right default by construction: it is the width
-- below which a region is not worth standing on, so it is also the width that
-- can only be crossed rather than walked along.
Portals.crossLimit = nil

local function vkey(p: Vector3): string
	return ("%.3f,%.3f,%.3f"):format(p.X, p.Y, p.Z)
end

local function hkey(x: number, y: number, z: number): number
	return x * 73856093 + y * 19349663 + z * 83492791
end

-- ------------------------------------------------------- 1. shared edges

-- Every edge held by two polygons of the SAME region.
--
-- Keyed by region as well as by endpoints. Two regions rounding to the same
-- corner would otherwise fake an adjacency between faces on different planes,
-- and the collision count below is kept so that stays a measured zero rather
-- than an assumed one.
local function sharedLinks(mesh: any, links: { any }, stats: any)
	local seen: { [string]: any } = {}
	local regionsOfEdge: { [string]: { [number]: boolean } } = {}

	for i, f in ipairs(mesh.tris) do
		for j = 1, f.n do
			local A, B = f.verts[j], f.verts[j % f.n + 1]
			if (B - A).Magnitude > 1e-9 then
				local ka, kb = vkey(A), vkey(B)
				local ek = (ka < kb) and (ka .. "|" .. kb) or (kb .. "|" .. ka)

				local rs = regionsOfEdge[ek]
				if not rs then rs = {}; regionsOfEdge[ek] = rs end
				rs[f.region] = true

				local key = f.region .. "#" .. ek
				local prev = seen[key]
				if not prev then
					seen[key] = { poly = i, a = A, b = B, count = 1 }
					stats.edges += 1
				else
					prev.count += 1
					if prev.count == 2 then
						-- ORIENTED FROM THE FIRST POLYGON'S WINDING, so a
						-- consumer always gets left and right the same way round
						-- when crossing from `a` into `b`. Taking it from
						-- whichever polygon happened to be reached second would
						-- flip the gate on half the mesh.
						links[#links + 1] = {
							kind = "shared", a = prev.poly, b = i,
							left = prev.a, right = prev.b,
							centre = (prev.a + prev.b) * 0.5,
							span = (prev.b - prev.a).Magnitude,
							drop = 0, count = 1, residual = 0,
						}
						stats.shared += 1
					else
						-- A THIRD OWNER IS A MANIFOLD FAILURE. Reported, never
						-- averaged into a link: three faces on one edge means
						-- the triangulation produced something that is not a
						-- surface, and a portal built over it would hide that.
						stats.overshared += 1
						stats.oversharedAt[#stats.oversharedAt + 1] =
							("r%03d %s (%d owners)"):format(f.region, ek, prev.count)
					end
				end
			end
		end
	end

	for _, rs in pairs(regionsOfEdge) do
		local n = 0
		for _ in pairs(rs) do n += 1 end
		if n > 1 then stats.crossRegionKeys += 1 end
	end
end

-- ----------------------------------------------------- 2. cell to polygon

-- Crossing-number test, in the region's own plane.
local function contains(P: { any }, qx: number, qy: number): boolean
	local n = #P
	local inside = false
	local ax, ay = P[n][1], P[n][2]
	for i = 1, n do
		local bx, by = P[i][1], P[i][2]
		if (ay > qy) ~= (by > qy) then
			local t = (qy - ay) / (by - ay)
			if qx < ax + t * (bx - ax) then inside = not inside end
		end
		ax, ay = bx, by
	end
	return inside
end

local function distToRing(P: { any }, qx: number, qy: number): number
	local n = #P
	local best = math.huge
	local ax, ay = P[n][1], P[n][2]
	for i = 1, n do
		local bx, by = P[i][1], P[i][2]
		local dx, dy = bx - ax, by - ay
		local L2 = dx * dx + dy * dy
		local t = 0
		if L2 > 1e-12 then
			t = math.clamp(((qx - ax) * dx + (qy - ay) * dy) / L2, 0, 1)
		end
		local ex, ey = qx - (ax + dx * t), qy - (ay + dy * t)
		local d = math.sqrt(ex * ex + ey * ey)
		if d < best then best = d end
		ax, ay = bx, by
	end
	return best
end

-- Which polygon each live cell stands on, by region.
--
-- Region by region, because a polygon and a cell only share a frame if they
-- share a region -- and `Rings.basis` is reused rather than reimplemented so
-- this measures in the frame the rings were classified in. Two copies of that
-- basis could disagree on handedness and silently mirror a whole region.
local function claimCells(mesh: any, data: any, stats: any): { [any]: number }
	local step = data.config.step
	local slack = step * Portals.claimSlack

	-- polygons, grouped by region and pre-projected
	local byRegion: { [number]: { any } } = {}
	for i, f in ipairs(mesh.tris) do
		local g = byRegion[f.region]
		if not g then
			local e1, e2 = Rings.basis(f.up)
			g = { e1 = e1, e2 = e2, polys = {} }
			byRegion[f.region] = g
		end
		local P = table.create(f.n)
		local lox, loy, hix, hiy = math.huge, math.huge, -math.huge, -math.huge
		for k = 1, f.n do
			local v = f.verts[k]
			local x, y = v:Dot(g.e1), v:Dot(g.e2)
			P[k] = { x, y }
			if x < lox then lox = x end
			if y < loy then loy = y end
			if x > hix then hix = x end
			if y > hiy then hiy = y end
		end
		g.polys[#g.polys + 1] = { idx = i, P = P,
			lox = lox - slack, loy = loy - slack, hix = hix + slack, hiy = hiy + slack }
	end

	local of: { [any]: number } = {}
	for _, grid in ipairs(data.grids) do
		for _, cell in ipairs(grid.cells) do
			local r = cell.region
			local g = r and byRegion[r]
			if not g then
				if r then stats.cellsNoPolygon += 1 end
				continue
			end
			stats.cellsTried += 1
			local qx, qy = cell.pos:Dot(g.e1), cell.pos:Dot(g.e2)
			local hit, bestD = nil, math.huge
			for _, P in ipairs(g.polys) do
				if qx >= P.lox and qx <= P.hix and qy >= P.loy and qy <= P.hiy then
					if contains(P.P, qx, qy) then
						-- INSIDE WINS OUTRIGHT. A cell genuinely within a
						-- polygon is never handed to a neighbour that happens
						-- to have a nearer edge.
						hit, bestD = P.idx, -1
						break
					end
					local d = distToRing(P.P, qx, qy)
					if d < bestD then hit, bestD = P.idx, d end
				end
			end
			if hit and bestD <= slack then
				of[cell] = hit
				if bestD >= 0 then stats.cellsNearby += 1 else stats.cellsInside += 1 end
			else
				stats.cellsNoPolygon += 1
			end
		end
	end
	return of
end

-- --------------------------------------------- 3 & 4. seams to portals

-- Contiguous runs within one polygon pair.
--
-- Two rooms joined on both sides of a pillar are ONE polygon pair and TWO
-- doorways. Flooding the midpoints apart keeps them apart; averaging the whole
-- bucket into a single portal would run it straight through the pillar.
local function groupsOf(mids: { Vector3 }, reach: number): { { number } }
	local G = reach
	local bucket: { [number]: { number } } = {}
	for i, m in ipairs(mids) do
		local k = hkey(math.floor(m.X / G), math.floor(m.Y / G), math.floor(m.Z / G))
		local b = bucket[k]
		if not b then b = {}; bucket[k] = b end
		b[#b + 1] = i
	end

	local taken = {}
	local out = {}
	for seed = 1, #mids do
		if not taken[seed] then
			taken[seed] = true
			local stack, members = { seed }, {}
			while #stack > 0 do
				local i = table.remove(stack) :: number
				members[#members + 1] = i
				local p = mids[i]
				local bx, by, bz = math.floor(p.X / G), math.floor(p.Y / G), math.floor(p.Z / G)
				for dx = -1, 1 do for dy = -1, 1 do for dz = -1, 1 do
					local b = bucket[hkey(bx + dx, by + dy, bz + dz)]
					if b then
						for _, j in ipairs(b) do
							if not taken[j] and (mids[j] - p).Magnitude <= reach then
								taken[j] = true
								stack[#stack + 1] = j
							end
						end
					end
				end end end
			end
			out[#out + 1] = members
		end
	end
	return out
end

-- The gate a group of cells describes.
--
-- ITS DIRECTION IS NOT SIMPLY THE SPREAD OF THE POINTS. A portal is the opening
-- you pass THROUGH, so it must lie across the direction of travel -- and travel
-- is known exactly, because every member of the group is a cell pair that says
-- which way you would step.
--
-- Taking the direction from the two most distant members alone was the first
-- version and it was wrong in the worst possible way: where a crossing's cells
-- run lengthwise ALONG the direction of travel -- an untraced strip you step
-- over, a stair -- the widest spread IS the travel direction, and nine gates on
-- case3 came out within ten degrees of pointing the way you walk rather than
-- across it. A gate turned ninety degrees is not a narrow gate, it is a wrong
-- one. Cocosulx spotted it in the drawing before it was measured.
--
-- `order` comes back sorted along the axis with each member's distance off the
-- line, which is what lets a group that turns a corner be cut at the corner.
local function fitSegment(mids: { Vector3 }, dirs: { Vector3 }, up: Vector3,
	members: { number }, step: number)

	local n = #members
	local c, travel = Vector3.zero, Vector3.zero
	for _, i in ipairs(members) do
		c += mids[i]
		travel += dirs[i]
	end
	c /= n

	-- The cells' own widest spread, which is where the opening actually lies...
	local far, fd = mids[members[1]], -1
	for _, i in ipairs(members) do
		local d = (mids[i] - c).Magnitude
		if d > fd then far, fd = mids[i], d end
	end
	local other, od = far, -1
	for _, i in ipairs(members) do
		local d = (mids[i] - far).Magnitude
		if d > od then other, od = mids[i], d end
	end

	-- ...used AS IS when it already lies across travel, and replaced when it does
	-- not. Travel decides WHICH of the two the spread is, and nothing more.
	--
	-- Forcing perpendicularity by projecting travel out of the spread was tried
	-- and is wrong, because it ROTATES the axis off the cells' own line whenever
	-- travel is the least bit skew. The gable ridge is 49 cells in a dead
	-- straight line along Z and its travel is (0.985, 0, -0.171); removing that
	-- -0.171 tilted the bar ten degrees, which over 24 studs is 3.9 studs away
	-- from the cells it is meant to span, and the splitter below then shredded
	-- one good gate into dozens chasing the error.
	--
	-- The cells' own line is the truth about WHERE the opening is. Travel is only
	-- needed to catch the case where the spread is not the opening at all: where
	-- a crossing's cells run lengthwise ALONG travel -- an untraced strip stepped
	-- over, a stair -- the widest spread IS the travel direction, and using it
	-- points the gate the way you walk. That was nine gates on case3, and there
	-- the cells have nothing to say about width, so the plane must answer instead.
	local ACROSS = 0.707      -- 45 degrees; beyond this the spread is travel, not width
	local axis, fromCells = nil, true
	if od > 1e-6 then
		local sp = (other - far).Unit
		if travel.Magnitude < 1e-6 or math.abs(sp:Dot(travel.Unit)) < ACROSS then
			axis = sp
		end
	end
	if not axis and travel.Magnitude > 1e-6 then
		fromCells = false
		-- The spread is along travel, or there is no spread: one cell wide. Turn
		-- travel a quarter turn in the floor plane; the cells cannot help here.
		local flat = travel - up * travel:Dot(up)
		if flat.Magnitude > 1e-6 then
			local a2 = up:Cross(flat.Unit)
			if a2.Magnitude > 1e-6 then axis = a2.Unit end
		end
	end
	if not axis then return c, c, 0, { members[1] }, { 0 }, true, travel end

	local tmin, tmax, resid = math.huge, -math.huge, 0
	local ts, off = {}, {}
	for _, i in ipairs(members) do
		local v = mids[i] - c
		local t = v:Dot(axis)
		ts[i] = t
		off[i] = (v - axis * t).Magnitude
		if t < tmin then tmin = t end
		if t > tmax then tmax = t end
		if off[i] > resid then resid = off[i] end
	end
	local order = table.clone(members)
	table.sort(order, function(a, b) return ts[a] < ts[b] end)

	-- half a step past the outermost member at each end, so the gate spans the
	-- cells rather than the line through their centres
	local h = step * 0.5
	return c + axis * (tmin - h), c + axis * (tmax + h), resid, order, off,
		fromCells, travel
end

-- How deep a bent group may be cut before it is emitted as it stands.
Portals.maxSplit = 4

-- Emit one gate per group, CUTTING A GROUP THAT DOES NOT LIE ON ONE LINE rather
-- than forcing a single segment through it.
--
-- A contiguity flood keeps two doorways either side of a pillar apart, but it
-- happily returns one run that turns a corner -- an untraced strip wrapping the
-- end of a wall, say -- and a straight gate through that does not lie on
-- walkable floor at all. Cutting at the member furthest from the fitted line
-- leaves every emitted gate within one step of its own cells.
--
-- The corner member is kept by BOTH halves, so the two gates meet there instead
-- of leaving a notch between them.
local function fitAll(mids: { Vector3 }, dirs: { Vector3 }, up: Vector3,
	members: { number }, step: number, out: { any }, depth: number?)

	local d = depth or 0
	local L, R, resid, order, off, fromCells, travel =
		fitSegment(mids, dirs, up, members, step)

	-- SPLITTING ONLY MAKES SENSE WHERE THE CELLS DREW THE LINE. When the axis
	-- came from travel instead -- the cells run along the way you walk, so they
	-- cannot say how wide the opening is -- they necessarily sit OFF the gate,
	-- and `resid` measures the depth of the crossing rather than an error. Left
	-- unguarded that fired on two-cell groups, which is nonsense on its face:
	-- two points always lie on their own line.
	if resid <= step or not fromCells or #members < 4 or d >= Portals.maxSplit then
		out[#out + 1] = { left = L, right = R, resid = resid,
			members = members, fromCells = fromCells, travel = travel }
		return
	end

	-- worst member, excluding the two ends: cutting at an end drops one point
	-- and refits the same line, which never terminates
	local cut, worst = 2, -1
	for k = 2, #order - 1 do
		if off[order[k]] > worst then cut, worst = k, off[order[k]] end
	end

	local a, b = {}, {}
	for k = 1, cut do a[#a + 1] = order[k] end
	for k = cut, #order do b[#b + 1] = order[k] end
	fitAll(mids, dirs, up, a, step, out, d + 1)
	fitAll(mids, dirs, up, b, step, out, d + 1)
end

-- One place that records how well a fitted group actually was a line, so seam
-- and bridge groups are held to the same standard instead of only the kind that
-- happened to be written first.
local function noteFit(stats: any, kind: string, lo: number, hi: number,
	resid: number, n: number, step: number, fromCells: boolean?)
	if fromCells == false then
		-- the cells never claimed to be on this line; see fitAll
		stats.gatesFromTravel += 1
		return
	end
	if resid > stats.worstResidual then
		stats.worstResidual = resid
		stats.worstResidualAt = ("%s f%04d-f%04d, %d cells"):format(kind, lo, hi, n)
	end
	if resid > step then
		stats.bentGroups += 1
		stats.bentAt[#stats.bentAt + 1] =
			("%s f%04d-f%04d residual %.2f over %d cells"):format(kind, lo, hi, resid, n)
	end
end

-- PORTALS ON THE POLYGONS' OWN EDGES. Severance says WHETHER two polygons
-- connect; this says WHERE, and the answer is geometry the mesh already has.
--
-- The fitted gates below were lines drawn through cell-pair midpoints, so they
-- floated up to two studs off both polygons, pointed along travel wherever the
-- cells ran that way (399 on case6) and came apart into dashed rows. A blue
-- shared portal has none of that because it IS an edge. This makes seams and
-- bridges edges too: take A's edges and B's edges that FACE each other --
-- antiparallel, B on A's outer side, within `gap` -- project one onto the other,
-- and walk the overlap. Where Severance's evidence for this pair (its cell-pair
-- midpoints, or the untraced cells of a bridge) lies within reach, the run is
-- open; where it stops -- a pillar, a rail -- the portal is cut. The gate is the
-- open run ON A's EDGE, and the matching stretch of B's edge rides along as
-- bLeft/bRight, so a funnel has both sides of the step.
Portals.edgeMatch = true
Portals.edgeAngle = 20    -- degrees off antiparallel two facing edges may be
Portals.seamGap = 2.5     -- studs in plane between facing edges of a seam
Portals.bridgeGap = 4.0   -- and of a bridge, which spans an untraced strip
Portals.edgeSample = 0.25 -- studs between verdict samples along the overlap
Portals.edgeBridge = 1.0  -- studs of missing evidence carried through when nothing stands in it

local bridgeRay = RaycastParams.new()
bridgeRay.FilterType = Enum.RaycastFilterType.Exclude

local function edgePortals(mesh: any, e: any, kind: string, gapMax: number,
	step: number, links: { any }, stats: any): number
	local A, B = mesh.tris[e.lo], mesh.tris[e.hi]
	if not A or not B then return 0 end
	do
		local ex = {}
		for _, n in ipairs({ "NVGN_Debug", "NVGN_Path", "PathStart", "PathEnd", "NVGN_Follower" }) do
			local x = workspace:FindFirstChild(n)
			if x then ex[#ex + 1] = x end
		end
		bridgeRay.FilterDescendantsInstances = ex
	end
	local up = A.up or Vector3.yAxis
	local function flat(v: Vector3): Vector3 return v - up * v:Dot(up) end
	local cosTol = math.cos(math.rad(Portals.edgeAngle))

	-- the evidence, hashed in plane
	local H = 1.0
	local grid: { [string]: { number } } = {}
	for i, m in ipairs(e.mids) do
		local f = flat(m)
		local k = math.floor(f.X / H) .. ":" .. math.floor(f.Z / H)
		local b = grid[k]
		if not b then b = {}; grid[k] = b end
		b[#b + 1] = i
	end
	local used = {}
	local function evidence(p: Vector3, reach: number): { number }
		local f = flat(p)
		local bx, bz = math.floor(f.X / H), math.floor(f.Z / H)
		local out = {}
		local rc = math.ceil(reach / H)
		for dx = -rc, rc do
			for dz = -rc, rc do
				for _, i in ipairs(grid[(bx + dx) .. ":" .. (bz + dz)] or {}) do
					if (flat(e.mids[i]) - f).Magnitude <= reach then out[#out + 1] = i end
				end
			end
		end
		return out
	end

	local made = 0
	local nA, nB = #A.verts, #B.verts
	for i = 1, nA do
		local a1, a2 = A.verts[i], A.verts[i % nA + 1]
		local da = flat(a2 - a1)
		local la = da.Magnitude
		if la < 1e-3 then continue end
		local ua = da / la
		local outward = ua:Cross(up) -- floor is left of travel; this points away
		for j = 1, nB do
			local b1, b2 = B.verts[j], B.verts[j % nB + 1]
			local db = flat(b2 - b1)
			if db.Magnitude < 1e-3 or ua:Dot(db.Unit) > -cosTol then continue end
			local o1, o2 = flat(b1 - a1):Dot(outward), flat(b2 - a1):Dot(outward)
			local gap = (o1 + o2) * 0.5
			if gap < -0.1 or gap > gapMax then continue end
			local t1, t2 = flat(b1 - a1):Dot(ua), flat(b2 - a1):Dot(ua)
			local lo, hi = math.max(0, math.min(t1, t2)), math.min(la, math.max(t1, t2))
			if hi - lo < Portals.edgeSample then continue end

			local reach = math.max(step * 1.5, gap * 0.5 + step)
			local ds = Portals.edgeSample
			local runStart, runDrops = nil, {}
			-- A SHORT HOLE IN THE EVIDENCE IS NOT A CUT. Two lattices meeting at a
			-- slight angle leave the cell pairs along one opening patchy, and cutting
			-- at every miss drew one edge as a dashed row of gates (Cocosulx). A gap
			-- up to edgeBridge is carried through unless something solid stands in
			-- it -- a ray along the edge at knee and at chest height, the rail and
			-- the pillar test.
			local function blocked(t1: number, t2: number): boolean
				local p1 = a1 + (a2 - a1) * (t1 / la) + outward * (gap * 0.5)
				local p2 = a1 + (a2 - a1) * (t2 / la) + outward * (gap * 0.5)
				for _, hgt in ipairs({ 0.4, 2.0 }) do
					local o1 = p1 + up * hgt
					if workspace:Raycast(o1, (p2 + up * hgt) - o1, bridgeRay) then return true end
				end
				return false
			end
			local function onB(p: Vector3): Vector3
				local d = b2 - b1
				local t = math.clamp((p - b1):Dot(d) / d:Dot(d), 0, 1)
				return b1 + d * t
			end
			local function emit(ts: number, te: number)
				-- REACH THE CORNERS. Cells near a polygon's corner rarely carry
				-- evidence, so every gate stopped short of its polygon's ends and an
				-- edge shared out between several polygons read as a dashed row.
				-- Stretch to the overlap's end when it is within edgeBridge and clear.
				if ts - lo <= Portals.edgeBridge and ts - lo > 1e-3 and not blocked(lo, ts) then ts = lo end
				if hi - te <= Portals.edgeBridge and hi - te > 1e-3 and not blocked(te, hi) then te = hi end
				if te - ts < ds * 0.5 then return end
				local aL, aR = a1 + (a2 - a1) * (ts / la), a1 + (a2 - a1) * (te / la)
				local dsum, dn = 0, 0
				for _, k in ipairs(runDrops) do dsum += e.drops[k]; dn += 1 end
				links[#links + 1] = {
					kind = kind, a = e.lo, b = e.hi,
					left = aL, right = aR, bLeft = onB(aR), bRight = onB(aL),
					centre = (aL + aR) * 0.5, span = (aR - aL).Magnitude,
					drop = dn > 0 and dsum / dn or 0, gap = gap,
					count = dn, residual = 0, fitted = true, edge = true,
				}
				made += 1
				stats.edgeLinks += 1
			end
			local lastHit = nil
			local t = lo
			while t <= hi + 1e-6 do
				local pA = a1 + (a2 - a1) * (t / la)
				local ev = evidence(pA + outward * (gap * 0.5), reach)
				if #ev > 0 then
					if runStart and lastHit and t - lastHit > ds * 1.5 then
						if t - lastHit > Portals.edgeBridge or blocked(lastHit, t) then
							emit(math.max(lo, runStart - ds * 0.5), math.min(hi, lastHit + ds * 0.5))
							runStart = nil
						else
							stats.edgeBridged += 1
						end
					end
					if not runStart then runStart = t; runDrops = {} end
					lastHit = t
					for _, k in ipairs(ev) do
						if not used[k] then used[k] = true; runDrops[#runDrops + 1] = k end
					end
				end
				t += ds
			end
			if runStart and lastHit then emit(math.max(lo, runStart - ds * 0.5), math.min(hi, lastHit + ds * 0.5)) end
		end
	end
	local nUsed = 0
	for _ in pairs(used) do nUsed += 1 end
	stats.edgeEvidence += #e.mids
	stats.edgeEvidenceUsed += nUsed
	return made
end

local function seamLinks(snap: any, of: { [any]: number }, step: number,
	links: { any }, stats: any, mesh: any)

	local buckets: { [string]: any } = {}
	for _, pr in ipairs(snap.pairs) do
		local pa, pb = of[pr.a], of[pr.b]
		if not pa or not pb then
			stats.pairsNoPolygon += 1
			continue
		end
		if pa == pb then
			-- both cells landed on the same face. Severance only keeps
			-- cross-REGION pairs and a polygon belongs to one region, so this
			-- can only be a claim that reached across a boundary; counted
			-- rather than turned into a self-link.
			stats.pairsSamePoly += 1
			continue
		end
		stats.pairsUsed += 1

		-- ordered, with the drop re-signed to match, so a step up from one side
		-- and down from the other are one link and not two
		local lo, hi, dn = pa, pb, pr.drop
		if lo > hi then lo, hi, dn = pb, pa, -dn end
		local k = lo .. ":" .. hi
		local e = buckets[k]
		if not e then
			e = { lo = lo, hi = hi, mids = {}, drops = {}, dirs = {},
				up = (lo == pa and pr.a or pr.b).normal or Vector3.yAxis }
			buckets[k] = e
		end
		e.mids[#e.mids + 1] = (pr.a.pos + pr.b.pos) * 0.5
		e.drops[#e.drops + 1] = dn
		-- which way you step, always from the low-numbered polygon
		local step_ = pr.b.pos - pr.a.pos
		e.dirs[#e.dirs + 1] = (lo == pa) and step_ or -step_
	end

	local keys = {}
	for k in pairs(buckets) do keys[#keys + 1] = k end
	table.sort(keys)   -- so two bakes list portals in the same order

	for _, k in ipairs(keys) do
		local e = buckets[k]
		if Portals.edgeMatch then
			local made = edgePortals(mesh, e, "seam", Portals.seamGap, step, links, stats)
			stats.seam += made
			if made > 0 then continue end
			stats.edgeFallback += 1
		end
		for _, members in ipairs(groupsOf(e.mids, step * Portals.groupReach)) do
			local gates = {}
			fitAll(e.mids, e.dirs, e.up, members, step, gates)
			if #gates > 1 then stats.gatesSplit += #gates - 1 end
			for _, gt in ipairs(gates) do
				local dsum = 0
				for _, i in ipairs(gt.members) do dsum += e.drops[i] end
				links[#links + 1] = {
					kind = "seam", a = e.lo, b = e.hi,
					left = gt.left, right = gt.right,
					centre = (gt.left + gt.right) * 0.5,
					span = (gt.right - gt.left).Magnitude,
					drop = dsum / #gt.members,
					count = #gt.members, residual = gt.resid, travel = gt.travel, fitted = gt.fromCells,
				}
				stats.seam += 1
				noteFit(stats, "seam", e.lo, e.hi, gt.resid, #gt.members, step, gt.fromCells)
			end
		end
	end
end

-- ------------------------------------------------------- 5. bridge links

-- Links across floor that exists but was never traced.
--
-- WHY THERE IS FLOOR WITH NO POLYGON ON IT. `Boundary.liveRegions` only traces a
-- region containing a solid k x k block of cells, k from `traceMinWidth` -- 3x3
-- at the shipped settings. A region narrower than that anywhere gets no
-- boundary, so no ring, so no polygon. On case3 that is 34 of 67 regions holding
-- 458 cells, and two of them are load bearing: r009 is the 0.5 x 24.0 stud flat
-- ridge of a gable roof sitting between its two pitched faces r003 and r004,
-- and r018 is an 11.5 x 0.5 strip between r005 and r007. Without these, four of
-- the largest polygons on the map are orphans and the navmesh comes out as 11
-- pieces where the cells say 9.
--
-- THE TRACE GATE IS RIGHT AND ITS CONSEQUENCE IS WRONG. You cannot stand around
-- on a half-stud ridge, which is what a polygon there would claim. You can
-- obviously step across it. Lowering `traceMinWidth` to fix this would trace
-- every handrail in the map to reach two ridges, so the distinction is drawn
-- here instead, where it actually lives.
--
-- ACROSS, NEVER ALONG, AND THE MEASURE IS DISTANCE. A link is emitted only where
-- the untraced floor between the two polygons is at most `traceMinWidth` deep --
-- one stride over the strip. Reaching a polygon at the far END of an untraced
-- corridor costs the corridor's whole length and is refused, because using it
-- would mean walking down the middle of something narrower than the agent,
-- which is exactly what the trace gate exists to forbid.
--
-- Counting CELLS instead of distance was the first rule here and it was wrong by
-- one cell: it required a single cell to touch both sides, which r009's
-- one-cell-wide ridge satisfies and r018's two-row strip does not, for no reason
-- an agent stepping over either of them would recognise.
local function bridgeLinks(snap: any, of: { [any]: number }, data: any, step: number,
	links: { any }, stats: any, mesh: any)

	local limit = Portals.crossLimit or data.config.traceMinWidth
		or data.config.minWidth or (step * 3)
	stats.crossLimit = limit

	-- The untraced floor: cells whose REGION never got a boundary.
	--
	-- NOT SIMPLY "EVERY CELL WITHOUT A POLYGON", which is what this was first and
	-- it was wrong. That set also holds the handful of cells inside TRACED regions
	-- that the claim in stage 2 missed -- 35 of 8268 on case3 -- and treating a
	-- claim failure as floor to bridge over produced links between two polygons of
	-- the SAME region, fitted through a scatter of missed cells rather than a
	-- strip. They were the worst fits in the map, residual 1.08 on a span of 2.42.
	-- A claim failure is a defect to report; it is not a bridge.
	local free = {}
	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			if cell.region and not of[cell] and not data.boundary[cell.region] then
				free[#free + 1] = cell
			end
		end
	end
	stats.bridgeCells = #free
	if #free == 0 then return end

	local reach = step * Portals.groupReach
	local G = reach
	local bucketCells: { [number]: { any } } = {}
	for _, cell in ipairs(free) do
		local p = cell.pos
		local k = hkey(math.floor(p.X / G), math.floor(p.Y / G), math.floor(p.Z / G))
		local b = bucketCells[k]
		if not b then b = {}; bucketCells[k] = b end
		b[#b + 1] = cell
	end
	local function neighbours(cell: any): { any }
		local p = cell.pos
		local out = {}
		local bx, by, bz = math.floor(p.X / G), math.floor(p.Y / G), math.floor(p.Z / G)
		for dx = -1, 1 do for dy = -1, 1 do for dz = -1, 1 do
			for _, q in ipairs(bucketCells[hkey(bx + dx, by + dy, bz + dz)] or {}) do
				if q ~= cell and (q.pos - p).Magnitude <= reach then out[#out + 1] = q end
			end
		end end end
		return out
	end

	-- Seeds: an untraced cell that some polygon's own cell pairs with directly.
	-- reachOf[cell][poly] = { d = studs of untraced floor walked to get here,
	-- src = the polygon-side cell it was entered from }.
	local reachOf: { [any]: { [number]: any } } = {}
	local queue: { any } = {}
	local function relax(cell: any, poly: number, d: number, src: any)
		if d > limit then return end
		local t = reachOf[cell]
		if not t then t = {}; reachOf[cell] = t end
		local cur = t[poly]
		if cur and cur.d <= d then return end
		t[poly] = { d = d, src = src }
		queue[#queue + 1] = { cell = cell, poly = poly }
	end
	for _, pr in ipairs(snap.pairs) do
		local pa, pb = of[pr.a], of[pr.b]
		if pa and not pb then relax(pr.b, pa, 0, pr.a)
		elseif pb and not pa then relax(pr.a, pb, 0, pr.b) end
	end

	-- Relaxation rather than a heap: the frontier is a few hundred cells under a
	-- hard distance cap, so the queue drains in a couple of passes and a heap
	-- would be more machinery than the problem has.
	local head = 1
	while head <= #queue do
		local e = queue[head]; head += 1
		local t = reachOf[e.cell]
		local cur = t and t[e.poly]
		if cur then
			for _, q in ipairs(neighbours(e.cell)) do
				relax(q, e.poly, cur.d + (q.pos - e.cell.pos).Magnitude, cur.src)
			end
		end
	end

	-- already linked directly: a bridge must not duplicate a seam
	local direct: { [string]: boolean } = {}
	for _, L in ipairs(links) do direct[L.a .. ":" .. L.b] = true end

	local buckets: { [string]: any } = {}
	local crossed, dead = 0, 0
	for cell, t in pairs(reachOf) do
		local ids = {}
		for poly in pairs(t) do ids[#ids + 1] = poly end
		table.sort(ids)
		if #ids < 2 then
			dead += 1
			continue
		end
		local used = false
		for i = 1, #ids - 1 do
			for j = i + 1, #ids do
				local lo, hi = ids[i], ids[j]
				-- the WHOLE crossing: in off one side and out the other
				if t[lo].d + t[hi].d > limit then
					stats.bridgeTooFar += 1
				elseif direct[lo .. ":" .. hi] then
					stats.bridgeDuplicate += 1
				else
					used = true
					local k = lo .. ":" .. hi
					local e = buckets[k]
					local up = t[lo].src.normal or Vector3.yAxis
					if not e then
						e = { lo = lo, hi = hi, mids = {}, drops = {}, dirs = {}, up = up }
						buckets[k] = e
					end
					e.mids[#e.mids + 1] = cell.pos
					-- measured between the two polygon-side cells, along the low
					-- side's own normal, so it means the same thing a seam drop does
					e.drops[#e.drops + 1] = (t[hi].src.pos - t[lo].src.pos):Dot(up)
					-- travel across the crossing, low side to high side
					e.dirs[#e.dirs + 1] = t[hi].src.pos - t[lo].src.pos
				end
			end
		end
		if used then crossed += 1 end
	end
	stats.bridgeCrossings = crossed
	stats.bridgeDeadEnd = dead

	local keys = {}
	for k in pairs(buckets) do keys[#keys + 1] = k end
	table.sort(keys)

	for _, k in ipairs(keys) do
		local e = buckets[k]
		if Portals.edgeMatch then
			local made = edgePortals(mesh, e, "bridge", Portals.bridgeGap, step, links, stats)
			stats.bridge += made
			if made > 0 then continue end
			stats.edgeFallback += 1
		end
		for _, members in ipairs(groupsOf(e.mids, step * Portals.groupReach)) do
			local gates = {}
			fitAll(e.mids, e.dirs, e.up, members, step, gates)
			if #gates > 1 then stats.gatesSplit += #gates - 1 end
			for _, gt in ipairs(gates) do
				local dsum = 0
				for _, i in ipairs(gt.members) do dsum += e.drops[i] end
				links[#links + 1] = {
					kind = "bridge", a = e.lo, b = e.hi,
					left = gt.left, right = gt.right,
					centre = (gt.left + gt.right) * 0.5,
					span = (gt.right - gt.left).Magnitude,
					drop = dsum / #gt.members,
					count = #gt.members, residual = gt.resid, travel = gt.travel, fitted = gt.fromCells,
				}
				stats.bridge += 1
				noteFit(stats, "bridge", e.lo, e.hi, gt.resid, #gt.members, step, gt.fromCells)
				-- A BRIDGE CHAINS TWO GATE CROSSINGS, so its height change is not bounded
				-- by Severance's 1.5 the way a seam's is -- in off one side and out the
				-- other can total 3.0. Four links on case3 come out at 2.75, which is two
				-- stair treads with an untraced one between them, and 2.75 is above the
				-- 2.0 step a Roblox humanoid climbs by default. Reported, never filtered:
				-- it is a fine link to fall DOWN and the direction is the consumer's to
				-- price, not ours to delete.
				if math.abs(links[#links].drop) > Agents.envelope().step then
					stats.bridgeSteep += 1
					stats.bridgeSteepAt[#stats.bridgeSteepAt + 1] =
						("f%04d-f%04d drop %+.2f over %d cells")
							:format(e.lo, e.hi, links[#links].drop, #gt.members)
				end
			end
		end
	end
end

-- ------------------------------------------------------------ components

-- ONE DOORWAY, ONE GATE. Seam and bridge gates are fitted per contiguity
-- group, and where two grids meet at a slight angle the cell pairs along one
-- opening come out with gaps just past `groupReach`, so a single doorway is cut
-- into a dashed row of short gates between the SAME two polygons -- Cocosulx
-- spotted rows of them on case6's steps. groupReach itself stays tight: it is
-- what keeps two doorways either side of a pillar apart.
--
-- So merge afterwards, and only what is provably one opening: same polygon
-- pair and kind, parallel (within mergeAngle), on one line (off it by at most
-- a step), the same height change (within mergeDrop), the gap between them at
-- most mergeGap -- and that gap clear at chest height, which is the pillar
-- test the contiguity rule was standing in for.
Portals.merge = true
Portals.mergeAngle = 10  -- degrees
Portals.mergeGap = 2.0   -- studs of gap along the gate line
Portals.mergeDrop = 0.25 -- studs of height-change difference
Portals.mergeLift = 2.0  -- studs above the gate the pillar ray runs

local function mergeGates(links: { any }, mesh: any, step: number, stats: any): { any }
	local rp = RaycastParams.new()
	rp.FilterType = Enum.RaycastFilterType.Exclude
	local dbg = workspace:FindFirstChild("NVGN_Debug")
	rp.FilterDescendantsInstances = dbg and { dbg } or {}
	local cosTol = math.cos(math.rad(Portals.mergeAngle))

	local out, groups, order = {}, {}, {}
	for _, L in ipairs(links) do
		-- an edge gate was cut where the evidence stopped, on purpose; merging it
		-- back across a rail lower than the pillar ray would undo exactly that
		if L.kind == "shared" or L.edge or L.span < 1e-4 then
			out[#out + 1] = L
		else
			local k = L.kind .. ":" .. L.a .. ":" .. L.b
			local g = groups[k]
			if not g then g = {}; groups[k] = g; order[#order + 1] = k end
			g[#g + 1] = L
		end
	end
	for _, k in ipairs(order) do
		local g = groups[k]
		if #g == 1 then out[#out + 1] = g[1]; continue end
		-- one axis for the pair, from the longest gate, oriented left->right
		local ref = g[1]
		for _, L in ipairs(g) do if L.span > ref.span then ref = L end end
		local ax = (ref.right - ref.left).Unit
		local up = mesh.tris[ref.a].up
		local o0 = ref.left
		local items = {}
		for _, L in ipairs(g) do
			local d = (L.right - L.left).Unit
			local lo, hi = L.left, L.right
			if d:Dot(ax) < 0 then lo, hi = hi, lo end
			items[#items + 1] = { L = L, t0 = (lo - o0):Dot(ax), t1 = (hi - o0):Dot(ax), lo = lo, hi = hi,
				par = math.abs(d:Dot(ax)) >= cosTol }
		end
		table.sort(items, function(x, y) return x.t0 < y.t0 end)
		local cur = nil
		local function flush()
			if cur then out[#out + 1] = cur.L end
		end
		for _, it in ipairs(items) do
			local ok = false
			if cur and cur.par and it.par then
				local gap = it.t0 - cur.t1
				-- distance of the next gate off the current one's line
				local v = it.lo - cur.lo
				local lateral = (v - ax * v:Dot(ax) - up * v:Dot(up)).Magnitude
				if gap <= Portals.mergeGap and lateral <= step
					and math.abs((it.L.drop or 0) - (cur.L.drop or 0)) <= Portals.mergeDrop then
					ok = true
					if gap > 0 then
						local a = cur.hi + up * Portals.mergeLift
						local b = it.lo + up * Portals.mergeLift
						if workspace:Raycast(a, b - a, rp) then ok = false; stats.mergeBlocked += 1 end
					end
				end
			end
			if ok then
				local A, B = cur.L, it.L
				local na, nb = A.count or 1, B.count or 1
				local hi = (it.t1 > cur.t1) and it.hi or cur.hi
				local t1 = math.max(it.t1, cur.t1)
				cur.L = {
					kind = A.kind, a = A.a, b = A.b,
					left = cur.lo, right = hi,
					centre = (cur.lo + hi) * 0.5,
					span = (hi - cur.lo).Magnitude,
					drop = ((A.drop or 0) * na + (B.drop or 0) * nb) / (na + nb),
					count = na + nb,
					residual = math.max(A.residual or 0, B.residual or 0),
					travel = (A.travel or Vector3.zero) + (B.travel or Vector3.zero),
					fitted = A.fitted and B.fitted, merged = (A.merged or 1) + (B.merged or 1),
				}
				cur.hi, cur.t1 = hi, t1
				stats.gatesMerged += 1
			else
				flush()
				cur = { L = it.L, lo = it.lo, hi = it.hi, t1 = it.t1, par = it.par }
			end
		end
		flush()
	end
	return out
end

local function components(nPoly: number, links: { any }): ({ number }, number)
	local parent = table.create(nPoly)
	for i = 1, nPoly do parent[i] = i end
	local function find(i: number): number
		local r = i
		while parent[r] ~= r do r = parent[r] end
		while parent[i] ~= r do local n = parent[i]; parent[i] = r; i = n end
		return r
	end
	for _, L in ipairs(links) do
		local ra, rb = find(L.a), find(L.b)
		if ra ~= rb then parent[rb] = ra end
	end
	local label, comp, n = {}, table.create(nPoly), 0
	for i = 1, nPoly do
		local r = find(i)
		local id = label[r]
		if not id then n += 1; id = n; label[r] = id end
		comp[i] = id
	end
	return comp, n
end

-- ---------------------------------------------------------------- build

-- Links for a convex mesh. `snap` is a `Severance.snapshot` of the same data.
--
-- ADDITIVE ONLY. Nothing here touches `verts`, `n`, `area` or `centre`; the mesh
-- that goes in is the mesh that comes out, and the portals sit beside it.
function Portals.build(mesh: any, data: any, snap: any): any
	local t0 = os.clock()
	local step = data.config.step

	local stats = {
		polys = #mesh.tris, edges = 0, shared = 0, seam = 0,
		overshared = 0, oversharedAt = {}, crossRegionKeys = 0,
		cellsTried = 0, cellsInside = 0, cellsNearby = 0, cellsNoPolygon = 0,
		pairs = #snap.pairs, pairsUsed = 0, pairsNoPolygon = 0, pairsSamePoly = 0,
		bridge = 0, bridgeCells = 0, bridgeCrossings = 0, bridgeDeadEnd = 0,
		bridgeDuplicate = 0, bridgeTooFar = 0, crossLimit = 0,
		bridgeSteep = 0, bridgeSteepAt = {}, gatesSplit = 0, gatesFromTravel = 0,
		bentGroups = 0, bentAt = {}, worstResidual = 0, worstResidualAt = "none",
		orphans = {}, pieces = 0, sevPieces = 0, sevSplit = {},
		gatesMerged = 0, mergeBlocked = 0,
		edgeLinks = 0, edgeFallback = 0, edgeEvidence = 0, edgeEvidenceUsed = 0, edgeBridged = 0,
		sevNarrow = 0, sevNarrowAt = {},
		seconds = 0,
	}

	local links: { any } = {}
	sharedLinks(mesh, links, stats)
	local of = claimCells(mesh, data, stats)
	seamLinks(snap, of, step, links, stats, mesh)
	bridgeLinks(snap, of, data, step, links, stats, mesh)
	if Portals.merge then
		links = mergeGates(links, mesh, step, stats)
		stats.seam, stats.bridge = 0, 0
		for _, L in ipairs(links) do
			if L.kind == "seam" then stats.seam += 1 elseif L.kind == "bridge" then stats.bridge += 1 end
		end
	end

	local degree = table.create(stats.polys, 0)
	for _, L in ipairs(links) do
		degree[L.a] = (degree[L.a] or 0) + 1
		degree[L.b] = (degree[L.b] or 0) + 1
	end
	-- NAMED, NOT COUNTED. A polygon nothing links to is a hole in the mesh, and
	-- "3 orphans" cannot be looked at in Studio the way "f0041 r006" can.
	for i = 1, stats.polys do
		if (degree[i] or 0) == 0 then
			stats.orphans[#stats.orphans + 1] =
				("f%04d r%03d %.1fsq"):format(i, mesh.tris[i].region, mesh.tris[i].area)
		end
	end

	local comp, pieces = components(stats.polys, links)
	stats.pieces = pieces

	-- THE CHECK THAT MATTERS, and it is against an independent answer: Severance
	-- decided connectivity from the CELLS, with a gate that never saw a polygon.
	-- Compared over the claimed cells only -- Severance covers all the cell
	-- regions and polygons exist for the traced ones, so its raw piece count is
	-- not the comparable number.
	--
	-- AN UPPER BOUND, NOT AN EQUALITY, and the difference is the point. Severance
	-- asks whether one cell is a step from the next and has no idea how wide the
	-- agent is; the mesh only exists where the floor is wide enough to stand on.
	-- So a Severance piece MAY legitimately span several polygon components, when
	-- what joins them is floor too narrow to trace. case4 does exactly this: its
	-- piece 17 runs through about thirty untraced regions of thirteen cells each,
	-- and refusing to link across them is the agent-width rule working, not a
	-- portal missing. A split with no untraced floor between the halves has no
	-- such excuse, and those two cases are reported differently below.
	--
	-- The converse -- one polygon component spanning several Severance pieces --
	-- cannot happen while every link is a subset of what Severance already
	-- joined, so it is asserted rather than expected.
	-- untraced floor per Severance piece, so a split can say which kind it is
	local narrow: { [number]: number } = {}
	for _, grid in ipairs(data.grids) do
		for _, cell in ipairs(grid.cells) do
			if cell.region and not of[cell] and not data.boundary[cell.region] then
				local s2 = snap.of[cell]
				if s2 then narrow[s2] = (narrow[s2] or 0) + 1 end
			end
		end
	end

	local sevSeen: { [number]: { [number]: boolean } } = {}
	local polySeen: { [number]: { [number]: boolean } } = {}
	for cell, pi in pairs(of) do
		local s = snap.of[cell]
		if s then
			local t = sevSeen[s]
			if not t then t = {}; sevSeen[s] = t end
			t[comp[pi]] = true
			local u = polySeen[comp[pi]]
			if not u then u = {}; polySeen[comp[pi]] = u end
			u[s] = true
		end
	end
	for s, t in pairs(sevSeen) do
		stats.sevPieces += 1
		local n = 0
		for _ in pairs(t) do n += 1 end
		if n > 1 then
			local nar = narrow[s] or 0
			if nar > 0 then
				stats.sevNarrow += 1
				stats.sevNarrowAt[#stats.sevNarrowAt + 1] =
					("piece %d spans %d components, %d untraced cells between them")
						:format(s, n, nar)
			else
				stats.sevSplit[#stats.sevSplit + 1] =
					("severance piece %d spread over %d polygon components, with no untraced floor between them")
						:format(s, n)
			end
		end
	end
	stats.merged = 0
	for _, u in pairs(polySeen) do
		local n = 0
		for _ in pairs(u) do n += 1 end
		if n > 1 then stats.merged += 1 end
	end

	stats.seconds = os.clock() - t0
	return { links = links, comp = comp, polyOf = of, stats = stats }
end

function Portals.report(res: any): string
	local s = res.stats
	local lines = {
		("portals   %d polys, %d links: %d shared, %d seam, %d bridge  (%.2fs)")
			:format(s.polys, #res.links, s.shared, s.seam, s.bridge, s.seconds),
		("  cells %d claimed (%d inside, %d within half a step), %d unclaimed")
			:format(s.cellsInside + s.cellsNearby, s.cellsInside, s.cellsNearby, s.cellsNoPolygon),
		("  severance pairs %d: %d used, %d no polygon, %d same polygon")
			:format(s.pairs, s.pairsUsed, s.pairsNoPolygon, s.pairsSamePoly),
		("  bridges: %d untraced cells, %d a crossing, %d dead ends; limit %.1f studs, %d over it, %d already direct")
			:format(s.bridgeCells, s.bridgeCrossings, s.bridgeDeadEnd,
				s.crossLimit, s.bridgeTooFar, s.bridgeDuplicate),
		("  %d components, severance says %d over the same cells (an upper bound: it does not know the agent's width)")
			:format(s.pieces, s.sevPieces),
		("  worst gate residual %.2f (%s), %d extra gates from cutting bent groups, %d gates squared to travel")
			:format(s.worstResidual, s.worstResidualAt, s.gatesSplit, s.gatesFromTravel),
		("  %d gates merged into their neighbours along one opening, %d merges refused by the pillar ray")
			:format(s.gatesMerged or 0, s.mergeBlocked or 0),
		("  edge-matched %d gates (%d evidence gaps bridged); %d polygon pairs fell back to fitting; %.0f%% of the crossing evidence lies on an edge gate")
			:format(s.edgeLinks or 0, s.edgeBridged or 0, s.edgeFallback or 0,
				100 * (s.edgeEvidenceUsed or 0) / math.max(1, s.edgeEvidence or 0)),
	}
	if s.overshared > 0 then
		lines[#lines + 1] = ("  !! %d edges owned by more than two polygons -- NOT A SURFACE")
			:format(s.overshared)
		for i = 1, math.min(#s.oversharedAt, 6) do
			lines[#lines + 1] = "     " .. s.oversharedAt[i]
		end
	end
	if s.crossRegionKeys > 0 then
		lines[#lines + 1] = ("  !! %d edge keys shared between regions -- the region tag is load bearing")
			:format(s.crossRegionKeys)
	end
	for _, m in ipairs(s.sevSplit) do
		lines[#lines + 1] = "  !! " .. m .. " -- LINKS MISSING"
	end
	if s.sevNarrow > 0 then
		lines[#lines + 1] = ("  !  %d severance pieces split by floor too narrow to trace -- agent width, not a missing link:")
			:format(s.sevNarrow)
		for i = 1, math.min(#s.sevNarrowAt, 6) do
			lines[#lines + 1] = "     " .. s.sevNarrowAt[i]
		end
	end
	if s.merged > 0 then
		lines[#lines + 1] = ("  !! %d polygon components span more than one severance piece -- links invented")
			:format(s.merged)
	end
	if #s.orphans > 0 then
		lines[#lines + 1] = ("  !! %d polygons with no link at all:"):format(#s.orphans)
		for i = 1, math.min(#s.orphans, 12) do
			lines[#lines + 1] = "     " .. s.orphans[i]
		end
		if #s.orphans > 12 then
			lines[#lines + 1] = ("     ... and %d more"):format(#s.orphans - 12)
		end
	end
	for i = 1, math.min(#s.bentAt, 6) do
		lines[#lines + 1] = "  !  group is not a line: " .. s.bentAt[i]
	end
	if s.bridgeSteep > 0 then
		lines[#lines + 1] = ("  !  %d bridges change height by more than the %.1f step -- two crossings chained:")
			:format(s.bridgeSteep, Agents.envelope().step)
		for i = 1, math.min(#s.bridgeSteepAt, 6) do
			lines[#lines + 1] = "     " .. s.bridgeSteepAt[i]
		end
	end
	return table.concat(lines, "\n")
end

return Portals
