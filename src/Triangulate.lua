--!strict
-- NVGN.Triangulate -- rings to triangles, one convex piece at a time.
--
-- Rings said which loop is the outside of a surface and which is a hole punched
-- in it. That is the whole input this needs. A region becomes one polygon with
-- holes, the holes are cut into the outer rim by bridges, and the result is ear
-- clipped.
--
-- WHY BRIDGES RATHER THAN A LIBRARY. A polygon with holes is not a polygon, and
-- every triangulator that accepts one either bridges internally or runs a
-- constrained Delaunay. Bridging is a dozen lines and it is exact: cut a channel
-- of zero width from the outer rim to the hole and the two rings become one
-- closed walk. The channel is traversed once in each direction, so the walk is
-- weakly simple rather than simple, and the ear test below is written to expect
-- that.
--
-- WHY EAR CLIPPING. It is O(n^2) and it produces long thin triangles, and
-- neither matters here. The largest ring on any map we bake is 40 corners, so
-- the quadratic term is nothing, and a navmesh triangle is a search node -- a
-- sliver costs a slightly worse heuristic and nothing else. Delaunay would buy
-- prettier triangles for a great deal more code and a numerical robustness
-- problem we do not currently have.
--
-- MEASURED IN THE REGION'S PLANE, using Rings' own basis function rather than a
-- second copy of it. Every ring of a region is flattened against the region
-- normal, clipped in 2D, and the triangles are read back out through the
-- original Vector3 for each vertex, so no coordinate is ever reconstructed from
-- the projection and the output sits exactly on the traced corners.
--
-- NOTHING IS REPAIRED. A region with no outer rim, or two of them, or a ring the
-- trace left open, is skipped and counted. Those are trace defects and Rings
-- already complains about them; inventing a surface here would hide the defect
-- behind a plausible-looking mesh.

local Triangulate = {}

local Rings = require(script.Parent:WaitForChild("Rings"))

-- A triangle this small is degenerate for our purposes and is dropped rather
-- than emitted. Against a 0.5 stud cell, a thousandth of a square stud is noise
-- from a bridge that ran along an existing edge.
Triangulate.minArea = 1e-3

-- Ear clipping can stall on a ring that self-intersects, which no amount of
-- retrying fixes. Bail after this many failed sweeps and report the ring.
Triangulate.maxStalls = 2

-- Flip diagonals to Delaunay after clipping. Off only to compare against the
-- raw ear-clip output; the flip pass has no downside worth a switch.
Triangulate.flip = true

type V2 = { x: number, y: number }

local function cross(o: V2, a: V2, b: V2): number
	return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
end

-- Inclusive, because the bridge duplicates two vertices and an exclusive test
-- would let a clipper cut straight through the channel.
local function inTriangle(a: V2, b: V2, c: V2, p: V2): boolean
	return cross(a, b, p) >= 0 and cross(b, c, p) >= 0 and cross(c, a, p) >= 0
end

-- Do two segments cross at interior points of both. Touching at an endpoint is
-- not a crossing: a bridge candidate always starts and ends on the boundary.
local function properCross(a: V2, b: V2, c: V2, d: V2): boolean
	local d1 = cross(c, d, a)
	local d2 = cross(c, d, b)
	local d3 = cross(a, b, c)
	local d4 = cross(a, b, d)
	return ((d1 > 0) ~= (d2 > 0)) and ((d3 > 0) ~= (d4 > 0))
end

-- Crossing number. `ring` is a list of V2.
local function inside(ring: { V2 }, p: V2): boolean
	local n = #ring
	local hit = false
	local j = n
	for i = 1, n do
		local a, b = ring[i], ring[j]
		if ((a.y > p.y) ~= (b.y > p.y))
			and (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) then
			hit = not hit
		end
		j = i
	end
	return hit
end

-- Cut one hole into the outer walk.
--
-- BRUTE FORCE, DELIBERATELY. The textbook construction casts a ray from the
-- hole's extreme vertex and then hunts for a visible reflex vertex when the
-- first candidate is blocked, which is three cases and an angle comparison.
-- Trying every pair and keeping the shortest visible one is one case, and at 40
-- outer corners against 20 hole corners it is a few thousand segment tests.
--
-- `rings` is every ring of the region, used only as obstacles: a bridge must not
-- cross the outer rim, the hole it is reaching, or any other hole.
local function bridge(walk: { number }, pts: { V2 }, hole: { number },
	rings: { { number } }): { number }?
	local bestA, bestB, bestD = nil, nil, math.huge
	for ai = 1, #walk do
		local ia = walk[ai]
		local A = pts[ia]
		for bi = 1, #hole do
			local ib = hole[bi]
			local B = pts[ib]
			local dx, dy = B.x - A.x, B.y - A.y
			local d = dx * dx + dy * dy
			if d < bestD and d > 0 then
				local ok = true
				for _, r in ipairs(rings) do
					local n = #r
					for i = 1, n do
						local ip, iq = r[i], r[(i % n) + 1]
						-- SKIP BY INDEX, not by coordinate. A bridge starts and ends
						-- on the boundary, so the two edges at each end touch it at a
						-- shared vertex. A cross product cannot tell that touch from a
						-- crossing -- one of its four determinants is exactly zero and
						-- the sign comparison then reads true -- and every candidate
						-- was being refused because of it.
						if ip ~= ia and ip ~= ib and iq ~= ia and iq ~= ib
							and properCross(A, B, pts[ip], pts[iq]) then
							ok = false
							break
						end
					end
					if not ok then break end
				end
				-- Not crossing an edge leaves one case: a bridge that runs along the
				-- boundary, or through a hole it never crosses because it grazes a
				-- vertex. The midpoint settles both.
				if ok then
					local mid = { x = (A.x + B.x) * 0.5, y = (A.y + B.y) * 0.5 }
					local ring = {}
					for k, iv in ipairs(rings[1]) do ring[k] = pts[iv] end
					if not inside(ring, mid) then ok = false end
				end
				if ok then
					bestA, bestB, bestD = ai, bi, d
				end
			end
		end
	end
	if not bestA then return nil end
	local bridgeA, bridgeB = walk[bestA], hole[bestB]

	-- outer[1..a], then the hole from b all the way round back to b, then
	-- outer[a] again. The hole keeps its own winding, which is opposite to the
	-- rim's, and that is what makes the combined walk close.
	local out = {}
	for i = 1, bestA do out[#out + 1] = walk[i] end
	local m = #hole
	for k = 0, m do out[#out + 1] = hole[((bestB - 1 + k) % m) + 1] end
	out[#out + 1] = walk[bestA]
	for i = bestA + 1, #walk do out[#out + 1] = walk[i] end
	return out, bridgeA, bridgeB
end

-- Is `d` inside the circle through a, b and c. `abc` must be counter-clockwise.
local function inCircle(a: V2, b: V2, c: V2, d: V2): boolean
	local ax, ay = a.x - d.x, a.y - d.y
	local bx, by = b.x - d.x, b.y - d.y
	local cx, cy = c.x - d.x, c.y - d.y
	return (ax * ax + ay * ay) * (bx * cy - cx * by)
		- (bx * bx + by * by) * (ax * cy - cx * ay)
		+ (cx * cx + cy * cy) * (ax * by - bx * ay) > 0
end

-- Flip diagonals until the triangulation is Delaunay.
--
-- WHY THIS AND NOT A BETTER CLIPPER. Ear clipping takes whichever ear it meets
-- first, so the shape of what it produces is an accident of scan order, and long
-- thin triangles are the usual result. Flipping fixes it after the fact and
-- comes with a guarantee no clipper can offer: among every triangulation of the
-- same corners, the Delaunay one maximises the SMALLEST angle. So there is no
-- arrangement of these vertices with fewer slivers than what this converges to.
--
-- It cannot invent quality that the corner set does not allow. Two corners half
-- a stud apart at the end of a twenty stud edge will always make a sliver, and
-- the only cure for that is adding vertices, which is a different algorithm and
-- a lot more of it.
--
-- CONSTRAINED. A ring edge is the floor's own boundary and a bridge is the
-- channel cut into a hole; flipping either one puts a triangle outside the
-- surface or straight across the hole. Both are refused by index.
local function delaunay(tris: { { number } }, pts: { V2 },
	fixed: { [number]: boolean }): number
	local function key(u: number, v: number): number
		if u > v then u, v = v, u end
		return u * 1000000 + v
	end
	local flips = 0
	for _ = 1, 200 do
		-- rebuilt every flip rather than patched. Two triangles change and the
		-- edges around them all move; patching that in place is where this kind
		-- of loop goes wrong, and there are only a handful of triangles here.
		local edge: { [number]: { number } } = {}
		for i, t in ipairs(tris) do
			for j = 1, 3 do
				local u, v = t[j], t[(j % 3) + 1]
				local k = key(u, v)
				local e = edge[k]
				if not e then e = {}; edge[k] = e end
				e[#e + 1] = i
			end
		end
		local did = false
		for k, e in pairs(edge) do
			if #e == 2 and not fixed[k] then
				local t1, t2 = tris[e[1]], tris[e[2]]
				local v = k % 1000000
				local u = (k - v) / 1000000
				local p2, p3 = nil, nil
				for _, x in ipairs(t1) do if x ~= u and x ~= v then p2 = x end end
				for _, x in ipairs(t2) do if x ~= u and x ~= v then p3 = x end end
				if p2 and p3 and p2 ~= p3 then
					local A, B = pts[u], pts[v]
					local C, D = pts[p2], pts[p3]
					-- the quad must be convex or the flip leaves the surface: u and v
					-- have to sit on opposite sides of the new diagonal
					local s1 = cross(C, D, A)
					local s2 = cross(C, D, B)
					if (s1 > 0) ~= (s2 > 0) and s1 ~= 0 and s2 ~= 0 then
						local a, b, c = A, B, C
						if cross(a, b, c) < 0 then a, b = b, a end
						if inCircle(a, b, c, D) then
							tris[e[1]] = { u, p3, p2 }
							tris[e[2]] = { v, p2, p3 }
							flips += 1
							did = true
							break
						end
					end
				end
			end
		end
		if not did then break end
	end
	return flips
end

-- Ear clip a counter-clockwise walk. Returns index triples into `pts`.
local function earClip(walk: { number }, pts: { V2 }): ({ { number } }, string?)
	local V = table.clone(walk)
	local tris = {}
	local stalls = 0
	while #V > 3 do
		local clipped = false
		local n = #V
		for i = 1, n do
			local ia = V[((i - 2) % n) + 1]
			local ib = V[i]
			local ic = V[(i % n) + 1]
			local a, b, c = pts[ia], pts[ib], pts[ic]
			if cross(a, b, c) > 0 then
				-- convex tip. It is an ear unless some other vertex of the walk
				-- lies in it. Only a REFLEX vertex can block: a convex one that
				-- landed inside would mean the walk crosses itself, and the
				-- bridge duplicates are convex-or-collinear by construction.
				local ear = true
				for j = 1, n do
					local iv = V[j]
					if iv ~= ia and iv ~= ib and iv ~= ic then
						local p = pts[iv]
						local prev = pts[V[((j - 2) % n) + 1]]
						local nxt = pts[V[(j % n) + 1]]
						if cross(prev, p, nxt) <= 0 and inTriangle(a, b, c, p) then
							ear = false
							break
						end
					end
				end
				if ear then
					tris[#tris + 1] = { ia, ib, ic }
					table.remove(V, i)
					clipped = true
					break
				end
			end
		end
		if not clipped then
			stalls += 1
			if stalls > Triangulate.maxStalls then
				return tris, ("stalled with %d vertices left"):format(#V)
			end
			-- One rescue, and only one: drop the sharpest vertex and carry on.
			-- A walk that stalls twice is self-intersecting and no local repair
			-- will finish it.
			table.remove(V, 1)
		end
	end
	if #V == 3 then tris[#tris + 1] = { V[1], V[2], V[3] } end
	return tris, nil
end

-- Triangulate every region of a simplified result.
--
-- Returns `{ tris, stats, complaints }`. A triangle carries its three world
-- corners, its centroid, its region, and the region normal, which is everything
-- a search node needs and everything a drawing needs.
function Triangulate.build(loops: { any }): any
	local stats = { regions = 0, done = 0, skipped = 0, tris = 0,
		holes = 0, unbridged = 0, dropped = 0, area = 0, flips = 0,
		minAngle = 180, slivers = 0 }
	local complaints = {}
	local tris = {}

	local byRegion, order = {}, {}
	for i, L in ipairs(loops) do
		if not byRegion[L.region] then
			byRegion[L.region] = {}
			order[#order + 1] = L.region
		end
		local g = byRegion[L.region]
		g[#g + 1] = i
	end

	for _, r in ipairs(order) do
		stats.regions += 1
		local idxs = byRegion[r]

		local outer, holes = nil, {}
		for _, i in ipairs(idxs) do
			local L = loops[i]
			if L.kind == "outer" then
				if outer then
					-- Rings already complained about this; say what it costs
					-- HERE rather than picking one rim and pretending.
					outer = false
				elseif outer == nil then
					outer = i
				end
			elseif L.kind == "hole" then
				holes[#holes + 1] = i
			end
		end

		if outer == nil or outer == false then
			stats.skipped += 1
			complaints[#complaints + 1] = (outer == false)
				and ("r%03d: more than one outer rim, not triangulated"):format(r)
				or ("r%03d: no outer rim, not triangulated"):format(r)
			continue
		end

		local up = loops[outer].regionUp or loops[outer].up
		local e1, e2 = Rings.basis(up)
		local origin = loops[outer].pts[1]

		-- Flatten every ring of the region into one shared vertex table, so a
		-- bridge can name a hole vertex and a rim vertex with the same index.
		local pts, world = {}, {}
		local ringIdx = {}
		local function flatten(L: any): { number }
			local out = {}
			for _, p in ipairs(L.pts) do
				local d = p - origin
				pts[#pts + 1] = { x = d:Dot(e1), y = d:Dot(e2) }
				world[#world + 1] = p
				out[#out + 1] = #pts
			end
			return out
		end

		local rim = flatten(loops[outer])
		ringIdx[#ringIdx + 1] = rim
		local holeRings = {}
		for _, i in ipairs(holes) do
			local h = flatten(loops[i])
			holeRings[#holeRings + 1] = h
			ringIdx[#ringIdx + 1] = h
			stats.holes += 1
		end

		-- Largest hole first. A big hole has the most ways to be blocked, and
		-- bridging it while the walk is still simple is the easy case.
		table.sort(holeRings, function(a, b) return #a > #b end)

		-- Edges the flip pass must not touch: every ring edge, because that is
		-- the floor's own boundary, and every bridge, because it is the channel
		-- cut into a hole.
		local fixed: { [number]: boolean } = {}
		local function hold(u: number, v: number)
			if u > v then u, v = v, u end
			fixed[u * 1000000 + v] = true
		end
		for _, ring in ipairs(ringIdx) do
			local n = #ring
			for i = 1, n do hold(ring[i], ring[(i % n) + 1]) end
		end

		local walk = rim
		for _, h in ipairs(holeRings) do
			local merged, ba, bb = bridge(walk, pts, h, ringIdx)
			if merged then
				walk = merged
				hold(ba :: number, bb :: number)
			else
				stats.unbridged += 1
				complaints[#complaints + 1] =
					("r%03d: a hole could not be bridged, filled in"):format(r)
			end
		end

		local out, err = earClip(walk, pts)
		if err then
			complaints[#complaints + 1] = ("r%03d: %s"):format(r, err)
		end
		if Triangulate.flip then
			stats.flips += delaunay(out, pts, fixed)
		end
		local made = 0
		for _, t in ipairs(out) do
			local A, B, C = world[t[1]], world[t[2]], world[t[3]]
			local a = 0.5 * (B - A):Cross(C - A).Magnitude
			if a < Triangulate.minArea then
				stats.dropped += 1
				continue
			end
			-- The SMALLEST ANGLE is the honest shape measure. Area says nothing
			-- about a sliver: a long thin triangle can have plenty of it.
			local ab, bc, ca = (B - A).Magnitude, (C - B).Magnitude, (A - C).Magnitude
			local worst = 180
			for _, t3 in ipairs({ { ab, ca, bc }, { bc, ab, ca }, { ca, bc, ab } }) do
				local x, y, z = t3[1], t3[2], t3[3]
				if x > 1e-9 and y > 1e-9 then
					local cosang = math.clamp((x * x + y * y - z * z) / (2 * x * y), -1, 1)
					local deg = math.deg(math.acos(cosang))
					if deg < worst then worst = deg end
				end
			end
			if worst < stats.minAngle then stats.minAngle = worst end
			if worst < 15 then stats.slivers += 1 end

			tris[#tris + 1] = { a = A, b = B, c = C, region = r, up = up,
				area = a, centre = (A + B + C) / 3, minAngle = worst }
			stats.area += a
			made += 1
		end
		stats.tris += made
		if made > 0 then stats.done += 1 end
	end

	return { tris = tris, stats = stats, complaints = complaints }
end

-- Total triangle area against total ring area is the check that matters: a
-- triangulation that lost a piece of floor, or filled in a hole, shows up here
-- and nowhere else.
function Triangulate.checkArea(loops: { any }, res: any): (number, number)
	local want = 0
	for _, L in ipairs(loops) do
		if L.kind == "outer" or L.kind == "hole" then
			want += L.area or 0
		end
	end
	return res.stats.area, want
end

function Triangulate.report(res: any, loops: { any }?): string
	local s = res.stats
	local lines = {
		("tri       %d regions, %d triangulated, %d skipped, %d triangles, %d holes")
			:format(s.regions, s.done, s.skipped, s.tris, s.holes),
	}
	if loops then
		local got, want = Triangulate.checkArea(loops, res)
		lines[#lines + 1] = ("  area %.1f of %.1f sq studs (%.2f%%)")
			:format(got, want, want > 0 and (got / want * 100) or 0)
	end
	lines[#lines + 1] = ("  smallest angle %.1f deg, %d triangles under 15 deg, %d flips")
		:format(s.minAngle, s.slivers, s.flips)
	if s.dropped > 0 then
		lines[#lines + 1] = ("  %d slivers under %.0e dropped"):format(s.dropped, Triangulate.minArea)
	end
	for _, c in ipairs(res.complaints) do
		lines[#lines + 1] = "  ! " .. c
	end
	return table.concat(lines, "\n")
end

return Triangulate
