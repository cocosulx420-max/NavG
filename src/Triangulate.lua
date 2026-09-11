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
		local A = pts[walk[ai]]
		for bi = 1, #hole do
			local B = pts[hole[bi]]
			local dx, dy = B.x - A.x, B.y - A.y
			local d = dx * dx + dy * dy
			if d < bestD and d > 0 then
				local ok = true
				for _, r in ipairs(rings) do
					local n = #r
					for i = 1, n do
						local P, Q = pts[r[i]], pts[r[(i % n) + 1]]
						if properCross(A, B, P, Q) then ok = false; break end
					end
					if not ok then break end
				end
				if ok then
					bestA, bestB, bestD = ai, bi, d
				end
			end
		end
	end
	if not bestA then return nil end

	-- outer[1..a], then the hole from b all the way round back to b, then
	-- outer[a] again. The hole keeps its own winding, which is opposite to the
	-- rim's, and that is what makes the combined walk close.
	local out = {}
	for i = 1, bestA do out[#out + 1] = walk[i] end
	local m = #hole
	for k = 0, m do out[#out + 1] = hole[((bestB - 1 + k) % m) + 1] end
	out[#out + 1] = walk[bestA]
	for i = bestA + 1, #walk do out[#out + 1] = walk[i] end
	return out
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
		holes = 0, unbridged = 0, dropped = 0, area = 0 }
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

		local walk = rim
		for _, h in ipairs(holeRings) do
			local merged = bridge(walk, pts, h, ringIdx)
			if merged then
				walk = merged
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
		local made = 0
		for _, t in ipairs(out) do
			local A, B, C = world[t[1]], world[t[2]], world[t[3]]
			local a = 0.5 * (B - A):Cross(C - A).Magnitude
			if a < Triangulate.minArea then
				stats.dropped += 1
				continue
			end
			tris[#tris + 1] = { a = A, b = B, c = C, region = r, up = up,
				area = a, centre = (A + B + C) / 3 }
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
	if s.dropped > 0 then
		lines[#lines + 1] = ("  %d slivers under %.0e dropped"):format(s.dropped, Triangulate.minArea)
	end
	for _, c in ipairs(res.complaints) do
		lines[#lines + 1] = "  ! " .. c
	end
	return table.concat(lines, "\n")
end

return Triangulate
