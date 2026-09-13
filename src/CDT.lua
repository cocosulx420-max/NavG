--!strict
-- NVGN.CDT -- constrained Delaunay triangulation with Ruppert refinement.
--
-- Triangulate.lua triangulates the traced boundary and nothing else. Its corner
-- set IS the boundary, forty corners for a large room, so the best it can ever
-- produce is a handful of enormous triangles. Its own comment says so: it
-- cannot invent quality the corner set does not allow, and the only cure is
-- adding vertices. This is that algorithm.
--
-- WHY A REAL CDT AND NOT A REFINEMENT OF THE EAR CLIP. Two reasons, and the
-- second is the one that pays. A CDT gives a genuine minimum-angle guarantee
-- under Ruppert refinement, where bisecting whatever the ear clipper happened to
-- emit only promises half the angle it started with. And a CDT takes hole rings
-- as CONSTRAINTS, directly -- so there is no bridging at all, and the
-- bridge-touches-boundary bug is deleted rather than fixed.
--
-- THE SHAPE OF THE OUTPUT IS WHAT THE FUNNEL NEEDS. Small triangles of similar
-- size, then merged into the largest convex polygons that exist, because the
-- funnel assumes straight-line travel inside a polygon is safe and the fewest
-- polygons means the fewest portals to cross.
--
-- NOTHING IS DELETED. Not a sliver, not a border triangle. A deleted polygon is
-- a hole, and a hole is an obstacle that does not exist in the map -- invisible
-- in the drawing, and a severed map if it lands in a doorway. Same call as
-- erosion, made the same way and for the same reason.

local CDT = {}

local Rings = require(script.Parent:WaitForChild("Rings"))
local Triangulate = require(script.Parent:WaitForChild("Triangulate"))

-- Refine before merging: add interior Steiner points until every triangle meets
-- the bounds below.
--
-- OFF, AND THE MEASUREMENT IS WHY. The idea was to add interior geometry so the
-- triangulation is well shaped and then merge it away into big convex polygons.
-- The first half works. The second cannot: Hertel-Mehlhorn dissolves EDGES and
-- never vertices, so every interior Steiner point is a pin that the merge has no
-- mechanism to pull out. Measured on case3, same floor, same invariants:
--
--     refined (2.0 studs / 22 deg)   2373 tris -> 729 polys, worst poly angle 18.4
--     not refined                     247 tris -> 100 polys, worst poly angle 10.9
--
-- Seven times the polygons, and every surplus polygon is a portal the funnel has
-- to cross and a node A* has to expand. What it buys is seven degrees on the
-- worst corner, which is a rendering concern and not a pathfinding one -- a
-- thin convex polygon costs a slightly worse heuristic and nothing else. The
-- bare 10.5 x 24.25 rectangle on case3 comes out as ONE polygon with this off
-- and as 104 with it on; Cocosulx spotted that from the drawing.
--
-- Recast draws the same line: its polygon mesh, the one that is searched, comes
-- from the contour alone, and the refined detail mesh exists only to sample
-- height. Turn this on when there is a detail mesh to build.
CDT.refine = false

-- Longest edge a finished triangle may have, in studs. Four grid cells. This is
-- the knob that makes the triangles SIMILAR IN SIZE. Only read when `refine`.
CDT.targetEdge = 2.0

-- Smallest interior angle a triangle may have, in degrees. Ruppert is proven to
-- terminate below about 20.7 and behaves well into the thirties. 22 sits just
-- above the proven range, which is why the cap is reported rather than silenced.
-- Only read when `refine`.
CDT.minAngle = 22

-- A constrained subsegment is never split below this. THIS IS THE TERMINATION
-- GUARD. Two ring edges meeting at a sharp angle encroach each other's midpoints
-- forever, each split making the next one worse; that is the known failure mode
-- of Ruppert on real input, and a floor plan is full of sharp corners.
CDT.minSegLen = 0.25

-- Hard cap on refinement operations per region. Hitting it is REPORTED, never
-- silenced: it means the refinement did not converge and the caller is holding a
-- mesh that does not meet the bounds it claims.
CDT.maxRefine = 60000

-- Corners a merged convex polygon may have. Convexity is the real constraint --
-- Triangulate caps at 4 because quads were asked for once, and a funnel wants
-- the opposite.
CDT.maxVerts = math.huge

-- Merging and decimation feed each other -- removing a vertex unblocks merges,
-- and merging changes which vertices have a convex link -- so they run in
-- alternating rounds to a fixed point rather than for a guessed number of
-- sweeps. This is only a safety stop, and hitting it is reported.
CDT.maxRounds = 64

-- How far off the straight line a boundary Steiner point may sit and still be
-- called collinear, as a fraction of the run it sits on.
--
-- THIS IS NOT A SIMPLIFICATION TOLERANCE. It only has to absorb the rounding in
-- a computed midpoint. `straighten` has already removed every traced corner
-- within 0.1 studs of its own chord, so anything still collinear to a
-- billionth is a split midpoint, and putting its parent edge back moves
-- nothing. Widening this would start moving where the floor ends, which is
-- PathSimplify's job and is already finished by the time this runs.
CDT.collinearExact = 1e-9

-- Drop a ring corner sitting closer than this to the line through its
-- neighbours. Same value and same reason as Triangulate.collinear: a corner four
-- hundredths of a stud off a straight edge is quantisation noise, and here it
-- would force a constrained subsegment far under targetEdge and drag a whole
-- neighbourhood of tiny triangles along with it.
CDT.collinear = 0.1

-- ---------------------------------------------------------------- primitives

local nxt = { 2, 3, 1 }
local prv = { 3, 1, 2 }

-- Vertex pair key. K is above any plausible vertex count for one region and
-- keeps the product exact in a double.
local K = 1048576
local function ekey(u: number, v: number): number
	if u > v then u, v = v, u end
	return u * K + v
end

local function cross2(ax: number, ay: number, bx: number, by: number,
	cx: number, cy: number): number
	return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
end

-- ------------------------------------------------------------------- mesh

-- Triangle-with-neighbours. `t[1..3]` are vertex indices counter-clockwise and
-- `t[3+i]` is the triangle across the edge OPPOSITE vertex i, which is the
-- directed edge (t[nxt[i]], t[prv[i]]). Every operation below is local to this,
-- which is the whole reason it exists: Triangulate's flip pass rebuilds a global
-- edge map per flip, and that is fine at forty triangles and hopeless at four
-- thousand.
export type Mesh = any

local function newMesh(): Mesh
	return {
		px = {}, py = {}, world = {},
		tri = {}, dead = {}, inside = {},
		vt = {},                 -- vertex -> one live triangle holding it
		seg = {},                -- ekey -> true for a constrained subsegment
		segs = {},               -- work list of {u, v}; stale entries fail `seg`
		nv = 0, nt = 0,
	}
end

local function addVertex(m: Mesh, x: number, y: number, w: Vector3?): number
	local i = m.nv + 1
	m.nv = i
	m.px[i] = x
	m.py[i] = y
	m.world[i] = w
	return i
end

local function newTri(m: Mesh, a: number, b: number, c: number): number
	local t = m.nt + 1
	m.nt = t
	m.tri[t] = { a, b, c }
	m.dead[t] = false
	m.inside[t] = false
	m.vt[a] = t; m.vt[b] = t; m.vt[c] = t
	return t
end

local function relink(m: Mesh, nb: number?, old: number, new: number)
	if not nb then return end
	local N = m.tri[nb]
	for i = 1, 3 do
		if N[3 + i] == old then N[3 + i] = new; return end
	end
end

-- a, b, c must be counter-clockwise. True when d is strictly inside their
-- circumcircle. Evaluated relative to d so the magnitudes stay small, the same
-- trick Rings.signedArea uses to stay honest a thousand studs from the origin.
local function inCircle(m: Mesh, a: number, b: number, c: number, d: number): boolean
	local dx, dy = m.px[d], m.py[d]
	local ax, ay = m.px[a] - dx, m.py[a] - dy
	local bx, by = m.px[b] - dx, m.py[b] - dy
	local cx, cy = m.px[c] - dx, m.py[c] - dy
	return (ax * ax + ay * ay) * (bx * cy - cx * by)
		- (bx * bx + by * by) * (ax * cy - cx * ay)
		+ (cx * cx + cy * cy) * (ax * by - bx * ay) > 0
end

-- Every live triangle holding `u`. Walked through the adjacency rather than
-- scanned, so it costs the vertex degree and not the mesh.
local function fan(m: Mesh, u: number): { number }
	local out, seen = {}, {}
	local start = m.vt[u]
	if not start or m.dead[start] then
		start = nil
		for t = 1, m.nt do
			if not m.dead[t] then
				local T = m.tri[t]
				if T[1] == u or T[2] == u or T[3] == u then start = t; break end
			end
		end
	end
	if not start then return out end
	local stack = { start }
	seen[start] = true
	while #stack > 0 do
		local t = stack[#stack]
		stack[#stack] = nil
		local T = m.tri[t]
		if not m.dead[t] and (T[1] == u or T[2] == u or T[3] == u) then
			out[#out + 1] = t
			m.vt[u] = t
			for i = 1, 3 do
				local nb = T[3 + i]
				if nb and not seen[nb] then seen[nb] = true; stack[#stack + 1] = nb end
			end
		end
	end
	return out
end

local function hasEdge(m: Mesh, u: number, v: number): boolean
	for _, t in ipairs(fan(m, u)) do
		local T = m.tri[t]
		if T[1] == v or T[2] == v or T[3] == v then return true end
	end
	return false
end

-- Steepest-descent walk to the triangle containing (x, y).
--
-- With `respect`, a constrained edge STOPS the walk and is handed back instead
-- of crossed. That is not a failure: Ruppert's rule is that a circumcentre you
-- cannot see means the segment in the way has to be split, and this is where
-- that gets found.
local function locate(m: Mesh, t: number?, x: number, y: number, respect: boolean)
	: (number?, number?, number?)
	if t and m.dead[t] then t = nil end
	if not t then
		for i = 1, m.nt do if not m.dead[i] then t = i; break end end
	end
	if not t then return nil end
	for _ = 1, 200000 do
		local T = m.tri[t]
		local best, bi = 0, nil
		for i = 1, 3 do
			local b, c = T[nxt[i]], T[prv[i]]
			local s = cross2(m.px[b], m.py[b], m.px[c], m.py[c], x, y)
			if s < best then best, bi = s, i end
		end
		if not bi then return t end
		local u, v = T[nxt[bi]], T[prv[bi]]
		if respect and m.seg[ekey(u, v)] then return nil, u, v end
		local nb = T[3 + bi]
		if not nb or m.dead[nb] then return nil end
		t = nb
	end
	return nil
end

-- Bowyer-Watson. Returns the new vertex and the triangles created.
--
-- The cavity never crosses a constrained edge, which is what makes this a
-- CONSTRAINED Delaunay insertion and not a Delaunay one. Each new triangle
-- inherits `inside` from the cavity triangle that owned its boundary edge, so a
-- cavity straddling a segment -- which is exactly what happens when a segment
-- midpoint is inserted -- still labels both sides correctly.
local function insertPoint(m: Mesh, x: number, y: number, w: Vector3?, hint: number?)
	: (number?, { number }?)
	local t0 = locate(m, hint, x, y, false)
	if not t0 then return nil end
	local p = addVertex(m, x, y, w)

	local cav, inCav = { t0 }, { [t0] = true }
	local qi = 1
	while qi <= #cav do
		local t = cav[qi]; qi += 1
		local T = m.tri[t]
		for i = 1, 3 do
			local nb = T[3 + i]
			if nb and not inCav[nb] and not m.dead[nb]
				and not m.seg[ekey(T[nxt[i]], T[prv[i]])] then
				local N = m.tri[nb]
				if inCircle(m, N[1], N[2], N[3], p) then
					inCav[nb] = true
					cav[#cav + 1] = nb
				end
			end
		end
	end

	-- The boundary is collected only once the cavity is FINAL. Collecting it as
	-- the flood runs records edges that a later triangle then swallows.
	local bnd = {}
	for _, t in ipairs(cav) do
		local T = m.tri[t]
		for i = 1, 3 do
			local nb = T[3 + i]
			if not nb or not inCav[nb] then
				bnd[#bnd + 1] = { T[nxt[i]], T[prv[i]], nb, m.inside[t], t }
			end
		end
	end
	for _, t in ipairs(cav) do m.dead[t] = true end

	local byU, byV, made = {}, {}, {}
	for _, e in ipairs(bnd) do
		local u, v, nb = e[1], e[2], e[3]
		local t = newTri(m, u, v, p)
		m.inside[t] = e[4]
		m.tri[t][6] = nb
		relink(m, nb, e[5], t)
		byU[u] = t; byV[v] = t
		made[#made + 1] = t
	end
	-- The fan closes on itself: the triangle across (v, p) is the one whose own
	-- u is this v, and the triangle across (p, u) is the one whose own v is this u.
	for _, t in ipairs(made) do
		local T = m.tri[t]
		T[4] = byU[T[2]]
		T[5] = byV[T[1]]
	end
	m.vt[p] = made[1]
	return p, made
end

-- Swap the diagonal of the quad around edge `i` of triangle `t`. Refuses when
-- the quad is not strictly convex, which is the only case where a flip would
-- leave the surface.
local function flip(m: Mesh, t: number, i: number): boolean
	local T = m.tri[t]
	local mt = T[3 + i]
	if not mt or m.dead[mt] then return false end
	local A, B, C = T[i], T[nxt[i]], T[prv[i]]
	-- A constrained edge is the floor's own boundary; swapping it puts a
	-- triangle across a wall.
	if m.seg[ekey(B, C)] then return false end
	local M = m.tri[mt]
	local j = nil
	for k = 1, 3 do if M[k] ~= B and M[k] ~= C then j = k; break end end
	if not j then return false end
	local D = M[j]
	local ax, ay = m.px[A], m.py[A]
	local bx, by = m.px[B], m.py[B]
	local cx, cy = m.px[C], m.py[C]
	local dx, dy = m.px[D], m.py[D]
	if cross2(ax, ay, bx, by, dx, dy) <= 0 then return false end
	if cross2(ax, ay, dx, dy, cx, cy) <= 0 then return false end

	local nCA, nAB = T[3 + nxt[i]], T[3 + prv[i]]
	local nBD, nDC = M[3 + nxt[j]], M[3 + prv[j]]

	m.tri[t] = { A, B, D, nBD, mt, nAB }
	m.tri[mt] = { A, D, C, nDC, nCA, t }
	relink(m, nBD, mt, t)
	relink(m, nCA, t, mt)
	m.vt[A] = t; m.vt[B] = t; m.vt[D] = t; m.vt[C] = mt
	return true
end

-- Every triangle edge the open segment u->v passes through, near end first.
--
-- The second return is a vertex sitting ON the segment. That is not an error and
-- not a tolerance problem: the traced boundary really does put a corner in the
-- middle of a straight run -- a seam stitch, or a hole ring touching the rim --
-- and a segment through a vertex crosses no edge at all, so the flip loop has
-- nothing to work on and would give up. The caller constrains the two halves.
local function crossings(m: Mesh, u: number, v: number): ({ { number } }, number?)
	local out = {}
	local ux, uy = m.px[u], m.py[u]
	local vx, vy = m.px[v], m.py[v]
	local ex, ey = vx - ux, vy - uy
	local elen = math.sqrt(ex * ex + ey * ey)
	if elen < 1e-12 then return out end

	-- On the segment: within a thousandth of a stud of the line, and strictly
	-- between the ends along it.
	local function onSeg(w: number): boolean
		local wx, wy = m.px[w] - ux, m.py[w] - uy
		if math.abs(ex * wy - ey * wx) > 1e-3 * elen then return false end
		local s = (wx * ex + wy * ey) / (elen * elen)
		return s > 1e-9 and s < 1 - 1e-9
	end

	local cur, ei = nil, nil
	for _, t in ipairs(fan(m, u)) do
		local T = m.tri[t]
		local k = (T[1] == u and 1) or (T[2] == u and 2) or 3
		local b, c = T[nxt[k]], T[prv[k]]
		if b == v or c == v then return out end
		if onSeg(b) then return out, b end
		if onSeg(c) then return out, c end
		-- THE CONE OUT OF u, NOT THE SIDE OF THE LINE THROUGH u AND v. The
		-- triangles around u partition the directions out of it, and the segment
		-- leaves through the one whose cone holds the direction u->v. Testing
		-- which side of the LINE b and c fall on also accepts the triangle
		-- pointing the OPPOSITE way, and the walk then sets off away from v and
		-- out through the super-triangle.
		if cross2(ux, uy, m.px[b], m.py[b], vx, vy) > 0
			and cross2(ux, uy, vx, vy, m.px[c], m.py[c]) > 0 then
			cur, ei = t, k
			break
		end
	end
	if not cur or not ei then return out end
	out[1] = { cur, ei }
	for _ = 1, 100000 do
		local T = m.tri[cur]
		local b, c = T[nxt[ei]], T[prv[ei]]
		local nb = T[3 + ei]
		if not nb or m.dead[nb] then break end
		local N = m.tri[nb]
		local j = nil
		for k = 1, 3 do if N[k] ~= b and N[k] ~= c then j = k; break end end
		if not j then break end
		local d = N[j]
		if d == v then break end
		if onSeg(d) then return out, d end
		-- b sits left of u->v. If the apex is also left, the segment leaves
		-- through (d, c); otherwise through (b, d).
		local ni = (cross2(ux, uy, vx, vy, m.px[d], m.py[d]) > 0) and prv[j] or nxt[j]
		out[#out + 1] = { nb, ni }
		cur, ei = nb, ni
	end
	return out
end

-- Force the edge u-v into the mesh and mark it constrained.
--
-- Sloan's method: flip whatever crosses it until nothing does. The crossing list
-- is rebuilt after each flip rather than patched -- a flip reshapes the strip it
-- came from, and patching that in place is exactly where this kind of loop goes
-- wrong. The list is a handful of edges, so rebuilding it is free.
local function insertSegment(m: Mesh, u: number, v: number, depth: number?): boolean
	if u == v then return false end
	local d = depth or 0
	if d > 64 then return false end
	for _ = 1, 20000 do
		if hasEdge(m, u, v) then break end
		local list, via = crossings(m, u, v)
		if via then
			-- A corner of the boundary lands inside this edge. Constrain the two
			-- halves instead; the union is the same locus and both pieces are
			-- real subsegments, which is what refinement would have made of it
			-- anyway.
			local a = insertSegment(m, u, via, d + 1)
			local b = insertSegment(m, via, v, d + 1)
			return a and b
		end
		if #list == 0 then break end
		local did = false
		for _, ce in ipairs(list) do
			if not m.dead[ce[1]] and flip(m, ce[1], ce[2]) then did = true; break end
		end
		if not did then break end
	end
	if not hasEdge(m, u, v) then return false end
	m.seg[ekey(u, v)] = true
	m.segs[#m.segs + 1] = { u, v }
	return true
end

-- Inside or outside, by PARITY of constrained crossings from the outside.
--
-- Nesting falls out: cross the outer rim and you are on the floor, cross a hole
-- ring as well and you are inside the pillar, which is outside again. The order
-- of the walk does not matter, because the parity of the crossings between two
-- triangles is path-independent when the constraints are closed curves, which
-- rings are.
local function markInside(m: Mesh, superVert: number)
	-- SEEDED FROM A LIVE TRIANGLE, found through the super-triangle's own corner.
	-- The triangle the mesh started as is long dead by now -- the first insertion
	-- kills it -- and a dead seed walks nowhere and marks nothing inside.
	local seed = nil
	for _, t in ipairs(fan(m, superVert)) do seed = t; break end
	if not seed then return end
	local depth = { [seed] = 0 }
	local q = { seed }
	local qi = 1
	while qi <= #q do
		local t = q[qi]; qi += 1
		local T = m.tri[t]
		for i = 1, 3 do
			local nb = T[3 + i]
			if nb and not m.dead[nb] and depth[nb] == nil then
				depth[nb] = depth[t] + (m.seg[ekey(T[nxt[i]], T[prv[i]])] and 1 or 0)
				q[#q + 1] = nb
			end
		end
	end
	for t = 1, m.nt do
		local d = depth[t]
		m.inside[t] = (not m.dead[t]) and d ~= nil and (d % 2 == 1) or false
	end
end

-- ------------------------------------------------------------- refinement

local function circumcentre(m: Mesh, a: number, b: number, c: number): (number?, number?)
	local ax, ay = m.px[a], m.py[a]
	local bx, by = m.px[b] - ax, m.py[b] - ay
	local cx, cy = m.px[c] - ax, m.py[c] - ay
	local d = 2 * (bx * cy - by * cx)
	if math.abs(d) < 1e-12 then return nil end
	local b2 = bx * bx + by * by
	local c2 = cx * cx + cy * cy
	return ax + (cy * b2 - by * c2) / d, ay + (bx * c2 - cx * b2) / d
end

-- Smallest interior angle in degrees, and the longest side.
local function quality(m: Mesh, a: number, b: number, c: number): (number, number)
	local ax, ay = m.px[a], m.py[a]
	local bx, by = m.px[b], m.py[b]
	local cx, cy = m.px[c], m.py[c]
	local ab = math.sqrt((bx - ax) ^ 2 + (by - ay) ^ 2)
	local bc = math.sqrt((cx - bx) ^ 2 + (cy - by) ^ 2)
	local ca = math.sqrt((ax - cx) ^ 2 + (ay - cy) ^ 2)
	local worst = 180
	local function ang(o1: number, o2: number, opp: number)
		if o1 < 1e-12 or o2 < 1e-12 then return end
		local v = (o1 * o1 + o2 * o2 - opp * opp) / (2 * o1 * o2)
		local d = math.deg(math.acos(math.clamp(v, -1, 1)))
		if d < worst then worst = d end
	end
	ang(ab, ca, bc)
	ang(ab, bc, ca)
	ang(bc, ca, ab)
	return worst, math.max(ab, bc, ca)
end

-- A point inside a segment's diametral circle encroaches it.
local function encroaches(m: Mesh, u: number, v: number, x: number, y: number): boolean
	return (m.px[u] - x) * (m.px[v] - x) + (m.py[u] - y) * (m.py[v] - y) < 0
end

-- Only the apexes of the segment's own two triangles can encroach it in a
-- Delaunay mesh, so that is all this looks at.
local function segEncroached(m: Mesh, u: number, v: number): boolean
	for _, t in ipairs(fan(m, u)) do
		local T = m.tri[t]
		if T[1] == v or T[2] == v or T[3] == v then
			for k = 1, 3 do
				local w = T[k]
				if w ~= u and w ~= v and encroaches(m, u, v, m.px[w], m.py[w]) then
					return true
				end
			end
		end
	end
	return false
end

-- Split a subsegment at its midpoint.
--
-- The mark comes off FIRST, so the insertion cavity may cross where the segment
-- used to be; without that the cavity stops at the old edge and the triangles on
-- the far side are never rebuilt. The two halves are marked after.
--
-- The midpoint's 3D position is the midpoint of its parents' 3D positions, not a
-- plane projection. Along a ring edge that is exact, and it stays exact through
-- repeated splits because every child carries one too.
local function splitSeg(m: Mesh, u: number, v: number): (number?, { number }?)
	local ux, uy = m.px[u], m.py[u]
	local vx, vy = m.px[v], m.py[v]
	if (vx - ux) ^ 2 + (vy - uy) ^ 2 < CDT.minSegLen * CDT.minSegLen then return nil end
	local hint = nil
	for _, t in ipairs(fan(m, u)) do
		local T = m.tri[t]
		if T[1] == v or T[2] == v or T[3] == v then hint = t; break end
	end
	local k = ekey(u, v)
	m.seg[k] = nil
	local wu, wv = m.world[u], m.world[v]
	local w = (wu and wv) and ((wu + wv) * 0.5) or nil
	local p, made = insertPoint(m, (ux + vx) * 0.5, (uy + vy) * 0.5, w, hint)
	if not p then
		m.seg[k] = true
		return nil
	end
	m.seg[ekey(u, p)] = true
	m.seg[ekey(p, v)] = true
	m.segs[#m.segs + 1] = { u, p }
	m.segs[#m.segs + 1] = { p, v }
	return p, made
end

-- A triangle whose worst angle sits between two constrained edges cannot be
-- improved -- the angle is the map's, not the mesh's -- and chasing it is how
-- Ruppert grinds forever in a sharp corner.
local function seditious(m: Mesh, T: { number }): boolean
	local worst, wi = 180, 0
	for i = 1, 3 do
		local a, b, c = T[prv[i]], T[i], T[nxt[i]]
		local u1x, u1y = m.px[a] - m.px[b], m.py[a] - m.py[b]
		local u2x, u2y = m.px[c] - m.px[b], m.py[c] - m.py[b]
		local m1 = math.sqrt(u1x * u1x + u1y * u1y)
		local m2 = math.sqrt(u2x * u2x + u2y * u2y)
		if m1 > 1e-12 and m2 > 1e-12 then
			local d = math.deg(math.acos(math.clamp(
				(u1x * u2x + u1y * u2y) / (m1 * m2), -1, 1)))
			if d < worst then worst, wi = d, i end
		end
	end
	if wi == 0 then return false end
	return m.seg[ekey(T[wi], T[prv[wi]])] ~= nil
		and m.seg[ekey(T[wi], T[nxt[wi]])] ~= nil
end

-- Ruppert. Encroached subsegments always go first -- that priority is what makes
-- the algorithm terminate rather than chase a circumcentre out of the domain.
--
-- Both work lists are CURSORS over growing arrays, never rescans. A rescan per
-- operation is quadratic, and there are thousands of operations.
local function refine(m: Mesh, stats: any): boolean
	local iters = 0
	local sp = 1
	local tq, ti = {}, 1
	for t = 1, m.nt do if m.inside[t] and not m.dead[t] then tq[#tq + 1] = t end end

	local function push(made: { number }?)
		if not made then return end
		for _, t in ipairs(made) do
			tq[#tq + 1] = t
			-- Re-queue the constrained edges around the change: a midpoint can
			-- encroach a DIFFERENT segment nearby, and the cursor has already
			-- walked past it.
			local T = m.tri[t]
			for i = 1, 3 do
				local a, b = T[nxt[i]], T[prv[i]]
				if m.seg[ekey(a, b)] then m.segs[#m.segs + 1] = { a, b } end
			end
		end
	end

	while iters < CDT.maxRefine do
		local worked = false

		while sp <= #m.segs and iters < CDT.maxRefine do
			local s = m.segs[sp]; sp += 1
			if m.seg[ekey(s[1], s[2])] and segEncroached(m, s[1], s[2]) then
				iters += 1
				local p, made = splitSeg(m, s[1], s[2])
				if p then stats.segSplits += 1; push(made); worked = true end
			end
		end

		local handled = false
		while ti <= #tq do
			local t = tq[ti]; ti += 1
			if not m.dead[t] and m.inside[t] then
				local T = m.tri[t]
				local minA, maxL = quality(m, T[1], T[2], T[3])
				if maxL > CDT.targetEdge
					or (minA < CDT.minAngle and not seditious(m, T)) then
					iters += 1
					handled = true
					worked = true
					local cx, cy = circumcentre(m, T[1], T[2], T[3])
					if cx and cy then
						local dst, bu, bv = locate(m, t, cx, cy, true)
						if bu and bv then
							local p, made = splitSeg(m, bu, bv)
							if p then
								stats.segSplits += 1
								push(made)
								tq[#tq + 1] = t
							end
						elseif dst and m.inside[dst] then
							local D = m.tri[dst]
							local ha, hb = nil, nil
							for i = 1, 3 do
								local a, b = D[nxt[i]], D[prv[i]]
								if m.seg[ekey(a, b)] and encroaches(m, a, b, cx, cy) then
									ha, hb = a, b
									break
								end
							end
							if ha and hb then
								local p, made = splitSeg(m, ha, hb)
								if p then
									stats.segSplits += 1
									push(made)
									tq[#tq + 1] = t
								end
							else
								local p, made = insertPoint(m, cx, cy, nil, dst)
								if p then stats.steiner += 1; push(made) end
							end
						end
					end
					break
				end
			end
		end

		if not worked then return true end
		if not handled and sp > #m.segs and ti > #tq then return true end
	end
	return false
end

-- ----------------------------------------------------------------- merging

-- Hertel-Mehlhorn, driven by a priority queue instead of a sweep.
--
-- The acceptance test is Triangulate's own `spliceFaces` -- splice along the
-- shared edge, keep it only if every corner of the result is still convex --
-- reused rather than written a second time. What is replaced is only the DRIVER:
-- `Triangulate.mergeConvex` rebuilds the whole edge-ownership map on every
-- single merge, which is nothing at forty triangles and quadratic at four
-- thousand.
--
-- Longest shared edge first is Recast's heuristic, kept: dissolving the longest
-- internal wall leaves the squarest face, and the squarest face has the most
-- ways to merge again.
local function mergeFaces(m: Mesh, faces: { any }, pts2: { any }): (number, { any })
	local splice = Triangulate.spliceFaces
	local owner: { [number]: { number } } = {}
	for i, f in ipairs(faces) do
		for j = 1, #f do
			local k = ekey(f[j], f[j % #f + 1])
			local e = owner[k]
			if not e then e = {}; owner[k] = e end
			e[#e + 1] = i
		end
	end

	local cand = {}
	for k, e in pairs(owner) do
		if #e == 2 and not m.seg[k] then
			local v = k % K
			local u = (k - v) / K
			local dx, dy = m.px[u] - m.px[v], m.py[u] - m.py[v]
			cand[#cand + 1] = { k, u, v, dx * dx + dy * dy }
		end
	end
	table.sort(cand, function(a, b) return a[4] > b[4] end)

	local merges = 0
	for _ = 1, 1000 do
		local did = 0
		for _, c in ipairs(cand) do
			local e = owner[c[1]]
			if e and #e == 2 and e[1] ~= e[2] then
				local i, j = e[1], e[2]
				local A, B = faces[i], faces[j]
				if A and B then
					local u, v = c[2], c[3]
					local mg = splice(A, B, u, v, pts2, CDT.maxVerts)
						or splice(A, B, v, u, pts2, CDT.maxVerts)
						or splice(B, A, u, v, pts2, CDT.maxVerts)
						or splice(B, A, v, u, pts2, CDT.maxVerts)
					-- A REPEATED CORNER MEANS THE TWO FACES SHARED MORE THAN ONE
					-- EDGE. Splicing along one of them leaves the other as a spur
					-- that runs out and straight back, and the convexity test
					-- passes it because a spur turns through zero degrees. The
					-- shape is not simple, so its area is wrong and a funnel
					-- string can be pulled around the outside of it.
					if mg then
						local once = {}
						for _, w in ipairs(mg) do
							if once[w] then mg = nil; break end
							once[w] = true
						end
					end
					if mg then
						faces[i] = mg
						faces[j] = false
						-- everything B owned is i's now
						for q = 1, #B do
							local oe = owner[ekey(B[q], B[q % #B + 1])]
							if oe then
								for z = 1, #oe do if oe[z] == j then oe[z] = i end end
							end
						end
						owner[c[1]] = nil
						merges += 1
						did += 1
					end
				end
			end
		end
		if did == 0 then break end
	end

	local out = {}
	for _, f in ipairs(faces) do if f then out[#out + 1] = f end end
	return merges, out
end

-- --------------------------------------------------------------- decimation

-- The boundary of the fan of faces around `v`, walked as one chain.
--
-- Each face contributes the path from the corner AFTER v round to the corner
-- BEFORE v -- the face minus v -- and consecutive faces of a fan share an
-- endpoint of that path, so the paths chain. An interior vertex chains into a
-- closed cycle that never mentions v; a boundary vertex chains into an open
-- path whose two ends are v's neighbours along the boundary.
--
-- Returns nil when the paths do not chain into exactly one run. That means the
-- faces around v are not a simple fan -- v is a pinch between two lobes -- and
-- there is no single polygon to replace them with.
local function linkOf(faces: { any }, fs: { number }, v: number): ({ number }?, boolean)
	local path, byStart, endOf = {}, {}, {}
	for _, fi in ipairs(fs) do
		local f = faces[fi]
		if not f then return nil, false end
		local n = #f
		if n < 3 then return nil, false end
		local iv = nil
		for i = 1, n do if f[i] == v then iv = i; break end end
		if not iv then return nil, false end
		local pth = table.create(n - 1)
		for k = 1, n - 1 do pth[k] = f[(iv + k - 1) % n + 1] end
		if byStart[pth[1]] then return nil, false end
		byStart[pth[1]] = fi
		endOf[fi] = pth[#pth]
		path[fi] = pth
	end

	local isEnd = {}
	for _, fi in ipairs(fs) do isEnd[endOf[fi]] = true end
	local head = nil
	for _, fi in ipairs(fs) do
		if not isEnd[path[fi][1]] then head = fi; break end
	end
	local closed = head == nil
	if closed then head = fs[1] end

	local order, seen = {}, {}
	local cur = head
	while cur and not seen[cur] do
		seen[cur] = true
		order[#order + 1] = cur
		cur = byStart[endOf[cur]]
	end
	if #order ~= #fs then return nil, false end

	local out = {}
	for oi, fi in ipairs(order) do
		local pth = path[fi]
		for k = (oi == 1) and 1 or 2, #pth do out[#out + 1] = pth[k] end
	end
	if closed and out[#out] == out[1] then out[#out] = nil end
	return out, closed
end

-- Remove every vertex whose fan is a convex polygon without it.
--
-- THIS IS THE PASS THAT TAKES THE SCAFFOLDING BACK OUT. Hertel-Mehlhorn deletes
-- edges and never vertices, so an interior Steiner point is a pin: as the faces
-- around it merge the angle there climbs toward 180 degrees, and the moment it
-- passes, the corner is reflex and every further merge is refused. A bare
-- rectangle with 138 refinement points in it came out as 111 polygons instead
-- of one, which is what this exists to fix.
--
-- WHAT IS ELIGIBLE, AND NOTHING ELSE:
--   * an interior point, touching no constrained edge;
--   * a boundary point whose two constrained edges are EXACTLY collinear, which
--     makes it a subsegment midpoint and makes putting the parent edge back a
--     no-op on the floor's outline.
-- A traced corner is never removed, and neither is a boundary vertex that is
-- only nearly collinear. That is boundary simplification -- it moves where the
-- floor ends -- and the drawn outline was already settled before this ran. Same
-- call as erosion, made the same way.
local function decimate(m: Mesh, faces: { any }): (number, { any })
	local incid: { [number]: { number } } = {}
	for i, f in ipairs(faces) do
		if f then
			for _, w in ipairs(f) do
				local e = incid[w]
				if not e then e = {}; incid[w] = e end
				e[#e + 1] = i
			end
		end
	end

	local removed = 0
	for v, fs in pairs(incid) do
		-- v's neighbours across constrained edges
		local ca, cb, cn = nil, nil, 0
		for _, fi in ipairs(fs) do
			local f = faces[fi]
			if f then
				local n = #f
				for i = 1, n do
					if f[i] == v then
						local a = f[i % n + 1]
						local b = f[(i + n - 2) % n + 1]
						if m.seg[ekey(v, a)] and a ~= ca and a ~= cb then
							cn += 1
							if ca then cb = a else ca = a end
						end
						if m.seg[ekey(v, b)] and b ~= ca and b ~= cb then
							cn += 1
							if ca then cb = b else ca = b end
						end
					end
				end
			end
		end

		local eligible = false
		if cn == 0 then
			eligible = true
		elseif cn == 2 and ca and cb then
			local ax, ay = m.px[ca], m.py[ca]
			local bx, by = m.px[cb], m.py[cb]
			local run = math.sqrt((bx - ax) ^ 2 + (by - ay) ^ 2)
			-- cross2 is twice the area, so the perpendicular offset is it over
			-- the run; the test wants that offset under a fraction of the run.
			local twiceArea = math.abs(cross2(ax, ay, bx, by, m.px[v], m.py[v]))
			eligible = run > 0 and twiceArea <= CDT.collinearExact * run * run
		end

		if eligible then
			local cyc, closed = linkOf(faces, fs, v)
			local c = cyc :: { number }
			local ok = cyc ~= nil and #c >= 3
			if ok then
				if cn == 2 then
					-- an open chain has to run between the two boundary
					-- neighbours, or the edge that closes it is not the one v
					-- was sitting on
					ok = not closed
						and ((c[1] == ca and c[#c] == cb) or (c[1] == cb and c[#c] == ca))
				else
					ok = closed
				end
			end
			if ok then
				local once = {}
				for _, w in ipairs(c) do
					if once[w] then ok = false; break end
					once[w] = true
				end
			end
			if ok then
				-- Convex, counter-clockwise, collinear allowed: the same
				-- predicate `spliceFaces` applies to a merge.
				local n = #c
				for i = 1, n do
					local a, b, d = c[((i - 2) % n) + 1], c[i], c[i % n + 1]
					if cross2(m.px[a], m.py[a], m.px[b], m.py[b],
						m.px[d], m.py[d]) < 0 then
						ok = false
						break
					end
				end
			end
			if ok then
				local keep = fs[1]
				local inFan = {}
				for _, fi in ipairs(fs) do inFan[fi] = true end
				faces[keep] = c
				for k = 2, #fs do faces[fs[k]] = false end
				if cn == 2 and ca and cb then
					m.seg[ekey(v, ca)] = nil
					m.seg[ekey(v, cb)] = nil
					m.seg[ekey(ca, cb)] = true
				end
				incid[v] = nil
				for _, w in ipairs(c) do
					local e = incid[w]
					if e then
						local out2, added = {}, false
						for _, fi in ipairs(e) do
							if inFan[fi] then
								if not added then
									out2[#out2 + 1] = keep
									added = true
								end
							else
								out2[#out2 + 1] = fi
							end
						end
						if not added then out2[#out2 + 1] = keep end
						incid[w] = out2
					end
				end
				removed += 1
			end
		end
	end

	local out = {}
	for _, f in ipairs(faces) do if f then out[#out + 1] = f end end
	return removed, out
end

-- ------------------------------------------------------------------- build

-- Mean offset of a region's CELLS along the region normal, measured from
-- `origin`. Steiner points are new geometry and have to land on the FLOOR, not
-- on a plane through the rim: a ramp's rim corners and its interior sit at
-- different heights, and fitting to the rim alone puts every interior point in
-- the air.
local function planeOffset(data: any, region: number, origin: Vector3, up: Vector3): number
	if not data or not data.grids then return 0 end
	local sum, n = 0, 0
	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			if cell.region == region then
				sum += (cell.pos - origin):Dot(up)
				n += 1
			end
		end
	end
	if n == 0 then return 0 end
	return sum / n
end

-- Mesh every region of a simplified result, refine it, and merge it into convex
-- polygons.
--
-- `data` is the bake, used only for the plane fit above; without it the plane
-- runs through the first traced corner, which is right for a flat region and
-- wrong for a ramp.
function CDT.build(loops: { any }, data: any?): any
	local stats = { regions = 0, done = 0, skipped = 0,
		tris = 0, polys = 0, holes = 0, steiner = 0, segSplits = 0, merges = 0,
		area = 0, straightened = 0, unconstrained = 0, stalled = 0, selfCross = 0,
		removed = 0, rounds = 0, unsettled = 0,
		minAngle = 180, slivers = 0, byN = {}, edgeHist = {} }
	local complaints = {}
	local polys = {}

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
				if outer then outer = false elseif outer == nil then outer = i end
			elseif L.kind == "hole" then
				holes[#holes + 1] = i
			end
		end
		if outer == nil or outer == false then
			stats.skipped += 1
			complaints[#complaints + 1] = (outer == false)
				and ("r%03d: more than one outer rim, not meshed"):format(r)
				or ("r%03d: no outer rim, not meshed"):format(r)
			continue
		end

		local up = loops[outer].regionUp or loops[outer].up
		local e1, e2 = Rings.basis(up)
		local origin = loops[outer].pts[1]

		local m = newMesh()
		local ringIdx = {}
		local loX, loY, hiX, hiY = math.huge, math.huge, -math.huge, -math.huge
		local uniq, seen = {}, {}

		-- Flattened first, inserted second. The super-triangle has to be sized
		-- off the finished extent, and the mesh's own vertex numbering only
		-- exists once points start going in, so the rings are held here as
		-- indices into `uniq` and remapped afterwards.
		local function addRing(L: any)
			local src = (CDT.collinear > 0)
				and Triangulate.straighten(L.pts, CDT.collinear) or L.pts
			stats.straightened += #L.pts - #src
			local ring = {}
			for _, p in ipairs(src) do
				local d = p - origin
				local x, y = d:Dot(e1), d:Dot(e2)
				-- Coincident corners would make a zero-length constraint, and a
				-- zero-length segment has no direction to insert along.
				local key = ("%.5f,%.5f"):format(x, y)
				local k = seen[key]
				if not k then
					uniq[#uniq + 1] = { x, y, p }
					k = #uniq
					seen[key] = k
				end
				ring[#ring + 1] = k
				if x < loX then loX = x end
				if y < loY then loY = y end
				if x > hiX then hiX = x end
				if y > hiY then hiY = y end
			end
			ringIdx[#ringIdx + 1] = ring
			return ring
		end

		addRing(loops[outer])
		for _, i in ipairs(holes) do
			addRing(loops[i])
			stats.holes += 1
		end

		-- A RING THAT CROSSES ITSELF IS A TRACE DEFECT AND IS SAID OUT LOUD.
		-- The parity fill still returns something for one -- it has to, the
		-- crossings make the two lobes read as inside and outside -- and that
		-- something is a mesh that does not match the map. Reported here rather
		-- than repaired, same rule as Rings.
		for ri, ring in ipairs(ringIdx) do
			local n = #ring
			for i = 1, n do
				for j = i + 2, n do
					if not (i == 1 and j == n) then
						local a, b = uniq[ring[i]], uniq[ring[i % n + 1]]
						local c, d = uniq[ring[j]], uniq[ring[j % n + 1]]
						local d1 = cross2(c[1], c[2], d[1], d[2], a[1], a[2])
						local d2 = cross2(c[1], c[2], d[1], d[2], b[1], b[2])
						local d3 = cross2(a[1], a[2], b[1], b[2], c[1], c[2])
						local d4 = cross2(a[1], a[2], b[1], b[2], d[1], d[2])
						if ((d1 > 0) ~= (d2 > 0)) and ((d3 > 0) ~= (d4 > 0)) then
							stats.selfCross += 1
							complaints[#complaints + 1] =
								("r%03d ring%d: edge %d crosses edge %d, the ring is not simple")
									:format(r, ri, i, j)
						end
					end
				end
			end
		end

		-- A super-triangle big enough that no input point is ever near its
		-- circumcircle, sized off the region's own extent.
		local ctrX, ctrY = (loX + hiX) * 0.5, (loY + hiY) * 0.5
		local rad = math.max(hiX - loX, hiY - loY, 1) * 8
		local s1 = addVertex(m, ctrX, ctrY + rad * 2, nil)
		local s2 = addVertex(m, ctrX - rad * 2, ctrY - rad, nil)
		local s3 = addVertex(m, ctrX + rad * 2, ctrY - rad, nil)
		local root = newTri(m, s1, s2, s3)

		-- INSERTION ASSIGNS THE VERTEX NUMBER. `insertPoint` creates the vertex
		-- it inserts, so the rings are remapped onto what it hands back; naming
		-- a vertex before it is in the mesh leaves the rings pointing at
		-- orphans that no triangle holds.
		local hint = root
		local vid = table.create(#uniq)
		for k, u in ipairs(uniq) do
			local p = insertPoint(m, u[1], u[2], u[3], hint)
			vid[k] = p
			if p then hint = m.vt[p] end
		end
		for _, ring in ipairs(ringIdx) do
			for i = #ring, 1, -1 do
				local p = vid[ring[i]]
				if p then ring[i] = p else table.remove(ring, i) end
			end
		end

		-- The ring vertices went in as points; now they become EDGES. Until this
		-- runs the mesh knows nothing about which side of a wall is floor.
		local failed = 0
		for _, ring in ipairs(ringIdx) do
			local n = #ring
			for i = 1, n do
				if not insertSegment(m, ring[i], ring[i % n + 1]) then failed += 1 end
			end
		end
		if failed > 0 then
			stats.unconstrained += failed
			complaints[#complaints + 1] =
				("r%03d: %d boundary edges could not be forced into the mesh"):format(r, failed)
		end

		markInside(m, s1)
		if CDT.refine and not refine(m, stats) then
			stats.stalled += 1
			complaints[#complaints + 1] =
				("r%03d: refinement hit the %d operation cap, bounds not met")
					:format(r, CDT.maxRefine)
		end

		local faces = {}
		for t = 1, m.nt do
			if m.inside[t] and not m.dead[t] then
				local T = m.tri[t]
				faces[#faces + 1] = { T[1], T[2], T[3] }
				local a, maxL = quality(m, T[1], T[2], T[3])
				if a < stats.minAngle then stats.minAngle = a end
				if a < 15 then stats.slivers += 1 end
				local b = math.min(8, math.max(1, math.floor(maxL / 0.5) + 1))
				stats.edgeHist[b] = (stats.edgeHist[b] or 0) + 1
			end
		end
		stats.tris += #faces

		local pts2 = table.create(m.nv)
		for i = 1, m.nv do pts2[i] = { x = m.px[i], y = m.py[i] } end
		-- MERGE AND DECIMATE UNTIL NEITHER MOVES. One merge can make a vertex's
		-- link convex and one removal can make two faces mergeable, so running
		-- either to exhaustion on its own leaves work on the table.
		local merged = faces
		local settled = false
		for round = 1, CDT.maxRounds do
			local mg, f2 = mergeFaces(m, merged, pts2)
			local rm, f3 = decimate(m, f2)
			stats.merges += mg
			stats.removed += rm
			merged = f3
			if mg == 0 and rm == 0 then
				stats.rounds = math.max(stats.rounds, round)
				settled = true
				break
			end
		end
		if not settled then
			stats.rounds = math.max(stats.rounds, CDT.maxRounds)
			stats.unsettled += 1
			complaints[#complaints + 1] =
				("r%03d: merge and decimation hit the %d round cap, still moving")
					:format(r, CDT.maxRounds)
		end

		-- Lift. Traced corners keep the exact Vector3 they were traced at, and
		-- segment midpoints keep the interpolation of theirs; only interior
		-- Steiner points are reconstructed, onto the region's own floor plane.
		local h = planeOffset(data, r, origin, up)
		local base = origin + up * h
		local cache = {}
		local function W(i: number): Vector3
			local w = m.world[i]
			if w then return w end
			local c = cache[i]
			if not c then
				c = base + e1 * m.px[i] + e2 * m.py[i]
				cache[i] = c
			end
			return c
		end

		local made = 0
		for _, f in ipairs(merged) do
			local n = #f
			local verts = table.create(n)
			for i = 1, n do verts[i] = W(f[i]) end

			-- Area and centroid by fanning from the first corner. For a CONVEX
			-- face the fan stays inside, so the area-weighted centroid is the
			-- real one, and the centroid is the search node.
			local area, acc = 0, Vector3.zero
			for i = 2, n - 1 do
				local A, B, C = verts[1], verts[i], verts[i + 1]
				local a = 0.5 * (B - A):Cross(C - A).Magnitude
				area += a
				acc += (A + B + C) / 3 * a
			end
			if area >= Triangulate.minArea then
				local worst = 180
				for i = 1, n do
					local a = verts[((i - 2) % n) + 1]
					local b = verts[i]
					local c = verts[(i % n) + 1]
					local u1, u2 = a - b, c - b
					local m1, m2 = u1.Magnitude, u2.Magnitude
					if m1 > 1e-9 and m2 > 1e-9 then
						local deg = math.deg(math.acos(
							math.clamp(u1:Dot(u2) / (m1 * m2), -1, 1)))
						if deg < worst then worst = deg end
					end
				end
				stats.byN[n] = (stats.byN[n] or 0) + 1
				polys[#polys + 1] = { verts = verts, n = n, region = r, up = up,
					area = area, centre = acc / area, minAngle = worst,
					-- kept so anything still expecting a triangle keeps working
					a = verts[1], b = verts[2], c = verts[3] }
				stats.area += area
				made += 1
			end
		end
		stats.polys += made
		if made > 0 then stats.done += 1 end
	end

	return { tris = polys, stats = stats, complaints = complaints }
end

function CDT.report(res: any, loops: { any }?): string
	local s = res.stats
	local lines = {
		("cdt       %d regions, %d meshed, %d skipped, %d tris -> %d polys, %d holes")
			:format(s.regions, s.done, s.skipped, s.tris, s.polys, s.holes),
		("  %d steiner, %d segment splits, %d merges, %d vertices decimated")
			:format(s.steiner, s.segSplits, s.merges, s.removed),
		("  %d corners straightened, settled in %d rounds")
			:format(s.straightened, s.rounds),
	}
	if loops then
		local want = 0
		for _, L in ipairs(loops) do
			if L.kind == "outer" or L.kind == "hole" then want += L.area or 0 end
		end
		lines[#lines + 1] = ("  area %.1f of %.1f sq studs (%.2f%%)")
			:format(s.area, want, want > 0 and (s.area / want * 100) or 0)
	end
	lines[#lines + 1] = CDT.refine
		and ("  smallest triangle angle %.1f deg, %d under 15 deg (bound %d)")
			:format(s.minAngle, s.slivers, CDT.minAngle)
		or ("  smallest triangle angle %.1f deg before merging, %d under 15 deg (not refined)")
			:format(s.minAngle, s.slivers)
	local hist = {}
	for b = 1, 8 do
		if s.edgeHist[b] then
			hist[#hist + 1] = (b < 8)
				and ("<%.1f:%d"):format(b * 0.5, s.edgeHist[b])
				or (">=4.0:%d"):format(s.edgeHist[b])
		end
	end
	if #hist > 0 and CDT.refine then
		lines[#lines + 1] = ("  longest edge, target %.1f   "):format(CDT.targetEdge)
			.. table.concat(hist, "  ")
	end
	local shape = {}
	for n = 3, 64 do
		if s.byN[n] then shape[#shape + 1] = ("%d-gon:%d"):format(n, s.byN[n]) end
	end
	if #shape > 0 then lines[#lines + 1] = "  " .. table.concat(shape, "  ") end
	if s.unconstrained > 0 then
		lines[#lines + 1] = ("  ! %d boundary edges missing from the mesh")
			:format(s.unconstrained)
	end
	for _, c in ipairs(res.complaints) do lines[#lines + 1] = "  ! " .. c end
	return table.concat(lines, "\n")
end

-- Exposed for the verification harness, which has to get inside the mesh to
-- assert what the report only summarises: that every polygon is convex, that
-- every internal edge is shared by exactly two, and that no constraint went
-- missing. Not part of the pipeline's own path.
CDT.internal = {
	newMesh = newMesh, addVertex = addVertex, newTri = newTri,
	locate = locate, insertPoint = insertPoint, insertSegment = insertSegment,
	fan = fan, hasEdge = hasEdge, flip = flip, crossings = crossings,
	markInside = markInside, refine = refine, mergeFaces = mergeFaces,
	decimate = decimate, linkOf = linkOf,
	quality = quality, ekey = ekey, cross2 = cross2,
}

return CDT
