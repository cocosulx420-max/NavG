--!strict
-- NVGN.EdgeKind -- what the far side of every polygon edge is, baked into the edge.
--
-- A portal exists where a polygon's boundary is an opening rather than a wall,
-- so something has to say which, for every stretch of every boundary edge.
--
-- WHY THE POLYGON EDGE AND NOT THE RAW FACE. FaceKind answered the same
-- question per raw cell-edge and then everything downstream had to be handed
-- back through a provenance chain to find which polygon edge it belonged to.
-- The objection to labelling the polygon directly was that the simplified
-- boundary drifts off the real floor edge -- but measured over all 1727 raw
-- nodes, 93% sit within an eighth of a stud of it and only 17 in the whole map
-- are beyond half a stud. The drift is real and rare, and the indirection cost
-- more than it bought.
--
-- WHAT THIS DOES NOT DECIDE. Walk and step connectivity comes from Severance,
-- which already tests every cell pair against a gate built for exactly that and
-- now keeps the pairs. This module is only asked about the boundary: wall, or
-- somewhere you could leave the floor, and how far down.
--
-- SPLIT, NEVER AVERAGE. An edge whose far side changes halfway along is cut
-- there. Taking whichever verdict owns the most of it would put a ledge link
-- along the part that is masonry, and about a quarter of edges are mixed.
-- Splitting is free: a collinear corner turns through zero degrees, which
-- `spliceFaces` already permits, so convexity, area and the boundary locus are
-- all unchanged. A boundary edge has no polygon opposite it, so no T-junction
-- can appear either.

local EdgeKind = {}

local Rings = require(script.Parent:WaitForChild("Rings"))
local LocalGrid = require(script.Parent:WaitForChild("LocalGrid"))

-- The probe box. THESE THREE NUMBERS WERE MEASURED, NOT CHOSEN, and the
-- measurements are worth keeping because both of the obvious values are wrong.
--
-- 0.5 DEEP. The old wall test used 1.0, reaching from 0.25 to 1.25 studs past
-- the cell centre -- two cells out. A wall standing that far away is real and
-- detecting it is still wrong, because it is not what blocks you leaving this
-- edge. False walls on known-open seams fell 18.0% -> 7.9% at 0.5. It does not
-- go lower: at 0.4 and 0.3 that figure flattens while real walls start being
-- lost, 18 then 81 then 195 of them.
--
-- 0.5 ABOVE THE FLOOR, top at 2.2. SVOLocal marks any leaf the geometry
-- touches, so a floor slab reads up to a full leaf higher than its real
-- surface, and at 0.2 the floor ACROSS a gap poked into the bottom of the box
-- and read as a wall. The evidence was bimodal and gave it away: 529 flush, 208
-- near, only 29 in between, then a second spike of 182 at 0.75-1.00 -- which is
-- the box's floor, not its side. Walls with nothing solid within 0.6 studs:
-- 201 -> 12.
EdgeKind.probeDepth = 0.5
EdgeKind.probeBottom = 0.5
EdgeKind.probeHeight = 1.7

-- How often to ask along an edge. Half a cell: fine enough to catch a doorway
-- in a long wall, coarse enough that a 20 stud edge is 80 questions.
EdgeKind.sampleStep = 0.25

-- Leaf size for the per-part trees. 1.0 costs 12k nodes and reads 18.0% of
-- known-open seams as wall; 0.5 costs 48k for 7.9%; 0.25 costs 170k and 5.2s
-- for 6.9%, which is not worth four times the bake.
EdgeKind.leaf = 0.5

-- Under this the floor did not drop at all -- it continued at this level and
-- simply is not navigable. Above the lattice noise between two grids with
-- different origins, far under STEP_UP.
EdgeKind.flatFall = 0.25

-- How far down to look before calling it a void, and the slab resolution of
-- that fallback scan. Only reached when no live cell was found below, where the
-- question is merely "is there ANY solid down there".
EdgeKind.maxFall = 24
EdgeKind.fallStep = 0.5

-- How far sideways a landing cell may sit from the slot straight off the edge.
-- Grids are built per FACE, each with its own lattice origin, so the cell below
-- is rarely exactly aligned with the one above.
EdgeKind.landingSpread = 0.75

-- Broadphase bucket for the part trees.
EdgeKind.bucket = 4

local function worldAABBHalf(cf: CFrame, size: Vector3): Vector3
	local e = size * 0.5
	local r, u, l = cf.RightVector, cf.UpVector, cf.LookVector
	return Vector3.new(
		math.abs(r.X)*e.X + math.abs(u.X)*e.Y + math.abs(l.X)*e.Z,
		math.abs(r.Y)*e.X + math.abs(u.Y)*e.Y + math.abs(l.Y)*e.Z,
		math.abs(r.Z)*e.X + math.abs(u.Z)*e.Y + math.abs(l.Z)*e.Z
	)
end

-- ------------------------------------------------------------- broadphase

local Index = {}
Index.__index = Index

local function buildIndex(trees: any, cell: number)
	local self = setmetatable({ cell = cell, map = {} }, Index)
	for part, tree in pairs(trees) do
		local h = worldAABBHalf(part.CFrame, part.Size)
		local lo, hi = part.Position - h, part.Position + h
		for x = math.floor(lo.X/cell), math.floor(hi.X/cell) do
		for y = math.floor(lo.Y/cell), math.floor(hi.Y/cell) do
		for z = math.floor(lo.Z/cell), math.floor(hi.Z/cell) do
			local k = x .. "," .. y .. "," .. z
			local e = self.map[k]
			if not e then e = {}; self.map[k] = e end
			e[#e + 1] = tree
		end end end
	end
	return self
end

function Index:gather(lo: Vector3, hi: Vector3, out: {any}, seen: {any})
	table.clear(out)
	table.clear(seen)
	local c = self.cell
	for x = math.floor(lo.X/c), math.floor(hi.X/c) do
	for y = math.floor(lo.Y/c), math.floor(hi.Y/c) do
	for z = math.floor(lo.Z/c), math.floor(hi.Z/c) do
		for _, t in ipairs(self.map[x .. "," .. y .. "," .. z] or {}) do
			if not seen[t] then seen[t] = true; out[#out + 1] = t end
		end
	end end end
	return out
end

-- ---------------------------------------------------------------- landing

-- Every live cell, bucketed by world X/Z at one step.
--
-- THE LANDING THAT MATTERS IS WALKABLE FLOOR, AND WE KNOW EXACTLY WHERE IT IS.
-- Measuring the fall by scanning voxels downward could not tell a landing 0.4
-- studs below from one at 0.0 -- the slabs are half a stud and the
-- representation over-claims by a leaf -- so 182 faces once reported a drop of
-- exactly zero, which is not a drop at all.
local function buildCellIndex(data: any)
	local step = data.config.step
	local map = {}
	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			local p = cell.pos
			local k = math.floor(p.X/step) .. ":" .. math.floor(p.Z/step)
			local e = map[k]
			if not e then e = {}; map[k] = e end
			e[#e + 1] = cell
		end
	end
	return map
end

local function landingAt(map: any, step: number, at: Vector3, fromY: number, tol: number)
	local spread = step * EdgeKind.landingSpread
	local r2 = spread * spread
	local best, bestY = nil, -math.huge
	local bx, bz = math.floor(at.X/step), math.floor(at.Z/step)
	for ox = -1, 1 do
		for oz = -1, 1 do
			for _, cell in ipairs(map[(bx + ox) .. ":" .. (bz + oz)] or {}) do
				local p = cell.pos
				local dx, dz = p.X - at.X, p.Z - at.Z
				if dx*dx + dz*dz <= r2 and p.Y <= fromY + tol and p.Y > bestY then
					best, bestY = cell, p.Y
				end
			end
		end
	end
	if not best then return nil end
	return best, fromY - bestY
end

-- ------------------------------------------------------------------ probe

local function probeAt(mid: Vector3, dir: Vector3): CFrame
	local up = (math.abs(dir.Y) > 0.99) and Vector3.xAxis or Vector3.yAxis
	return CFrame.lookAt(mid, mid + dir, up)
end

local function anySolid(cands: {any}, cf: CFrame, size: Vector3): boolean
	for _, t in ipairs(cands) do
		if t:overlapsBox(cf, size) then return true end
	end
	return false
end

-- ------------------------------------------------------------------ build

-- Crossing-number test in the polygon's own plane, for the outward check below.
local function contains(P: {any}, qx: number, qy: number): boolean
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

-- Label and split every boundary edge of every polygon, in place.
--
-- `mesh` is the CDT result. Polygons gain vertices at the split points and a
-- parallel `edgeKind` / `edgeFall` / `edgeLanding` per edge.
function EdgeKind.build(mesh: any, data: any, trees: any): any
	local step = data.config.step
	local tol = data.config.flushTol or 0.3
	local STEP_UP = LocalGrid.STEP_UP
	local idx = buildIndex(trees, EdgeKind.bucket)
	local cells = buildCellIndex(data)

	local stats = { polys = 0, edges = 0, samples = 0, split = 0, added = 0,
		internal = 0, wall = 0, step = 0, drop = 0, ledge = 0, none = 0,
		void = 0, flipped = 0 }
	local out, seen = {}, {}

	-- an edge shared by two polygons is interior floor and needs no probing
	local shared = {}
	for _, f in ipairs(mesh.tris) do
		for j = 1, f.n do
			local a, b = f.verts[j], f.verts[j % f.n + 1]
			local ka = ("%.3f,%.3f,%.3f"):format(a.X, a.Y, a.Z)
			local kb = ("%.3f,%.3f,%.3f"):format(b.X, b.Y, b.Z)
			local k = (ka < kb) and (ka .. "|" .. kb) or (kb .. "|" .. ka)
			shared[k] = (shared[k] or 0) + 1
		end
	end

	for _, f in ipairs(mesh.tris) do
		stats.polys += 1
		local up = f.up
		local e1, e2 = Rings.basis(up)
		local origin = f.verts[1]
		local P = {}
		for i = 1, f.n do
			local d = f.verts[i] - origin
			P[i] = { d:Dot(e1), d:Dot(e2) }
		end

		-- WHICH WAY IS OUT, ESTABLISHED RATHER THAN ASSUMED. Deriving it from
		-- the winding works right up until one region comes back wound the other
		-- way, and then its whole rim is labelled against the inside of itself
		-- with nothing to show for it. One containment test settles it.
		local sign = 1
		do
			local a, b = P[1], P[2]
			local dx, dy = b[1] - a[1], b[2] - a[2]
			local m = math.sqrt(dx*dx + dy*dy)
			if m > 1e-9 then
				local mx, my = (a[1]+b[1])*0.5, (a[2]+b[2])*0.5
				-- right of travel, in the (e1,e2) plane
				local nx, ny = dy/m, -dx/m
				if contains(P, mx + nx*0.01, my + ny*0.01) then
					sign = -1
					stats.flipped += 1
				end
			end
		end

		local newVerts, kinds, falls, lands = {}, {}, {}, {}
		for j = 1, f.n do
			local A, B = f.verts[j], f.verts[j % f.n + 1]
			local a2, b2 = P[j], P[j % f.n + 1]
			local dx, dy = b2[1] - a2[1], b2[2] - a2[2]
			local len = math.sqrt(dx*dx + dy*dy)
			newVerts[#newVerts + 1] = A
			stats.edges += 1

			local ka = ("%.3f,%.3f,%.3f"):format(A.X, A.Y, A.Z)
			local kb = ("%.3f,%.3f,%.3f"):format(B.X, B.Y, B.Z)
			local key = (ka < kb) and (ka .. "|" .. kb) or (kb .. "|" .. ka)
			if (shared[key] or 0) > 1 or len < 1e-6 then
				-- INDEXED, NOT APPENDED. `t[#t + 1] = nil` is a no-op in Lua, so
				-- appending a nil fall leaves that array one short and every
				-- later entry lines up against the wrong edge. `#kinds` is the
				-- authority; the other two are written at the same index.
				local ki = #kinds + 1
				kinds[ki] = "internal"
				falls[ki] = nil
				lands[ki] = nil
				stats.internal += 1
				continue
			end

			-- outward, in world space
			local outX, outY = (dy/len) * sign, (-dx/len) * sign
			local dir = (e1 * outX + e2 * outY)
			if dir.Magnitude < 1e-9 then dir = Vector3.xAxis else dir = dir.Unit end

			local nSamp = math.max(1, math.ceil(len / EdgeKind.sampleStep))
			local size = Vector3.new(step * 0.9, EdgeKind.probeHeight, EdgeKind.probeDepth)
			local segK, segF, segL, segT = {}, {}, {}, {}

			for s = 0, nSamp - 1 do
				local t = (s + 0.5) / nSamp
				local p = A:Lerp(B, t)
				stats.samples += 1

				-- PUSHED OUT PAST THE FLOOR'S OWN EDGE. The polygon boundary
				-- runs through the CENTRES of the outermost cells -- `polyline`
				-- insets every raw node half a step into the region -- so a box
				-- hung straight off it covers the cell you are standing on
				-- rather than the one beyond. Half a step out puts its near
				-- face where the floor actually ends, which is what the tuned
				-- depth was measured against.
				local mid = p + dir * (step * 0.5 + EdgeKind.probeDepth * 0.5)
					+ Vector3.yAxis * (EdgeKind.probeBottom + EdgeKind.probeHeight * 0.5)
				local cf = probeAt(mid, dir)
				local h = worldAABBHalf(cf, size)
				local cands = idx:gather(cf.Position - h, cf.Position + h, out, seen)

				local k, fall, land
				if anySolid(cands, cf, size) then
					k = "wall"
				else
					local at = p + dir * step
					local L, drop = landingAt(cells, step, at, p.Y, tol)
					if L and drop < EdgeKind.flatFall then
						k, fall, land = "none", drop, L
					elseif L and drop <= STEP_UP then
						k, fall, land = "step", drop, L
					elseif L then
						k, fall, land = "drop", drop, L
					else
						-- nowhere walkable below. Voxels can still say whether
						-- there is ANY solid down there, which separates a ledge
						-- over rubble from one over nothing. It does not price a
						-- link, because nothing there can be stood on.
						local colSize = Vector3.new(step * 0.9, EdgeKind.fallStep, EdgeKind.probeDepth)
						local depth = nil
						local y = 0
						while y < EdgeKind.maxFall do
							-- one full step out, the same slot the landing
							-- lookup uses, so the column clears the slab the
							-- polygon is standing on instead of reporting it
							local c = p + dir * step
								- Vector3.yAxis * (y + EdgeKind.fallStep * 0.5)
							local ccf = probeAt(c, dir)
							local ch = worldAABBHalf(ccf, colSize)
							local cc = idx:gather(ccf.Position - ch, ccf.Position + ch, out, seen)
							if anySolid(cc, ccf, colSize) then depth = y; break end
							y += EdgeKind.fallStep
						end
						k, fall = "ledge", depth or math.huge
						if not depth then stats.void += 1 end
					end
				end

				-- run-length: only a change starts a new stretch.
				-- Indexed off `segK`, for the same reason as above: `fall` is
				-- nil on a wall, and appending nil is a no-op that would shift
				-- every later height onto the wrong stretch.
				if #segK > 0 and segK[#segK] == k then
					segT[#segT] = t
				else
					local si = #segK + 1
					segK[si] = k
					segF[si] = fall
					segL[si] = land
					segT[si] = t
				end
			end

			for si = 1, #segK do
				local ki = #kinds + 1
				kinds[ki] = segK[si]
				falls[ki] = segF[si]
				lands[ki] = segL[si]
				local c = segK[si]
				if c == "wall" then stats.wall += 1
				elseif c == "step" then stats.step += 1
				elseif c == "drop" then stats.drop += 1
				elseif c == "ledge" then stats.ledge += 1
				else stats.none += 1 end
				-- a cut point, except after the last stretch: the edge already
				-- ends at B and the next iteration emits it
				if si < #segK then
					newVerts[#newVerts + 1] = A:Lerp(B, segT[si])
					stats.added += 1
				end
			end
			if #segK > 1 then stats.split += 1 end
		end

		f.verts = newVerts
		f.n = #newVerts
		f.edgeKind = kinds
		f.edgeFall = falls
		f.edgeLanding = lands
		f.a, f.b, f.c = newVerts[1], newVerts[2], newVerts[3]
	end

	return stats
end

function EdgeKind.report(stats: any): string
	return table.concat({
		("edges     %d polys, %d edges -> %d labelled stretches (%d edges split, %d corners added)")
			:format(stats.polys, stats.edges,
				stats.internal + stats.wall + stats.step + stats.drop + stats.ledge + stats.none,
				stats.split, stats.added),
		("  %d internal, %d wall, %d step, %d drop, %d ledge, %d none  (%d over nothing at all)")
			:format(stats.internal, stats.wall, stats.step, stats.drop,
				stats.ledge, stats.none, stats.void),
		("  %d samples, %d polygons wound the other way"):format(stats.samples, stats.flipped),
	}, "\n")
end

return EdgeKind
