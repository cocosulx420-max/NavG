--!strict
-- NVGN.FaceKind -- why the floor stops here, decided per RAW BOUNDARY FACE.
--
-- A portal exists where the floor's edge is an opening rather than a wall, so
-- something has to say which. `LocalGrid.classifyNodes` already answers it and
-- answers it wrong often enough to be unusable, for two structural reasons.
--
-- DROP WAS THE ELSE OF WALL. `classifyNodes` ends on
--   `if above then wallMask ... else dropMask ... end`
-- and there is no test anywhere for "can you leave the floor here". Every
-- failure of the wall probe -- a shape it cannot test, an API that returned
-- nothing -- silently became *yes you can*, which is fail-open on the one
-- verdict that has to be fail-safe. `below`, real positive evidence of a ledge,
-- is computed there and then overwritten before it reaches the output.
--
-- AND THE PROBE COULD NOT SEE MESHES. Exact tests covered Part and WedgePart;
-- everything else fell through to GetPartsInPart, which this project already
-- measured as missing ~46% of solids. Between 76 and 235 of case3's 748 drop
-- faces had a solid sitting in the probe box, and nothing could say which.
--
-- WHY SVOLocal AND NOT THE GLOBAL SVO. The global tree is wrong in both
-- directions at once: 57% of its leaves are cells the geometry only grazes
-- (touch-marking at a 1 stud leaf), and 1995 leaf cells are missing on thin
-- slabs because its precise path prunes an entire subtree on a single
-- GetPartsInPart false negative -- one miss at depth 3 deletes an 8x8 stud
-- patch of wall. SVOLocal is a tree per PART built in THAT PART'S OWN FRAME, so
-- a rotated wall is voxelized tightly instead of smeared across world-aligned
-- cells, and only 49 of case3's 432 parts need the unreliable precise path
-- against 146 in the global tree.
--
-- A FACE IS THE RIGHT GRANULARITY AND THE POLYGON EDGE IS NOT. A face is one
-- cell edge: it sits exactly where the floor ends and it has exactly one
-- verdict. The polygon boundary is the SIMPLIFIED boundary, measured up to
-- 0.500 studs off these faces, and at a doorjamb half a stud is the difference
-- between the opening and the wall beside it. Labelling the simplified edges is
-- what produced 74 "mixed" verdicts that told a portal builder nothing.

local FaceKind = {}

local SVOLocal = require(script.Parent:WaitForChild("SVOLocal"))
local LocalGrid = require(script.Parent:WaitForChild("LocalGrid"))

-- How far past the floor's edge to look, in studs.
--
-- ONE CELL, AND THIS IS THE KNOB THAT MATTERED. `classifyNodes` used 1.0 to
-- "catch a set-back pillar", which reaches from 0.25 to 1.25 studs past the cell
-- centre -- two cells out. A wall standing that far away is real, and detecting
-- it is still wrong: it is not what blocks you stepping off this edge. Measured
-- on case3's known-open seam faces, false walls fell 18.0% -> 7.9% going from
-- 1.0 to 0.5, while leaf size and probe height between them moved it barely at
-- all.
--
-- It does not go lower. At 0.4 and 0.3 the seam figure flattens (7.7%) while
-- walls start being LOST -- 18 real walls became dropoffs at 0.5, 81 at 0.4, 195
-- at 0.3 -- because the box no longer spans the cell it is asking about.
FaceKind.probeDepth = 0.5

FaceKind.probeHeight = 1.7

-- Leaf size for the per-part trees.
--
-- Half the default, matching LocalGrid's cell step. 1.0 costs 12k nodes and
-- 0.55s and reads 18.0% of known-open seams as wall; 0.5 costs 48k and 1.4s for
-- 7.9%; 0.25 costs 170k and 5.2s for 6.9%, which is not worth four times the
-- bake.
FaceKind.leaf = 0.5

-- How far above the floor the probe starts. Box top stays at 2.2.
--
-- IT HAS TO CLEAR ONE LEAF, because SVOLocal marks any leaf the geometry
-- touches, so a floor slab reads up to a full leaf HIGHER than its real surface.
-- At 0.2 the floor on the OTHER SIDE of a gap pokes into the bottom of the box
-- and reads as a wall standing in the gap. The evidence was bimodal and gave it
-- away: distance from the probe centre to the nearest solid clumped at 529
-- flush (0-0.25) and 208 near (0.25-0.50), then only 29 in 0.50-0.75 and a
-- second spike of 182 at 0.75-1.00 -- which is the box's FLOOR, not its side.
--
-- Measured on case3, top held at 2.2:
--
--   bottom 0.20 | wall 948 | walls with nothing within 0.6: 201 | fall-0 drops 182
--   bottom 0.50 | wall 755 | walls with nothing within 0.6:  12 | fall-0 drops 222
--   bottom 1.00 | wall 687 | walls with nothing within 0.6:   0 | fall-0 drops 272
--
-- The seam check sits at 41 of 522 for every row, so this costs nothing there.
-- 1.0 buys the last twelve and starts turning real walls into dropoffs instead.
--
-- The price is not seeing an obstacle shorter than half a stud, which is honest
-- rather than a loss: at this leaf that detection was never real, and anything
-- under STEP_UP is a step you walk up rather than a wall you stop at.
FaceKind.probeBottom = 0.5

-- Cap the probe at the headroom the cell actually has.
--
-- A CEILING IS NOT A WALL. Regions here are split by headroom class as well as
-- by plane, so a seam very often sits exactly where a low ceiling begins -- and
-- a 2 stud probe standing on the floor reaches straight into that overhang and
-- calls it a wall. Measured on case3: with no cap, 180 of 522 KNOWN-OPEN seam
-- faces came back `wall`, and RAISING the probe made it worse (202), because
-- raising the floor of the box raises its roof too.
--
-- Whatever is above `cell.clearance` is the thing that defined the clearance.
-- It is already accounted for -- it is why this cell is in the region it is in
-- -- and asking about it again here only turns a doorway into a wall.
FaceKind.clampToClearance = true

-- Margin under the ceiling, so the probe does not graze the surface that set
-- the clearance in the first place.
FaceKind.clearanceMargin = 0.3

-- A probe shorter than this is not asking a meaningful question, so the face is
-- left to the drop test rather than answered from a sliver.
FaceKind.minProbeHeight = 0.5

-- How far down to look for something to land on before calling it a void.
FaceKind.maxFall = 24

-- Resolution of the fallback voxel scan. Only reached when no live cell was
-- found below, where the question is merely "is there ANY solid down there".
FaceKind.fallStep = 0.5

-- Under this, the floor did not drop at all -- it continued at this level and
-- simply is not navigable. Set above the lattice noise between two grids with
-- different origins and far under STEP_UP.
FaceKind.flatFall = 0.25

-- How far sideways a landing cell may sit from the slot straight off the edge.
-- Grids are built per FACE, each with its own lattice origin, so the cell below
-- is rarely exactly aligned with the one above; 0.75 of a step tolerates that
-- without reaching into the next slot along.
FaceKind.landingSpread = 0.75

-- How far out to measure `near`. Beyond this the answer is "nothing close" and
-- the exact number stops meaning anything.
FaceKind.nearRange = 2.5

-- Broadphase bucket size. Parts are filed by world AABB so a face tests a
-- handful of trees rather than all 432.
FaceKind.bucket = 4

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
			e[#e + 1] = { part, tree }
		end end end
	end
	return self
end

-- Candidate trees overlapping a world AABB, minus `skip`.
function Index:gather(lo: Vector3, hi: Vector3, skip: Instance?, out: {any}, seen: {any})
	table.clear(out)
	table.clear(seen)
	local c = self.cell
	for x = math.floor(lo.X/c), math.floor(hi.X/c) do
	for y = math.floor(lo.Y/c), math.floor(hi.Y/c) do
	for z = math.floor(lo.Z/c), math.floor(hi.Z/c) do
		for _, e in ipairs(self.map[x .. "," .. y .. "," .. z] or {}) do
			local part = e[1]
			if part ~= skip and not seen[part] then
				seen[part] = true
				out[#out + 1] = e[2]
			end
		end
	end end end
	return out
end

-- ------------------------------------------------------------------ probe

-- A box `size` sitting at `mid` with its LookVector along `dir`.
--
-- The lateral axis falls out of `CFrame.lookAt`, which uses world up. That
-- matches `classifyNodes`, whose constants these are, so the two stay
-- comparable; on a steeply tilted floor it means the probe stands upright
-- rather than square to the surface, which is the existing behaviour and not a
-- change this pass is making.
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

local function aabbOf(cf: CFrame, size: Vector3): (Vector3, Vector3)
	local h = worldAABBHalf(cf, size)
	return cf.Position - h, cf.Position + h
end

-- ---------------------------------------------------------------- landing

-- Every live cell, bucketed by world X/Z at one step.
--
-- THE LANDING THAT MATTERS IS WALKABLE FLOOR, AND WE KNOW EXACTLY WHERE IT IS.
-- Measuring the fall by scanning voxels downward could not tell a landing 0.4
-- studs below from one at 0.0 -- the slabs are half a stud and the
-- representation over-claims by a leaf -- so 182 faces reported a drop of
-- exactly zero, which is not a drop at all. Cells carry an exact position, so
-- the answer is a lookup and not a measurement.
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

-- The highest live cell at or below `fromY`, in the slot `at`. Returns the cell
-- and the exact drop, or nil.
local function landingAt(map: any, step: number, at: Vector3, fromY: number, tol: number)
	local spread = step * FaceKind.landingSpread
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

-- ------------------------------------------------------------------ build

-- Label every face of every traced region.
--
-- `boundary` is `data.boundary`, the table Boundary.faces produced. Faces are
-- rewritten in place: `kind` is replaced and `fall` / `near` are added.
--
-- SEAMS ARE NOT RE-DECIDED. A face is `edge` when live floor of ANOTHER region
-- sits directly across it, which comes from cell adjacency and never consults a
-- probe. It is the one verdict that was already trustworthy, and it is also the
-- ground truth this pass is checked against: a seam is known open, so the wall
-- test must never fire on one.
function FaceKind.build(data: any, trees: any): any
	local step = data.config.step
	local tol = data.config.flushTol or 0.3
	local STEP_UP = LocalGrid.STEP_UP
	local idx = buildIndex(trees, FaceKind.bucket)
	local cells = buildCellIndex(data)

	-- which part each cell's floor came from, so the downward scan can ignore
	-- the slab the agent is standing on. Its voxels over-claim past its own
	-- edge, and without this every drop would report a landing at zero.
	local partOf = {}
	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do partOf[cell] = g.part end
	end

	local stats = { faces = 0, wall = 0, drop = 0, edge = 0, none = 0, step = 0, ledge = 0,
		void = 0, changed = 0, wasDropNowWall = 0, wasWallNowDrop = 0,
		seamProbed = 0, seamFalseWall = 0, queries = 0, tooLow = 0 }
	local out, seen = {}, {}

	for _, entry in pairs(data.boundary) do
		for _, f in ipairs(entry.faces) do
			stats.faces += 1
			local was = f.kind
			local cell = f.cell
			local ctr = (f.a + f.b) * 0.5
			local d = ctr - cell.pos

			if d.Magnitude < 1e-6 then
				f.kind = "none"
				f.fall = nil
				f.near = nil
				stats.none += 1
				continue
			end
			local dir = d.Unit

			-- the wall probe, whatever the face already says. A seam gets
			-- probed too, because that is the check that says whether the
			-- voxels are over-claiming enough to close a real doorway.
			local top = FaceKind.probeBottom + FaceKind.probeHeight
			if FaceKind.clampToClearance and cell.clearance then
				top = math.min(top, cell.clearance - FaceKind.clearanceMargin)
			end
			local h = top - FaceKind.probeBottom
			local solid = false
			if h >= FaceKind.minProbeHeight then
				local mid = cell.pos + dir * (step * 0.5 + FaceKind.probeDepth * 0.5)
					+ Vector3.yAxis * (FaceKind.probeBottom + h * 0.5)
				local cf = probeAt(mid, dir)
				local size = Vector3.new(step * 0.9, h, FaceKind.probeDepth)
				local lo, hi = aabbOf(cf, size)
				local cands = idx:gather(lo, hi, partOf[cell], out, seen)
				stats.queries += #cands
				solid = anySolid(cands, cf, size)
			else
				stats.tooLow += 1
			end

			if was == "edge" then
				stats.seamProbed += 1
				if solid then stats.seamFalseWall += 1 end
				stats.edge += 1
				f.fall = nil
				f.near = nil
				continue
			end

			if solid then
				f.kind = "wall"
				f.fall = nil
				f.near = 0
				stats.wall += 1
				if was == "drop" then stats.wasDropNowWall += 1 end
			else
				-- NOTHING BESIDE YOU IS HALF THE ANSWER. The other half is what
				-- is underneath and how far down, and those are three different
				-- things to a pathfinder, not one.
				local at = cell.pos + dir * step
				local land, fall = landingAt(cells, step, at, cell.pos.Y, tol)
				local edgePt = cell.pos + dir * (step * 0.5 + FaceKind.probeDepth * 0.5)

				if land and fall < FaceKind.flatFall then
					-- The floor did not drop. It continued at this level and is
					-- simply not navigable -- clearance or pruning ended the
					-- region here, not geometry. Not a wall, not a fall, and
					-- nowhere to build a portal to.
					f.kind = "none"
					f.fall = fall
					f.landing = land
					stats.none += 1
				elseif land and fall <= STEP_UP then
					-- You walk down it AND back up, so this is a TWO-WAY link.
					f.kind = "step"
					f.fall = fall
					f.landing = land
					stats.step += 1
				elseif land then
					f.kind = "drop"
					f.fall = fall
					f.landing = land
					stats.drop += 1
					if was == "wall" then stats.wasWallNowDrop += 1 end
				else
					-- No walkable floor below. Voxels can still say whether
					-- there is ANY solid down there, which separates a ledge
					-- over rubble from a ledge over nothing. It does not price
					-- a link, because nothing down there can be walked on.
					local colSize = Vector3.new(step * 0.9, FaceKind.fallStep, FaceKind.probeDepth)
					local depth = nil
					local y = 0
					while y < FaceKind.maxFall do
						local c = edgePt - Vector3.yAxis * (y + FaceKind.fallStep * 0.5)
						local ccf = probeAt(c, dir)
						local clo, chi = aabbOf(ccf, colSize)
						local cc = idx:gather(clo, chi, partOf[cell], out, seen)
						stats.queries += #cc
						if anySolid(cc, ccf, colSize) then depth = y; break end
						y += FaceKind.fallStep
					end
					-- LEDGE, NOT VOID: on case3, 216 of these 273 faces have
					-- solid underneath and only 57 have nothing at all. Both
					-- are places you can leave the floor and not land anywhere
					-- you could stand, which is what the name has to say.
					-- `fall` is the depth to that solid, or infinite for a true
					-- void.
					f.kind = "ledge"
					f.fall = depth or math.huge
					f.landing = nil
					stats.ledge += 1
					if not depth then stats.void += 1 end
					if was == "wall" then stats.wasWallNowDrop += 1 end
				end

				-- how close the nearest solid actually is, so a portal builder
				-- can be careful without this pass inventing a category.
				local best = FaceKind.nearRange
				local ncands = idx:gather(
					ctr - Vector3.new(best, best, best),
					ctr + Vector3.new(best, best, best), partOf[cell], out, seen)
				for _, t in ipairs(ncands) do
					local dd = t:nearestSolid(
						ctr + Vector3.yAxis * (FaceKind.probeBottom + FaceKind.probeHeight * 0.5), best)
					if dd and dd < best then best = dd end
				end
				f.near = (best < FaceKind.nearRange) and best or nil
			end

			if f.kind ~= was then stats.changed += 1 end
		end
	end

	return stats
end

function FaceKind.report(stats: any): string
	local lines = {
		("kind      %d faces: %d wall, %d step, %d drop, %d ledge, %d edge, %d none  (%d changed)")
			:format(stats.faces, stats.wall, stats.step, stats.drop, stats.ledge,
				stats.edge, stats.none, stats.changed),
		("  %d were drop and are wall, %d were wall and are not")
			:format(stats.wasDropNowWall, stats.wasWallNowDrop),
		("  step = two-way, walk down and back up (<= %.1f).  drop = one-way, priced by height.")
			:format(LocalGrid.STEP_UP),
		("  none = floor continues at this level and is not navigable.")
		, ("  ledge = nowhere walkable below (%d of them over nothing at all).")
			:format(stats.void)
	}
	-- THE CHECK THAT MATTERS. A seam has live floor of another region across it,
	-- decided by cell adjacency with no probe involved, so it is known open. A
	-- wall verdict on one is the voxels over-claiming a real doorway shut.
	if stats.seamProbed > 0 then
		local pct = stats.seamFalseWall / stats.seamProbed * 100
		lines[#lines + 1] = ("  seam check: %d of %d known-open faces read as WALL (%.1f%%)%s")
			:format(stats.seamFalseWall, stats.seamProbed, pct,
				(stats.seamFalseWall > 0) and "  <-- over-claim closing real openings" or "  -- clean")
	end
	return table.concat(lines, "\n")
end

return FaceKind
