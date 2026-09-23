--!strict
-- NVGN.Solid -- exact "does this box touch that part" tests, shared.
--
-- Moved out of LocalGrid unchanged, so the global grid can use the very same
-- narrow phase the local grid's kill test was tuned with.

local Solid = {}

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

Solid.boxOverlap = boxOverlap
Solid.isBlock = isBlock
Solid.supportHalf = supportHalf
Solid.wedgeOverlap = wedgeOverlap
Solid.isWedge = isWedge
Solid.boxPoints = boxPoints
Solid.wedgePoints = wedgePoints
Solid.hullsApart = hullsApart

return Solid
