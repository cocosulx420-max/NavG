--!strict
-- NVGN.Thickness -- how much ground is under your feet, in studs.
--
-- `maxD` in the design note. For every walkable cell, the distance to the
-- nearest REAL edge of the floor. The offset reads it to decide how far it can
-- afford to push a wall inward: a wide floor can give up an agent radius
-- without noticing, and a narrow ledge cannot give up anything at all.
--
-- TRUE EUCLIDEAN, by the two-pass lower-envelope transform. Stepping outward
-- through 4 neighbours measures a diamond and through 8 measures a rounded
-- square, and both are wrong in the same place: the diagonals, which is exactly
-- where the rotated walls are. This computes the real distance instead, in
-- linear time, with no kernel and no error to quote.
--
-- WHAT COUNTS AS AN EDGE IS WALL AND DROP, NEVER A REGION SEAM. A seam is a
-- bookkeeping line where one plane bucket ends and the next begins; the floor
-- runs straight through it. Seeding it would report a wide floor as thin all
-- along a seam and make the offset carve a trench down the middle of open
-- ground. Walls and dropoffs are the two places the floor actually stops.
--
-- Measured in each grid's OWN lattice, which is the point of the local grids:
-- a rotated slab's edges lie on whole lattice lines of its own frame, so the
-- transform runs on a square lattice and the answer needs no correction.

local Thickness = {}

-- The lattice distance is measured to the centre of the first cell OUTSIDE the
-- floor, and the floor actually ends halfway between that cell and the last one
-- inside. So half a step comes off every reading. Exact where the boundary runs
-- along a lattice line, which after LocalGrid turns the grid is most of them.
Thickness.halfStep = true

-- Stand-in for infinity. Not math.huge: the envelope arithmetic divides
-- differences of these, and inf minus inf is a nan that silently eats a whole
-- row. Large enough to lose to any real distance on any grid we build.
local FAR = 1e12

local DIR4 = { { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 } }
local DIR8 = {
	{ 1, 0 }, { 1, 1 }, { 0, 1 }, { -1, 1 },
	{ -1, 0 }, { -1, -1 }, { 0, -1 }, { 1, -1 },
}

-- One dimension of the squared-distance transform: the lower envelope of the
-- parabolas f[q] + (x - q)^2. `v` holds the parabolas still on the envelope and
-- `z` the crossings between them, so each is pushed and popped at most once.
local function envelope(f: {number}, n: number, d: {number},
	v: {number}, z: {number})
	local k = 0
	v[0] = 0
	z[0] = -FAR
	z[1] = FAR
	for q = 1, n - 1 do
		local s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k])
		while k > 0 and s <= z[k] do
			k -= 1
			s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k])
		end
		k += 1
		v[k] = q
		z[k] = s
		z[k + 1] = FAR
	end
	k = 0
	for q = 0, n - 1 do
		while z[k + 1] < q do k += 1 end
		local dq = q - v[k]
		d[q] = dq * dq + f[v[k]]
	end
end

-- Ground thickness for every live cell, written to `cell.thick` in studs.
--
-- A cell whose grid has no wall or drop anywhere gets `math.huge`: the floor
-- genuinely does not end inside this grid, and inventing a number there would
-- be a guess the offset would then act on. Counted and reported instead.
-- `o.seedWallOnly` seeds from WALL faces alone and `o.field` names where the
-- answer is written. Distance-to-wall is a different question from ground
-- thickness: an erosion must not pull back from a ledge or a region seam, so it
-- needs a field that never saw them. Same transform, different seeds.
function Thickness.build(data: any, cfg: any?, o: any?): any
	local opt = o or {}
	local field = opt.field or "thick"
	local wallOnly = opt.seedWallOnly == true
	-- Fall back to the BAKE's own config for anything the caller's config does
	-- not carry. Pipeline hands its own resolved options down, and those hold
	-- overrides and the root but not `step`, which LocalGrid resolves.
	local c = cfg or data.config
	local dc = data.config or {}
	local step = c.step or dc.step
	local conn = c.connectivity or dc.connectivity
	local dirs = (conn == 8) and DIR8 or DIR4
	local nd = #dirs

	local stats = { grids = 0, cells = 0, seeds = 0, unbounded = 0,
		min = math.huge, max = 0, sum = 0 }

	for _, g in ipairs(data.grids) do
		local cells = {}
		for _, cell in ipairs(g.cells) do
			if cell.region then cells[#cells + 1] = cell end
		end
		if #cells == 0 then continue end
		stats.grids += 1

		-- lattice bounds, one ring wider so a seed just outside the floor fits
		local ulo, uhi, vlo, vhi = math.huge, -math.huge, math.huge, -math.huge
		for _, cell in ipairs(cells) do
			if cell.ui < ulo then ulo = cell.ui end
			if cell.ui > uhi then uhi = cell.ui end
			if cell.vi < vlo then vlo = cell.vi end
			if cell.vi > vhi then vhi = cell.vi end
		end
		ulo -= 1; uhi += 1; vlo -= 1; vhi += 1
		local W, H = uhi - ulo + 1, vhi - vlo + 1

		-- Seed grid. FAR everywhere, 0 at the lattice slot immediately across a
		-- wall or drop face -- that slot is outside the floor, and the floor's
		-- true edge is the half step between it and the cell that named it.
		local f = table.create(W * H, FAR)
		local seeded = 0
		for _, cell in ipairs(cells) do
			-- Wall-only seeding drops the STEP directions too. A stair riser is
			-- solid and so reads as a wall, but it is one you climb, and eroding
			-- back from it cuts every tread off from the next.
			local mask = wallOnly
				and bit32.band(cell.wallMask or 0, bit32.bnot(cell.stepMask or 0))
				or bit32.bor(cell.wallMask or 0, cell.dropMask or 0)
			if mask ~= 0 then
				for bit = 1, nd do
					if bit32.band(mask, bit32.lshift(1, bit - 1)) ~= 0 then
						local d = dirs[bit]
						local x = cell.ui + d[1] - ulo
						local y = cell.vi + d[2] - vlo
						if x >= 0 and x < W and y >= 0 and y < H then
							local i = y * W + x + 1
							if f[i] ~= 0 then f[i] = 0; seeded += 1 end
						end
					end
				end
			end
		end
		stats.seeds += seeded

		if seeded == 0 then
			-- no wall and no drop in this whole grid: the floor does not end here
			for _, cell in ipairs(cells) do
				cell[field] = math.huge
				stats.unbounded += 1
				stats.cells += 1
			end
			continue
		end

		-- columns, then rows. The second pass runs over the first pass's output,
		-- which is what makes the composed result the true 2D transform rather
		-- than two independent 1D ones.
		local col = table.create(H)
		local outc = table.create(H)
		local v = table.create(math.max(W, H) + 1)
		local z = table.create(math.max(W, H) + 2)
		for x = 0, W - 1 do
			for y = 0, H - 1 do col[y] = f[y * W + x + 1] end
			envelope(col, H, outc, v, z)
			for y = 0, H - 1 do f[y * W + x + 1] = outc[y] end
		end
		local row = table.create(W)
		local outr = table.create(W)
		for y = 0, H - 1 do
			local base = y * W
			for x = 0, W - 1 do row[x] = f[base + x + 1] end
			envelope(row, W, outr, v, z)
			for x = 0, W - 1 do f[base + x + 1] = outr[x] end
		end

		local half = Thickness.halfStep and (step * 0.5) or 0
		for _, cell in ipairs(cells) do
			local x = cell.ui - ulo
			local y = cell.vi - vlo
			local d2 = f[y * W + x + 1]
			local t = math.max(math.sqrt(d2) * step - half, 0)
			cell[field] = t
			stats.cells += 1
			stats.sum += t
			if t < stats.min then stats.min = t end
			if t > stats.max then stats.max = t end
		end
	end

	stats.mean = stats.cells > 0 and (stats.sum / stats.cells) or 0
	return stats
end

-- A histogram, because a mean says nothing about whether the thin places exist.
-- What the offset cares about is the low end: how much floor is narrow enough
-- that an agent radius cannot be taken off it.
function Thickness.histogram(data: any, edges: {number}?): any
	local e = edges or { 0.5, 1, 1.5, 2, 3, 4, 6, 8 }
	local bins = table.create(#e + 1, 0)
	local n, unbounded = 0, 0
	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			if cell.region and cell.thick then
				n += 1
				if cell.thick == math.huge then
					unbounded += 1
				else
					local b = #e + 1
					for i, lim in ipairs(e) do
						if cell.thick < lim then b = i; break end
					end
					bins[b] += 1
				end
			end
		end
	end
	return { edges = e, bins = bins, cells = n, unbounded = unbounded }
end

function Thickness.report(stats: any, hist: any?): string
	local lines = {
		("thick     %d cells over %d grids, %d seeds, min %.2f mean %.2f max %.2f")
			:format(stats.cells, stats.grids, stats.seeds,
				stats.min == math.huge and 0 or stats.min, stats.mean, stats.max),
	}
	if stats.unbounded > 0 then
		lines[#lines + 1] = ("  %d cells in grids with no wall or drop at all -- unbounded")
			:format(stats.unbounded)
	end
	if hist then
		local parts = {}
		local prev = 0
		for i, lim in ipairs(hist.edges) do
			parts[#parts + 1] = ("%.1f-%.1f:%d"):format(prev, lim, hist.bins[i])
			prev = lim
		end
		parts[#parts + 1] = (">%.1f:%d"):format(prev, hist.bins[#hist.bins])
		lines[#lines + 1] = "  " .. table.concat(parts, "  ")
	end
	return table.concat(lines, "\n")
end

return Thickness
