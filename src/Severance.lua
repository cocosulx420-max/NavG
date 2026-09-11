--!strict
-- NVGN.Severance -- did anything get cut off, and was it cut off by us.
--
-- The offset moves every wall boundary inward. Most of the time that just
-- trims a rim. Sometimes it closes a corridor, and a closed corridor is
-- invisible in every other statistic the pipeline produces: the corner count
-- barely moves, the deviation does not change, and the drawing still looks like
-- a floor. It shows up later as an agent that refuses to walk somewhere, which
-- is the most expensive way to find out.
--
-- So: union-find over cell adjacency, once before the offset and once after,
-- and report what changed. REPORT, NEVER REPAIR. A severed layer is a tuning
-- failure and has to be visible as one; quietly reconnecting it would hide the
-- exact thing this module exists to surface.
--
-- THIS READS CELLS, NOT POLYGONS. The rings are a description of the floor;
-- the cells are the floor. An offset that cuts a passage does so in the cells
-- whether or not the rings happen to show it.
--
-- ANNIHILATION IS COUNTED SEPARATELY FROM SEVERANCE, because `pieces > 1` is
-- false for zero exactly as it is for one. A component the offset erased
-- entirely would otherwise pass the severance test silently.

local Severance = {}

-- THE ADJACENCY GATE -- when two cells are close enough that you could have
-- walked straight from one to the other. In-plane separation is the reach, and
-- the along-normal term is a step up or down.
--
-- These are the numbers purge uses to make a staircase read as ONE component
-- instead of one component per tread, and RegionLink repeats them for the same
-- reason. RegionLink is not required here because it takes the old pipeline's
-- `walk` array and `regionOf` map rather than LocalGrid's grids, so there is no
-- shared function to call -- but the two are the same physical constant, and
-- moving one without the other is a bug.
Severance.stepPlane = 0.75
Severance.stepNormal = 0.6

-- Bucket edge for the spatial hash. Must be at least the gate reach, or a
-- neighbour could sit outside the 3x3x3 block that gets scanned.
local function bucketSize(): number
	return math.max(Severance.stepPlane, Severance.stepNormal)
end

local function key(x: number, y: number, z: number): number
	return x * 73856093 + y * 19349663 + z * 83492791
end

-- Cells that count as floor: in a region, and kept by the caller's filter.
--
-- `keep` is how the offset gets measured. Before the offset it is nil and
-- every live cell counts; after it, it answers whether a cell still lies inside
-- the offset polygons. The module never computes that itself -- it takes the
-- verdict and reports the consequence.
local function gather(data: any, keep: ((any) -> boolean)?): {any}
	local out = {}
	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			if cell.region and (not keep or keep(cell)) then
				out[#out + 1] = cell
			end
		end
	end
	return out
end

-- Connected components of the walkable cells, by union-find.
--
-- Components are numbered in FIRST-SEEN ORDER over the cell array, which
-- LocalGrid already fixes, so the same bake numbers them the same way twice.
-- Numbering them by size instead would shuffle ids on every tie, and case5 has
-- plenty of ties.
function Severance.snapshot(data: any, keep: ((any) -> boolean)?): any
	local cells = gather(data, keep)
	local n = #cells

	local parent = table.create(n)
	local rank = table.create(n)
	for i = 1, n do
		parent[i] = i
		rank[i] = 0
	end

	local function find(i: number): number
		local r = i
		while parent[r] ~= r do r = parent[r] end
		-- path compression, so a long staircase does not cost a walk per query
		while parent[i] ~= r do
			local nxt = parent[i]
			parent[i] = r
			i = nxt
		end
		return r
	end

	local function union(a: number, b: number)
		local ra, rb = find(a), find(b)
		if ra == rb then return end
		if rank[ra] < rank[rb] then ra, rb = rb, ra end
		parent[rb] = ra
		if rank[ra] == rank[rb] then rank[ra] += 1 end
	end

	local G = bucketSize()
	local hash: { [number]: {number} } = {}
	for i = 1, n do
		local p = cells[i].pos
		local k = key(math.floor(p.X / G), math.floor(p.Y / G), math.floor(p.Z / G))
		local b = hash[k]
		if not b then b = {}; hash[k] = b end
		b[#b + 1] = i
	end

	local plane2 = Severance.stepPlane * Severance.stepPlane
	local normTol = Severance.stepNormal
	local tested = 0

	for i = 1, n do
		local a = cells[i]
		local p = a.pos
		local up = a.normal or Vector3.yAxis
		local bx, by, bz = math.floor(p.X / G), math.floor(p.Y / G), math.floor(p.Z / G)
		for ox = -1, 1 do
			for oy = -1, 1 do
				for oz = -1, 1 do
					local b = hash[key(bx + ox, by + oy, bz + oz)]
					if b then
						for _, j in ipairs(b) do
							-- j > i only: every pair is then tested once, from
							-- the lower index, and the gate is symmetric enough
							-- that testing it from the other side adds nothing.
							if j > i then
								local dv = cells[j].pos - p
								local dn = dv:Dot(up)
								local flat = dv - up * dn
								if flat:Dot(flat) <= plane2 and math.abs(dn) <= normTol then
									tested += 1
									union(i, j)
								end
							end
						end
					end
				end
			end
		end
	end

	-- label in first-seen order
	local label: { [number]: number } = {}
	local comp = table.create(n)
	local sizes = {}
	for i = 1, n do
		local r = find(i)
		local id = label[r]
		if not id then
			id = #sizes + 1
			label[r] = id
			sizes[id] = 0
		end
		comp[i] = id
		sizes[id] += 1
	end

	-- cell -> component, for the comparison. Keyed by the cell itself so the
	-- two snapshots line up without depending on index order, which a filtered
	-- snapshot does not preserve.
	local of: { [any]: number } = {}
	for i = 1, n do of[cells[i]] = comp[i] end

	return {
		cells = cells, comp = comp, of = of, sizes = sizes,
		pieces = #sizes, cellCount = n, links = tested,
	}
end

-- What the offset did to connectivity.
--
-- Correspondence between the two snapshots is by CELL, never by component id.
-- Ids are assigned independently in each pass and an offset that deletes the
-- first cell of a component renumbers everything after it, so matching on id
-- would report a catastrophe on every run.
--
-- A before-component is SEVERED when its surviving cells end up in more than
-- one after-component, and ANNIHILATED when none of its cells survive at all.
function Severance.compare(before: any, after: any): any
	local severed, annihilated, shrunk = {}, {}, {}
	local orphans = 0

	-- for each before-component: which after-components its cells landed in
	local landed = {}   -- [beforeId] = { [afterId] = count }
	local survivors = {}
	local total = {}
	for i = 1, before.cellCount do
		local b = before.comp[i]
		total[b] = (total[b] or 0) + 1
		local a = after.of[before.cells[i]]
		if a then
			survivors[b] = (survivors[b] or 0) + 1
			local t = landed[b]
			if not t then t = {}; landed[b] = t end
			t[a] = (t[a] or 0) + 1
		end
	end

	-- Any after-cell that was not a before-cell. Cannot happen when the offset
	-- only removes cells, which is the only thing it is allowed to do, so this
	-- counts a violation of that rule rather than a normal outcome.
	for i = 1, after.cellCount do
		if before.of[after.cells[i]] == nil then orphans += 1 end
	end

	for b = 1, before.pieces do
		local surv = survivors[b] or 0
		local size = total[b] or 0
		if surv == 0 then
			annihilated[#annihilated + 1] = { id = b, cells = size }
		else
			local t = landed[b]
			local parts = 0
			for _ in pairs(t) do parts += 1 end
			if parts > 1 then
				-- sizes of the fragments, largest first, so the report says
				-- whether this split off a room or a single cell
				local frags = {}
				for _, cnt in pairs(t) do frags[#frags + 1] = cnt end
				table.sort(frags, function(x, y) return x > y end)
				severed[#severed + 1] = { id = b, cells = size, parts = parts, frags = frags }
			end
			if surv < size then
				shrunk[#shrunk + 1] = { id = b, cells = size, lost = size - surv }
			end
		end
	end

	return {
		beforePieces = before.pieces, afterPieces = after.pieces,
		beforeCells = before.cellCount, afterCells = after.cellCount,
		lost = before.cellCount - after.cellCount,
		severed = severed, annihilated = annihilated, shrunk = shrunk,
		orphans = orphans,
		clean = #severed == 0 and #annihilated == 0 and orphans == 0,
	}
end

-- The one-line verdict, then every failure named. A severance is listed
-- individually however many there are: the whole point is that it is visible,
-- and a count alone does not say which passage went.
function Severance.report(cmp: any): string
	local lines = {
		("sever     %d pieces -> %d, %d cells -> %d (%d lost)")
			:format(cmp.beforePieces, cmp.afterPieces,
				cmp.beforeCells, cmp.afterCells, cmp.lost),
	}
	if cmp.clean then
		lines[#lines + 1] = "  nothing severed, nothing annihilated"
	end
	for _, a in ipairs(cmp.annihilated) do
		lines[#lines + 1] = ("  !! component %d ANNIHILATED, %d cells gone"):format(a.id, a.cells)
	end
	for _, s in ipairs(cmp.severed) do
		local f = {}
		for i = 1, math.min(#s.frags, 4) do f[#f + 1] = tostring(s.frags[i]) end
		if #s.frags > 4 then f[#f + 1] = "..." end
		lines[#lines + 1] = ("  !! component %d SEVERED into %d, %d cells -> %s")
			:format(s.id, s.parts, s.cells, table.concat(f, " + "))
	end
	if cmp.orphans > 0 then
		lines[#lines + 1] = ("  !! %d cells present after but not before -- the offset added floor")
			:format(cmp.orphans)
	end
	return table.concat(lines, "\n")
end

-- SELF-TEST: a snapshot compared against itself must report no change.
--
-- Written and run BEFORE the offset exists, because a checker built alongside
-- the thing it checks tends to agree with it by construction. If this cannot
-- pass on unchanged data it cannot be trusted on changed data.
function Severance.selfTest(data: any): (boolean, string)
	local a = Severance.snapshot(data)
	local b = Severance.snapshot(data)
	local fails = {}
	if a.pieces ~= b.pieces then
		fails[#fails + 1] = ("pieces %d vs %d -- not deterministic"):format(a.pieces, b.pieces)
	end
	if a.cellCount ~= b.cellCount then
		fails[#fails + 1] = ("cells %d vs %d"):format(a.cellCount, b.cellCount)
	end
	for i = 1, math.min(a.cellCount, b.cellCount) do
		if a.comp[i] ~= b.comp[i] then
			fails[#fails + 1] = ("cell %d labelled %d then %d"):format(i, a.comp[i], b.comp[i])
			break
		end
	end
	local cmp = Severance.compare(a, b)
	if not cmp.clean or cmp.lost ~= 0 then
		fails[#fails + 1] = "compare reported a change against identical data"
	end
	if #fails > 0 then
		return false, "selfTest FAILED:\n  " .. table.concat(fails, "\n  ")
	end
	return true, ("selfTest passed: %d cells, %d pieces, stable across two runs")
		:format(a.cellCount, a.pieces)
end

return Severance
