--!strict
-- NVGN.Cull -- drop the regions that are not worth pathing through.
--
-- A bake finds every surface an agent could physically stand on, which is not
-- the same set as the surfaces worth putting in a graph. case3 comes out of
-- LocalGrid with 82 regions and roughly half of them are window ledges, the top
-- of a stringer, a crouch space under a handrail, or a two-cell scrap beside a
-- step. An NPC never needs to path INTO any of them: it can stand on the floor
-- beside one and already be in reach of anything standing there.
--
-- TWO TESTS, AND THE SECOND ONE IS THE WHOLE DESIGN.
--
-- REACH. For every cell of a region, how far is the nearest cell of some OTHER
-- region. Take the worst of those. If even the most buried cell of a region is
-- within arm's length of somewhere else, then standing outside is as good as
-- standing inside, and the region buys nothing but another node in the graph.
-- Cocosulx's framing, and it is a better question than any measure of size or
-- shape: it asks what the region is FOR.
--
-- SEVERANCE, CHECKED AFTER EVERY SINGLE DROP. The obvious companion test is
-- "and it must not be a bridge", computed up front as articulation points of
-- the region graph. That was measured and it is wrong twice over. It protects
-- regions that are only bridges because a plane seam gave the region graph an
-- edge that is not a real one, and it fails to protect a staircase, because no
-- SINGLE tread is a cut vertex even though the flight falls apart without them.
--
-- So do not predict it. Drop one region, run the connectivity snapshot, and put
-- it straight back if a component split. Smallest first, so the junk gets the
-- first chance to go and whatever is holding the map together protects itself.
-- On case3 that holds back 15 regions: 14 stair treads and the 2 regions
-- Cocosulx flagged by hand as the ones most likely to be lost by mistake.
--
-- Measured on case3 against 38 regions Cocosulx marked as not worth keeping and
-- 2 marked as must-keep: all 38 dropped, both keeps held, 8 further regions
-- dropped (six of them 5 cells or less), no stairs lost, 5.0% of the floor.
--
-- NOT DESTRUCTIVE, the same way Erode is not: the region id moves to
-- `regionWas` and `cell.culled` is set, so a bake can be drawn either way.

local Cull = {}

local Severance = require(script.Parent:WaitForChild("Severance"))

-- How far outside help can reach, in studs. A region every part of which is
-- within this of another region is redundant.
--
-- 1.75 rather than 1.25, which was the first value tried. The three biggest
-- regions on Cocosulx's list sit between 1.12 and 1.58, and the stair treads
-- bottom out at 1.41. THAT IS THE UNCOMFORTABLE PART: at 1.75 the threshold is
-- above the treads rather than below them, and they survive only because the
-- severance check catches them afterwards. It holds on case3 and the margin is
-- not wide, so re-measure this on a new map rather than assuming.
Cull.reach = 1.75

-- A region smaller than this is dropped whatever its reach. Cheap, and it
-- catches the scraps that are so small they have no interior to measure.
Cull.minCells = 15

-- Distance to the nearest cell of a DIFFERENT region, worst case over a region.
--
-- Straight world-space distance, not lattice distance: the outside help may be
-- standing on another part at another angle, which is exactly the case a ledge
-- beside a wall presents.
function Cull.reachOf(data: any): { [number]: number }
	local G = math.max(Cull.reach, 1)
	local all = {}
	for _, g in ipairs(data.grids) do
		for _, c in ipairs(g.cells) do
			if c.region then all[#all + 1] = c end
		end
	end
	local hash: { [string]: { any } } = {}
	local function key(p: Vector3): string
		return ("%d,%d,%d"):format(
			math.floor(p.X / G), math.floor(p.Y / G), math.floor(p.Z / G))
	end
	for _, c in ipairs(all) do
		local k = key(c.pos)
		local t = hash[k]
		if not t then t = {}; hash[k] = t end
		t[#t + 1] = c
	end

	local out: { [number]: number } = {}
	for _, a in ipairs(all) do
		local p = a.pos
		local bx, by, bz = math.floor(p.X / G), math.floor(p.Y / G), math.floor(p.Z / G)
		local best = math.huge
		for ox = -1, 1 do
			for oy = -1, 1 do
				for oz = -1, 1 do
					local t = hash[("%d,%d,%d"):format(bx + ox, by + oy, bz + oz)]
					if t then
						for _, b in ipairs(t) do
							if b.region ~= a.region then
								local d = (b.pos - p).Magnitude
								if d < best then best = d end
							end
						end
					end
				end
			end
		end
		-- an isolated region has no other region anywhere near it, so it is
		-- unreachable from outside and must be kept
		if best > (out[a.region] or -1) then out[a.region] = best end
	end
	return out
end

-- Drop what is redundant, keep what the graph needs.
function Cull.apply(data: any, cfg: any?): any
	local c = cfg or {}
	local reachMax = c.cullReach or Cull.reach
	local minCells = c.cullMinCells or Cull.minCells

	local byRegion: { [number]: { any } } = {}
	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			local r = cell.region or cell.regionWas
			if r then
				-- restore first, so a second call with different settings is
				-- not culling an already culled mask
				cell.region = r
				cell.regionWas = r
				cell.culled = false
				local t = byRegion[r]
				if not t then t = {}; byRegion[r] = t end
				t[#t + 1] = cell
			end
		end
	end

	local reach = Cull.reachOf(data)
	local cand = {}
	for r, cells in pairs(byRegion) do
		if reach[r] <= reachMax or #cells < minCells then
			cand[#cand + 1] = r
		end
	end
	-- SMALLEST FIRST. The order decides the answer, because each drop is judged
	-- against the graph the previous drops left behind. Giving the least
	-- valuable region the first chance to go is what lets a chain of scraps
	-- unravel while a chain of treads does not.
	table.sort(cand, function(a, b) return #byRegion[a] < #byRegion[b] end)

	local stats = { regions = 0, candidates = #cand, dropped = 0, held = 0,
		cells = 0, cellsDropped = 0, reach = reachMax, minCells = minCells,
		heldIds = {}, droppedIds = {} }
	for r, cells in pairs(byRegion) do
		stats.regions += 1
		stats.cells += #cells
	end

	local base = Severance.snapshot(data)
	for _, r in ipairs(cand) do
		local cells = byRegion[r]
		for _, cell in ipairs(cells) do cell.region = nil end
		local after = Severance.snapshot(data)
		if #Severance.compare(base, after).severed > 0 then
			for _, cell in ipairs(cells) do cell.region = r end
			stats.held += 1
			stats.heldIds[#stats.heldIds + 1] = r
		else
			for _, cell in ipairs(cells) do cell.culled = true end
			stats.dropped += 1
			stats.cellsDropped += #cells
			stats.droppedIds[#stats.droppedIds + 1] = r
			base = after
		end
	end
	table.sort(stats.heldIds)
	table.sort(stats.droppedIds)
	return stats
end

-- Put every culled region back.
function Cull.restore(data: any)
	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			if cell.regionWas then
				cell.region = cell.regionWas
				cell.culled = false
			end
		end
	end
end

function Cull.report(stats: any): string
	local lines = {
		("cull      reach %.2f, min %d cells -- %d of %d regions dropped, %d held")
			:format(stats.reach, stats.minCells, stats.dropped, stats.regions,
				stats.held),
		("  %d of %d cells removed (%.1f%%)"):format(stats.cellsDropped,
			stats.cells, stats.cells > 0 and (stats.cellsDropped / stats.cells * 100) or 0),
	}
	if stats.held > 0 then
		lines[#lines + 1] = "  held by severance: " .. table.concat(stats.heldIds, ", ")
	end
	return table.concat(lines, "\n")
end

return Cull
