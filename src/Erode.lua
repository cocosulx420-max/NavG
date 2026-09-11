--!strict
-- NVGN.Erode -- take the agent's width off the CELLS, before anything is traced.
--
-- The offset used to happen after simplification: trace the floor's edge, fit
-- lines to it, then push the wall lines inward and re-intersect the corners.
-- Every hard part of that existed only because of the ordering.
--
--   * the face kind had to survive simplification, so 13634 raw nodes voted on
--     530 edges and some edges came out part masonry and part ledge
--   * moving lines needed corner re-intersection, a miter limit, a near-parallel
--     fallback, and a rule for corners whose edges disagreed
--   * measuring the cost needed a point-in-polygon test, and the polygon already
--     excludes 11% of the cells it was traced from, so the severance baseline
--     had to be run through the same test to cancel that out
--   * and the coin-toss on cells lying exactly on a polygon edge reported the
--     offset ADDING floor
--
-- Do it here and all of it goes. A cell knows what is beside it, because that is
-- where the wall masks were computed. Remove the cells an agent's centre cannot
-- occupy, then trace what is left. The boundary is the offset boundary by
-- construction, there is nothing to keep consistent, and the severance check
-- compares cells to cells, which is what it was built for.
--
-- SEEDED FROM WALLS ALONE. A dropoff must not pull the floor back -- an agent
-- may stand on the lip of a ledge -- and a seam is not an edge of the floor at
-- all. Those two rules used to be `if kind == ...` branches in the offset. Here
-- they are just seeds that are not planted.
--
-- The quantisation objection answers itself: the surviving mask is on the 0.5
-- stud lattice, but the simplifier already turns a quantised mask into sub-cell
-- accurate lines -- that is what it does for the traced boundary today, at 0.633
-- studs worst deviation. Nothing is lost by moving the erosion upstream of it.

local Erode = {}

local Thickness = require(script.Parent:WaitForChild("Thickness"))

-- How far back from a wall an agent's centre may not go, in studs.
--
-- 0.5 is ONE CELL, Cocosulx's call: take a node off and let pathfinding handle
-- the rest. It is well under the 1.2 stud radius a 2 stud character wants, so
-- this is deliberately not the full clearance -- the runtime is expected to keep
-- its own distance. Raising it to 1.2 makes this an exact bake-time guarantee
-- and deletes anything narrower than 2.4 studs.
Erode.radius = 0.5

-- Where the wall distance is written. Kept off `thick`, which answers a
-- different question and is seeded from dropoffs as well.
Erode.field = "wallDist"

-- Remove every cell whose centre sits closer to a wall than `radius`.
--
-- NOT DESTRUCTIVE. The cell keeps its data and its region id moves to
-- `regionWas`, so the same bake can be traced eroded or un-eroded and the two
-- compared. Boundary and Severance both gate on `cell.region`, so clearing it is
-- all it takes to be gone.
function Erode.apply(data: any, cfg: any?): any
	local c = cfg or data.config
	local radius = (cfg and cfg.erode) or Erode.radius
	Thickness.build(data, c, { field = Erode.field, seedWallOnly = true })

	local stats = { cells = 0, removed = 0, kept = 0, radius = radius,
		unbounded = 0, regionsEmptied = 0 }
	local before, after = {}, {}
	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			local r = cell.region or cell.regionWas
			if not r then continue end
			-- restore first, so a second call with a different radius is not
			-- eroding an already eroded mask
			cell.region = r
			cell.regionWas = r
			stats.cells += 1
			before[r] = (before[r] or 0) + 1
			local d = cell[Erode.field]
			if d == math.huge then
				-- no wall anywhere in this grid, so nothing to stand back from
				stats.unbounded += 1
			end
			if d ~= nil and d ~= math.huge and d < radius then
				cell.region = nil
				cell.eroded = true
				stats.removed += 1
			else
				cell.eroded = false
				stats.kept += 1
				after[r] = (after[r] or 0) + 1
			end
		end
	end
	for r, n in pairs(before) do
		if n > 0 and (after[r] or 0) == 0 then stats.regionsEmptied += 1 end
	end
	return stats
end

-- Put every eroded cell back, so a result can be traced un-eroded again.
function Erode.restore(data: any)
	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			if cell.regionWas then
				cell.region = cell.regionWas
				cell.eroded = false
			end
		end
	end
end

-- A cell filter for Severance: did this cell survive the erosion.
function Erode.keepTest(): (any) -> boolean
	return function(cell: any): boolean
		return not cell.eroded
	end
end

function Erode.report(stats: any): string
	return ("erode     %.2f studs -- %d cells in, %d removed (%.1f%%), %d left%s")
		:format(stats.radius, stats.cells, stats.removed,
			stats.cells > 0 and (stats.removed / stats.cells * 100) or 0, stats.kept,
			stats.regionsEmptied > 0
				and (("; %d regions emptied"):format(stats.regionsEmptied)) or "")
end

return Erode
