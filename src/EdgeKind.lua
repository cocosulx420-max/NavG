--!strict
-- NVGN.EdgeKind -- is this polygon edge a wall, a ledge, or a seam.
--
-- The offset treats the three completely differently. A WALL is pushed inward
-- by the agent radius, because standing inside masonry is not a thing. A DROPOFF
-- is not pushed at all: an agent may walk the lip of a ledge, and standing off
-- from every ledge as though it were a wall removed 12.4% of the cells on the
-- old test map, from exactly the places worth keeping. A SEAM is where the floor
-- continues into the next region, so offsetting it would open a gap between two
-- polygons that are supposed to share an edge.
--
-- Boundary already decides this per face. What it cannot do is keep the answer
-- through simplification: 13634 raw nodes become 530 corners, and the simplifier
-- takes an array of Vector3 and hands corners back.
--
-- MATCHED GEOMETRICALLY, NOT BY INDEX. The obvious method is to say edge k
-- covers raw nodes idx[k] through idx[k+1] and read their kinds. That method is
-- already known to be wrong here: where a boundary pinches, the raw walk goes
-- out along a spur and back, so consecutive corners land on raw indices that run
-- BACKWARDS, and case3 has such a pinch. Pipeline.measure hit this and reports
-- 29.9 studs of error on a 2.5 stud edge when it trusted the index span. So each
-- raw node is instead assigned to the simplified edge it actually lies nearest,
-- which needs no correspondence and cannot be fooled by one.
--
-- A MIXED EDGE IS REPORTED, NOT HIDDEN. A merge can flatten a run that was part
-- masonry and part ledge into one straight edge, and there is no honest single
-- answer for it. The purity is kept alongside the verdict so the offset, and
-- anyone reading the report, can see how much of the edge actually voted.

local EdgeKind = {}

-- Ranked for the tie-break, most cautious first.
--
-- WALL WINS A TIE ON PURPOSE. Offsetting something that turns out to be a ledge
-- costs a strip of walkable floor, which is erosion and safe. NOT offsetting
-- something that turns out to be a wall puts an agent inside geometry, which is
-- the failure the offset exists to prevent. When the votes are level, take the
-- expensive answer over the broken one.
EdgeKind.priority = { wall = 1, edge = 2, drop = 3, none = 4 }

-- Below this share of agreement an edge is counted as mixed. Not a threshold
-- that changes the verdict -- the majority still wins -- only the line at which
-- the disagreement is worth printing.
EdgeKind.mixedBelow = 0.8

local function nearestEdge(q: Vector3, pts: {Vector3}, last: number, n: number): number
	local best, bi = math.huge, 1
	for i = 1, last do
		local a = pts[i]
		local d = pts[(i % n) + 1] - a
		local dd = d:Dot(d)
		local t = dd > 1e-12 and math.clamp((q - a):Dot(d) / dd, 0, 1) or 0
		local dist = (q - (a + d * t)).Magnitude
		if dist < best then
			best = dist
			bi = i
			if best <= 1e-9 then break end
		end
	end
	return bi
end

-- Assign a kind to every edge of every loop, in place.
--
-- Writes `L.edgeKind[i]` and `L.edgePurity[i]` for the edge leaving corner i.
-- An edge that no raw node landed on keeps `"none"` at zero purity rather than
-- borrowing its neighbour's answer: an invented edge, such as one a closure
-- pass added, describes no traced face and should not claim to.
function EdgeKind.assign(loops: {any}): any
	local stats = { edges = 0, wall = 0, drop = 0, edge = 0, none = 0,
		mixed = 0, unvoted = 0, loops = 0 }

	for _, L in ipairs(loops) do
		local pts, poly, kinds = L.pts, L.poly, L.polyKind
		local n = #pts
		if n < 2 or not poly or not kinds then continue end
		stats.loops += 1
		local last = L.closed and n or n - 1

		local votes = table.create(last)
		local totals = table.create(last)
		for i = 1, last do
			votes[i] = {}
			totals[i] = 0
		end

		-- which raw nodes landed on each edge, kept so the offset can read the
		-- ground thickness behind the edge without repeating this search
		local en = table.create(last)
		for i = 1, last do en[i] = {} end

		for k, q in ipairs(poly) do
			local kind = kinds[k] or "none"
			local i = nearestEdge(q, pts, last, n)
			votes[i][kind] = (votes[i][kind] or 0) + 1
			totals[i] += 1
			local t = en[i]
			t[#t + 1] = k
		end

		local ek = table.create(last)
		local ep = table.create(last)
		for i = 1, last do
			local tot = totals[i]
			if tot == 0 then
				ek[i] = "none"
				ep[i] = 0
				stats.unvoted += 1
				stats.none += 1
			else
				local bestKind, bestCount = "none", -1
				for kind, count in pairs(votes[i]) do
					local better = count > bestCount
					if count == bestCount then
						-- deterministic, and biased to the cautious answer
						better = (EdgeKind.priority[kind] or 99)
							< (EdgeKind.priority[bestKind] or 99)
					end
					if better then
						bestKind = kind
						bestCount = count
					end
				end
				local purity = bestCount / tot
				ek[i] = bestKind
				ep[i] = purity
				if stats[bestKind] then stats[bestKind] += 1 end
				if purity < EdgeKind.mixedBelow then stats.mixed += 1 end
			end
			stats.edges += 1
		end

		L.edgeKind = ek
		L.edgePurity = ep
		L.edgeNodes = en
	end

	return stats
end

function EdgeKind.report(stats: any): string
	return ("edges     %d edges -- wall %d, drop %d, seam %d, none %d; %d mixed under %.0f%% agreement, %d unvoted")
		:format(stats.edges, stats.wall, stats.drop, stats.edge, stats.none,
			stats.mixed, EdgeKind.mixedBelow * 100, stats.unvoted)
end

return EdgeKind
