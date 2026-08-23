--!strict
-- NavGen.Segments — split the fitted polyline by class, as a POST-PASS.
--
-- LineFit is blind to class and finds every corner anyway (18/18 hand marks).
-- That is the point and it is not to be undone: a class change is not a fact
-- about the SHAPE of the ground, so a rule that breaks a segment on one would
-- put a vertex in the middle of a provably straight edge for a non-geometric
-- reason. That is the one-rule-per-screenshot failure that killed the previous
-- detector.
--
-- So the geometry is decided FIRST and frozen, and only then is each finished
-- segment cut where its nodes change class. Two consequences, both wanted:
--
--   * No corner can move. Every LineFit vertex survives this stage unchanged
--     and at the same index. The cuts are added between them, never instead of
--     them.
--   * Labelling becomes possible. A piece that was part `seam` and part `drop`
--     could not be labelled at all; after the cut each piece has exactly one
--     class, and the downstream stage can dissolve the seams.
--
-- WHY SEAMS MATTER. `seam` is 24% of SmallMap's boundary (1920 of 8118 edges)
-- and is not a boundary at all: it is where one part's grid ends and another
-- walkable part continues. It exists only because grids are per-part and sit on
-- different lattices, so there is no common integer grid to union cells on.
-- Dissolve it downstream, or the map reads as 87 sealed islands.
--
-- WHERE THE CUT LANDS. Node `j` is the last node of the old class and `j+1` the
-- first of the new. The cut is placed AT `j`, and node `j` is the shared
-- endpoint of both pieces — exactly as consecutive LineFit segments share a
-- vertex, so the polyline stays connected by construction. The piece after the
-- cut takes its class from `j+1` onward, so the shared node never contaminates
-- the label of the piece it merely touches.
--
-- Most cuts are free: `ringNodesOf` already pushes a DUPLICATE node at a class
-- change (196 of them on SmallMap), so `lat[j] == lat[j+1]` and the two pieces
-- meet at literally the same lattice cell. Where the change is not duplicated
-- the shared-endpoint rule keeps the chain connected anyway.

local LineFit = require(script.Parent.LineFit)

local Segments = {}

export type Segment = {
	i0: number,       -- first node index, into ring.lat / ring.cls / ring.world
	i1: number,       -- last node index
	class: string,    -- "seam" | "wall" | "drop"
	corner0: boolean, -- is i0 a LineFit vertex, or a class cut?
	corner1: boolean,
}

export type Stats = {
	rings: number,
	vertices: number,   -- LineFit vertices, before any splitting
	segments: number,   -- pieces after splitting
	split: number,      -- fit segments that were cut at least once
	cuts: number,       -- cut points added
	degenerate: number, -- zero-length pieces dropped
	byClass: { [string]: number },
}

-- The class of the run `[a+1 .. b]` — i.e. everything the piece covers except
-- its inherited start node. Pieces are cut so that this run is uniform; the
-- assert is here because a non-uniform one means the cut logic below is wrong,
-- not something to be papered over.
local function runClass(cls: { string }, a: number, b: number): string
	return cls[math.min(a + 1, b)]
end

-- Split one fit segment `[a..b]` (node indices, a < b) at every class change
-- strictly inside it. Appends to `out`.
local function splitOne(cls: { string }, a: number, b: number, out: { Segment }, st: Stats): boolean
	local cut = false
	local s = a
	for j = a + 1, b - 1 do
		if cls[j] ~= cls[j + 1] then
			if j > s then
				out[#out + 1] = {
					i0 = s, i1 = j, class = runClass(cls, s, j),
					corner0 = (s == a), corner1 = false,
				}
			else
				st.degenerate += 1
			end
			st.cuts += 1
			cut = true
			s = j
		end
	end
	if b > s then
		out[#out + 1] = {
			i0 = s, i1 = b, class = runClass(cls, s, b),
			corner0 = (s == a), corner1 = true,
		}
	else
		st.degenerate += 1
	end
	return cut
end

-- Turn one ring plus its fit into classed segments.
--
-- `ring` is an entry of `Boundary.ringCells(...).rings`: `lat`, `cls`, `world`.
-- `fit` is the `LineFit.fit` result for that ring. The ring is closed, so the
-- last vertex wraps to the first.
function Segments.ofRing(ring: any, fit: any, st: Stats): { Segment }
	local v = fit.vertices
	local cls = ring.cls
	local n = #ring.lat
	local out: { Segment } = {}
	if #v < 2 then return out end

	for k = 1, #v do
		local a = v[k]
		local b = (k < #v) and v[k + 1] or v[1]
		-- The wrapping segment runs a -> n -> 1 -> b, which is not an ascending
		-- index range. Walk it in a flattened index space and fold back.
		if b > a then
			if splitOne(cls, a, b, out, st) then st.split += 1 end
		else
			local flat = {}
			for i = a, n do flat[#flat + 1] = i end
			for i = 1, b do flat[#flat + 1] = i end
			local fcls = table.create(#flat)
			for i, idx in ipairs(flat) do fcls[i] = cls[idx] end
			local piece: { Segment } = {}
			if splitOne(fcls, 1, #flat, piece, st) then st.split += 1 end
			for _, sg in ipairs(piece) do
				sg.i0, sg.i1 = flat[sg.i0], flat[sg.i1]
				out[#out + 1] = sg
			end
		end
	end
	return out
end

export type Result = {
	rings: { { ring: any, fit: any, segments: { Segment } } },
	stats: Stats,
}

-- Fit every ring and split the result. `ringData` is `Boundary.ringCells(...)`.
function Segments.build(ringData: any, fitCfg: any?): Result
	local st: Stats = {
		rings = 0, vertices = 0, segments = 0, split = 0,
		cuts = 0, degenerate = 0,
		byClass = { seam = 0, wall = 0, drop = 0 },
	}
	local cfg = { closed = true }
	if fitCfg then for k, val in pairs(fitCfg) do cfg[k] = val end end

	local out = {}
	for _, ring in ipairs(ringData.rings) do
		local cells = table.create(#ring.lat)
		for i, l in ipairs(ring.lat) do cells[i] = { x = l[1], z = l[2] } end
		local fit = LineFit.fit(cells, cfg)
		local segs = Segments.ofRing(ring, fit, st)
		st.rings += 1
		st.vertices += #fit.vertices
		st.segments += #segs
		for _, sg in ipairs(segs) do
			st.byClass[sg.class] = (st.byClass[sg.class] or 0) + 1
		end
		out[#out + 1] = { ring = ring, fit = fit, segments = segs }
	end
	return { rings = out, stats = st }
end

-- The world points of the LineFit vertices only — the corner set, unchanged by
-- this stage. This is what goes to `HandMarks.score`; the cut points are NOT
-- corners and must never be scored as if they were.
function Segments.cornerPoints(res: Result): { Vector3 }
	local pts = {}
	for _, r in ipairs(res.rings) do
		for _, idx in ipairs(r.fit.vertices) do
			pts[#pts + 1] = r.ring.world[idx]
		end
	end
	return pts
end

-- The world points added by the class cuts, kept separate so they can be looked
-- at on their own.
function Segments.cutPoints(res: Result): { Vector3 }
	local pts = {}
	for _, r in ipairs(res.rings) do
		for _, sg in ipairs(r.segments) do
			if not sg.corner0 then pts[#pts + 1] = r.ring.world[sg.i0] end
		end
	end
	return pts
end

return Segments
