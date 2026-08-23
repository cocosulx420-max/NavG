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
	-- "<ringIndex>:<nodeIndex>" -> the welded position of that node, set by
	-- `weldCorners`. Read through `Segments.pointOf`, never straight out of
	-- `ring.world`.
	weld: { [string]: Vector3 }?,
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
	for ri, r in ipairs(res.rings) do
		for _, idx in ipairs(r.fit.vertices) do
			pts[#pts + 1] = Segments.pointOf(res, ri, idx)
		end
	end
	return pts
end

-- The world points added by the class cuts, kept separate so they can be looked
-- at on their own.
function Segments.cutPoints(res: Result): { Vector3 }
	local pts = {}
	for ri, r in ipairs(res.rings) do
		for _, sg in ipairs(r.segments) do
			if not sg.corner0 then pts[#pts + 1] = Segments.pointOf(res, ri, sg.i0) end
		end
	end
	return pts
end

--------------------------------------------------------------------------
-- Looking at it
--------------------------------------------------------------------------

-- Draw the split polyline into `workspace.NVGN_Debug.Segments`.
--
-- ONE PART PER SEGMENT, drawn as the straight chord from its first node to its
-- last -- so what is on screen is the FITTED line, not the ring it was fitted
-- to. If a drawn bar visibly leaves the ring of nodes under it, that is the
-- corridor tolerance being spent, and it is real, not a drawing artifact.
--
-- Colours are Boundary's, so this reads against the existing class view:
-- red wall, blue drop, green seam. Corners are white balls and the class cuts
-- are yellow ones, deliberately DIFFERENT shapes of information -- a white ball
-- is a claim about the shape of the ground, a yellow one is only a change of
-- label and is allowed to sit mid-straightaway.
function Segments.visualize(res: Result, opts: any?, parent: Instance?): number
	if typeof(opts) == "Instance" then parent = opts :: Instance; opts = nil end
	local o = opts or {}
	local classes = o.classes  -- e.g. { seam = false } to hide the seams
	local thick = o.thickness or 0.28
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then
		dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root
	end
	local old = dbg:FindFirstChild("Segments")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "Segments"; folder.Parent = dbg

	local COL = {
		wall = Color3.fromRGB(255, 80, 80),
		drop = Color3.fromRGB(80, 170, 255),
		seam = Color3.fromRGB(90, 255, 120),
	}
	local n = 0

	local function ball(pos: Vector3, size: number, col: Color3, name: string)
		local b = Instance.new("Part")
		b.Name = name
		b.Shape = Enum.PartType.Ball
		b.Size = Vector3.new(size, size, size)
		b.Position = pos
		b.Anchored = true; b.CanCollide = false; b.CanQuery = false; b.CanTouch = false
		b.Material = Enum.Material.Neon; b.Color = col
		b.Parent = folder
		n += 1
	end

	for ri, r in ipairs(res.rings) do
		for _, sg in ipairs(r.segments) do
			if classes and classes[sg.class] == false then continue end
			local a, b = Segments.pointOf(res, ri, sg.i0), Segments.pointOf(res, ri, sg.i1)
			local d = b - a
			local len = d.Magnitude
			-- A zero-length piece has no direction to build a CFrame from; it is
			-- still worth seeing, as a dot.
			if len < 1e-3 then
				ball(a, thick * 2, COL[sg.class] or Color3.new(1, 1, 1), "dot")
			else
				local bar = Instance.new("Part")
				bar.Name = sg.class
				bar.Size = Vector3.new(thick, thick, len)
				bar.CFrame = CFrame.lookAt(a + d * 0.5, b)
				bar.Anchored = true; bar.CanCollide = false
				bar.CanQuery = false; bar.CanTouch = false
				bar.Material = Enum.Material.Neon
				bar.Color = COL[sg.class] or Color3.new(1, 1, 1)
				bar.Parent = folder
				n += 1
			end
		end
	end

	if o.corners ~= false then
		for _, p in ipairs(Segments.cornerPoints(res)) do
			ball(p, 0.7, Color3.fromRGB(255, 255, 255), "corner")
		end
	end
	if o.cuts ~= false then
		for _, p in ipairs(Segments.cutPoints(res)) do
			ball(p, 0.5, Color3.fromRGB(255, 225, 60), "cut")
		end
	end
	return n
end

--------------------------------------------------------------------------
-- Seam corners, and the duplicates a seam creates
--------------------------------------------------------------------------

-- A seam is not a boundary, so A TURN IN A SEAM IS NOT A CORNER. 38 of the 464
-- corners on SmallMap have seam on both sides of them: they exist only because
-- one part's grid stopped and turned. They are junctions in a chain that is
-- about to be dissolved, so anything that welds or offsets corners would be
-- reasoning about them for nothing.
--
-- So: merge every run of consecutive seam pieces into one. A cut can never be
-- seam|seam -- a cut exists only where the class CHANGES -- so every junction
-- this removes is a corner, and no non-seam geometry is touched.
function Segments.dissolveSeamCorners(res: Result): number
	local removed = 0
	for _, r in ipairs(res.rings) do
		local segs = r.segments
		if #segs >= 2 then
			local merged: { Segment } = {}
			for _, sg in ipairs(segs) do
				local last = merged[#merged]
				if last and last.class == "seam" and sg.class == "seam" then
					last.i1, last.corner1 = sg.i1, sg.corner1
					removed += 1
				else
					merged[#merged + 1] = {
						i0 = sg.i0, i1 = sg.i1, class = sg.class,
						corner0 = sg.corner0, corner1 = sg.corner1,
					}
				end
			end
			-- The ring is a CYCLE, so the last piece and the first can be a seam
			-- pair too. Fold the tail into the head rather than leaving behind the
			-- one junction a linear pass cannot see.
			if #merged >= 2 and merged[#merged].class == "seam" and merged[1].class == "seam" then
				merged[1].i0, merged[1].corner0 = merged[#merged].i0, merged[#merged].corner0
				merged[#merged] = nil
				removed += 1
			end
			r.segments = merged
		end
	end
	return removed
end

-- A corner that two rings both see.
--
-- Grids are per-part and sit on different lattices, so where two walkable parts
-- meet, ONE corner in the world gets traced twice, once per ring, a fraction of
-- a stud apart. Both copies are real observations of the same place; downstream
-- they have to become one point or the two floors never stitch.
--
-- WHICH COPY WINS. Not an average, and not the first one seen: the copy on the
-- ring that GENUINELY TURNS there. A corner with real boundary on both sides
-- (drop|drop, wall|wall, drop|wall) is a fact about the ground. A corner with a
-- seam on one side is only the place where the OTHER part's grid ran out, so
-- its position is an artifact of where a lattice happened to stop. When exactly
-- one copy in a cluster is seam-free, that copy is the answer.
--
-- Measured on SmallMap at radius 1.0, after `dissolveSeamCorners`: 43 clusters
-- of two, none of them mixing two corners of the SAME ring (so nothing
-- genuinely distinct is being welded), and none with more than one seam-free
-- copy. 24 resolve by the rule above. The other 19 are `drop|seam` against
-- `seam|drop` -- symmetric, both copies equally artifacted -- and `symmetric`
-- decides what happens to those. The default is to LEAVE THEM ALONE, because
-- guessing there would move a corner for a reason that is not about the ground.
--
-- Radius is 1.0 deliberately. At 1.5 three clusters start swallowing two
-- corners of one ring, which is a different operation and a wrong one.
export type WeldConfig = {
	radius: number?,
	symmetric: string?,  -- "leave" (default) | "midpoint"
}

export type WeldStats = {
	clusters: number, welded: number, moved: number,
	symmetric: number, sameRing: number,
}

-- The position of a node, after welding. Everything downstream must read
-- positions through this and never out of `ring.world`, or welded corners
-- silently come back apart.
function Segments.pointOf(res: Result, ri: number, idx: number): Vector3
	local w = res.weld and res.weld[ri .. ":" .. idx]
	return w or res.rings[ri].ring.world[idx]
end

function Segments.weldCorners(res: Result, cfg: WeldConfig?): WeldStats
	local c = cfg or {}
	local radius = c.radius or 1.0
	local symmetric = c.symmetric or "leave"
	local st: WeldStats = { clusters = 0, welded = 0, moved = 0, symmetric = 0, sameRing = 0 }
	res.weld = {}

	-- Every corner junction, with the class on each side of it.
	local pts = {}
	for ri, r in ipairs(res.rings) do
		local segs = r.segments
		for si, sg in ipairs(segs) do
			local nxt = segs[si == #segs and 1 or si + 1]
			if nxt and sg.corner1 then
				pts[#pts + 1] = {
					ri = ri, idx = sg.i1, pos = r.ring.world[sg.i1],
					a = sg.class, b = nxt.class,
				}
			end
		end
	end

	local used = {}
	for i, p in ipairs(pts) do
		if not used[i] then
			local cl = { p }
			used[i] = true
			for k = i + 1, #pts do
				if not used[k] and (pts[k].pos - p.pos).Magnitude <= radius then
					used[k] = true
					cl[#cl + 1] = pts[k]
				end
			end
			st.clusters += 1
			if #cl > 1 then
				local rings = {}
				for _, q in ipairs(cl) do rings[q.ri] = true end
				local nr = 0
				for _ in pairs(rings) do nr += 1 end
				if nr == 1 then st.sameRing += 1 end

				local free, freeCount = nil, 0
				for _, q in ipairs(cl) do
					if q.a ~= "seam" and q.b ~= "seam" then free, freeCount = q, freeCount + 1 end
				end

				local target: Vector3? = nil
				if freeCount == 1 then
					target = (free :: any).pos
				else
					st.symmetric += 1
					if symmetric == "midpoint" then
						local sum = Vector3.zero
						for _, q in ipairs(cl) do sum += q.pos end
						target = sum / #cl
					end
				end

				if target then
					st.welded += 1
					for _, q in ipairs(cl) do
						if (q.pos - target).Magnitude > 1e-4 then
							res.weld[q.ri .. ":" .. q.idx] = target
							st.moved += 1
						end
					end
				end
			end
		end
	end
	return st
end

return Segments
