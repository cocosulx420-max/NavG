--!strict
-- NVGN.Boundary -- trace a region's boundary on LocalGrid's OWN data.
--
-- Stage one of the LocalGrid-native tracer: cells -> directed boundary faces ->
-- closed loops. No lattice of its own, so nothing is re-quantized and no cell
-- collapses into a shared slot.
--
-- A face is emitted where a cell has no same-region neighbour, and the test is
-- SYMMETRIC: adjacency is collected as undirected pairs first and both sides are
-- marked interior, so the two cells can never disagree about the face between
-- them. The per-cell masks are read for labelling only.
--
-- Symmetry is the whole invariant. classifyNodes probes from each cell's own
-- plane with its own tolerance, so cell A can find B while B does not find A.
-- Emitting straight from those masks puts a T-junction in the boundary, which
-- leaves a node a walk can enter but not leave, and one such node unbalances the
-- rest of the region and shreds it into open paths.
--
-- Faces are welded on exact position: a face corner is bit identical to its
-- neighbour's, across part seams included, so chaining is a hash join.

local Boundary = {}

-- classifyNodes writes its bits over these, in this order
local DIR4 = { {1,0}, {0,1}, {-1,0}, {0,-1} }
-- the four corner directions, probed so a diagonal touch reads as CONNECTED
local DIAG = { {1,1}, {-1,1}, {-1,-1}, {1,-1} }

-- WIDTH, NOT AREA. This used to compare the region's cell count against
-- minWidth squared, which is an area test wearing a width test's name: it asks
-- whether a region is BIG, and a small region that is perfectly wide enough to
-- stand in fails it. A 2 by 2 stud crawl space needs sixteen cells at a half
-- step and a fifteen cell one was thrown away, taking a whole traversable
-- pocket out of the navmesh before the tracer ever saw it.
--
-- The real question is the one pruneNarrow asks of a cell: does a square of
-- side traceMinWidth fit inside the region. Squares are tested per grid, on the
-- grid's own lattice indices, because that is the only place cells are indexed;
-- a region spanning parts is judged on the widest square any single grid holds.
--
-- traceMinWidth is separate from minWidth on purpose. minWidth is a standing
-- agent's shoulders and prunes handrails at the CELL level; this gate decides
-- whether a surviving region is worth tracing, and a crawl space is narrower
-- than shoulders by definition.
local function liveRegions(data: any): { [number]: boolean }
	if data.liveCache then return data.liveCache end
	local c = data.config
	local step = c.step or 0.5
	local minW = c.traceMinWidth or c.minWidth or 0
	local k = math.max(1, math.ceil(minW / step))

	local out = {}
	if k <= 1 then
		for r in ipairs(data.stats.regionSizes or {}) do out[r] = true end
		data.liveCache = out
		return out
	end

	for _, g in ipairs(data.grids) do
		-- occupancy per region on THIS grid's lattice
		local occ: { [number]: { [string]: boolean } } = {}
		for _, cell in ipairs(g.cells) do
			local r = cell.region
			if r and not out[r] then
				local o = occ[r]
				if not o then o = {}; occ[r] = o end
				o[(cell.ui or 0) .. ":" .. (cell.vi or 0)] = true
			end
		end
		for r, o in pairs(occ) do
			for key in pairs(o) do
				local u0, v0 = key:match("^(-?%d+):(-?%d+)$")
				u0, v0 = tonumber(u0), tonumber(v0)
				local full = true
				for du = 0, k - 1 do
					for dv = 0, k - 1 do
						if not o[(u0 + du) .. ":" .. (v0 + dv)] then full = false break end
					end
					if not full then break end
				end
				if full then out[r] = true break end
			end
		end
	end
	data.liveCache = out
	return out
end
Boundary.liveRegions = liveRegions

local function nodeKey(p: Vector3): string
	return string.format("%d:%d:%d",
		math.round(p.X * 256), math.round(p.Y * 256), math.round(p.Z * 256))
end
Boundary.nodeKey = nodeKey

-- Where the neighbour in local direction d would sit, in world space.
local function neighbourPos(g: any, cell: any, d: {number}): Vector3
	if not g.fallback and g.u and g.v then
		return cell.pos + g.u * (d[1] * g.step) + g.v * (d[2] * g.step)
	end
	return cell.pos + Vector3.new(d[1] * g.step, 0, d[2] * g.step)
end

-- Which of `cell`'s four directions faces the point `p`. Used to mark the far
-- side of an adjacency, whose own lattice may be rotated or offset against this
-- one.
--
-- CAPPED on probeRadius. Nearest-of-four alone is not good enough: where the far
-- cell's lattice is rotated against this one, the nearest direction can be the
-- wrong one, and marking it interior deletes a face that genuinely exists. That
-- is what rounds a square corner off into a diagonal, because the corner cell's
-- own exposed face is suppressed and the boundary walks around a cell that is
-- there.
--
-- Returning nil leaves the far side unmarked, which can leave an asymmetric pair
-- the walk has to close later. That is the cheaper failure by a wide margin:
-- uncapped cost 1173 corners to save 21 asymmetric nodes.
local function directionTo(g: any, cell: any, p: Vector3, r2: number): number?
	local best, bd = nil, math.huge
	for bit, d in ipairs(DIR4) do
		local dd = (neighbourPos(g, cell, d) - p).Magnitude
		if dd < bd then bd = dd; best = bit end
	end
	if best and bd * bd <= r2 then return best end
	return nil
end

-- One directed segment per boundary face, wound so the region lies on the LEFT.
--
-- The outward direction of the face is o, and the travel direction is up x o:
-- up x (up x o) is -o, so the region side ends up on the left of travel. `up` is
-- the surface normal rather than u x v, which keeps the winding consistent even
-- where a grid's in-plane frame is left-handed.
--
-- Pass one collects adjacency and marks BOTH cells interior in the direction
-- that faces the other. Pass two emits a face for every direction left unmarked.
-- A neighbour in a different region is not adjacency, so a region seam is a
-- boundary like any wall or drop.
function Boundary.faces(data: any)
	local c = data.config
	local step = c.step
	local r2 = (c.probeRadius * step) ^ 2
	local tol = c.flushTol

	-- world XZ buckets, so a neighbour is found without knowing which grid owns it
	local keep = liveRegions(data)
	local live: { [string]: {any} } = {}
	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			if cell.region and keep[cell.region] then
				local k = math.floor(cell.pos.X) .. ":" .. math.floor(cell.pos.Z)
				local b = live[k]; if not b then b = {}; live[k] = b end
				b[#b + 1] = { cell = cell, g = g }
			end
		end
	end

	local interior: { [any]: {boolean} } = {}
	local diag: { [any]: {[any]: boolean} } = {}
	local function mark(cell: any, bit: number)
		local t = interior[cell]
		if not t then t = {}; interior[cell] = t end
		t[bit] = true
	end

	local stats = { faces = 0, cells = 0, wall = 0, drop = 0, edge = 0,
		pairs_ = 0, asymmetric = 0, diagonals = 0 }

	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			if cell.region and keep[cell.region] then
				for bit, d in ipairs(DIR4) do
					local p = neighbourPos(g, cell, d)
					local bx, bz = math.floor(p.X), math.floor(p.Z)
					local found, fg = nil, nil
					local bd = math.huge
					for ox = -1, 1 do for oz = -1, 1 do
						for _, e in ipairs(live[(bx + ox) .. ":" .. (bz + oz)] or {}) do
							local q = e.cell
							if q ~= cell and q.region == cell.region then
								local dx, dz = q.pos.X - p.X, q.pos.Z - p.Z
								local dd = dx * dx + dz * dz
								if dd <= r2 and math.abs(q.pos.Y - p.Y) <= tol and dd < bd then
									bd = dd; found = q; fg = e.g
								end
							end
						end
					end end
					if found then
						stats.pairs_ += 1
						mark(cell, bit)
						-- the far side, in ITS own direction indexing
						local back = directionTo(fg, found, cell.pos, r2)
						if back then mark(found, back) else stats.asymmetric += 1 end
					end
				end
				-- 8-CONNECTED, for pinches only. A diagonal neighbour shares no face,
				-- so it can never suppress one -- doing that would punch holes in the
				-- boundary. What it decides is what happens where two cells touch at a
				-- single corner: under 4-connectivity that corner is a pinch the walk
				-- has to guess at, and under 8 the two cells are connected, so the
				-- boundary passes around the pair instead of between them.
				for _, d in ipairs(DIAG) do
					local p = neighbourPos(g, cell, d)
					local bx, bz = math.floor(p.X), math.floor(p.Z)
					for ox = -1, 1 do for oz = -1, 1 do
						for _, en in ipairs(live[(bx + ox) .. ":" .. (bz + oz)] or {}) do
							local q = en.cell
							if q ~= cell and q.region == cell.region then
								local dx, dz = q.pos.X - p.X, q.pos.Z - p.Z
								if dx * dx + dz * dz <= r2 and math.abs(q.pos.Y - p.Y) <= tol then
									local t = diag[cell]
									if not t then t = {}; diag[cell] = t end
									t[q] = true
									local t2 = diag[q]
									if not t2 then t2 = {}; diag[q] = t2 end
									t2[cell] = true
									stats.diagonals += 1
								end
							end
						end
					end end
				end
			end
		end
	end

	local out = {}
	for _, g in ipairs(data.grids) do
		local u = g.u or Vector3.xAxis
		local v = g.v or Vector3.zAxis
		local up = g.n or u:Cross(v)
		for _, cell in ipairs(g.cells) do
			local r = cell.region
			if r and keep[r] then
				local mine = interior[cell]
				local wm, dm, em = cell.wallMask or 0, cell.dropMask or 0, cell.edgeMask or 0
				local any = false
				for bit, d in ipairs(DIR4) do
					if not (mine and mine[bit]) then
						local list = out[r]
						if not list then list = {}; out[r] = list end
						local o = (u * d[1] + v * d[2])
						local ctr = cell.pos + o * (step / 2)
						local t = up:Cross(o)
						local m = bit32.lshift(1, bit - 1)
						local kind = (bit32.band(em, m) ~= 0 and "edge")
							or (bit32.band(wm, m) ~= 0 and "wall")
							or (bit32.band(dm, m) ~= 0 and "drop") or "none"
						if stats[kind] then stats[kind] += 1 end
						stats.faces += 1
						any = true
						list[#list + 1] = {
							a = ctr - t * (step / 2),
							b = ctr + t * (step / 2),
							up = up, cell = cell, kind = kind, dir = bit,
						}
					end
				end
				if any then stats.cells += 1 end
			end
		end
	end
	return out, stats, diag
end

-- The 8-CONNECTED border set: every cell missing a neighbour in any of the eight
-- directions, not just the four it has faces on.
--
-- A cell whose four faces are all covered can still touch empty space at a
-- corner. It contributes no face, so face tracing never sees it and it reads as
-- interior, which leaves gaps along every diagonal. It is a border cell all the
-- same, so it is reported here separately from the tracing.
--
-- This does NOT feed the loops. A diagonal touch has no edge to emit, and the
-- chaining depends on faces meeting at shared corners.
function Boundary.borderCells(data: any)
	local c = data.config
	local step = c.step
	local r2 = (c.probeRadius * step) ^ 2
	local tol = c.flushTol
	local keep = liveRegions(data)
	local live: { [string]: {any} } = {}
	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			if cell.region and keep[cell.region] then
				local k = math.floor(cell.pos.X) .. ":" .. math.floor(cell.pos.Z)
				local b = live[k]; if not b then b = {}; live[k] = b end
				b[#b + 1] = cell
			end
		end
	end
	local function hasNeighbour(g, cell, d)
		local p = neighbourPos(g, cell, d)
		local bx, bz = math.floor(p.X), math.floor(p.Z)
		for ox = -1, 1 do for oz = -1, 1 do
			for _, q in ipairs(live[(bx + ox) .. ":" .. (bz + oz)] or {}) do
				if q ~= cell and q.region == cell.region then
					local dx, dz = q.pos.X - p.X, q.pos.Z - p.Z
					if dx * dx + dz * dz <= r2 and math.abs(q.pos.Y - p.Y) <= tol then
						return true
					end
				end
			end
		end end
		return false
	end
	local out, stats = {}, { border = 0, ortho = 0, diagonalOnly = 0 }
	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			if cell.region and keep[cell.region] then
				local ortho, diagonal = false, false
				for _, d in ipairs(DIR4) do
					if not hasNeighbour(g, cell, d) then ortho = true; break end
				end
				if not ortho then
					for _, d in ipairs(DIAG) do
						if not hasNeighbour(g, cell, d) then diagonal = true; break end
					end
				end
				if ortho or diagonal then
					out[cell] = { region = cell.region, grid = g, ortho = ortho }
					stats.border += 1
					if ortho then stats.ortho += 1 else stats.diagonalOnly += 1 end
				end
			end
		end
	end
	return out, stats
end

-- Give every face endpoint a canonical node id.
--
-- Two tiers, because the endpoints fail to coincide for two unrelated reasons.
-- WELD_EPS collapses float noise: a face corner and its neighbour's agree to
-- about 1e-5 studs, which straddles any fixed rounding boundary and would
-- otherwise split one node in two. SEAM_TOL then stitches genuine gaps where a
-- region crosses from one part's grid to another, and it is applied ONLY to
-- nodes whose in and out degree already disagree, so a healthy loop can never
-- be damaged by it.
--
-- SEAM_TOL reaches FURTHER THAN ONE STEP, which is only safe because of what it
-- is allowed to touch. Real seam gaps measure just over a step -- 0.500, 0.504
-- and 0.705 studs in r004 -- so a tolerance below one step refuses every one of
-- them and leaves the region as open paths rather than a polygon.
--
-- The protection is not the distance. It is that this tier only ever considers
-- nodes whose in and out degree already disagree, and only pairs them when their
-- imbalances have OPPOSITE sign, so it can only ever join an end that is missing
-- an outgoing edge to one missing an incoming edge. Every node on a healthy loop
-- is balanced and is never a candidate, whatever it is standing next to.
local WELD_EPS = 0.01
local SEAM_STEPS = 1.6

function Boundary.weld(faces: {any}, step: number)
	local seamTol = SEAM_STEPS * step
	local ids = {}
	local pos: {Vector3} = {}
	local hash: { [string]: {number} } = {}
	local function bkey(p: Vector3, s: number): string
		return math.floor(p.X / s) .. ":" .. math.floor(p.Y / s) .. ":" .. math.floor(p.Z / s)
	end
	local function canonical(p: Vector3): number
		local bx = math.floor(p.X / WELD_EPS)
		local by = math.floor(p.Y / WELD_EPS)
		local bz = math.floor(p.Z / WELD_EPS)
		for ox = -1, 1 do for oy = -1, 1 do for oz = -1, 1 do
			for _, id in ipairs(hash[(bx+ox)..":"..(by+oy)..":"..(bz+oz)] or {}) do
				if (pos[id] - p).Magnitude <= WELD_EPS then return id end
			end
		end end end
		pos[#pos + 1] = p
		local id = #pos
		local k = bkey(p, WELD_EPS)
		local b = hash[k]; if not b then b = {}; hash[k] = b end
		b[#b + 1] = id
		return id
	end
	for i, f in ipairs(faces) do
		ids[i] = { a = canonical(f.a), b = canonical(f.b) }
	end

	-- second tier: only nodes that are already unbalanced
	local indeg, outdeg = {}, {}
	for _, e in ipairs(ids) do
		outdeg[e.a] = (outdeg[e.a] or 0) + 1
		indeg[e.b] = (indeg[e.b] or 0) + 1
	end
	local loose = {}
	for id = 1, #pos do
		if (indeg[id] or 0) ~= (outdeg[id] or 0) then loose[#loose + 1] = id end
	end
	local remap, stitched = {}, 0
	for x = 1, #loose do
		local i = loose[x]
		if not remap[i] then
			local best, bd = nil, seamTol
			for y = x + 1, #loose do
				local j = loose[y]
				if not remap[j] then
					local d = (pos[j] - pos[i]).Magnitude
					-- opposite imbalance, or the merge just moves the problem
					local si = (indeg[i] or 0) - (outdeg[i] or 0)
					local sj = (indeg[j] or 0) - (outdeg[j] or 0)
					if d <= bd and si * sj < 0 then best = j; bd = d end
				end
			end
			if best then remap[best] = i; stitched += 1 end
		end
	end
	if stitched > 0 then
		for _, e in ipairs(ids) do
			e.a = remap[e.a] or e.a
			e.b = remap[e.b] or e.b
		end
	end
	return ids, { nodes = #pos - stitched, stitched = stitched }
end

-- Chain one region's faces into closed loops.
--
-- Most nodes join exactly two faces and the walk is forced. Where more meet, the
-- node is a PINCH: two cells of the same region touching at a single corner.
--
-- 8-CONNECTED, so a corner touch is a connection. The walk takes the SHALLOWEST
-- turn there, which carries the boundary around the outside of the pair. Taking
-- the sharpest instead cuts between them, which threads the boundary through a
-- join that is not a gap and leaves a spurious notch a cell wide.
--
-- At an ordinary node there is only one unused continuation and the angle never
-- comes into it.
function Boundary.chain(faces: {any}, ids: {any}, cw: boolean?)
	local outAt: { [number]: {number} } = {}
	local inAt: { [number]: {number} } = {}
	for i in ipairs(faces) do
		local ka, kb = ids[i].a, ids[i].b
		local b = outAt[ka]; if not b then b = {}; outAt[ka] = b end
		b[#b + 1] = i
		local c = inAt[kb]; if not c then c = {}; inAt[kb] = c end
		c[#c + 1] = i
	end

	local used = {}
	local loops, broken = {}, 0
	local function pick(atKey: number, din: Vector3?, up: Vector3): number?
		local cand = outAt[atKey]
		if not cand then return nil end
		local live = {}
		for _, i in ipairs(cand) do
			if not used[i] then live[#live + 1] = i end
		end
		if #live == 0 then return nil end
		if #live == 1 or not din then return live[1] end
		-- a pinch: keep the diagonally touching cells together by turning as
		-- little as possible
		local best, bestAng = nil, nil
		for _, i in ipairs(live) do
			local f = faces[i]
			local dout = (f.b - f.a)
			if dout.Magnitude > 1e-9 then
				dout = dout.Unit
				local rev = -din
				local ang = math.atan2(rev:Cross(dout):Dot(up), rev:Dot(dout))
				if ang <= 1e-9 then ang += 2 * math.pi end
				if cw then ang = 2 * math.pi - ang end
				if bestAng == nil or ang > bestAng then bestAng = ang; best = i end
			end
		end
		return best or live[1]
	end

	for seed = 1, #faces do
		if not used[seed] then
			local seq, cur, din = {}, seed, nil
			local startKey = ids[seed].a
			local closed = false
			while cur and not used[cur] do
				used[cur] = true
				local f = faces[cur]
				seq[#seq + 1] = cur
				local endKey = ids[cur].b
				local d = f.b - f.a
				din = (d.Magnitude > 1e-9) and d.Unit or din
				if endKey == startKey then closed = true; break end
				cur = pick(endKey, din, f.up)
			end
			-- An open path is extended BACKWARDS from its seed as well. Without
			-- this a single unbalanced node shreds the rest of the region: each
			-- later seed lands mid-path and reports its own fragment, so one
			-- defect is counted hundreds of times and the real count is hidden.
			if not closed then
				local head = startKey
				while true do
					local nxt = nil
					for _, i in ipairs(inAt[head] or {}) do
						if not used[i] then nxt = i; break end
					end
					if not nxt then break end
					used[nxt] = true
					table.insert(seq, 1, nxt)
					head = ids[nxt].a
				end
				broken += 1
			end
			loops[#loops + 1] = { faces = seq, closed = closed }
		end
	end
	table.sort(loops, function(x, y) return #x.faces > #y.faces end)
	return loops, broken
end

-- Angles live on a 180-degree axis: a boundary direction has no head or tail.
local function angDiff(a: number, b: number): number
	local d = math.abs(a - b) % 180
	return (d > 90) and (180 - d) or d
end
Boundary.angDiff = angDiff

-- Hard corners: vertices the tangent window must not sample across.
--
-- Measured against the NET direction of the faces either side, never the single
-- step between two faces. On a lattice every step turns by exactly 90 degrees,
-- so a single-step test marks every vertex of a 45 degree staircase as a corner
-- and shatters a run that is genuinely straight. Over a window the staircase's
-- alternation cancels and its net direction is the diagonal, while a real L
-- corner still reads 90 degrees.
--
-- Every vertex past the threshold is flagged. A window-based detector has width,
-- so one geometric corner fires as a short run of flags, and every vertex in that
-- run becomes a hard boundary for the fitting window.
--
-- The two ends of an open path are hard boundaries too: there is nothing beyond
-- them to fit to.
function Boundary.corners(trace: any, cfg: any?)
	local W = (cfg and cfg.cornerWindow) or 3
	local thr = math.cos(math.rad((cfg and cfg.cornerAngle) or 45))
	local stats = { corners = 0, vertices = 0 }
	for _, e in pairs(trace) do
		for _, lp in ipairs(e.loops) do
			local F = lp.faces
			local n = #F
			local dir = {}
			for i, fi in ipairs(F) do
				local f = e.faces[fi]
				local d = f.b - f.a
				dir[i] = (d.Magnitude > 1e-9) and d.Unit or Vector3.zero
			end
			-- how sharply the boundary turns at each vertex, as a dot product:
			-- +1 straight on, 0 a right angle, -1 a reversal
			local dot = {}
			for v = 1, n do
				stats.vertices += 1
				local before, after = Vector3.zero, Vector3.zero
				for x = 0, W - 1 do
					local j = v - x
					if lp.closed then j = ((j - 1) % n) + 1 elseif j < 1 then j = nil end
					if j then before += dir[j] end
				end
				for x = 1, W do
					local j = v + x
					if lp.closed then j = ((j - 1) % n) + 1 elseif j > n then j = nil end
					if j then after += dir[j] end
				end
				if before.Magnitude > 1e-6 and after.Magnitude > 1e-6 then
					dot[v] = before.Unit:Dot(after.Unit)
				end
			end

			local C = {}
			for v = 1, n do
				if dot[v] and dot[v] < thr then
					C[v] = true
					stats.corners += 1
				end
			end
			if not lp.closed and not C[n] then
				C[n] = true
				stats.corners += 1
			end
			lp.corner = C
			lp.turnDot = dot
		end
	end
	return stats
end

-- A direction per FACE, fitted from the faces either side of it ALONG THE LOOP.
--
-- The window is arc length, not a radius in space, and it is clamped to less
-- than half the loop. That is what makes a small ring safe: a spatial window
-- wraps a ring smaller than itself and returns the ring's diameter instead of a
-- local direction, so cells on opposite sides of a hole come back with the same
-- tangent. An arc-length window cannot reach round to the far side.
--
-- The window is also TRUNCATED AT HARD CORNERS, so no fit ever samples across
-- one. A window that straddles a corner averages the two edges into a direction
-- belonging to neither, which rounds the corner off and leaves the weld nothing
-- crisp to intersect. Truncated, the two edges meet at a real 90 degrees.
--
-- Fitted in the loop's own plane, from the mean of its faces' normals, so the
-- two principal axes are a real 2-D fit rather than a 3-D one that spends a
-- degree of freedom on the surface normal.
function Boundary.tangents(trace: any, cfg: any?)
	local window = (cfg and cfg.window) or 1.5
	local step = (cfg and cfg.step) or 0.5
	local k0 = math.max(1, math.round(window / step))
	local stats = { fitted = 0, loops = 0, clamped = 0, shortLoops = 0, linSum = 0,
		truncated = 0 }

	for _, e in pairs(trace) do
		for _, lp in ipairs(e.loops) do
			local F = lp.faces
			local n = #F
			stats.loops += 1

			-- loop plane and an in-plane basis
			local nrm = Vector3.zero
			for _, fi in ipairs(F) do nrm += e.faces[fi].up end
			nrm = (nrm.Magnitude > 1e-6) and nrm.Unit or Vector3.yAxis
			local f1 = e.faces[F[1]]
			local d1 = f1.b - f1.a
			local e1 = d1 - nrm * d1:Dot(nrm)
			if e1.Magnitude < 1e-6 then
				e1 = Vector3.xAxis - nrm * Vector3.xAxis:Dot(nrm)
				if e1.Magnitude < 1e-6 then e1 = Vector3.zAxis - nrm * Vector3.zAxis:Dot(nrm) end
			end
			e1 = e1.Unit
			local e2 = nrm:Cross(e1)

			-- one sample per face, at its midpoint, spaced one step apart
			local px, py = {}, {}
			for i, fi in ipairs(F) do
				local f = e.faces[fi]
				local m = (f.a + f.b) * 0.5
				px[i] = m:Dot(e1); py[i] = m:Dot(e2)
			end

			-- never let the window reach round to the far side of the loop
			local k = math.min(k0, math.floor((n - 1) / 2))
			if k < k0 then stats.clamped += 1 end
			local C = lp.corner or {}
			local tangent, linearity = {}, {}
			if k >= 1 then
				for i = 1, n do
					-- grow outward from i, stopping at the first hard corner vertex.
					-- going left from face j crosses vertex j-1; going right from
					-- face j crosses vertex j.
					local idx = { i }
					local j = i
					for _ = 1, k do
						local v = ((j - 2) % n) + 1
						if C[v] then break end
						if (not lp.closed) and j - 1 < 1 then break end
						j = v
						table.insert(idx, 1, j)
					end
					j = i
					for _ = 1, k do
						if C[j] then break end
						if (not lp.closed) and j + 1 > n then break end
						j = (j % n) + 1
						idx[#idx + 1] = j
						if j == i then break end
					end
					local m = #idx
					if m < k * 2 + 1 then stats.truncated += 1 end
					local sx, sy = 0, 0
					for x = 1, m do sx += px[idx[x]]; sy += py[idx[x]] end
					if m == 1 then
						-- a single face between two corners: its own direction IS the
						-- tangent, and an axis-aligned segment is exactly linear
						local f = e.faces[F[i]]
						local d = f.b - f.a
						tangent[i] = math.deg(math.atan2(d:Dot(e2), d:Dot(e1))) % 180
						linearity[i] = 1
						stats.fitted += 1
						stats.linSum += 1
					elseif m == 2 then
						-- two samples have no covariance to speak of; the chord between
						-- them is the answer, and it reads a diagonal as 45 degrees
						local ax = px[idx[2]] - px[idx[1]]
						local ay = py[idx[2]] - py[idx[1]]
						tangent[i] = math.deg(math.atan2(ay, ax)) % 180
						linearity[i] = 1
						stats.fitted += 1
						stats.linSum += 1
					else
						local mx, my = sx / m, sy / m
						local sxx, sxy, syy = 0, 0, 0
						for x = 1, m do
							local dx, dy = px[idx[x]] - mx, py[idx[x]] - my
							sxx += dx * dx; sxy += dx * dy; syy += dy * dy
						end
						local tr = sxx + syy
						local disc = math.max(0, tr * tr / 4 - (sxx * syy - sxy * sxy))
						local l1 = tr / 2 + math.sqrt(disc)
						local l2 = tr / 2 - math.sqrt(disc)
						local ang
						if math.abs(sxy) > 1e-12 then ang = math.atan2(l1 - sxx, sxy)
						else ang = (sxx >= syy) and 0 or math.pi / 2 end
						tangent[i] = math.deg(ang) % 180
						linearity[i] = (l1 > 1e-12) and (1 - l2 / l1) or 0
						stats.fitted += 1
						stats.linSum += linearity[i]
					end
				end
			else
				stats.shortLoops += 1
			end
			lp.tangent = tangent
			lp.linearity = linearity
			lp.basis = { n = nrm, e1 = e1, e2 = e2 }
		end
	end
	stats.meanLinearity = stats.linSum / math.max(1, stats.fitted)
	return stats
end

-- Cut each loop into ARCS wherever the direction turns.
--
-- An arc is a contiguous run of the loop, and it knows its neighbours because
-- arcs are stored in loop order: arc N is followed by arc N+1, and the last is
-- followed by the first on a closed loop. Nothing downstream has to rediscover
-- adjacency from endpoint distances.
--
-- Compared against the arc's ANCHOR direction, never a running mean: a running
-- mean drifts along a curve and swallows the whole thing.
--
-- The walk starts at the sharpest turn in the loop, so the cut set is a property
-- of the geometry rather than of whichever face the trace happened to seed on.
-- Greedy segmentation from an arbitrary start is not reproducible.
function Boundary.arcs(trace: any, cfg: any?)
	local tol = (cfg and cfg.tol) or 20
	local stats = { arcs = 0, loops = 0, oneArc = 0, faces = 0 }
	for _, e in pairs(trace) do
		for _, lp in ipairs(e.loops) do
			local n = #lp.faces
			local T = lp.tangent or {}
			stats.loops += 1

			local startAt, bestTurn = 1, -1
			if lp.closed then
				for i = 1, n do
					local j = (i % n) + 1
					local a, b = T[i], T[j]
					if a and b then
						local d = angDiff(a, b)
						if d > bestTurn then bestTurn = d; startAt = j end
					end
				end
			end

			local arcs = {}
			local cur, anchor = nil, nil
			for x = 0, n - 1 do
				local i = ((startAt - 1 + x) % n) + 1
				local t = T[i]
				local cut = (cur == nil) or (t == nil) or (anchor == nil)
					or (angDiff(t, anchor) > tol)
				if cut then
					cur = { i }
					anchor = t
					arcs[#arcs + 1] = cur
				else
					cur[#cur + 1] = i
				end
				stats.faces += 1
			end
			lp.arcs = arcs
			stats.arcs += #arcs
			if #arcs == 1 then stats.oneArc += 1 end
		end
	end
	return stats
end

-- The LINES of each loop, fitted on the NODES.
--
-- A loop's nodes are its vertices, in order: node i is where face i starts, and
-- the step from node i to node i+1 IS face i. Fitting on the nodes rather than
-- on face midpoints means a line is a run of vertices, which is what a polygon
-- edge actually is, and the endpoints a weld needs are nodes rather than points
-- interpolated between them.
--
-- Four things happen per loop, in order:
--   1. a corner is found AT a node, from the net direction of the steps arriving
--      against the net of those leaving. Net, not the single step either side:
--      on a lattice every step turns 90 degrees, so a single-step test marks
--      every vertex of a 45 degree staircase and shatters a straight run.
--   2. the loop is cut into runs at those corners. A corner ENDS its run.
--   3. each run is split at its worst node until no node sits further than
--      devTol from its own line's chord, then adjacent pieces are MERGED back
--      wherever the combined chord still fits.
--   4. one direction per line, a total least squares fit over that line's own
--      nodes, copied onto each node it owns.
--
-- DEVIATION IN STUDS, not an angle against a running anchor. An angular cut
-- fires on the jitter of a staircase and sheds a fragment each time, and it
-- cannot tell how far the drawn chord actually strays from the boundary, which
-- is the only thing that matters. Splitting on deviation and merging back is
-- also what stops a run continuing past a corner the angular detector missed: a
-- line that overshoots a 90 degree corner has a huge deviation and is cut.
--
-- devTol is measured from the fit, so half a step is the natural bound: that is
-- how far a single-step staircase of any tread length strays from its own fitted
-- line. Anything beyond it is a bend rather than quantization.
--
-- Lines PARTITION the nodes, so each node carries exactly one line id and the
-- line order round the loop is line 1, 2, 3 with the last meeting the first.
function Boundary.nodeLines(trace: any, cfg: any?)
	local c = cfg or {}
	local step = c.step or 0.5
	local devTol = c.devTol or (step * 0.6)
	local W = c.cornerWindow or 3
	local thr = math.cos(math.rad(c.cornerAngle or 45))
	local suppress = c.suppress ~= false
	local revThr = c.reversalDot or -0.7
	-- 40, not 45: on a midpoint polyline a square corner of the region shows up
	-- as a 45 degree chamfer rather than a 90 degree step
	local stepAngle = c.stepAngle or 40
	local stats = { loops = 0, nodes = 0, corners = 0, kept = 0, lines = 0, fitted = 0,
		splits = 0, merges = 0, snapped = 0, chamfers = 0, linSum = 0, oneLine = 0 }

	for _, e in pairs(trace) do
		for _, lp in ipairs(e.loops) do
			local n = #lp.faces
			stats.loops += 1
			stats.nodes += n

			-- NODES SIT ON FACE MIDPOINTS, not on face corners.
			--
			-- A corner polyline can only step along the lattice axes, so a 45 degree
			-- boundary comes out as a zigzag that strays half a step from the line it
			-- is approximating, and every fit downstream has to tolerate that. Joining
			-- face midpoints instead lets two perpendicular faces of one cell connect
			-- diagonally, so the same boundary comes out as an exactly straight run
			-- and its deviation collapses to nothing.
			--
			-- It also puts a node half a step inside the cell edge rather than on the
			-- lattice corner, which is where the wall is, and it makes the node to
			-- cell mapping one to one: node i belongs to face i and nothing else.
			local node, dir = {}, {}
			for i, fi in ipairs(lp.faces) do
				local f = e.faces[fi]
				node[i] = (f.a + f.b) * 0.5
			end
			for i = 1, n do
				local j = (i % n) + 1
				if (not lp.closed) and i == n then
					dir[i] = dir[i - 1] or Vector3.zero
				else
					local d = node[j] - node[i]
					dir[i] = (d.Magnitude > 1e-9) and d.Unit or Vector3.zero
				end
			end

			-- The turn between the two steps meeting AT a node. On a lattice this is
			-- 90 degrees or nothing, so it says exactly where a turn is available --
			-- unlike the net-direction window, which says whether a turn is real but
			-- can put its minimum a node to either side of one.
			local function adjTurn(i)
				if (not lp.closed) and i <= 1 then return -1 end
				local a, b = dir[((i - 2) % n) + 1], dir[i]
				if a.Magnitude < 0.5 or b.Magnitude < 0.5 then return -1 end
				return math.deg(math.acos(math.clamp(a:Dot(b), -1, 1)))
			end

			-- plane and in-plane basis for the loop
			local nrm = Vector3.zero
			for _, fi in ipairs(lp.faces) do nrm += e.faces[fi].up end
			nrm = (nrm.Magnitude > 1e-6) and nrm.Unit or Vector3.yAxis
			local seed = (dir[1].Magnitude > 1e-6) and dir[1] or Vector3.xAxis
			local e1 = seed - nrm * seed:Dot(nrm)
			if e1.Magnitude < 1e-6 then
				e1 = Vector3.xAxis - nrm * Vector3.xAxis:Dot(nrm)
				if e1.Magnitude < 1e-6 then e1 = Vector3.zAxis - nrm * Vector3.zAxis:Dot(nrm) end
			end
			e1 = e1.Unit
			local e2 = nrm:Cross(e1)
			local px, py = {}, {}
			for i = 1, n do px[i] = node[i]:Dot(e1); py[i] = node[i]:Dot(e2) end

			-- 1. corners, at nodes
			local corner, turnDot = {}, {}
			for i = 1, n do
				if (not lp.closed) and (i == 1 or i == n) then
					corner[i] = true
					stats.corners += 1
				else
					local before, after = Vector3.zero, Vector3.zero
					for x = 1, W do
						local j = i - x
						if lp.closed then j = ((j - 1) % n) + 1 elseif j < 1 then j = nil end
						if j then before += dir[j] end
					end
					for x = 0, W - 1 do
						local j = i + x
						if lp.closed then j = ((j - 1) % n) + 1 elseif j > n then j = nil end
						if j then after += dir[j] end
					end
					if before.Magnitude > 1e-6 and after.Magnitude > 1e-6 then
						local dp = before.Unit:Dot(after.Unit)
						turnDot[i] = dp
						-- ONLY A REVERSAL IS A HARD SPLIT.
						--
						-- A fixed window cannot find an ordinary corner: near the ends
						-- of a run it reaches across the neighbouring corner into the
						-- previous line and reports a turn that is not there, and
						-- clipping it leaves too few steps to read the run's direction
						-- at all. A staircase whose period exceeds the window fails
						-- either way, so no window length works everywhere. Ordinary
						-- corners come from the deviation split below, which is
						-- scale-free. A reversal is different: it is a tip, it cannot be
						-- confused with a staircase, and a fit through it is meaningless.
						if dp < revThr and adjTurn(i) >= stepAngle then
							corner[i] = true
							stats.corners += 1
						end
					end
				end
			end

			-- Reversals need no suppression: every one of them is a real tip. On a
			-- ribbon one cell wide the window spans its whole width and each of the
			-- four tips reads as a reversal, which is correct.

			-- 2. runs between corners. A corner ENDS its run.
			local runs = {}
			if lp.closed then
				local cuts = {}
				for i = 1, n do if corner[i] then cuts[#cuts + 1] = i end end
				if #cuts == 0 then
					-- a closed loop with no corner at all still needs one cut, or it
					-- is a single line biting its own tail
					local sharp, at = math.huge, 1
					for i = 1, n do
						local dp = turnDot[i]
						if dp and dp < sharp and adjTurn(i) >= stepAngle then sharp = dp; at = i end
					end
					corner[at] = true
					cuts = { at }
					stats.kept += 1
				end
				for ci = 1, #cuts do
					local from = (cuts[ci] % n) + 1
					local upto = cuts[(ci % #cuts) + 1]
					local run, j, guard = {}, from, 0
					while guard <= n do
						run[#run + 1] = j
						if j == upto then break end
						j = (j % n) + 1
						guard += 1
					end
					if #run > 0 then runs[#runs + 1] = run end
				end
			else
				local run = {}
				for i = 1, n do
					run[#run + 1] = i
					if corner[i] and i > 1 then
						runs[#runs + 1] = run
						run = {}
					end
				end
				if #run > 0 then runs[#runs + 1] = run end
			end

			-- 3. split each run where its nodes bow off their own chord, then merge
			-- neighbours back wherever the combined chord still fits
			-- Deviation is measured from the piece's own LEAST SQUARES line, not from
			-- the chord between its end nodes, and the difference decides both of the
			-- ways this can go wrong.
			--
			-- A monotone staircase lies entirely on ONE side of its endpoint chord: a
			-- 45 degree run reaches 0.354 and a long-tread run approaches a full step,
			-- so a chord test splits clean diagonals. The same staircase STRADDLES its
			-- least squares line and only reaches half a step. A real bend does not
			-- straddle anything -- the fit cannot pass through the middle of a corner
			-- -- so its deviation stays large. Chord distance cannot tell a 0.39 stud
			-- bend from staircase quantization; distance from the fit can.
			local function devOf(run, a, b)
				local m = b - a + 1
				if m < 3 then return 0, a end
				local sx, sy = 0, 0
				for x = a, b do sx += px[run[x]]; sy += py[run[x]] end
				local mx, my = sx / m, sy / m
				local sxx, sxy, syy = 0, 0, 0
				for x = a, b do
					local dx, dy = px[run[x]] - mx, py[run[x]] - my
					sxx += dx * dx; sxy += dx * dy; syy += dy * dy
				end
				local tr = sxx + syy
				local disc = math.max(0, tr * tr / 4 - (sxx * syy - sxy * sxy))
				local l1 = tr / 2 + math.sqrt(disc)
				local ax, ay
				if math.abs(sxy) > 1e-12 then ax, ay = sxy, l1 - sxx
				elseif sxx >= syy then ax, ay = 1, 0
				else ax, ay = 0, 1 end
				local mag = math.sqrt(ax * ax + ay * ay)
				if mag < 1e-12 then return 0, a end
				ax, ay = ax / mag, ay / mag
				local worst, at = 0, a
				for x = a, b do
					local dx, dy = px[run[x]] - mx, py[run[x]] - my
					local off = math.abs(dx * -ay + dy * ax)
					if off > worst then worst = off; at = x end
				end
				return worst, at
			end

			local lines = {}
			for _, run in ipairs(runs) do
				local pieces = {}
				local stack = { { 1, #run } }
				while #stack > 0 do
					local seg = table.remove(stack)
					local a, b = seg[1], seg[2]
					if b - a < 2 then
						pieces[#pieces + 1] = { a, b }
					else
						local worst, at = devOf(run, a, b)
						if worst > devTol then
							-- Put the break on a real lattice turn when one is within reach.
							-- The worst-deviation node is the right neighbourhood but not
							-- always the vertex itself, and a break one node off a turn is a
							-- line that wraps a cell around its own corner.
							local bestAt, bestD = at, math.huge
							for o = -2, 2 do
								local cand = at + o
								if cand > a and cand < b and adjTurn(run[cand]) >= stepAngle then
									if math.abs(o) < bestD then bestD = math.abs(o); bestAt = cand end
								end
							end
							at = bestAt
						end
						if at <= a then at = a + 1 end
						if at >= b then at = b - 1 end
						if worst <= devTol then
							pieces[#pieces + 1] = { a, b }
						else
							-- the break node ENDS the first piece, so the pieces stay a
							-- partition rather than sharing a vertex
							stack[#stack + 1] = { at + 1, b }
							stack[#stack + 1] = { a, at }
							stats.splits += 1
						end
					end
				end
				table.sort(pieces, function(x, y) return x[1] < y[1] end)
				local changed = true
				while changed do
					changed = false
					for x = 1, #pieces - 1 do
						local A, Bp = pieces[x], pieces[x + 1]
						local worst = devOf(run, A[1], Bp[2])
						if worst <= devTol then
							pieces[x] = { A[1], Bp[2] }
							table.remove(pieces, x + 1)
							stats.merges += 1
							changed = true
							break
						end
					end
				end
				for _, p in ipairs(pieces) do
					local ln = {}
					for x = p[1], p[2] do ln[#ln + 1] = run[x] end
					lines[#lines + 1] = ln
				end
			end

			-- A CHAMFER BELONGS TO THE LINE IT TURNS INTO.
			--
			-- On a midpoint polyline a square corner of the region is a single 0.354
			-- step cutting across it. That node is the turn, not part of the run
			-- arriving at it: its outgoing step already heads off in the new
			-- direction, and its cell sits past the corner. Left on the earlier line
			-- it reads as that line reaching a cell into the feature beyond.
			if #lines > 1 then
				local chamfer = step * 0.75
				for li = 1, #lines do
					local cur = lines[li]
					local nxt = lines[(li % #lines) + 1]
					if #cur > 1 and nxt ~= cur then
						local last = cur[#cur]
						local prev = cur[#cur - 1]
						if (node[last] - node[prev]).Magnitude < chamfer then
							table.remove(cur)
							table.insert(nxt, 1, last)
							stats.chamfers += 1
						end
					end
				end
			end

			-- 4. one direction per line, over that line's own nodes
			local tangent, linearity = {}, {}
			for _, ln in ipairs(lines) do
				local m = #ln
				local ang, lin
				if m == 1 then
					local d = dir[ln[1]]
					ang = math.deg(math.atan2(d:Dot(e2), d:Dot(e1))) % 180
					lin = 1
				else
					local sx, sy = 0, 0
					for _, i in ipairs(ln) do sx += px[i]; sy += py[i] end
					local mx, my = sx / m, sy / m
					local sxx, sxy, syy = 0, 0, 0
					for _, i in ipairs(ln) do
						local dx, dy = px[i] - mx, py[i] - my
						sxx += dx * dx; sxy += dx * dy; syy += dy * dy
					end
					local tr = sxx + syy
					local disc = math.max(0, tr * tr / 4 - (sxx * syy - sxy * sxy))
					local l1 = tr / 2 + math.sqrt(disc)
					local l2 = tr / 2 - math.sqrt(disc)
					if math.abs(sxy) > 1e-12 then
						ang = math.deg(math.atan2(l1 - sxx, sxy)) % 180
					else
						ang = (sxx >= syy) and 0 or 90
					end
					lin = (l1 > 1e-12) and (1 - l2 / l1) or 0
				end
				for _, i in ipairs(ln) do tangent[i] = ang; linearity[i] = lin end
				stats.fitted += m
				stats.linSum += lin * m
			end

			lp.node = node
			lp.nCorner = corner
			lp.nTurnDot = turnDot
			lp.nTangent = tangent
			lp.nLinearity = linearity
			lp.basis = { n = nrm, e1 = e1, e2 = e2 }
			lp.lines = lines
			stats.lines += #lines
			if #lines == 1 then stats.oneLine += 1 end
		end
	end
	stats.meanLinearity = stats.linSum / math.max(1, stats.fitted)
	return stats
end

-- Faces and loops for every region. data.boundary[r] = { loops = ..., faces = ... }
function Boundary.trace(data: any, cfg: any?)
	local cw = cfg and cfg.cw or false
	local step = data.config.step
	local byRegion, fstats = Boundary.faces(data)
	local out = {}
	local stats = { regions = 0, loops = 0, closed = 0, broken = 0, stitched = 0, nodes = 0,
		faces = fstats.faces, borderCells = fstats.cells,
		wall = fstats.wall, drop = fstats.drop, edge = fstats.edge,
		unlabelled = fstats.none, adjacency = fstats.pairs_, asymmetric = fstats.asymmetric }
	for r, faces in pairs(byRegion) do
		local ids, wstats = Boundary.weld(faces, step)
		local loops, broken = Boundary.chain(faces, ids, cw)
		out[r] = { faces = faces, ids = ids, loops = loops }
		stats.regions += 1
		stats.loops += #loops
		stats.broken += broken
		stats.closed += (#loops - broken)
		stats.stitched += wstats.stitched
		stats.nodes += wstats.nodes
	end
	data.boundary = out
	data.stats.boundaryFaces = stats.faces
	data.stats.boundaryLoops = stats.loops
	data.stats.boundaryBroken = stats.broken
	return out, stats
end

return Boundary
