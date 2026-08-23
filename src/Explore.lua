--!strict
-- NavGen.Explore -- TEST. Corner finding by exploration, off neighbour lookups
-- and nothing else.
--
-- NOT PART OF THE PIPELINE YET. Written to be judged and thrown away if it is
-- not good.
--
-- The idea, in Cocosulx's words: you explore, and you determine things from the
-- exploration. You do not need a precomputed order, and you do not need to
-- explore all of it -- only far enough to find the pattern break or hold.
--
-- WHAT THIS DELETES from the previous attempt:
--   * chains, and materialising the walk into arrays
--   * open vs closed, endpoints, and cutting at branches
--   * `throughDot`, a constant that paired edges at a branch by angle. That was
--     answering a topology question with a tolerance. A branch needs no rule:
--     you grow down each option and the one that does not break is the one that
--     continues. Growth is the proof, same as everywhere else.
--
-- The two rules are LineFit's, unchanged in meaning:
--
--   RULE 1, direction lock (hard, no rollback). A run may never use two
--   opposite directions on an axis. Checked BEFORE committing the step.
--
--   RULE 2, corridor (soft, always rolls back). Test EVERY node of the run so
--   far against the chord from the run's start to the candidate, because the
--   failure appears BEHIND the walker. Roll back to the argmax -- where the two
--   regimes parted ways -- not to where belief ran out. Guard: strictly < 2.
--
-- They are reimplemented here rather than called, because `LineFit` takes an
-- array and this stage never builds one. That is a real duplication and a real
-- risk: if the rules are ever changed, they are changed in two places.
--
-- The boundary is a node with a wall or a dropoff. There is no seam here and
-- there is nothing to ignore later, because a seam is never counted.
--
-- GEOMETRY IS JUDGED ON THE PART'S OWN LATTICE. This is the whole reason
-- `LocalGrid` samples each block part on its own axes: a rotated part's rim
-- lands on whole lattice lines, so there is no staircase to fit away and the
-- integers are EXACT, not rounded.
--
-- An earlier version of this file rounded world positions onto one global XZ
-- lattice instead, to keep the corridor test in integers. That threw the
-- guarantee away and manufactured staircases out of straight lines: a measured
-- 32-node run stepping a dead-straight (-0.99, +0.14) each time came out as
-- (-29,-10) (-30,-10) (-31,-10) (-32,-9) ... and rule 2 broke it 1.45 cells off
-- a chord it should have been exactly on. The rule was right; the input was a
-- lie.
--
-- So: a run is measured in the lattice of the grid it STARTED on.
--   * While the run stays on that part -- almost all of it -- coordinates are
--     the cell's own `ui, vi`. Exact integers, zero rounding.
--   * Where a run crosses onto another part there is no shared lattice, so the
--     foreign node is converted into the run's frame. To keep that conversion
--     from being the same half-cell lie at a smaller scale, every coordinate is
--     carried at SUBDIV units per cell, so a converted node is off by at most
--     1/(2*SUBDIV) of a cell instead of 1/2. Native coordinates stay exact --
--     they are just multiplied by SUBDIV.
--
-- Everything stays integer, so the corridor test is still an exact cross product
-- and identical input still gives byte-identical output.
--
-- Y is carried on the output and is never part of a decision.

local Explore = {}

local DIR8 = {
	{ 1, 0 }, { 1, 1 }, { 0, 1 }, { -1, 1 },
	{ -1, 0 }, { -1, -1 }, { 0, -1 }, { 1, -1 },
}

export type Config = {
	step: number?,
	minDot: number?,     -- same side; plateau [-0.50, -0.10]
	radius: number?,     -- neighbour radius in steps; must admit the diagonal
	tolNum: number?,
	tolDen: number?,
	probe: number?,      -- how far to explore a branch option before judging it
	maxRuns: number?,    -- tripwire; default 4 runs per node
}

-- Sub-cell units. Native lattice coordinates are exact at any value; this only
-- sets how finely a FOREIGN node lands when converted into the run's frame.
local SUBDIV = 8

local DEFAULT = {
	minDot = -0.30,
	radius = 1.45,
	tolNum = 1,
	tolDen = 1,
	probe = 12,
}

--------------------------------------------------------------------------
-- The world: nodes and who is next to whom
--------------------------------------------------------------------------

local function facing(g: any, cell: any): Vector3?
	local blk = bit32.bor(cell.wallMask or 0, cell.dropMask or 0)
	local s = Vector3.zero
	for bit, d in ipairs(DIR8) do
		if bit32.band(blk, bit32.lshift(1, bit - 1)) ~= 0 then
			local w
			if not g.fallback and g.u and g.v then
				w = g.u * d[1] + g.v * d[2]
			else
				w = Vector3.new(d[1], 0, d[2])
			end
			s += w.Unit
		end
	end
	if s.Magnitude < 1e-3 then return nil end
	return s.Unit
end

export type World = {
	pos: { Vector3 },
	wall: { boolean },
	drop: { boolean },
	u: { Vector3? },        -- the owning grid's face axes, kept for drawing
	nrm: { Vector3? },
	gid: { number },        -- which grid owns this node
	ui: { number },         -- its cell on THAT grid's lattice, exact
	vi: { number },
	frames: { any },        -- per-grid origin/u/v, to convert a foreign node
	wallMask: { number },
	dropMask: { number },
	cell: { { x: number, z: number } },
	out: { Vector3? },
	nbr: { { number } },
	step: number,
	n: number,
}

-- Neighbours, by world proximity and nothing else. Grid identity plays no part;
-- two nodes on differently-rotated parts are neighbours if they are next to each
-- other in the world and the void is on the same side of both.
--
-- The same-side test is what keeps the two faces of a thin wall apart. Without
-- it they fuse into one line.
function Explore.world(localData: any, cfg: Config?): World
	local c: any = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	local step = c.step or (localData.config and localData.config.step) or 1

	local pos, out, wall, drop = {}, {}, {}, {}
	local uAx, nAx, wMask, dMask = {}, {}, {}, {}
	local gid, ui, vi = {}, {}, {}
	local frames = {}
	for _, g in pairs(localData.grids) do
		frames[#frames + 1] = g
		local gi = #frames
		for _, cell in ipairs(g.cells) do
			if cell.wall or cell.dropoff then
				pos[#pos + 1] = cell.pos
				out[#pos] = facing(g, cell)
				wall[#pos] = cell.wall or false
				drop[#pos] = cell.dropoff or false
				-- kept so the view can lie the node flat on ITS OWN face, the way
				-- LocalGrid draws it, instead of inventing a shape for it
				uAx[#pos] = (not g.fallback) and g.u or nil
				nAx[#pos] = (not g.fallback) and g.n or nil
				wMask[#pos] = cell.wallMask or 0
				dMask[#pos] = cell.dropMask or 0
				-- THE EXACT COORDINATES: the cell's own place on its own lattice
				gid[#pos] = gi
				ui[#pos] = cell.ui
				vi[#pos] = cell.vi
			end
		end
	end

	local n = #pos

	local B = step * 1.5
	local hash: { [string]: { number } } = {}
	for i = 1, n do
		local p = pos[i]
		local k = math.floor(p.X/B) .. ":" .. math.floor(p.Y/B) .. ":" .. math.floor(p.Z/B)
		local b = hash[k]
		if not b then b = {}; hash[k] = b end
		b[#b + 1] = i
	end

	-- Neighbours come in two kinds and the difference matters.
	--
	-- A DIAGONAL LINK HAS TO BE ADMITTED, or a boundary running at 45 degrees has
	-- no links at all -- its neighbours sit at sqrt(2) and a 4-connected radius
	-- cannot see them, so every node on it becomes an island.
	--
	-- BUT AN ADMITTED DIAGONAL IS ALSO A SHORTCUT ACROSS EVERY STAIRCASE JOG, and
	-- those shortcuts let a walk step diagonally backwards, which trips rule 1 and
	-- chops runs to nothing: without this the mean run was 4.1 nodes, with 680
	-- dirlock breaks over 1475 runs.
	--
	-- THE CHORD RULE, and there is no tolerance in it: drop a diagonal link when
	-- its two ends already share a 4-connected neighbour. Going the long way round
	-- through that shared neighbour IS the chain; the diagonal is the shortcut.
	local R = step * c.radius
	local nearR = step * 1.05
	local near, diag = table.create(n), table.create(n)
	for i = 1, n do
		local p = pos[i]
		local bx, by, bz = math.floor(p.X/B), math.floor(p.Y/B), math.floor(p.Z/B)
		near[i], diag[i] = {}, {}
		for dx = -1, 1 do for dy = -1, 1 do for dz = -1, 1 do
			local b = hash[(bx+dx) .. ":" .. (by+dy) .. ":" .. (bz+dz)]
			if b then
				for _, j in ipairs(b) do
					if j ~= i then
						local d = (pos[j] - p).Magnitude
						if d <= R and ((not out[i]) or (not out[j]) or out[i]:Dot(out[j]) > c.minDot) then
							if d <= nearR then
								table.insert(near[i], j)
							else
								table.insert(diag[i], j)
							end
						end
					end
				end
			end
		end end end
	end

	local nearSet = table.create(n)
	for i = 1, n do
		local t = {}
		for _, j in ipairs(near[i]) do t[j] = true end
		nearSet[i] = t
	end

	local nbr = table.create(n)
	for i = 1, n do
		local a = {}
		for _, j in ipairs(near[i]) do a[#a + 1] = j end
		for _, j in ipairs(diag[i]) do
			local chord = false
			for k in pairs(nearSet[i]) do
				if nearSet[j][k] then chord = true break end
			end
			if not chord then a[#a + 1] = j end
		end
		nbr[i] = a
	end

	return { pos = pos, wall = wall, drop = drop, u = uAx, nrm = nAx,
		wallMask = wMask, dropMask = dMask,
		gid = gid, ui = ui, vi = vi, frames = frames,
		out = out, nbr = nbr, step = step, n = n }
end

--------------------------------------------------------------------------
-- Coordinates, in the frame of whichever part the run started on
--------------------------------------------------------------------------

-- Native node: exact, just scaled to sub-cell units. Foreign node: projected
-- onto the frame's own axes and rounded to the nearest sub-cell unit, which is
-- the only place any rounding happens at all.
local function coordIn(W: World, frame: number, i: number): (number, number)
	if W.gid[i] == frame then
		return W.ui[i] * SUBDIV, W.vi[i] * SUBDIV
	end
	local g = W.frames[frame]
	local step = W.step
	local p = W.pos[i]
	if g.fallback or not g.u or not g.v or not g.origin then
		return math.round(p.X / step * SUBDIV), math.round(p.Z / step * SUBDIV)
	end
	local rel = p - g.origin
	local a = rel:Dot(g.u) / step - 0.5
	local b = rel:Dot(g.v) / step - 0.5
	return math.round(a * SUBDIV), math.round(b * SUBDIV)
end

--------------------------------------------------------------------------
-- The two rules
--------------------------------------------------------------------------

-- Rule 1. Directions used so far, as four slots: +x, -x, +z, -z.
local function locked(used: { boolean }, dx: number, dz: number): boolean
	if dx > 0 and used[2] then return true end
	if dx < 0 and used[1] then return true end
	if dz > 0 and used[4] then return true end
	if dz < 0 and used[3] then return true end
	return false
end

local function mark(used: { boolean }, dx: number, dz: number)
	if dx > 0 then used[1] = true elseif dx < 0 then used[2] = true end
	if dz > 0 then used[3] = true elseif dz < 0 then used[4] = true end
end

-- Rule 2. Every node of the run against the chord from the run's start to the
-- candidate. Integer cross product against a rational tolerance, cross
-- multiplied -- no floats in the decision, no square root taken.
local function corridor(W: World, frame: number, run: { number }, cand: number, tolNum: number, tolDen: number): (boolean, number?)
	local ax, az = coordIn(W, frame, run[1])
	local kx, kz = coordIn(W, frame, cand)
	local dx, dz = kx - ax, kz - az
	if dx == 0 and dz == 0 then return false, nil end
	local best, bestPos = -1, nil
	for p = 2, #run do
		local qx, qz = coordIn(W, frame, run[p])
		local cr = dx * (qz - az) - dz * (qx - ax)
		if cr < 0 then cr = -cr end
		if cr > best then best, bestPos = cr, p end
	end
	if bestPos == nil then return false, nil end
	-- The tolerance is in CELLS, so it is scaled to sub-cell units to match.
	local tn = tolNum * SUBDIV
	local dd = dx * dx + dz * dz
	local fail = (best * best * tolDen * tolDen) > (tn * tn * dd)
	return fail, bestPos
end

--------------------------------------------------------------------------
-- Exploring
--------------------------------------------------------------------------

local function edgeKey(a: number, b: number): string
	return (a < b) and (a .. ":" .. b) or (b .. ":" .. a)
end

export type Run = {
	nodes: { number },
	reason: string,   -- "dirlock" | "corridor" | "guard" | "dead end" | "probe" | "fork"
}

-- Grow one run from `start`, having arrived from `cameFrom` (0 = nowhere).
--
-- AT A BRANCH there is no rule and no constant. Every candidate is explored in
-- turn, up to `probe` steps, under these same two rules; the one that survives
-- longest is the one the line continues along. A branch that immediately breaks
-- is a different line arriving, and it is simply not taken.
local function grow(W: World, start: number, cameFrom: number, c: any, probe: boolean?, walked: { [string]: boolean }?): Run
	-- THE RUN'S FRAME is the lattice of the part it started on. It is fixed for
	-- the life of the run, so the whole run is measured against one ruler.
	local frame = W.gid[start]
	local run = { start }
	local used = { false, false, false, false }
	local prev, cur = cameFrom, start
	local seen = { [start] = true }
	local budget = probe and c.probe or math.huge
	local steps = 0

	while steps < budget do
		local cands = {}
		for _, j in ipairs(W.nbr[cur]) do
			if j ~= prev and not seen[j] then
				-- An edge something already walked is not a way forward. Without
				-- this a closed boundary laps forever, re-finding the same
				-- corners: the run's own `seen` only stops it revisiting a node
				-- WITHIN one run, and each new run starts with a fresh one.
				local blocked = walked ~= nil and walked[edgeKey(cur, j)] or false
				if not blocked then cands[#cands + 1] = j end
			end
		end
		if #cands == 0 then return { nodes = run, reason = "dead end" } end

		local nxt: number
		if #cands == 1 then
			nxt = cands[1]
		elseif probe then
			-- A PROBE DOES NOT PROBE. It is measuring how far this option runs
			-- before the pattern breaks, and hitting a fork is itself a reason to
			-- stop measuring. Without this the recursion has no bottom -- it
			-- overflowed the stack on the first real run.
			return { nodes = run, reason = "fork" }
		else
			-- EXPLORE EACH ONE, take the one that goes furthest before breaking.
			local bestLen, bestJ = -1, cands[1]
			for _, j in ipairs(cands) do
				local r = grow(W, j, cur, c, true, walked)
				if #r.nodes > bestLen then bestLen, bestJ = #r.nodes, j end
			end
			nxt = bestJ
		end

		local ax, az = coordIn(W, frame, cur)
		local bx, bz = coordIn(W, frame, nxt)
		local dx, dz = bx - ax, bz - az

		-- RULE 1, before committing. A step within the frame's own part is
		-- exactly +/-SUBDIV or 0; a converted one can carry a unit or two of
		-- rounding, so anything under half a cell is not a direction.
		local dead = SUBDIV // 2
		local sx = (dx > dead) and 1 or ((dx < -dead) and -1 or 0)
		local sz = (dz > dead) and 1 or ((dz < -dead) and -1 or 0)
		if locked(used, sx, sz) then
			return { nodes = run, reason = "dirlock" }
		end

		-- RULE 2, against the whole run behind.
		local fail, bestPos = corridor(W, frame, run, nxt, c.tolNum, c.tolDen)
		if fail then
			local offset = (bestPos :: number) - 1
			if offset < 2 then return { nodes = run, reason = "guard" } end
			local cut = {}
			for i = 1, bestPos :: number do cut[i] = run[i] end
			return { nodes = cut, reason = "corridor" }
		end

		mark(used, sx, sz)
		run[#run + 1] = nxt
		seen[nxt] = true
		prev, cur = cur, nxt
		steps += 1
	end
	return { nodes = run, reason = "probe" }
end

Explore._grow = grow

export type Result = {
	corners: { Vector3 },
	runs: number,
	stats: { [string]: number },
	world: World,
}

-- Sweep the map. Every node is a legitimate place to start exploring from; a
-- node is only used as a seed once something has not already explored through
-- it, so the whole boundary gets covered without walking any of it twice.
--
-- The backward bootstrap is LineFit's and the reason is the same: an arbitrary
-- seed usually lands mid-run, and a forward-only pass would call that a corner.
-- So grow BACKWARD first, and start the real run from wherever that broke.
function Explore.corners(W: World, cfg: Config?): Result
	local c: any = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end

	-- WHAT MAKES THIS TERMINATE is edges, not nodes. Every edge of the boundary
	-- gets walked exactly once; a run refuses an edge already walked, so a closed
	-- boundary stops itself after one lap instead of re-finding the same corners
	-- forever. That was the first version's bug and it hung Studio for 120s.
	-- A TRIPWIRE, not a rule. Every run must claim at least one fresh edge, so
	-- the sweep cannot need more runs than there are edges. If it ever does, the
	-- termination invariant is broken and this stops in a second instead of
	-- wedging Studio, which it did once and which costs the user their session.
	local maxRuns = c.maxRuns or (W.n * 4)
	local walked: { [string]: boolean } = {}
	local touched = table.create(W.n, false)
	local corners: { Vector3 } = {}
	local stats: { [string]: number } = {}
	local runs = 0

	local function claim(run: { number })
		for i = 1, #run - 1 do
			walked[edgeKey(run[i], run[i + 1])] = true
			touched[run[i]] = true
		end
		touched[run[#run]] = true
	end

	for seed = 1, W.n do
		if not touched[seed] then
			-- BACKWARD BOOTSTRAP, LineFit's, and for its reason: an arbitrary seed
			-- usually lands mid-run, and starting forward from it would call that
			-- spot a corner. Grow backward under the same rules and start from
			-- wherever that broke. The bootstrap is thrown away, not claimed.
			local anchor = seed
			do
				local back = grow(W, seed, 0, c, false, walked)
				anchor = back.nodes[#back.nodes]
				-- THE ANCHOR IS A RULE BREAK TOO. It is where growth stopped
				-- walking BACKWARD, under these same two rules -- the same fact
				-- about the ground, found from the other side. Throwing it away
				-- silently dropped real 90-degree corners: measured, four corners
				-- picked out by hand were all anchors, and three of them were also
				-- where the lap came back round and dead-ended.
				if back.reason == "corridor" or back.reason == "dirlock" then
					corners[#corners + 1] = W.pos[anchor]
				end
			end

			local cur, from = anchor, 0
			while true do
				local r = grow(W, cur, from, c, false, walked)
				runs += 1
				if runs > maxRuns then
					error(string.format("Explore: %d runs on %d nodes -- a run is not claiming a fresh edge", runs, W.n))
				end
				stats[r.reason] = (stats[r.reason] or 0) + 1
				if #r.nodes < 2 then
					touched[cur] = true
					break
				end
				claim(r.nodes)
				local last = r.nodes[#r.nodes]
				-- A CORNER IS WHERE A RULE BROKE THE RUN, and nowhere else.
				-- `dead end` means only that the sweep had already walked
				-- everything ahead -- bookkeeping, not geometry -- and emitting
				-- there put a corner at the seam between two runs. That alone was
				-- 667 of 1475 runs and most of a 1142-corner count against the
				-- ring pipeline's 464.
				if r.reason == "corridor" or r.reason == "dirlock" then
					corners[#corners + 1] = W.pos[last]
				end
				from = r.nodes[#r.nodes - 1]
				cur = last
				-- Progress is guaranteed because every run claims at least one
				-- fresh edge, and there are finitely many edges.
			end
		end
	end

	return { corners = corners, runs = runs, stats = stats, world = W }
end

--------------------------------------------------------------------------
-- Looking at it
--------------------------------------------------------------------------

-- The ordinary node view, with the corners called out.
--
-- Drawn exactly the way `LocalGrid.visualizeClasses` draws a node: a flat tile
-- lying on the part's OWN face, oriented by that grid's axes, in the usual
-- palette -- red wall, blue dropoff, purple both. A node is a patch of ground,
-- so it is drawn as a patch of ground.
--
-- A corner is the SAME tile, green and a little wider. It is not a different
-- kind of thing and should not be a different kind of shape.
function Explore.visualize(res: Result, opts: any?, parent: Instance?): number
	if typeof(opts) == "Instance" then parent = opts :: Instance; opts = nil end
	local o = opts or {}
	local nodeW = o.nodeW or 0.9
	local cornerW = o.cornerW or 1.3
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then
		dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root
	end
	local old = dbg:FindFirstChild("Explore")
	if old then old:Destroy() end
	local f = Instance.new("Folder"); f.Name = "Explore"; f.Parent = dbg

	local WALL = Color3.fromRGB(255, 70, 70)
	local DROP = Color3.fromRGB(70, 160, 255)
	local BOTH = Color3.fromRGB(220, 90, 255)
	local CORNER = Color3.fromRGB(0, 255, 90)

	local W = res.world
	local step = W.step
	local lift = Vector3.new(0, 0.12, 0)

	-- A corner is identified by POSITION, because that is all `corners` carries.
	-- Rounded to a tenth of a stud so a node and its own corner cannot miss each
	-- other on a float comparison.
	local isCorner: { [string]: boolean } = {}
	local function key(p: Vector3): string
		return string.format("%.1f:%.1f:%.1f", p.X, p.Y, p.Z)
	end
	for _, p in ipairs(res.corners) do isCorner[key(p)] = true end

	local n = 0
	for i = 1, W.n do
		local p = W.pos[i]
		local corner = isCorner[key(p)]
		local col, w
		if corner then
			col, w = CORNER, cornerW
		else
			w = nodeW
			if W.wall[i] and W.drop[i] then col = BOTH
			elseif W.wall[i] then col = WALL
			else col = DROP end
		end
		local dot = Instance.new("Part")
		dot.Anchored = true; dot.CanCollide = false; dot.CanQuery = false; dot.CanTouch = false
		dot.Material = Enum.Material.SmoothPlastic
		dot.Color = col
		dot.Size = Vector3.new(w * step, 0.08, w * step)
		if W.u[i] and W.nrm[i] then
			dot.CFrame = CFrame.fromMatrix(p + lift, W.u[i] :: Vector3, W.nrm[i] :: Vector3)
		else
			dot.CFrame = CFrame.new(p + lift)
		end
		dot.Name = corner and "corner"
			or string.format("w%d_d%d", W.wallMask[i], W.dropMask[i])
		dot.Parent = f
		n += 1
	end
	return n
end

return Explore
