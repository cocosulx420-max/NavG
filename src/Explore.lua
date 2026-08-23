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
-- Geometry is judged in PLAN VIEW on the world XZ lattice at `step` -- integers,
-- so the corridor test stays exact. Y is carried on the output and is never part
-- of a decision. FLAGGED, NOT BLESSED: this is my choice, not a measured one.

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
}

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

	local pos, out = {}, {}
	for _, g in pairs(localData.grids) do
		for _, cell in ipairs(g.cells) do
			if cell.wall or cell.dropoff then
				pos[#pos + 1] = cell.pos
				out[#pos] = facing(g, cell)
			end
		end
	end

	local n = #pos
	local cellOf = table.create(n)
	for i = 1, n do
		cellOf[i] = {
			x = math.floor(pos[i].X / step + 0.5),
			z = math.floor(pos[i].Z / step + 0.5),
		}
	end

	local B = step * 1.5
	local hash: { [string]: { number } } = {}
	for i = 1, n do
		local p = pos[i]
		local k = math.floor(p.X/B) .. ":" .. math.floor(p.Y/B) .. ":" .. math.floor(p.Z/B)
		local b = hash[k]
		if not b then b = {}; hash[k] = b end
		b[#b + 1] = i
	end

	local R = step * c.radius
	local nbr = table.create(n)
	for i = 1, n do
		local p = pos[i]
		local bx, by, bz = math.floor(p.X/B), math.floor(p.Y/B), math.floor(p.Z/B)
		local a = {}
		for dx = -1, 1 do for dy = -1, 1 do for dz = -1, 1 do
			local b = hash[(bx+dx) .. ":" .. (by+dy) .. ":" .. (bz+dz)]
			if b then
				for _, j in ipairs(b) do
					if j ~= i and (pos[j] - p).Magnitude <= R then
						if (not out[i]) or (not out[j]) or out[i]:Dot(out[j]) > c.minDot then
							a[#a + 1] = j
						end
					end
				end
			end
		end end end
		nbr[i] = a
	end

	return { pos = pos, cell = cellOf, out = out, nbr = nbr, step = step, n = n }
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
local function corridor(W: World, run: { number }, cand: number, tolNum: number, tolDen: number): (boolean, number?)
	local a = W.cell[run[1]]
	local k = W.cell[cand]
	local dx, dz = k.x - a.x, k.z - a.z
	if dx == 0 and dz == 0 then return false, nil end
	local best, bestPos = -1, nil
	for p = 2, #run do
		local q = W.cell[run[p]]
		local cr = dx * (q.z - a.z) - dz * (q.x - a.x)
		if cr < 0 then cr = -cr end
		if cr > best then best, bestPos = cr, p end
	end
	if bestPos == nil then return false, nil end
	local dd = dx * dx + dz * dz
	local fail = (best * best * tolDen * tolDen) > (tolNum * tolNum * dd)
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

		local a, b = W.cell[cur], W.cell[nxt]
		local dx, dz = b.x - a.x, b.z - a.z

		-- RULE 1, before committing.
		if locked(used, dx, dz) then
			return { nodes = run, reason = "dirlock" }
		end

		-- RULE 2, against the whole run behind.
		local fail, bestPos = corridor(W, run, nxt, c.tolNum, c.tolDen)
		if fail then
			local offset = (bestPos :: number) - 1
			if offset < 2 then return { nodes = run, reason = "guard" } end
			local cut = {}
			for i = 1, bestPos :: number do cut[i] = run[i] end
			return { nodes = cut, reason = "corridor" }
		end

		mark(used, dx, dz)
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
			end

			local cur, from = anchor, 0
			while true do
				local r = grow(W, cur, from, c, false, walked)
				runs += 1
				stats[r.reason] = (stats[r.reason] or 0) + 1
				if #r.nodes < 2 then
					touched[cur] = true
					break
				end
				claim(r.nodes)
				local last = r.nodes[#r.nodes]
				corners[#corners + 1] = W.pos[last]
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

function Explore.visualize(res: Result, parent: Instance?): number
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then
		dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root
	end
	local old = dbg:FindFirstChild("Explore")
	if old then old:Destroy() end
	local f = Instance.new("Folder"); f.Name = "Explore"; f.Parent = dbg
	local n = 0
	for _, p in ipairs(res.corners) do
		local b = Instance.new("Part")
		b.Name = "corner"; b.Shape = Enum.PartType.Ball
		b.Size = Vector3.new(0.8, 0.8, 0.8); b.Position = p
		b.Anchored = true; b.CanCollide = false; b.CanQuery = false; b.CanTouch = false
		b.Material = Enum.Material.Neon; b.Color = Color3.fromRGB(0, 255, 120)
		b.Parent = f
		n += 1
	end
	return n
end

return Explore
