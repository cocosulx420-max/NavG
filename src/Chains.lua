--!strict
-- NavGen.Chains -- the boundary as a neighbour graph, with no rings in it.
--
-- THE BOUNDARY IS: a node with a wall or a dropoff. That is the whole
-- definition. `LocalGrid` already says the rest in its own words -- "neither
-- means the floor simply continues, WHETHER OR NOT IT CONTINUES ONTO A
-- DIFFERENT PART" -- and this stage finally takes it at its word.
--
-- There is no `seam` class here, because a seam is not a boundary. It was never
-- a node the ring trace found; it was a label the ring trace was forced to
-- invent, because a ring follows ONE part's rim and has to call the far side of
-- that rim something. 24% of the old boundary was seam, and every one of those
-- nodes was a place where the floor simply continues.
--
-- Everything the seam class caused went with it:
--   * 87 sealed islands, because each part's rim closed on itself
--   * 38 "corners" that were a turn in a seam
--   * 43 duplicate corners, one per lattice, needing a weld pass to pick between
--
-- WHAT A RING ACTUALLY PROVIDED was ORDER -- a sequence for the line fit to walk
-- along. Nothing else. So order is all this stage has to replace, and a chain of
-- degree-2 neighbours provides it directly. Measured on SmallMap: 5850 of 6046
-- boundary nodes (96.8%) have exactly two neighbours.
--
-- ADJACENCY, and the two rules in it.
--
--   SAME SIDE. Two boundary nodes chain together only if the void is on the same
--   side of both -- the dot of their facing directions is not strongly negative.
--   Without this the two faces of a thin wall fuse into one line, which is 1095
--   of the 1152 spurious branches on this map. The threshold is a PLATEAU, not a
--   tuning: anywhere in [-0.50, -0.10] gives identical output. It only matters
--   near -0.71, where opposite faces start re-fusing.
--
--   THE CHORD RULE. Diagonal neighbours have to be admitted or a boundary
--   running at 45 degrees is not a chain at all -- every node on it becomes its
--   own fragment, because its neighbours are at sqrt(2) and a 4-connected radius
--   cannot see them. But admitting them blindly links i to i+2 across every
--   staircase jog: 1376 branches instead of 78. So a diagonal link is dropped
--   when its two ends already share a 4-connected neighbour. That link is a
--   shortcut across a jog, not a link in the chain. No tolerance in it.
--
-- A CHAIN RUNS THROUGH A BRANCH, it does not stop at one. Cutting at every
-- branch looks principled and is not: it forced both ends of 257 open chains to
-- be vertices -- an open chain's endpoints are vertices by LineFit's contract --
-- and that alone doubled the corner count from 464 to 924 while producing 163
-- two-node stubs. A branch is not a corner. It is a place where a THIRD line
-- arrives, and the two lines that were already there carry straight on.
--
-- So at a branch, the arriving edge is paired with the edge that most nearly
-- continues it, and the walk goes through. An edge with no such continuation --
-- the plank actually running into the ground -- is the only thing that ends a
-- chain there.
--
-- THE FIT IS IN PLAN VIEW. `LineFit` is 2D and integer by contract, and a chain
-- that climbs a staircase is neither planar nor on any part's lattice -- there
-- is no common integer grid across parts in 3D. But in PLAN VIEW there is one:
-- the world XZ lattice at `step`. That is also the right frame on its own
-- merits, because a staircase's boundary in plan view is a straight line, which
-- is exactly what it should fit as. Y is carried along and put back on the
-- output vertex; it is never part of the decision.

local LineFit = require(script.Parent.LineFit)

local Chains = {}

local DIR8 = {
	{ 1, 0 }, { 1, 1 }, { 0, 1 }, { -1, 1 },
	{ -1, 0 }, { -1, -1 }, { 0, -1 }, { 1, -1 },
}

export type Config = {
	step: number?,
	minDot: number?,       -- same-side threshold; plateau is [-0.50, -0.10]
	nearMult: number?,     -- 4-connected radius, in steps
	diagMult: number?,     -- 8-connected radius, in steps
	throughDot: number?,   -- how straight an edge pair must be to walk through
}

local DEFAULT = {
	minDot = -0.30,
	nearMult = 1.05,
	diagMult = 1.45,
	throughDot = -0.5,
}

export type Node = {
	pos: Vector3,
	out: Vector3?,     -- the direction the boundary faces, in world space
	wall: boolean,
	drop: boolean,
}

export type Chain = {
	nodes: { number },  -- indices into `Chains.Result.nodes`, in order
	closed: boolean,
}

export type Stats = {
	nodes: number, edges: number,
	deg0: number, deg1: number, deg2: number, deg3plus: number,
	chains: number, cycles: number, open: number,
	junctions: number, fragments: number,
}

export type Result = {
	nodes: { Node },
	adj: { { number } },
	chains: { Chain },
	stats: Stats,
	step: number,
}

-- The direction the boundary faces: the average of the directions in which this
-- node has a wall or a dropoff, taken on the OWNING GRID'S axes and returned in
-- world space, so nodes from differently-rotated parts are directly comparable.
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

-- Build the graph and walk it into ordered chains.
function Chains.build(localData: any, cfg: Config?): Result
	local c: any = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	local step = c.step or (localData.config and localData.config.step) or 1

	local nodes: { Node } = {}
	for _, g in pairs(localData.grids) do
		for _, cell in ipairs(g.cells) do
			if cell.wall or cell.dropoff then
				nodes[#nodes + 1] = {
					pos = cell.pos, out = facing(g, cell),
					wall = cell.wall or false, drop = cell.dropoff or false,
				}
			end
		end
	end

	-- Spatial hash in WORLD space. Grid identity plays no part in adjacency;
	-- that is the whole point of this stage.
	local B = step * 1.5
	local hash: { [string]: { number } } = {}
	local function bucketOf(p: Vector3): (number, number, number)
		return math.floor(p.X / B), math.floor(p.Y / B), math.floor(p.Z / B)
	end
	for i, n in ipairs(nodes) do
		local bx, by, bz = bucketOf(n.pos)
		local k = bx .. ":" .. by .. ":" .. bz
		local b = hash[k]
		if not b then b = {}; hash[k] = b end
		b[#b + 1] = i
	end

	local nearR, diagR = step * c.nearMult, step * c.diagMult
	local near: { { number } } = {}
	local diag: { { number } } = {}
	for i, n in ipairs(nodes) do
		near[i], diag[i] = {}, {}
		local bx, by, bz = bucketOf(n.pos)
		for dx = -1, 1 do for dy = -1, 1 do for dz = -1, 1 do
			local b = hash[(bx + dx) .. ":" .. (by + dy) .. ":" .. (bz + dz)]
			if b then
				for _, j in ipairs(b) do
					if j ~= i then
						local q = nodes[j]
						local d = (q.pos - n.pos).Magnitude
						if d <= diagR then
							local ok = (not n.out) or (not q.out) or n.out:Dot(q.out) > c.minDot
							if ok then
								if d <= nearR then
									near[i][#near[i] + 1] = j
								else
									diag[i][#diag[i] + 1] = j
								end
							end
						end
					end
				end
			end
		end end end
	end

	-- THE CHORD RULE.
	local nearSet: { { [number]: boolean } } = {}
	for i, a in ipairs(near) do
		local s = {}
		for _, j in ipairs(a) do s[j] = true end
		nearSet[i] = s
	end
	local adj: { { number } } = {}
	for i = 1, #nodes do
		local a = {}
		for _, j in ipairs(near[i]) do a[#a + 1] = j end
		for _, j in ipairs(diag[i]) do
			local chord = false
			for k in pairs(nearSet[i]) do
				if nearSet[j][k] then chord = true break end
			end
			if not chord then a[#a + 1] = j end
		end
		adj[i] = a
	end

	local st: Stats = {
		nodes = #nodes, edges = 0, deg0 = 0, deg1 = 0, deg2 = 0, deg3plus = 0,
		chains = 0, cycles = 0, open = 0, junctions = 0, fragments = 0,
	}
	for i = 1, #nodes do
		local d = #adj[i]
		st.edges += d
		if d == 0 then st.deg0 += 1
		elseif d == 1 then st.deg1 += 1
		elseif d == 2 then st.deg2 += 1
		else st.deg3plus += 1; st.junctions += 1 end
	end
	st.edges = st.edges // 2

	-- PAIR THE EDGES AT EVERY BRANCH. Two edges continue each other when the
	-- walk arrives along one and leaves along the other in nearly the same
	-- direction, so the test is on the dot of the two outgoing directions:
	-- -1 is dead straight through. Greedy, most-straight pair first, and an
	-- edge stays unpaired if nothing continues it.
	--
	-- `throughDot` is the one number here. It is not doing fine work -- a
	-- continuation is near -1 and an arriving third line is near 0 -- so
	-- anywhere in the middle behaves the same.
	local throughDot: number = c.throughDot or -0.5
	local pairAt: { { [number]: number } } = {}
	for i = 1, #nodes do
		local a = adj[i]
		pairAt[i] = {}
		if #a >= 3 then
			local dirs = {}
			for k, j in ipairs(a) do
				local d = nodes[j].pos - nodes[i].pos
				dirs[k] = (d.Magnitude > 1e-6) and d.Unit or Vector3.zero
			end
			local cands = {}
			for x = 1, #a do
				for y = x + 1, #a do
					cands[#cands + 1] = { x = x, y = y, dot = dirs[x]:Dot(dirs[y]) }
				end
			end
			table.sort(cands, function(p1, p2) return p1.dot < p2.dot end)
			local taken = {}
			for _, cd in ipairs(cands) do
				if cd.dot < throughDot and not taken[cd.x] and not taken[cd.y] then
					taken[cd.x], taken[cd.y] = true, true
					pairAt[i][a[cd.x]] = a[cd.y]
					pairAt[i][a[cd.y]] = a[cd.x]
				end
			end
		end
	end

	-- Where does the walk go, arriving at `cur` from `prev`?
	local function stepFrom(prev: number, cur: number): number?
		local a = adj[cur]
		if #a == 2 then
			local nxt = a[1]
			if nxt == prev then nxt = a[2] end
			return nxt
		elseif #a >= 3 then
			return pairAt[cur][prev]
		end
		return nil
	end

	-- WALK. Every edge belongs to exactly one chain. Start from the edges that
	-- nothing continues -- a degree-1 end, or an unpaired edge at a branch --
	-- then whatever is left over is a cycle.
	local seenEdge: { [string]: boolean } = {}
	local function edgeKey(a: number, b: number): string
		return (a < b) and (a .. ":" .. b) or (b .. ":" .. a)
	end
	local chains: { Chain } = {}

	local function walkFrom(i: number, first: number, closed: boolean)
		seenEdge[edgeKey(i, first)] = true
		local run = { i, first }
		local prev, cur = i, first
		while true do
			local nxt = stepFrom(prev, cur)
			if nxt == nil then break end
			local k = edgeKey(cur, nxt)
			if seenEdge[k] then break end
			seenEdge[k] = true
			prev, cur = cur, nxt
			if closed and cur == i then break end
			run[#run + 1] = cur
		end
		chains[#chains + 1] = { nodes = run, closed = closed }
		if closed then st.cycles += 1 else
			st.open += 1
			if #run <= 3 then st.fragments += 1 end
		end
	end

	for i = 1, #nodes do
		local a = adj[i]
		if #a == 1 then
			if not seenEdge[edgeKey(i, a[1])] then walkFrom(i, a[1], false) end
		elseif #a >= 3 then
			for _, j in ipairs(a) do
				-- an edge nothing continues is where a chain genuinely ends
				if pairAt[i][j] == nil and not seenEdge[edgeKey(i, j)] then
					walkFrom(i, j, false)
				end
			end
		end
	end
	-- Anything still unwalked has no loose end anywhere on it: a cycle.
	for i = 1, #nodes do
		for _, j in ipairs(adj[i]) do
			if not seenEdge[edgeKey(i, j)] then walkFrom(i, j, true) end
		end
	end
	st.chains = #chains

	return { nodes = nodes, adj = adj, chains = chains, stats = st, step = step }
end

--------------------------------------------------------------------------
-- The fit
--------------------------------------------------------------------------

export type FitResult = {
	chain: Chain,
	vertices: { Vector3 },   -- world positions of the fitted corners
	indices: { number },     -- and where they sit in `chain.nodes`
}

-- Fit every chain in PLAN VIEW on the world XZ lattice. See the header: there is
-- no common integer grid across parts in 3D, there is one in plan, and plan is
-- the frame a staircase should be judged in anyway.
function Chains.fit(res: Result, cfg: any?): ({ FitResult }, { number })
	local step = res.step
	local out: { FitResult } = {}
	local verts = 0
	for _, ch in ipairs(res.chains) do
		local cells = table.create(#ch.nodes)
		for i, ni in ipairs(ch.nodes) do
			local p = res.nodes[ni].pos
			cells[i] = {
				x = math.floor(p.X / step + 0.5),
				z = math.floor(p.Z / step + 0.5),
			}
		end
		local c: any = { closed = ch.closed }
		if cfg then for k, v in pairs(cfg) do c[k] = v end end
		local fit = LineFit.fit(cells, c)
		local pts = table.create(#fit.vertices)
		for i, vi in ipairs(fit.vertices) do
			pts[i] = res.nodes[ch.nodes[vi]].pos
		end
		verts += #pts
		out[#out + 1] = { chain = ch, vertices = pts, indices = fit.vertices }
	end
	return out, { verts }
end

-- Every fitted corner, as world points. This is what goes to `HandMarks.score`.
function Chains.cornerPoints(fits: { FitResult }): { Vector3 }
	local pts = {}
	for _, f in ipairs(fits) do
		for _, p in ipairs(f.vertices) do pts[#pts + 1] = p end
	end
	return pts
end

--------------------------------------------------------------------------
-- Looking at it
--------------------------------------------------------------------------

-- Chains in dim navy, fitted corners as white balls, branch nodes magenta and
-- chain ends orange -- so what is a corner and what is merely where a chain
-- stopped stay visually separate.
function Chains.visualize(res: Result, fits: { FitResult }?, parent: Instance?): number
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then
		dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root
	end
	local old = dbg:FindFirstChild("Chains")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "Chains"; folder.Parent = dbg

	local DIM = Color3.fromRGB(60, 70, 90)
	local JUNC = Color3.fromRGB(255, 0, 255)
	local ENDP = Color3.fromRGB(255, 150, 0)
	local CORNER = Color3.fromRGB(255, 255, 255)
	local n = 0

	local function ball(p: Vector3, s: number, col: Color3, name: string)
		local b = Instance.new("Part")
		b.Name = name; b.Shape = Enum.PartType.Ball
		b.Size = Vector3.new(s, s, s); b.Position = p
		b.Anchored = true; b.CanCollide = false; b.CanQuery = false; b.CanTouch = false
		b.Material = Enum.Material.Neon; b.Color = col; b.Parent = folder
		n += 1
	end
	local function bar(a: Vector3, b: Vector3, t: number, col: Color3)
		local d = b - a
		local L = d.Magnitude
		if L < 1e-3 then return end
		local p = Instance.new("Part")
		p.Name = "link"; p.Size = Vector3.new(t, t, L)
		p.CFrame = CFrame.lookAt(a + d * 0.5, b)
		p.Anchored = true; p.CanCollide = false; p.CanQuery = false; p.CanTouch = false
		p.Material = Enum.Material.Neon; p.Color = col; p.Parent = folder
		n += 1
	end

	for _, ch in ipairs(res.chains) do
		for i = 1, #ch.nodes - 1 do
			bar(res.nodes[ch.nodes[i]].pos, res.nodes[ch.nodes[i + 1]].pos, 0.12, DIM)
		end
		if ch.closed and #ch.nodes > 2 then
			bar(res.nodes[ch.nodes[#ch.nodes]].pos, res.nodes[ch.nodes[1]].pos, 0.12, DIM)
		end
	end
	for i, node in ipairs(res.nodes) do
		local d = #res.adj[i]
		if d >= 3 then ball(node.pos, 0.8, JUNC, "junction")
		elseif d <= 1 then ball(node.pos, 0.6, ENDP, "chainEnd")
		else ball(node.pos, 0.22, DIM, "node") end
	end
	if fits then
		for _, p in ipairs(Chains.cornerPoints(fits)) do
			ball(p, 0.7, CORNER, "corner")
		end
	end
	return n
end

return Chains
