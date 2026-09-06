--!strict
-- NVGN.NodeWalk -- walkability by a brick test against the NODES themselves.
--
-- This is deliberately NOT WalkLocal. WalkLocal probes against real BaseParts;
-- this probes against the voxelised node set. Testing node-against-node is what
-- recovers thin trim and seams that a solid BasePart occludes but its own
-- voxelisation does not.
--
-- Pipeline:
--   SVO per part at LEAF  ->  expand every node to uniform LEAF cells
--   ->  stand a W x H x W brick LIFT above each cell's own face
--   ->  reject if it touches ANY other cell
--   ->  purge isolated / thin / small
--
-- The probe axis is the NODE'S FACE NORMAL (SVOLocal.nodeFaceNormal), not a
-- "most skyward part axis" guess. That distinction is the whole ballgame on
-- sloped surfaces: with the axis guess the roofs of case3 scored 156/15325
-- walkable, with the face normal they score 3647/15325. On a grid-aligned
-- floor the two vectors are identical, so flat ground is unaffected.

local SVOLocal = require(script.Parent.SVOLocal)

local NodeWalk = {}
local UP = Vector3.new(0, 1, 0)

-- Tuned. These took real work to find -- do not casually change them.
local DEFAULT = {
	leaf = 0.5,        -- SVO leaf size, and the uniform cell size after expansion
	pad = 0.01,        -- SVOLocal contact pad
	blockW = 0.3,      -- brick width and thickness
	lift = 0.1,        -- gap under EVERY brick, all modes

	-- Max surface slope an NPC will walk, in degrees from world up. Anything
	-- steeper is rejected BEFORE the cascade, which is both the correct answer
	-- and a saving: a wall face is 90 deg and would otherwise cost up to four
	-- overlap queries to reject. Measured off the cell's own normal, so it is
	-- only as good as that normal -- see the world-Y rescue in `cells`.
	maxSlope = 65,

	-- Locomotion modes, tried in order. First fit wins, so a cell an NPC can
	-- stand on costs ONE probe and only a fully blocked cell costs three.
	--
	-- `h` is the brick itself; the 0.1 lift means the real clearance demanded is
	-- h + lift = 5.1 / 3.6 / 2.1. That gap is deliberate twice over: it keeps an
	-- NPC's head off the ceiling, and it stops cells that overlap at the very
	-- bottom of the face from registering as a hit.
	--
	-- The last rung is the ORIGINAL 1.4 gate and it stays. `walkable = false`
	-- means it never enters the navmesh -- it is the floor of the whole system:
	-- no NPC of any size may ever occupy less than 1.5 studs of clearance. Its
	-- job now is diagnostic. A cell that clears 1.4 but fails prone is a real
	-- surface that is too tight for anything, and it is drawn BLACK so those
	-- near-misses stay visible instead of silently vanishing from the bake.
	modes = {
		{ name = "stand",  h = 5.0 },
		{ name = "crouch", h = 3.5 },
		{ name = "prone",  h = 2.0 },
		{ name = "gate",   h = 1.4, walkable = false },
	},

	minNbr = 2,        -- pass 1: fewer in-plane neighbours than this = isolated
	patch = 2,         -- pass 2: must sit in a full patch x patch in-plane block
	minRegion = 6,     -- pass 3: components smaller than this are dropped

	-- Stage 3: a region below this many cells is not worth a polygon. Regions are
	-- strictly coplanar, so a small one is NOT mergeable into a big neighbour --
	-- the neighbour is on a different plane by definition. Dropping is the only
	-- honest option. 6 cells at leaf 0.5 is 1.5 studs^2.
	minRegionCells = 6,

	-- Agent footprint. The brick is a CLEARANCE probe -- 0.3 wide, deliberately
	-- thin, so it measures headroom and nothing else. This is the BODY: an NPC is
	-- about 2 studs across (HumanoidRootPart is 2 x 2 x 1). A region has to be
	-- able to house that box somewhere, at its own mode's height, or nothing can
	-- actually stand in it however much headroom the thin brick found.
	agentWidth = 2.0,
	requireFit = true,   -- cull regions that cannot house the footprint anywhere

	-- Face normalization (unions and meshes only). faceTol grows a face against
	-- its SEED normal; faceAdopt is the looser gate for pulling an on-plane cell
	-- in, since a cell at a step edge has a poor local normal; facePlane is how
	-- far off the plane a cell may sit and still belong to it.
	faceRadius = 1.5,
	faceTol = 15,
	faceAdopt = 25,
	facePlane = 0.35,
	-- a face may be small: 6 cells at leaf 0.5 is the same floor minRegionCells
	-- uses. 20 silently skipped every face on a 4-stud union.
	minFaceCells = 6,

	-- where the face plane sits among its cells' offsets, as a quantile. Below 1.0
	-- the plane cuts into the surface and the probe buries whatever is above it,
	-- so this stays at the outermost cell: a floating face is a cosmetic error, a
	-- buried one disappears.
	faceSeat = 1.0,

	stepPlane = 0.75,  -- component link: max in-plane separation
	stepNormal = 0.6,  -- component link: max offset ALONG the normal (one step)
	tol = 0.2,         -- lattice probe tolerance when asking "is there a cell here"

	maxCells = 250000, -- refuse to materialise more than this (see notes below)
}

-- Guard rail. Expanding SVO nodes to leaf size is safe on detailed geometry and
-- catastrophic on large flat parts: a 16-stud baseplate node is 32^3 = 32768
-- cells, and a Baseplate collapses to ~16k such nodes. Always scope to a model,
-- never the whole workspace, and let maxCells stop you if you forget.

-- How far above a face the tallest brick reaches. Anything that culls or chunks
-- cells MUST use this, not a hardcoded number: a cell this far from a surface
-- can still block the standing probe, and dropping it invents clearance that
-- does not exist.
function NodeWalk.reach(cfg): number
	local c = cfg or DEFAULT
	local tallest = 0
	for _, m in ipairs(c.modes or DEFAULT.modes) do tallest = math.max(tallest, m.h) end
	return (c.lift or DEFAULT.lift) + tallest
end

local function merged(cfg)
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

-- Only used when nodeFaceNormal declines (interior cells, and slanted meshes
-- where it reports a bounding-box direction). Picks the most skyward of the
-- part's six axis directions -- note this can never exceed 54.7 degrees of
-- tilt, so it cannot describe a genuinely steep surface.
local function fallbackUp(cf: CFrame): Vector3
	local best, bestD = UP, -math.huge
	for _, a in ipairs({ cf.RightVector, cf.UpVector, cf.LookVector }) do
		for _, sign in ipairs({ 1, -1 }) do
			local n = a * sign
			local d = n:Dot(UP)
			if d > bestD then bestD = d; best = n end
		end
	end
	return best
end

-- The two in-plane lattice directions for a cell, taken from the cell's OWN
-- frame. Using world axes here deletes every sloped surface wholesale.
local function basis(c): (Vector3?, Vector3?)
	local n = c.up
	local u, v
	-- a normalized cell carries its FACE's grid axes in `frame`; those are the
	-- in-plane directions, and the part's own axes no longer describe the surface
	local f = c.frame or c.cf
	for _, a in ipairs({ f.RightVector, f.UpVector, f.LookVector }) do
		if math.abs(a:Dot(n)) < 0.5 then
			if not u then u = a else v = a end
		end
	end
	return u, v
end

function NodeWalk.collect(target: Instance): {BasePart}
	local parts = {}
	for _, d in ipairs(target:GetDescendants()) do
		if d:IsA("BasePart") and d.CanCollide then table.insert(parts, d) end
	end
	return parts
end

-- Build the SVO and flatten it to uniform `leaf` cells, each tagged with its
-- source part and its face normal.
function NodeWalk.cells(parts: {BasePart}, cfg)
	local c = merged(cfg)
	local trees = SVOLocal.fromParts(parts, c.leaf, c.pad)
	local out, maxEdge = {}, 0
	for part, tree in pairs(trees) do
		tree:forEachNode(function(cf: CFrame, edge: number)
			maxEdge = math.max(maxEdge, edge)
			local n = math.max(1, math.round(edge / c.leaf))
			if n == 1 then
				table.insert(out, { cf = cf, part = part, tree = tree })
			else
				local o = -edge * 0.5 + c.leaf * 0.5
				for i = 0, n - 1 do for j = 0, n - 1 do for k = 0, n - 1 do
					table.insert(out, {
						cf = cf * CFrame.new(o + i*c.leaf, o + j*c.leaf, o + k*c.leaf),
						part = part, tree = tree,
					})
				end end end
			end
		end)
	end
	-- A BOX AXIS IS ONLY THE SURFACE ON A BOX. nodeFaceNormal can name exactly one
	-- of the six BOUNDING-BOX axes. On a part whose collision shape is its box
	-- that is the surface. On a union or a mesh it is a guess, and the dangerous
	-- case is not the one that returns nil -- it is the one that returns a
	-- confident wrong answer, because realNormal is then true and every later
	-- safeguard stands down.
	--
	-- Measured on case5's Union: its box is rotated (Up = -0.747, 0.656, 0.105)
	-- but its geometry is not -- there is a flat horizontal face at y 8.362, and
	-- raycasts onto it return (0, 1, 0) everywhere. nodeFaceNormal handed back the
	-- box's Up for all 33 cells there, 49 degrees off the real surface. That wrong
	-- normal survives the slope filter (49 < maxSlope), then buckets the cells onto
	-- a tilted plane in `regions` and gives Contour a lattice basis skewed through
	-- the surface, which collapsed 22 of the 33 cells onto occupied slots.
	--
	-- So on a non-exact part the box answer is not evidence. Treat it as no answer
	-- and let the world-Y rescue below decide, which is exactly the test that
	-- settles it: nothing above in the column means it IS a top surface. A cell the
	-- rescue cannot vouch for stays unproven and is refused rather than guessed.
	local fellBack, nilCells, distrusted = 0, {}, 0
	for _, cell in ipairs(out) do
		local n = SVOLocal.nodeFaceNormal(cell.part, cell.cf, c.leaf)
		if n and cell.tree and cell.tree.exact == false then
			n = nil; distrusted += 1
		end
		cell.realNormal = n ~= nil
		if not n then
			n = fallbackUp(cell.cf); fellBack += 1
			table.insert(nilCells, cell)
		end
		cell.up = n
		cell.face = cell.cf.Position + n * (c.leaf * 0.5)
	end

	-- WORLD-Y TOP RESCUE.
	-- nodeFaceNormal can only name one of the six BOUNDING-BOX axes, so a part
	-- whose geometry is rotated relative to its box gets no normal at all on a
	-- face that is genuinely flat. The tell is world Y: if nothing sits above a
	-- cell in its own vertical column, it IS a top surface, whatever the box
	-- says. Only cells that got no real normal are considered, so the 99.98%
	-- that nodeFaceNormal already handles are untouched.
	local rescued = 0
	if #nilCells > 0 then
		local COL = c.leaf * 0.5
		local col = {}
		local function key(p) return math.floor(p.X / c.leaf) * 65536 + math.floor(p.Z / c.leaf) end
		for _, cell in ipairs(out) do
			local k = key(cell.cf.Position)
			local b = col[k]; if not b then b = {}; col[k] = b end
			table.insert(b, cell)
		end
		for _, cell in ipairs(nilCells) do
			local p = cell.cf.Position
			local clear = true
			for dx = -1, 1 do
				for dz = -1, 1 do
					local b = col[key(p + Vector3.new(dx * c.leaf, 0, dz * c.leaf))]
					if b then
						for _, q in ipairs(b) do
							local d = q.cf.Position - p
							if q ~= cell and math.abs(d.X) < COL and math.abs(d.Z) < COL and d.Y > 0.1 then
								clear = false; break
							end
						end
					end
					if not clear then break end
				end
				if not clear then break end
			end
			if clear then
				-- the cell is a rotated cube, so its top along world Y is the
				-- projection of its half-extents, not simply +leaf/2
				local cf = cell.cf
				local extY = (c.leaf * 0.5) * (math.abs(cf.RightVector.Y)
					+ math.abs(cf.UpVector.Y) + math.abs(cf.LookVector.Y))
				cell.up = UP
				cell.face = Vector3.new(cf.Position.X, cf.Position.Y + extY, cf.Position.Z)
				cell.worldTop = true
				-- Re-seat the node's own frame so its top face IS world up -- but only
				-- the up axis is corrected. The cells lie on the PART's lattice, so the
				-- horizontal axes must keep the part's yaw or the node stops lining up
				-- with the surface it belongs to (and with its own neighbours). Take
				-- whichever part axis is most horizontal, flatten it, and use it as X.
				--
				-- `cf` itself stays as the SVO produced it: that is the volume this
				-- cell occupies as a BLOCKER for other probes, and rotating it would
				-- move solid matter around.
				local flattest, bestDot = nil, math.huge
				for _, a in ipairs({ cf.RightVector, cf.UpVector, cf.LookVector }) do
					local d = math.abs(a:Dot(UP))
					if d < bestDot then bestDot = d; flattest = a end
				end
				local horiz = flattest - UP * flattest:Dot(UP)
				local right = (horiz.Magnitude > 1e-4) and horiz.Unit or Vector3.xAxis
				cell.frame = CFrame.fromMatrix(cf.Position, right, UP)
				rescued += 1
			end
		end
	end
	return out, { maxEdge = maxEdge, fellBack = fellBack, rescued = rescued,
	              distrusted = distrusted }
end

-- The brick test. Materialises the cells as query-only proxies, probes each,
-- and tears the proxies down again. Returns only the cells that came back clear.
function NodeWalk.probe(cells, cfg, state)
	local c = merged(cfg)
	local reach = NodeWalk.reach(c)

	state = state or {}
	if not state.started then
		state.started = true
		state.cursor = 0
		state.walk, state.gated = {}, {}
		state.counts = {}
		for _, m in ipairs(c.modes) do state.counts[m.name] = 0 end
		state.probesRun, state.steep, state.buried, state.unproven = 0, 0, 0, 0
		state.fits = 0

		-- CHUNK PLAN. Materialising every cell at once is what kills Studio: on
		-- case5 that is 2.2M proxies, roughly 9 GB of Part. Split the model into
		-- spatial chunks and only stand up one chunk's proxies at a time. Each
		-- chunk carries a HALO of `reach` studs so a cell at a boundary still sees
		-- everything that could block its tallest brick.
		local mn = Vector3.new(math.huge, math.huge, math.huge)
		local mx = Vector3.new(-math.huge, -math.huge, -math.huge)
		for _, cell in ipairs(cells) do
			local p = cell.cf.Position
			mn = Vector3.new(math.min(mn.X,p.X), math.min(mn.Y,p.Y), math.min(mn.Z,p.Z))
			mx = Vector3.new(math.max(mx.X,p.X), math.max(mx.Y,p.Y), math.max(mx.Z,p.Z))
		end
		-- aim for roughly half the proxy budget per chunk
		local want = math.max(1, math.ceil(#cells / math.max(1, c.maxCells * 0.5)))
		local n = math.max(1, math.ceil(math.sqrt(want)))
		local spanX = (mx.X - mn.X) / n + 0.001
		local spanZ = (mx.Z - mn.Z) / n + 0.001
		local probeList, blockList = {}, {}
		for i = 0, n*n - 1 do probeList[i] = {}; blockList[i] = {} end
		for idx, cell in ipairs(cells) do
			local p = cell.cf.Position
			local ix = math.clamp(math.floor((p.X - mn.X)/spanX), 0, n-1)
			local iz = math.clamp(math.floor((p.Z - mn.Z)/spanZ), 0, n-1)
			table.insert(probeList[iz*n + ix], idx)
			local x0 = ((p.X - (mn.X + ix*spanX)) < reach) and math.max(0, ix-1) or ix
			local x1 = (((mn.X + (ix+1)*spanX) - p.X) < reach) and math.min(n-1, ix+1) or ix
			local z0 = ((p.Z - (mn.Z + iz*spanZ)) < reach) and math.max(0, iz-1) or iz
			local z1 = (((mn.Z + (iz+1)*spanZ) - p.Z) < reach) and math.min(n-1, iz+1) or iz
			for x = x0, x1 do for z = z0, z1 do table.insert(blockList[z*n + x], idx) end end
		end
		local peak = 0
		for i = 0, n*n - 1 do peak = math.max(peak, #blockList[i]) end
		state.probeList, state.blockList, state.nchunk, state.peak = probeList, blockList, n*n, peak
	end

	local orphan = workspace:FindFirstChild("__nvgn_proxies")
	if orphan then orphan:Destroy() end

	local proxy = Instance.new("Part")
	proxy.Size = Vector3.new(c.leaf, c.leaf, c.leaf)
	proxy.Anchored = true
	proxy.CanCollide, proxy.CanTouch = false, false
	proxy.CanQuery = true
	proxy.Transparency = 1
	proxy.CastShadow = false

	-- one brick per mode, built once and reused -- resizing a Part per probe is
	-- far more expensive than keeping four around
	local bricks, bodies = {}, {}
	for i, m in ipairs(c.modes) do
		local b = Instance.new("Part")
		b.Size = Vector3.new(c.blockW, m.h, c.blockW)
		b.Anchored = true
		b.CanCollide, b.CanQuery, b.CanTouch = false, false, false
		b.Transparency = 1
		b.Parent = workspace
		bricks[i] = b
		-- the same height, but the width of an actual body
		local g = b:Clone()
		g.Size = Vector3.new(c.agentWidth, m.h, c.agentWidth)
		g.Parent = workspace
		bodies[i] = g
	end

	local cosMax = math.cos(math.rad(c.maxSlope))
	local t0 = os.clock()
	local budget = c.budget or math.huge

	while state.cursor < state.nchunk and (os.clock() - t0) < budget do
		local ci = state.cursor
		local pl, bl = state.probeList[ci], state.blockList[ci]
		if #pl > 0 then
			local holder = Instance.new("Folder")
			holder.Name = "__nvgn_proxies"
			for _, idx in ipairs(bl) do
				local p = proxy:Clone()
				p.CFrame = cells[idx].cf
				p.Parent = holder
			end
			holder.Parent = workspace

			local op = OverlapParams.new()
			op.FilterType = Enum.RaycastFilterType.Include
			op.FilterDescendantsInstances = { holder }
			op.RespectCanCollide = false
			op.MaxParts = 1

			for _, idx in ipairs(pl) do
				local cell = cells[idx]
				local up = cell.up

				-- 1. buried: something of the cell's OWN part sits on its face. A free
				--    tree walk, and it removes ~80% of cells before any spatial query.
				if cell.tree and cell.tree:containsPoint(cell.face + up * 0.05) then
					cell.buried = true
					state.buried += 1

				-- 2. too steep to stand on at any clearance
				elseif up:Dot(UP) < cosMax then
					cell.steep = true
					state.steep += 1

				-- 3. FAIL CLOSED. No real face normal and the world-Y test did not
				--    rescue it, so its "up" is a guess. fallbackUp can never exceed
				--    54.74 deg, so a guessed normal silently reads as walkable no
				--    matter how steep the surface really is -- that invents floor on
				--    a wall. Refuse rather than guess.
				elseif (not cell.realNormal) and (not cell.worldTop) then
					cell.unproven = true
					state.unproven += 1

				else
					local seed = (math.abs(up:Dot(cell.cf.UpVector)) > 0.99)
						and cell.cf.LookVector or cell.cf.UpVector
					local right = up:Cross(seed).Unit
					-- cascade: stand, crouch, prone, then the 1.4 gate. First fit wins.
					for i, m in ipairs(c.modes) do
						local brick = bricks[i]
						brick.CFrame = CFrame.fromMatrix(cell.face + up * (c.lift + m.h * 0.5), right, up)
						state.probesRun += 1
						if #workspace:GetPartsInPart(brick, op) == 0 then
							cell.mode = m.name
							cell.modeIndex = i
							cell.clearance = c.lift + m.h
							cell.walkable = (m.walkable ~= false)
							state.counts[m.name] += 1
							if cell.walkable then
								-- can a BODY stand here, not just a thin pole? One extra
								-- query, same centre, same height, agent width.
								local g = bodies[i]
								g.CFrame = brick.CFrame
								cell.fits = #workspace:GetPartsInPart(g, op) == 0
								if cell.fits then state.fits += 1 end
								table.insert(state.walk, cell)
							else
								table.insert(state.gated, cell)
							end
							break
						end
					end
				end
			end
			holder:Destroy()
		end
		state.cursor += 1
	end

	for _, b in ipairs(bricks) do b:Destroy() end
	for _, b in ipairs(bodies) do b:Destroy() end
	state.done = state.cursor >= state.nchunk

	return state.walk, {
		counts = state.counts, probesRun = state.probesRun, cells = #cells,
		gated = state.gated, steep = state.steep, buried = state.buried,
		unproven = state.unproven, chunks = state.nchunk, peak = state.peak,
		fits = state.fits, done = state.done, cursor = state.cursor,
	}, state
end

-- Purge isolated cells, thin bands, and small patches. Deliberately NOT a
-- reachability filter: a disconnected island may be genuinely reachable by
-- jumping, so islands are kept and only unstandable geometry is removed.
function NodeWalk.purge(walk, cfg)
	local c = merged(cfg)
	local n = #walk
	local alive = table.create(n, true)
	for i = 1, n do alive[i] = true end

	local CELLSZ = 1.0
	local hash = {}
	local function key(x, y, z) return x * 73856093 + y * 19349663 + z * 83492791 end
	for i, cell in ipairs(walk) do
		local k = key(math.floor(cell.face.X/CELLSZ), math.floor(cell.face.Y/CELLSZ), math.floor(cell.face.Z/CELLSZ))
		local b = hash[k]; if not b then b = {}; hash[k] = b end
		table.insert(b, i)
	end
	local function near(v: Vector3, r: number)
		local out = {}
		for x = math.floor((v.X-r)/CELLSZ), math.floor((v.X+r)/CELLSZ) do
		for y = math.floor((v.Y-r)/CELLSZ), math.floor((v.Y+r)/CELLSZ) do
		for z = math.floor((v.Z-r)/CELLSZ), math.floor((v.Z+r)/CELLSZ) do
			local b = hash[key(x, y, z)]
			if b then for _, i in ipairs(b) do table.insert(out, i) end end
		end end end
		return out
	end
	local function hasFaceAt(p: Vector3): boolean
		for _, i in ipairs(near(p, c.tol)) do
			if alive[i] and (walk[i].face - p).Magnitude <= c.tol then return true end
		end
		return false
	end

	-- A world-top cell sits on its PART's lattice, which for a rotated part is
	-- not aligned to anything useful -- on the union that produced this case the
	-- row step is 0.500 but the row-to-row step is 0.707 and drops 0.05 in Y. A
	-- fixed +/-leaf probe can never find those neighbours, so count by distance
	-- instead. Only world-top cells take this path.
	local function neighboursByRadius(cell): number
		local p = cell.face
		local cnt = 0
		for _, i in ipairs(near(p, c.leaf * 1.7)) do
			if alive[i] then
				local q = walk[i].face
				if q ~= p then
					local d = q - p
					local horiz = Vector3.new(d.X, 0, d.Z).Magnitude
					if horiz > 1e-4 and horiz <= c.leaf * 1.6 and math.abs(d.Y) <= c.leaf * 0.7 then
						cnt += 1
					end
				end
			end
		end
		return cnt
	end

	-- pass 1: isolated
	-- A NORMALIZED cell is not a world-top special case: it sits on a fitted face
	-- grid with real in-plane axes, so it takes the ordinary lattice path whatever
	-- its tilt. Sending it through the world-top branch instead judges it by world
	-- Y, which on a tilted face finds no neighbours at all -- that cut all 77
	-- walkable cells of Unioncase_2 as a one-cell-wide strip.
	local cutIso, kill = 0, {}
	for i, cell in ipairs(walk) do
		if cell.normalized or not cell.worldTop then
			local u, v = basis(cell)
			if u and v then
				local cnt = 0
				for _, d in ipairs({ u, -u, v, -v }) do
					if hasFaceAt(cell.face + d * c.leaf) then cnt += 1 end
				end
				if cnt < c.minNbr then kill[i] = true; cutIso += 1 end
			end
		end
	end
	for i in pairs(kill) do alive[i] = false end

	-- pass 2: thin -- must sit in a full patch x patch in-plane block, in at
	-- least one of the four quadrant orientations
	local cutThin; cutThin = 0; kill = {}
	local span = c.patch - 1
	for i, cell in ipairs(walk) do
		if alive[i] and (cell.normalized or not cell.worldTop) then
			local u, v = basis(cell)
			if u and v then
				local okq = false
				for _, su in ipairs({ 1, -1 }) do
					for _, sv in ipairs({ 1, -1 }) do
						local full = true
						for a = 0, span do
							for b = 0, span do
								if not (a == 0 and b == 0) then
									if not hasFaceAt(cell.face + (u*su*a + v*sv*b) * c.leaf) then
										full = false; break
									end
								end
							end
							if not full then break end
						end
						if full then okq = true; break end
					end
					if okq then break end
				end
				if not okq then kill[i] = true; cutThin += 1 end
			end
		end
	end
	for i in pairs(kill) do alive[i] = false end

	-- PASS 2b: world-top cells, judged as a GROUP rather than one at a time.
	--
	-- These sit on their part's rotated lattice, so a per-cell width test punishes
	-- the ends of a perfectly good face -- it cut the four corners of a 2x11 strip
	-- that is obviously one surface. Cells that are adjacent and coplanar ARE the
	-- same face, so cluster them and judge the cluster: if any member sits in a
	-- full patch x patch neighbourhood the whole cluster has real width and is
	-- kept intact, corners included. A genuine one-cell-wide line has no such
	-- member anywhere and is dropped whole.
	local cutStrip, clusters = 0, 0
	do
		local seen = {}
		for i, cell in ipairs(walk) do
			if alive[i] and cell.worldTop and not cell.normalized and not seen[i] then
				clusters += 1
				local group, stack = {}, { i }
				seen[i] = true
				while #stack > 0 do
					local j = table.remove(stack) :: number
					table.insert(group, j)
					local pj = walk[j].face
					for _, k in ipairs(near(pj, c.leaf * 1.7)) do
						if alive[k] and walk[k].worldTop and not seen[k] then
							local d = walk[k].face - pj
							local horiz = Vector3.new(d.X, 0, d.Z).Magnitude
							if horiz <= c.leaf * 1.6 and math.abs(d.Y) <= c.leaf * 0.7 then
								seen[k] = true; table.insert(stack, k)
							end
						end
					end
				end
				local wide = false
				for _, j in ipairs(group) do
					if neighboursByRadius(walk[j]) >= (c.patch * c.patch - 1) then wide = true; break end
				end
				if not wide then
					for _, j in ipairs(group) do alive[j] = false; cutStrip += 1 end
				end
			end
		end
	end

	-- pass 3: small components. The link tolerates a step ALONG the normal, so a
	-- staircase reads as one region instead of one component per tread.
	local comp, ncomp, sizes = {}, 0, {}
	for i = 1, n do
		if alive[i] and not comp[i] then
			ncomp += 1; comp[i] = ncomp
			local stack, cnt = { i }, 0
			while #stack > 0 do
				local j = table.remove(stack) :: number
				cnt += 1
				local cj = walk[j]
				for _, k in ipairs(near(cj.face, 0.85)) do
					if alive[k] and not comp[k] then
						local dv = walk[k].face - cj.face
						local dn = dv:Dot(cj.up)
						local dp = (dv - cj.up * dn).Magnitude
						if dp <= c.stepPlane and math.abs(dn) <= c.stepNormal then
							comp[k] = ncomp; table.insert(stack, k)
						end
					end
				end
			end
			sizes[ncomp] = cnt
		end
	end
	local cutSmall = 0
	for i = 1, n do
		if alive[i] and sizes[comp[i]] < c.minRegion then alive[i] = false; cutSmall += 1 end
	end

	local kept, survivors = 0, 0
	for i = 1, n do if alive[i] then kept += 1 end end
	for _, s in pairs(sizes) do if s >= c.minRegion then survivors += 1 end end

	return alive, {
		input = n, kept = kept,
		cutIsolated = cutIso, cutThin = cutThin, cutSmall = cutSmall,
		cutStrip = cutStrip, topClusters = clusters,
		components = ncomp, componentsKept = survivors,
		comp = comp, sizes = sizes,
	}
end

-- FACE NORMALIZATION -- unions and meshes only, and it runs BEFORE the probe.
--
-- A union's cells sit on the PART's lattice. Where that lattice is rotated
-- relative to the surface, the cells come out as a staircase: measured on
-- case5's Union, in-row steps of 0.500 against row-to-row steps of 0.707, plus
-- whole rows missing where the surface passes between lattice layers. Contour
-- then builds its own lattice from that and squashes it -- 22 of 33 cells
-- collapsed onto occupied slots, leaving a one-cell-wide strip that contours as
-- two coincident edges at 0.0 degrees.
--
-- Placement matters as much as the method. A cell here has either a
-- bounding-box normal (wrong whenever the box is rotated relative to the
-- geometry) or none at all, and a cell with no trusted normal is refused as
-- `unproven` rather than guessed. Run after the probe and that refusal has
-- already happened: on Unioncase_2 it discarded 27 of 54 surface cells before
-- this stage could see them. Fitting the face FIRST gives every cell a normal
-- taken from the surface, so the probe has something real to stand its brick on.
--
-- The steps, and the reason each one is not simpler:
--
--   1. A NORMAL PER CELL, BY RAYCAST. A PCA of the neighbourhood describes the
--      VOXELS, and the voxels of an off-axis face are a staircase, so the fit
--      inherits the staircase's bias -- it read 12.8 degrees on a face that is
--      flat. One short ray onto the cell's own part reports the real surface;
--      it must start outside the solid or it returns nothing. PCA is the
--      fallback for cells the ray misses.
--   2. GROW A FACE AGAINST ITS SEED normal, never against the neighbour's.
--      Comparing to the neighbour lets a flood fill walk a gradient: on
--      Unioncase it joined a 49 degree ramp to flat ground in sub-15-degree
--      steps and returned one 2436-cell blob fitting at RMS 1.677. Seeded, the
--      same cells split into a 0.8 degree face at RMS 0.199 and a 43.6 degree
--      face at RMS 0.433.
--   3. MERGE COPLANAR faces even when they do not touch. Two rows of a ramp came
--      back as a separate 84-cell face purely for not being 8-connected -- they
--      were the ramp's missing lines.
--   4. ADOPT on-plane cells, but only where the local normal AGREES. Plane
--      distance alone is not membership: a horizontal face otherwise adopts
--      whatever a ramp happens to have at that height, which over-filled a
--      1134-slot face with 1512 cells.
--   5. SEAT the plane above its cells, on the surface points the rays found.
--      Seating on the mean pushes half the cells of a stepped face BELOW the
--      surface, into solid, where the probe's buried test rightly rejects them
--      -- that lost 45 of 63 cells and the face vanished. Seating on the strict
--      maximum lets one stray cell lift the whole face, and clamping the lift
--      cuts into the rim instead. A high quantile does neither.
--   6. ONE GRID PER FACE -- direction, phase and origin fixed once, and every
--      cell placed through it. Reconstructing an adopted cell without the grid's
--      phase offsets it by a fraction of a cell and combs the seams.
--
-- KNOWN LIMIT. Faces of one object are related, so a face that is already planar
-- in the voxels can pin the height of a stepped one along their shared seam.
-- That is what `anchorSeam` does, and it only ever raises a face: every attempt
-- to lower one buried it outright (126 walkable cells to 64, twice). On
-- Unioncase_2 the ramp sits 0.107 studs off its surface and the stepped top
-- still 0.638, because the correction there is downward and therefore refused.
-- The cell faces are only 0.181 studs above that surface, so the excess is in
-- the plane FIT, most likely pulled up by adopted cells -- that is where to look
-- next, not at the seating rule.
local function planeFit(pts)
	local m = Vector3.zero
	for _, p in ipairs(pts) do m += p end
	m /= #pts
	local xx, xy, xz, yy, yz, zz = 0, 0, 0, 0, 0, 0
	for _, p in ipairs(pts) do
		local d = p - m
		xx += d.X*d.X; xy += d.X*d.Y; xz += d.X*d.Z
		yy += d.Y*d.Y; yz += d.Y*d.Z; zz += d.Z*d.Z
	end
	local function mul(v)
		return Vector3.new(xx*v.X + xy*v.Y + xz*v.Z, xy*v.X + yy*v.Y + yz*v.Z, xz*v.X + yz*v.Y + zz*v.Z)
	end
	local a = Vector3.new(1, 0, 0)
	for _ = 1, 40 do local n = mul(a); if n.Magnitude < 1e-12 then break end; a = n.Unit end
	local b = Vector3.new(0, 0, 1)
	for _ = 1, 40 do
		local n = mul(b); n = n - a * n:Dot(a)
		if n.Magnitude < 1e-12 then break end
		b = n.Unit
	end
	local nr = a:Cross(b)
	if nr.Magnitude < 1e-9 then return nil end
	nr = nr.Unit
	if nr.Y < 0 then nr = -nr; a = -a end
	return nr, m, a.Unit, nr:Cross(a.Unit).Unit
end

function NodeWalk.normalizeFaces(walk, alive, cfg)
	local c = merged(cfg)
	local LEAF, R = c.leaf, c.faceRadius
	local stats = { candidates = 0, faces = 0, moved = 0, collapsed = 0,
	                adopted = 0, anchored = 0, rayHits = 0, rayMissed = 0 }

	-- candidates: cells of parts whose collision shape is not their box. Before
	-- the probe there is no `alive`, so take the EXPOSED ones -- nothing solid of
	-- the same part directly above. That is the set a face is made of.
	local idx = {}
	for i, cell in ipairs(walk) do
		if (alive == nil or alive[i]) and cell.tree and cell.tree.exact == false then
			if alive ~= nil or not cell.tree:containsPoint(cell.cf.Position + UP * LEAF) then
				table.insert(idx, i)
			end
		end
	end
	stats.candidates = #idx
	if #idx < c.minFaceCells then return stats end

	local P = {}
	for n, i in ipairs(idx) do P[n] = walk[i].face or walk[i].cf.Position end

	local hash = {}
	local function bucket(p, dx, dy, dz)
		return (math.floor(p.X/R)+dx) .. "," .. (math.floor(p.Y/R)+dy) .. "," .. (math.floor(p.Z/R)+dz)
	end
	for n, p in ipairs(P) do
		local k = bucket(p, 0, 0, 0)
		hash[k] = hash[k] or {}
		table.insert(hash[k], n)
	end
	local function near(p)
		local out = {}
		for dx = -1, 1 do for dy = -1, 1 do for dz = -1, 1 do
			for _, n in ipairs(hash[bucket(p, dx, dy, dz)] or {}) do
				if (P[n] - p).Magnitude <= R then table.insert(out, n) end
			end
		end end end
		return out
	end

	-- 1. a normal per cell, and where the surface actually is
	local ln, hp = {}, {}
	local rp = RaycastParams.new()
	rp.FilterType = Enum.RaycastFilterType.Include
	rp.IgnoreWater = true
	for n, p in ipairs(P) do
		local cell = walk[idx[n]]
		rp.FilterDescendantsInstances = { cell.part }
		local dir = cell.up
		local res = workspace:Raycast(p + dir * LEAF, -dir * (LEAF * 2.2), rp)
		if res and res.Instance == cell.part then
			local nr = res.Normal
			if nr.Y < 0 then nr = -nr end
			ln[n], hp[n] = nr, res.Position
			stats.rayHits += 1
		else
			local nb = near(p)
			if #nb >= 6 then
				local pts = {}
				for _, o in ipairs(nb) do table.insert(pts, P[o]) end
				ln[n] = planeFit(pts)
			end
			stats.rayMissed += 1
		end
	end

	-- 2. grow faces against the seed normal
	local key = {}
	for n, p in ipairs(P) do
		key[math.round(p.X/LEAF) .. "," .. math.round(p.Y/LEAF) .. "," .. math.round(p.Z/LEAF)] = n
	end
	local TOL = math.cos(math.rad(c.faceTol))
	local order = {}
	for n in ipairs(P) do if ln[n] then table.insert(order, n) end end
	table.sort(order, function(x, y) return ln[x].Y > ln[y].Y end)

	local seen, groups = {}, {}
	for _, s in ipairs(order) do
		if not seen[s] then
			local anchor = ln[s]
			local comp, stack = {}, { s }
			seen[s] = true
			while #stack > 0 do
				local j = table.remove(stack)
				table.insert(comp, j)
				local q = P[j]
				local a, b, d = math.round(q.X/LEAF), math.round(q.Y/LEAF), math.round(q.Z/LEAF)
				for dx = -1, 1 do for dy = -1, 1 do for dz = -1, 1 do
					local nk = key[(a+dx) .. "," .. (b+dy) .. "," .. (d+dz)]
					if nk and not seen[nk] and ln[nk] and ln[nk]:Dot(anchor) > TOL then
						seen[nk] = true
						table.insert(stack, nk)
					end
				end end end
			end
			if #comp >= c.minFaceCells then table.insert(groups, comp) end
		end
	end

	-- 3. merge coplanar groups, connected or not
	local planes = {}
	for _, g in ipairs(groups) do
		local pts = {}
		for _, n in ipairs(g) do table.insert(pts, P[n]) end
		local N, M = planeFit(pts)
		if N then
			local placed = false
			for _, pl in ipairs(planes) do
				if pl.N:Dot(N) > math.cos(math.rad(10))
					and math.abs((M - pl.M):Dot(pl.N)) < c.facePlane then
					for _, n in ipairs(g) do table.insert(pl.members, n) end
					placed = true
					break
				end
			end
			if not placed then table.insert(planes, { N = N, M = M, members = table.clone(g) }) end
		end
	end
	table.sort(planes, function(x, y) return #x.members > #y.members end)

	-- 4. adopt, then 5. seat
	local claimed, faces = {}, {}
	for _, pl in ipairs(planes) do
		local pts = {}
		for _, n in ipairs(pl.members) do table.insert(pts, P[n]) end
		local N, M = planeFit(pts)
		if N then
			local memb, mark = {}, {}
			for _, n in ipairs(pl.members) do memb[#memb+1] = n; mark[n] = true end
			for n, p in ipairs(P) do
				if not mark[n] and not claimed[n] and ln[n]
					and math.abs((p - M):Dot(N)) <= c.facePlane
					and ln[n]:Dot(N) > math.cos(math.rad(c.faceAdopt)) then
					memb[#memb+1] = n; mark[n] = true
					stats.adopted += 1
				end
			end
			if #memb >= c.minFaceCells then
				for _, n in ipairs(memb) do claimed[n] = true end
				local fp = {}
				for _, n in ipairs(memb) do table.insert(fp, P[n]) end
				local FN, FM, U, V = planeFit(fp)
				if FN then
					-- the surface's own normal beats the voxel fit; the voxel fit
					-- still supplies the in-plane axes
					local acc = Vector3.zero
					for _, n in ipairs(memb) do if ln[n] then acc += ln[n] end end
					if acc.Magnitude > 1e-6 then
						local TN = acc.Unit
						if TN:Dot(FN) < 0 then TN = -TN end
						FN = TN
						U = U - FN * U:Dot(FN)
						if U.Magnitude <= 1e-6 then U = FN:Cross(Vector3.new(1, 0, 0)) end
						if U.Magnitude <= 1e-6 then U = FN:Cross(Vector3.new(0, 0, 1)) end
						U = U.Unit
						V = FN:Cross(U).Unit
					end
					-- Seat on the CELL FACES, never on the raycast hits. Those rays are
					-- fired along the cell's pre-fit up, which is world up, so on a steep
					-- face a vertical ray lands further down the slope and its hit point
					-- sits BELOW the plane. Seating there drops the plane under the
					-- surface and the probe buries the face whole: on Unioncase's 49
					-- degree ramp that silently lost all 1050 of its cells while the flat
					-- face beside it came through untouched. A cell's own face is on the
					-- surface by construction, so the outermost of them always clears.
					local offs = {}
					for _, n in ipairs(memb) do table.insert(offs, (P[n] - FM):Dot(FN)) end
					table.sort(offs)
					FM = FM + FN * offs[math.clamp(math.ceil(#offs * c.faceSeat), 1, #offs)]
					-- planarity in the voxels: the confidence used to rank faces
					local acc2 = 0
					for _, n in ipairs(memb) do
						local d = (P[n] - FM):Dot(FN)
						acc2 += d * d
					end
					table.insert(faces, { memb = memb, FN = FN, FM = FM, U = U, V = V,
					                      rms = math.sqrt(acc2 / #memb) })
				end
			end
		end
	end

	-- 6. seat the flattest face first, pull the rest onto it, lay one grid each
	table.sort(faces, function(x, y) return x.rms < y.rms end)
	local seated = {}
	for fi, f in ipairs(faces) do
		if fi > 1 and #seated > 0 then
			local deltas = {}
			for _, n in ipairs(f.memb) do
				for _, g in ipairs(seated) do
					for _, o in ipairs(g.memb) do
						if (P[o] - P[n]).Magnitude <= LEAF * 1.8 then
							local pg = P[o] - g.FN * ((P[o] - g.FM):Dot(g.FN))
							table.insert(deltas, (pg - f.FM):Dot(f.FN))
						end
					end
				end
			end
			if #deltas >= 3 then
				table.sort(deltas)
				-- the junction is the TOP of the seam: a neighbour within a leaf of a
				-- steep face is well down its slope, so anything lower drags this
				-- plane through its own surface. Never move downward -- the face's
				-- own seating already clears every one of its cells.
				local lift = math.max(deltas[#deltas], 0)
				if lift > 0 then
					f.FM = f.FM + f.FN * lift
					stats.anchored += 1
				end
			end
		end
		table.insert(seated, f)

		local FN, FM, U, V = f.FN, f.FM, f.U, f.V
		local fp = {}
		for _, n in ipairs(f.memb) do table.insert(fp, P[n]) end
		local sx, sy, cnt = 0, 0, 0
		for a = 1, #fp do for b = a + 1, #fp do
			local d = fp[b] - fp[a]
			if d.Magnitude <= LEAF * 1.2 then
				local t = 4 * math.atan2(d:Dot(V), d:Dot(U))
				sx += math.cos(t); sy += math.sin(t); cnt += 1
			end
		end end
		local ang = (cnt > 0) and (math.atan2(sy, sx) / 4) or 0
		local GU = (U * math.cos(ang) + V * math.sin(ang)).Unit
		local GV = FN:Cross(GU).Unit
		local function phase(ax)
			local cx, cy = 0, 0
			for _, p in ipairs(fp) do
				local fr = (((p - FM):Dot(ax)) / LEAF) % 1
				cx += math.cos(2*math.pi*fr); cy += math.sin(2*math.pi*fr)
			end
			return (math.atan2(cy, cx) / (2*math.pi)) % 1
		end
		local pu, pv = phase(GU), phase(GV)

		local slot = {}
		for _, n in ipairs(f.memb) do
			local d = P[n] - FM
			local iu = math.round(d:Dot(GU)/LEAF - pu)
			local iv = math.round(d:Dot(GV)/LEAF - pv)
			local k = iu .. "," .. iv
			local prev = slot[k]
			if prev then
				-- two cells over one in-plane spot: keep the outermost
				stats.collapsed += 1
				if (P[n] - FM):Dot(FN) > (P[prev.n] - FM):Dot(FN) then
					if alive then alive[idx[prev.n]] = false end
					slot[k] = { n = n, iu = iu, iv = iv }
				elseif alive then
					alive[idx[n]] = false
				end
			else
				slot[k] = { n = n, iu = iu, iv = iv }
			end
		end
		for _, s in pairs(slot) do
			local pos = FM + GU * ((s.iu + pu) * LEAF) + GV * ((s.iv + pv) * LEAF)
			local cell = walk[idx[s.n]]
			cell.face = pos
			cell.up = FN
			cell.frame = CFrame.fromMatrix(pos, GU, FN)
			cell.normalized = true
			-- the normal comes from the fitted SURFACE now, not a box axis, so the
			-- probe may trust it instead of refusing the cell as unproven
			cell.realNormal = true
			stats.moved += 1
		end
		stats.faces += 1
	end

	return stats
end

local ROOT_NAME = "NVGN_Debug"
local SECTION = "BrickWalk"

-- Recast stage 3: partition the surviving cells into REGIONS.
--
-- A region is the largest patch you could draw one outline around. Cells join a
-- region when all three hold:
--   * same locomotion mode   -- stand and prone are different areas to an NPC
--   * same plane             -- same normal AND same offset along it, so two
--                               floors stacked above each other stay separate
--   * connected              -- you can step from one to the next
--
-- Deliberately STRICT about the plane: each stair tread is its own region.
-- Cocosulx's call -- pathfinding resolves step-to-step, and it keeps every
-- region a flat 2D problem for contouring, which is the whole point of stage 4.
function NodeWalk.regions(walk, alive, cfg)
	local c = merged(cfg)
	local n = #walk

	-- bucket by (mode, normal direction, plane offset)
	local QN, QD = 100, 1 / (c.leaf * 0.5)
	local planes = {}
	for i = 1, n do
		if alive == nil or alive[i] then
			local cell = walk[i]
			local u = cell.up
			local d = cell.face:Dot(u)
			local k = string.format("%s|%d,%d,%d|%d", cell.mode or "?",
				math.round(u.X*QN), math.round(u.Y*QN), math.round(u.Z*QN), math.round(d*QD))
			local b = planes[k]; if not b then b = {}; planes[k] = b end
			table.insert(b, i)
		end
	end

	local regionOf, regions = {}, {}
	for _, members in pairs(planes) do
		-- local hash so connectivity is cheap inside the plane
		local hash = {}
		local CS = c.leaf * 2
		local function key(p) return math.floor(p.X/CS)*73856093 + math.floor(p.Y/CS)*19349663 + math.floor(p.Z/CS)*83492791 end
		for _, i in ipairs(members) do
			local k = key(walk[i].face)
			local b = hash[k]; if not b then b = {}; hash[k] = b end
			table.insert(b, i)
		end
		local seen = {}
		for _, seedIdx in ipairs(members) do
			if not seen[seedIdx] then
				local id = #regions + 1
				local cells, stack = {}, { seedIdx }
				seen[seedIdx] = true
				while #stack > 0 do
					local j = table.remove(stack) :: number
					table.insert(cells, j)
					regionOf[j] = id
					local pj = walk[j].face
					for dx = -1, 1 do for dy = -1, 1 do for dz = -1, 1 do
						local b = hash[key(pj + Vector3.new(dx*CS, dy*CS, dz*CS))]
						if b then
							for _, k2 in ipairs(b) do
								if not seen[k2] and (walk[k2].face - pj).Magnitude <= c.leaf * 1.6 then
									seen[k2] = true; table.insert(stack, k2)
								end
							end
						end
					end end end
				end
				local cell = walk[seedIdx]
				table.insert(regions, {
					id = id, cells = cells, size = #cells,
					mode = cell.mode, normal = cell.up,
					area = #cells * c.leaf * c.leaf,
				})
			end
		end
	end

	-- does the region house a body anywhere? one cell that fits is enough --
	-- that is what "the region can hold an NPC" means.
	for _, r in ipairs(regions) do
		r.fitCells = 0
		for _, i in ipairs(r.cells) do
			if walk[i].fits then r.fitCells += 1 end
		end
	end

	-- cull regions too small to deserve a polygon
	local kept, culled, culledCells, culledNoFit = {}, 0, 0, 0
	for _, r in ipairs(regions) do
		local noFit = c.requireFit and r.fitCells == 0
		if noFit then culledNoFit += 1 end
		if r.size < c.minRegionCells or noFit then
			for _, i in ipairs(r.cells) do
				regionOf[i] = nil
				if alive then alive[i] = false end
			end
			culled += 1
			culledCells += r.size
		else
			table.insert(kept, r)
		end
	end

	table.sort(kept, function(a, b) return a.size > b.size end)
	return kept, regionOf, {
		total = #regions, kept = #kept, culled = culled,
		culledCells = culledCells, minRegionCells = c.minRegionCells,
		culledNoFit = culledNoFit, agentWidth = c.agentWidth,
	}
end

-- Paints one colour per region, so you can see the partition rather than a
-- uniform green field. Colours are spread by the golden ratio so neighbouring
-- ids never look alike.
function NodeWalk.drawRegions(walk, regions, regionOf, opts)
	opts = opts or {}
	local leaf = opts.leaf or DEFAULT.leaf
	local name = opts.name or "regions"
	local root = workspace:FindFirstChild(ROOT_NAME)
	if not root then root = Instance.new("Folder"); root.Name = ROOT_NAME; root.Parent = workspace end
	local sec = root:FindFirstChild("Regions")
	if not sec then
		sec = Instance.new("Folder"); sec.Name = "Regions"; sec.Parent = root
	end
	-- clear only THIS model's folder; drawing case5 must not wipe case3
	local prev = sec:FindFirstChild(name)
	if prev then prev:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = name; folder.Parent = sec

	local colour = {}
	for _, r in ipairs(regions) do
		colour[r.id] = Color3.fromHSV(((r.id * 0.61803398875) % 1), 0.72, 0.98)
	end

	local proto = Instance.new("Part")
	proto.Size = Vector3.new(leaf - 0.06, leaf - 0.06, leaf - 0.06)
	proto.Anchored = true
	proto.CanCollide, proto.CanQuery, proto.CanTouch = false, false, false
	proto.CastShadow = false
	proto.Material = Enum.Material.SmoothPlastic

	local drawn = 0
	for i, cell in ipairs(walk) do
		local id = regionOf[i]
		if id then
			local p = proto:Clone()
			p.CFrame = (cell.frame or cell.cf) + cell.up * 0.08
			p.Color = colour[id]
			p:SetAttribute("region", id)
			p:SetAttribute("mode", cell.mode)
			p.Parent = folder
			drawn += 1
		end
	end
	return { drawn = drawn, folder = folder }
end

function NodeWalk.clear(name: string?)
	local root = workspace:FindFirstChild(ROOT_NAME)
	local sec = root and root:FindFirstChild(SECTION)
	if not sec then return 0 end
	if name == nil then
		local n = #sec:GetChildren()
		sec:ClearAllChildren()
		return n
	end
	local fo = sec:FindFirstChild(name)
	if not fo then return 0 end
	local n = #fo:GetChildren()
	fo:Destroy()
	return n
end

function NodeWalk.draw(walk, alive, opts)
	opts = opts or {}
	local name: string = opts.name or "walk"
	local leaf: number = opts.leaf or DEFAULT.leaf
	local inset: number = opts.inset or 0.06
	-- A cell is INSIDE its part, so a walkable top cell is coplanar with the
	-- surface and z-fights into invisibility. Push it out along its own normal.
	local lift: number = opts.lift or 0.08
	-- one colour per locomotion mode, so the bake reads at a glance
	local MODE_COLOUR = opts.modeColour or {
		stand  = Color3.fromRGB(70, 210, 100),   -- green
		crouch = Color3.fromRGB(240, 180, 50),   -- amber
		prone  = Color3.fromRGB(210, 70, 60),    -- red
		gate   = Color3.fromRGB(12, 12, 14),     -- black: clears 1.4, fails prone
	}
	local GREEN = opts.green or Color3.fromRGB(70, 210, 100)

	NodeWalk.clear(name)
	local root = workspace:FindFirstChild(ROOT_NAME)
	if not root then
		root = Instance.new("Folder"); root.Name = ROOT_NAME; root.Parent = workspace
	end
	local sec = root:FindFirstChild(SECTION)
	if not sec then
		sec = Instance.new("Folder"); sec.Name = SECTION; sec.Parent = root
	end
	local folder = Instance.new("Folder")
	folder.Name = name

	local proto = Instance.new("Part")
	proto.Size = Vector3.new(leaf - inset, leaf - inset, leaf - inset)
	proto.Anchored = true
	proto.CanCollide, proto.CanQuery, proto.CanTouch = false, false, false
	proto.CastShadow = false
	proto.Material = Enum.Material.SmoothPlastic
	proto.Color = GREEN

	local function emit(cell, walkableFlag)
		local p = proto:Clone()
		-- a rescued cell draws on its RE-SEATED frame, so its top face is world up
		p.CFrame = (cell.frame or cell.cf) + cell.up * lift
		if cell.mode then
			p.Color = MODE_COLOUR[cell.mode] or GREEN
			p:SetAttribute("mode", cell.mode)
			p:SetAttribute("clearance", cell.clearance)
		end
		p:SetAttribute("walkable", walkableFlag)
		p:SetAttribute("part", cell.part.Name)
		p.Parent = folder
	end

	local n, nGated = 0, 0
	for i, cell in ipairs(walk) do
		if (alive == nil) or alive[i] then
			emit(cell, true)
			n += 1
		end
	end
	-- gated cells are drawn but never walkable -- they are the near-misses
	if opts.gated then
		for _, cell in ipairs(opts.gated) do
			emit(cell, false)
			nGated += 1
		end
	end

	folder.Parent = sec
	return { drawn = n + nGated, walkable = n, gated = nGated, folder = folder }
end

-- One call: build, probe, purge, draw. `target` is a Model or Folder -- scope it
-- to the thing under test, never the whole workspace.
function NodeWalk.run(target: Instance, cfg)
	local c = merged(cfg)
	local t = {}

	local t0 = os.clock()
	local parts = NodeWalk.collect(target)
	local cells, info = NodeWalk.cells(parts, c)
	t.build = os.clock() - t0

	t0 = os.clock()
	local walk, pinfo = NodeWalk.probe(cells, c)
	t.probe = os.clock() - t0

	t0 = os.clock()
	local alive, stats = NodeWalk.purge(walk, c)
	t.purge = os.clock() - t0

	t0 = os.clock()
	local drawn
	if c.draw ~= false then
		drawn = NodeWalk.draw(walk, alive, {
			name = c.name or target.Name, leaf = c.leaf, gated = pinfo.gated,
		})
	end
	t.draw = os.clock() - t0

	return {
		cells = cells, walk = walk, alive = alive,
		stats = {
			parts = #parts, cells = #cells, maxEdge = info.maxEdge,
			fellBack = info.fellBack, rescued = info.rescued, walkable = #walk,
			kept = stats.kept, cutIsolated = stats.cutIsolated,
			cutThin = stats.cutThin, cutSmall = stats.cutSmall,
			components = stats.components, componentsKept = stats.componentsKept,
			drawn = drawn and drawn.drawn or 0,
			modes = pinfo.counts, probesRun = pinfo.probesRun,
			gated = #pinfo.gated, steep = pinfo.steep,
			buried = pinfo.buried, unproven = pinfo.unproven,
			chunks = pinfo.chunks, peak = pinfo.peak, fits = pinfo.fits,
		},
		gated = pinfo.gated,
		times = t,
		config = c,
	}
end

return NodeWalk

