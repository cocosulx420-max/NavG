--!strict
-- NVGN.Leaps -- the one-way links: dropping off an edge, and jumping.
--
-- Severance joins floor you can WALK between -- a step of at most the largest
-- profile step. Anything taller was simply not linked, so a ledge you can drop
-- off or jump onto was a dead end, and case5's c004 could only be reached by a
-- drop the graph could not express. These links fill that in.
--
-- Walked along every polygon edge that is the region's boundary, one sample per
-- `sample` studs. From each sample the body walks OUTWARD, distance by distance
-- (`tries`), and at each distance two questions are asked:
--
--   PASS   the lowest height the body can cross from the rim to here at: a
--          blade from its feet to its head (`body`, the smallest crouch of any
--          profile) swept clear, and the column over the rim clear up to it. Up
--          to the envelope jump; nothing passes -> a wall, stop.
--   FALL   from that height, straight down. What it finds decides:
--            a lip or cornice off the mesh within a step    keep going out
--            the same floor or a walkable neighbour         a seam's job, stop
--            a lower polygon                                DROP (over a rail if
--                                                           the pass was high)
--            a polygon up to the jump above, or across a    JUMP
--            gap
--          and an UP ray from the landing back to the fall's start must hit
--          nothing -- a ray that starts inside a part flies through it
--          (Cocosulx's gateA_bad).
--
-- Every drop records `vault`, the height it had to clear first (0 for a plain
-- step off), so a profile that cannot jump that high refuses it; a jump records
-- its rise and gap. The ENVELOPE (Agents) decides what is looked for, each
-- profile what it uses. Consecutive samples on one edge that land on the same
-- polygon at a similar height become one gate, drawn on both sides, with `via`
-- the path the body takes between them, so the arrow goes over what it clears.

local Leaps = {}

local Agents = require(script.Parent:WaitForChild("Agents"))

Leaps.sample = 1.0      -- studs between samples along a rim
Leaps.dropCap = 300     -- deepest a ray looks when drop is unlimited
Leaps.landTol = 0.75    -- how far a hit may sit off a polygon's plane
Leaps.groupRise = 0.75  -- samples merge while their landing heights agree this well
-- Distances out from the rim the body is tried at. Up to dropReach it steps (or
-- vaults) off and falls; beyond it, it has to jump the distance.
Leaps.tries = { 0.6, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5 }
Leaps.dropReach = 3.5
Leaps.jumpStep = 0.5    -- studs between the tries past dropReach, out to jumpDistance
Leaps.passStep = 0.5    -- studs between the heights the pass is tried at
Leaps.lift = 0.3        -- studs off a surface a ray runs, clear of the surface itself
Leaps.blade = 0.2       -- studs; width of the body sweep, a thin blade from feet to head
-- THE FALL IS AS WIDE AS THE BODY. A ray down the centre slipped past an eave
-- slab 0.9 studs away and "landed" 50 studs below, straight through the
-- overhang the body would hit (Cocosulx's dropoffbad). Rays at this share of
-- the smallest radius, out and to both sides, must reach the landing too.
Leaps.fallRadius = 0.9
Leaps.detour = 2.0      -- a walk bridge must save a way round longer than this x the walk...
Leaps.detourSlack = 3.0 -- ...plus this many studs
Leaps.walkMin = 2       -- samples (studs of rim) a walk bridge needs
-- Debugging: a set of polygon indices; every probe from their rims is logged
-- step by step into Leaps.traceLog.
Leaps.tracePolys = nil :: { [number]: boolean }?
Leaps.traceLog = {} :: { string }

local function vkey(p: Vector3): string
	return ("%.3f,%.3f,%.3f"):format(p.X, p.Y, p.Z)
end

local function heightAt(f: any, p: Vector3): number
	local c, up = f.centre, f.up or Vector3.yAxis
	if math.abs(up.Y) < 1e-3 then return c.Y end
	return c.Y - ((p.X - c.X) * up.X + (p.Z - c.Z) * up.Z) / up.Y
end

local function inPlan(f: any, p: Vector3): boolean
	local v = f.verts
	local n = #v
	for k = 1, n do
		local a, b = v[k], v[k % n + 1]
		if (b.X - a.X) * (p.Z - a.Z) - (b.Z - a.Z) * (p.X - a.X) > 1e-4 then return false end
	end
	return true
end

function Leaps.build(mesh: any, data: any, res: any, debugExclude: { Instance }?): any
	local env = Agents.envelope()
	local stats = { rims = 0, samples = 0, drops = 0, jumpsUp = 0, jumpsAcross = 0, vaults = 0,
		dropSamples = 0, jumpSamples = 0, offMesh = 0, duplicate = 0, walls = 0, rays = 0, seconds = 0,
		caught = 0, walks = 0 }
	local t0 = os.clock()
	local tris = mesh.tris
	local body = env.crouch
	if body == math.huge then body = 3 end

	-- polygons hashed by plan bounds, 4 stud buckets
	local B = 4
	local bucket: { [string]: { number } } = {}
	for i, f in ipairs(tris) do
		local lo, hi = f.verts[1], f.verts[1]
		for _, v in ipairs(f.verts) do lo = lo:Min(v); hi = hi:Max(v) end
		for x = math.floor(lo.X / B), math.floor(hi.X / B) do
			for z = math.floor(lo.Z / B), math.floor(hi.Z / B) do
				local k = x .. ":" .. z
				local b = bucket[k]
				if not b then b = {}; bucket[k] = b end
				b[#b + 1] = i
			end
		end
	end
	local function locate(p: Vector3): number?
		local best, bd = nil, math.huge
		for _, i in ipairs(bucket[math.floor(p.X / B) .. ":" .. math.floor(p.Z / B)] or {}) do
			local f = tris[i]
			if inPlan(f, p) then
				local d = math.abs(heightAt(f, p) - p.Y)
				if d < Leaps.landTol and d < bd then best, bd = i, d end
			end
		end
		return best
	end

	-- links that already join a pair, either way
	local joined: { [string]: boolean } = {}
	local adj: { [number]: { any } } = {}
	for _, L in ipairs(res.links) do
		joined[math.min(L.a, L.b) .. ":" .. math.max(L.a, L.b)] = true
		adj[L.a] = adj[L.a] or {}; table.insert(adj[L.a], { to = L.b, at = L.centre })
		adj[L.b] = adj[L.b] or {}; table.insert(adj[L.b], { to = L.a, at = L.centre })
	end
	-- A WALK BRIDGE MUST SAVE A DETOUR. Over unmeshed floor between two polygons
	-- the walk links already join a short way round, it adds a gate and no
	-- route; kept only when the way round is longer than detour x the straight
	-- walk plus detourSlack. Measured through the link centres, bounded.
	local function detour(i: number, j: number, from: Vector3, to: Vector3): boolean
		local direct = (to - from).Magnitude
		local limit = Leaps.detour * direct + Leaps.detourSlack
		local best: { [number]: number } = { [i] = 0 }
		local at: { [number]: Vector3 } = { [i] = from }
		local open = { i }
		while #open > 0 do
			local k, bi = nil, 0
			for n2, x in ipairs(open) do if not k or best[x] < best[k] then k, bi = x, n2 end end
			table.remove(open, bi)
			if k == j then return best[k] + (to - at[k]).Magnitude > limit end
			for _, e in ipairs(adj[k :: number] or {}) do
				local c = best[k :: number] + (e.at - at[k :: number]).Magnitude
				if c <= limit and (best[e.to] == nil or c < best[e.to]) then
					if best[e.to] == nil then open[#open + 1] = e.to end
					best[e.to] = c
					at[e.to] = e.at
				end
			end
		end
		return true
	end

	local rp = RaycastParams.new()
	rp.FilterType = Enum.RaycastFilterType.Exclude
	rp.FilterDescendantsInstances = debugExclude or {}
	rp.IgnoreWater = true
	local UP = Vector3.yAxis
	local depth = math.min(env.drop, Leaps.dropCap)
	local function ray(o: Vector3, d: Vector3): RaycastResult?
		stats.rays += 1
		return workspace:Raycast(o, d, rp)
	end

	-- Can the body go from over the rim `p` to over `q` with its feet at
	-- p.Y + h? A thin blade from its feet to its head, swept along: rays at
	-- three heights let whatever stood between them through.
	local bladeSize = Vector3.new(Leaps.blade, body - 0.1 - Leaps.lift, Leaps.blade)
	local bladeMid = (Leaps.lift + body - 0.1) * 0.5
	local function passes(p: Vector3, q: Vector3, h: number): boolean
		local d = q - p
		if d.Magnitude < 1e-3 then return true end
		stats.rays += 1
		return workspace:Blockcast(CFrame.new(p + UP * (h + bladeMid)), bladeSize, d, rp) == nil
	end
	local function lowestPass(p: Vector3, q: Vector3): number?
		local h = 0
		while h <= env.jump + 1e-6 do
			-- a ceiling over the rim lower than the head: no higher pass either
			if h > 0 and ray(p + UP * 0.1, UP * (h + body)) then return nil end
			if passes(p, q, h) then return h end
			h += Leaps.passStep
		end
		return nil
	end

	-- the tries, out to the envelope's jump distance
	local tries = table.clone(Leaps.tries)
	do
		local d = Leaps.dropReach + Leaps.jumpStep
		while d <= env.jumpDistance + 1e-6 do tries[#tries + 1] = d; d += Leaps.jumpStep end
	end

	-- One sample: walk outward from rim point `p` on polygon `i`.
	-- Returns target polygon, kind ("drop" | "up" | "across"), landing, the
	-- point the body passes through, and the height it cleared.
	local radius = (env.radius == math.huge) and 1 or env.radius
	-- THE FALL IS AS WIDE AS THE BODY: its footprint, a flat box, swept down
	-- from where it steps off to a step over the landing. An eave, a slab
	-- edge or the ledge it is still over catches it on any side, where rays
	-- round it slipped past (Cocosulx's dropoffbad).
	local footSize = Vector3.new(2 * radius * Leaps.fallRadius, 0.2, 2 * radius * Leaps.fallRadius)
	local function fallClear(from: Vector3, land: Vector3, outward: Vector3): boolean
		local drop = from.Y - (land.Y + env.step)
		if drop <= 0 then return true end
		stats.rays += 1
		return workspace:Blockcast(CFrame.lookAt(from, from + outward), footSize, -UP * drop, rp) == nil
	end

	local tracing = false
	local function note(fmt: string, ...)
		if tracing then table.insert(Leaps.traceLog, fmt:format(...)) end
	end
	local function probe(p: Vector3, outward: Vector3, i: number): (number?, string?, Vector3?, { Vector3 }?, number)
		tracing = Leaps.tracePolys ~= nil and (Leaps.tracePolys :: any)[i] == true
		note("f%d p(%.2f,%.2f,%.2f) out(%.2f,%.2f)", i, p.X, p.Y, p.Z, outward.X, outward.Z)
		local gapSeen = false
		local crossed = false -- walked over floor that is not on the mesh
		for _, past in ipairs(tries) do
			local q = p + outward * past
			local h = lowestPass(p, q)
			if not h then note("  %.1f no pass: wall", past); stats.walls += 1; return nil, nil, nil, nil, 0 end
			local from = q + UP * (h + Leaps.lift)
			local hit = ray(from, -UP * (h + Leaps.lift + depth))
			if not hit then note("  %.1f pass %.1f, nothing below", past, h); gapSeen = true; continue end
			local rel = hit.Position.Y - p.Y
			local j = locate(hit.Position)
			note("  %.1f pass %.1f, fall hits %s rel %.2f on %s", past, h, hit.Instance.Name, rel, j and ("f" .. j) or "no polygon")
			if j == i then
				-- still our own floor: nothing to leave yet
				if gapSeen then return nil, nil, nil, nil, 0 end
				continue
			end
			if rel >= -env.step and rel <= env.step then
				if j then
					if not gapSeen then
						-- walkable onto the neighbour. A seam's job -- unless there is no
						-- seam: a roof valley whose cells died (Cocosulx's image 20), or a
						-- steep panel's top a lip below the roof above it (image 22),
						-- leaves them unlinked, and the walk is a two-way bridge.
						if joined[math.min(i, j) .. ":" .. math.max(i, j)] or h > env.step
							or not detour(i, j, p, hit.Position) then
							stats.duplicate += 1
							return nil, nil, nil, nil, 0
						end
						stats.walks += 1
						local apex = h + Leaps.lift
						return j, "walk", hit.Position, (h > 0) and { p + UP * apex, q + UP * apex } or { q + UP * apex }, h
					end
				else
					stats.offMesh += 1 -- a lip, a cornice, a rail's top: keep going out
					crossed = true
					continue
				end
			elseif rel < -env.step then
				gapSeen = true
				if not j then stats.offMesh += 1; continue end
			elseif not j then
				-- something high and off the mesh we got over: keep going out
				continue
			end
			-- a landing on polygon j: the way down must be open
			local land = hit.Position
			if ray(land + UP * 0.05, from - (land + UP * 0.05)) then return nil, nil, nil, nil, 0 end
			-- too close to something the body would catch on: try further out
			if not fallClear(from, land, outward) then stats.caught += 1; crossed = true; continue end
			if joined[math.min(i, j) .. ":" .. math.max(i, j)] then
				stats.duplicate += 1
				return nil, nil, nil, nil, 0
			end
			-- THE PATH THE CHECKS PROVED: up the column over the rim to the pass
			-- height, across at it, then down the fall column. Drawn as is, so an
			-- arrow never cuts a corner the body does not.
			local apex = h + Leaps.lift
			local via = (h > 0) and { p + UP * apex, q + UP * apex } or { q + UP * apex }
			if rel < -env.step and past <= Leaps.dropReach + 1e-6 then
				return j, "drop", land, via, h
			elseif rel > env.step and past <= Leaps.dropReach + 1e-6 then
				return j, "up", land, via, h
			else
				return j, "across", land, via, h
			end
		end
		return nil, nil, nil, nil, 0
	end

	-- boundary edges: an edge key used once inside its region
	local uses: { [string]: number } = {}
	for _, f in ipairs(tris) do
		for k = 1, #f.verts do
			local a, b = f.verts[k], f.verts[k % #f.verts + 1]
			local ka, kb = vkey(a), vkey(b)
			local ek = f.region .. "#" .. ((ka < kb) and (ka .. "|" .. kb) or (kb .. "|" .. ka))
			uses[ek] = (uses[ek] or 0) + 1
		end
	end

	local out = {}
	local function emit(kind: string, a: number, b: number, run: { any }, oneWay: boolean)
		if #run == 0 then return end
		local first, last = run[1], run[#run]
		local L = {
			kind = kind, a = a, b = b,
			left = first.from, right = last.from,
			bLeft = last.to, bRight = first.to,
			centre = (first.from + last.from) * 0.5,
			span = (last.from - first.from).Magnitude,
			rise = 0, gap = 0, count = #run, residual = 0, fitted = true, edge = true,
			oneWay = oneWay, type = kind, vault = 0,
		}
		local rs, gs = 0, 0
		for _, s in ipairs(run) do
			rs += s.to.Y - s.from.Y
			gs += Vector3.new(s.to.X - s.from.X, 0, s.to.Z - s.from.Z).Magnitude
			if (s.vault or 0) > L.vault then L.vault = s.vault end
		end
		L.rise, L.gap = rs / #run, gs / #run
		-- the path the body takes, averaged over samples that share its shape
		local nv = run[1].via and #run[1].via or 0
		if nv > 0 then
			local acc, cnt = table.create(nv, Vector3.zero), 0
			for _, s2 in ipairs(run) do
				if s2.via and #s2.via == nv then
					for k2 = 1, nv do acc[k2] += s2.via[k2] end
					cnt += 1
				end
			end
			for k2 = 1, nv do acc[k2] /= cnt end
			L.via = acc
			L.over = acc[nv]
		end
		L.drop = -L.rise
		out[#out + 1] = L
	end

	for i, f in ipairs(tris) do
		local v = f.verts
		local n = #v
		for k = 1, n do
			local a, b = v[k], v[k % n + 1]
			local ka, kb = vkey(a), vkey(b)
			local ek = f.region .. "#" .. ((ka < kb) and (ka .. "|" .. kb) or (kb .. "|" .. ka))
			if uses[ek] ~= 1 then continue end
			local d = Vector3.new(b.X - a.X, 0, b.Z - a.Z)
			local len = d.Magnitude
			if len < 0.5 then continue end
			stats.rims += 1
			local ua = d / len
			local outward = ua:Cross(UP) -- floor is left of travel
			local m = math.max(1, math.floor(len / Leaps.sample))
			local cur: any = nil
			local function flush()
				if cur then
					if cur.kind == "walk" then
						-- one sample is a corner graze, not a way across
						if #cur.samples >= Leaps.walkMin then
							emit("bridge", i, cur.target, cur.samples, false)
						end
					elseif cur.kind == "drop" then
						emit("drop", i, cur.target, cur.samples, true)
						stats.drops += 1
						if cur.vaulted then stats.vaults += 1 end
						if cur.canJumpUp then
							local rev = {}
							for s = #cur.samples, 1, -1 do
								local q = cur.samples[s]
								local rv = {}
								for k2 = #(q.via or {}), 1, -1 do rv[#rv + 1] = q.via[k2] end
								rev[#rev + 1] = { from = q.to, to = q.from, via = rv }
							end
							emit("jump", cur.target, i, rev, true)
							stats.jumpsUp += 1
						end
					else
						emit("jump", i, cur.target, cur.samples, true)
						if cur.kind == "up" then stats.jumpsUp += 1 else stats.jumpsAcross += 1 end
					end
					cur = nil
				end
			end
			for s = 0, m - 1 do
				local t = (s + 0.5) / m
				local p = a:Lerp(b, t)
				stats.samples += 1
				local target, kind, land, via, vault = probe(p, outward, i)
				if target and land then
					if kind == "drop" then stats.dropSamples += 1 else stats.jumpSamples += 1 end
					local sample = { from = p, to = land, via = via, vault = vault }
					-- a plain step off can be jumped back up; a vault over a rail
					-- cannot be reversed by the same jump
					local canUp = kind == "drop" and (p.Y - land.Y) <= env.jump and vault <= env.step
					if cur and cur.target == target and cur.kind == kind
						and math.abs((land.Y - p.Y) - (cur.last.to.Y - cur.last.from.Y)) <= Leaps.groupRise
						and (vault > env.step) == cur.vaulted then
						cur.samples[#cur.samples + 1] = sample
						cur.last = sample
						cur.canJumpUp = cur.canJumpUp and canUp
					else
						flush()
						cur = { target = target, kind = kind, samples = { sample }, last = sample,
							canJumpUp = canUp, vaulted = vault > env.step }
					end
				else
					flush()
				end
			end
			flush()
		end
	end

	-- a ledge found from below and as the reverse of a drop from above is one jump
	local kept: { [string]: boolean } = {}
	for _, L in ipairs(out) do
		local c = L.centre
		local a, b = L.a, L.b
		if not L.oneWay then
			-- found from both sides: the same crossing, keyed by its middle
			a, b = math.min(a, b), math.max(a, b)
			c = (L.centre + (L.bLeft + L.bRight) * 0.5) * 0.5
		end
		local key = L.kind .. ":" .. a .. ":" .. b .. ":" .. math.floor(c.X / 2) .. ":" .. math.floor(c.Z / 2)
		if kept[key] then
			stats.duplicate += 1
		else
			kept[key] = true
			res.links[#res.links + 1] = L
		end
	end
	stats.seconds = os.clock() - t0
	res.leaps = stats
	return stats
end

function Leaps.report(s: any): string
	return ("leaps     %d rim edges, %d samples: %d drops (%d over a rail), %d jumps up, %d jumps across, %d walk samples over unmeshed floor (%d drop samples, %d jump samples); %d off the mesh, %d already walkable, %d walls, %d falls the body would catch; %d rays  (%.1fs)")
		:format(s.rims, s.samples, s.drops, s.vaults, s.jumpsUp, s.jumpsAcross, s.walks, s.dropSamples, s.jumpSamples,
			s.offMesh, s.duplicate, s.walls, s.caught, s.rays, s.seconds)
end

return Leaps
