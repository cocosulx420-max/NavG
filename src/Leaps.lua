--!strict
-- NVGN.Leaps -- the one-way links: dropping off an edge, and jumping.
--
-- Severance joins floor you can WALK between -- a step of at most the largest
-- profile step. Anything taller was simply not linked, so a ledge you can drop
-- off or jump onto was a dead end, and case5's c004 could only be reached by a
-- drop the graph could not express. These links fill that in.
--
-- Walked along every polygon edge that is the region's boundary, one sample per
-- `sample` studs, looking just past the rim:
--
--   drop         a ray straight down lands on another polygon more than a step
--                lower. One-way, rim -> landing, with its measured depth.
--   jump up      the reverse of a drop no deeper than the envelope jump.
--                One-way, landing -> rim.
--   jump across  nothing to land on straight down, so look outward up to the
--                envelope jumpDistance for a landing no higher than the jump,
--                with a clear line at chest height. One-way.
--
-- MEASUREMENTS, NOT VERDICTS: every link carries rise, gap and span, and the
-- limits used here are the ENVELOPE (Agents), so each NPC profile decides at
-- path time whether it can use one. Consecutive samples on one edge that land on
-- the same polygon at a similar height become one gate, drawn on both sides.

local Leaps = {}

local Agents = require(script.Parent:WaitForChild("Agents"))

Leaps.sample = 1.0      -- studs between samples along a rim
Leaps.past = 0.6        -- studs past the rim the down ray starts
Leaps.dropCap = 300     -- deepest a ray looks when drop is unlimited
Leaps.landTol = 0.75    -- how far a hit may sit off a polygon's plane
Leaps.chest = 2.0       -- height of the clear-line test for jumps
Leaps.groupRise = 0.75  -- samples merge while their landing heights agree this well
-- How far out past the rim a drop may start. The first that lands on the mesh
-- wins, so a cornice or a lip under the rim is stepped past rather than landed on.
Leaps.pastTries = { 0.6, 1.5, 2.5, 3.5 }
Leaps.knee = 0.5        -- height of the low clear-line test

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
	local stats = { rims = 0, samples = 0, drops = 0, jumpsUp = 0, jumpsAcross = 0,
		dropSamples = 0, jumpSamples = 0, offMesh = 0, duplicate = 0, seconds = 0 }
	local t0 = os.clock()
	local tris = mesh.tris

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
	local function locate(p: Vector3, skip: number?): number?
		local best, bd = nil, math.huge
		for _, i in ipairs(bucket[math.floor(p.X / B) .. ":" .. math.floor(p.Z / B)] or {}) do
			if i ~= skip then
				local f = tris[i]
				if inPlan(f, p) then
					local d = math.abs(heightAt(f, p) - p.Y)
					if d < Leaps.landTol and d < bd then best, bd = i, d end
				end
			end
		end
		return best
	end

	-- links that already join a pair, either way
	local joined: { [string]: boolean } = {}
	for _, L in ipairs(res.links) do
		joined[math.min(L.a, L.b) .. ":" .. math.max(L.a, L.b)] = true
	end

	local rp = RaycastParams.new()
	rp.FilterType = Enum.RaycastFilterType.Exclude
	rp.FilterDescendantsInstances = debugExclude or {}
	rp.IgnoreWater = true
	local UP = Vector3.yAxis
	local depth = math.min(env.drop, Leaps.dropCap)

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
			oneWay = oneWay, type = kind,
		}
		local rs, gs = 0, 0
		for _, s in ipairs(run) do
			rs += s.to.Y - s.from.Y
			gs += Vector3.new(s.to.X - s.from.X, 0, s.to.Z - s.from.Z).Magnitude
		end
		L.rise, L.gap = rs / #run, gs / #run
		-- the point in the air just past the rim, for a forward-then-down arrow
		local ov, no = Vector3.zero, 0
		for _, s2 in ipairs(run) do if s2.over then ov += s2.over; no += 1 end end
		if no > 0 then L.over = ov / no end
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
			-- runs per target, per kind: { target, kind, samples }
			local cur: any = nil
			local function flush()
				if cur then
					if cur.kind == "drop" then
						emit("drop", i, cur.target, cur.samples, true)
						stats.drops += 1
						if cur.canJumpUp then
							local rev = {}
							for s = #cur.samples, 1, -1 do
								local q = cur.samples[s]
								rev[#rev + 1] = { from = q.to, to = q.from, over = q.over }
							end
							emit("jump", cur.target, i, rev, true)
							stats.jumpsUp += 1
						end
					else
						emit("jump", i, cur.target, cur.samples, true)
						stats.jumpsAcross += 1
					end
					cur = nil
				end
			end
			for s = 0, m - 1 do
				local t = (s + 0.5) / m
				local p = a:Lerp(b, t)
				stats.samples += 1
				local target, kind, land, over = nil, nil, nil, nil
				-- THE WAY DOWN MUST BE OPEN. A ray that starts inside a part flies
				-- straight through it, so a probe just past the top edge of a roof
				-- panel -- already inside the building -- "landed" on the floor
				-- inside, 10.6 studs down (Cocosulx's gateA_bad). Every drop now
				-- needs the step off the rim clear at knee and chest height, and an
				-- UP ray from the landing back to the start that hits nothing: no
				-- roof, no ceiling in between.
				local lastHit = nil
				for _, past in ipairs(Leaps.pastTries) do
					local q = p + outward * past
					if workspace:Raycast(p + UP * Leaps.knee, outward * past, rp)
						or workspace:Raycast(p + UP * Leaps.chest, outward * past, rp) then
						break -- a wall or a parapet: no stepping off here
					end
					local hit = workspace:Raycast(q + UP * 0.25, -UP * (depth + 0.25), rp)
					lastHit = hit
					if not hit then break end
					local h = p.Y - hit.Position.Y
					if h <= env.step then break end -- walkable, a seam's job
					local up = (q + UP * 0.25) - (hit.Position + UP * 0.05)
					if workspace:Raycast(hit.Position + UP * 0.05, up, rp) then break end
					local j = locate(hit.Position, i)
					if j and not joined[math.min(i, j) .. ":" .. math.max(i, j)] then
						target, kind, land, over = j, "drop", hit.Position, q
						break
					elseif j then
						stats.duplicate += 1
						break
					end
					stats.offMesh += 1 -- a lip or a cornice: try further out
				end
				if not target and (not lastHit or (p.Y - lastHit.Position.Y) > env.step) then
					-- nothing to step down onto: try a jump outward
					local chest = p + UP * Leaps.chest
					for dist = 1.0, env.jumpDistance, 0.5 do
						local o2 = p + outward * dist
						if workspace:Raycast(chest, outward * dist, rp) then break end
						local top = o2 + UP * (env.jump + 0.25)
						local h2 = workspace:Raycast(top, -UP * (env.jump + env.step + 0.5), rp)
						if h2 then
							local rise = h2.Position.Y - p.Y
							if rise <= env.jump and rise >= -env.step then
								-- head room over the take-off and a clear column over the
								-- landing: no jumping through a ceiling
								local clearTake = not workspace:Raycast(p + UP * 0.1, UP * (math.max(rise, 0) + Leaps.chest + 1), rp)
								local clearLand = not workspace:Raycast(h2.Position + UP * 0.05, top - (h2.Position + UP * 0.05), rp)
								local clearLine = not workspace:Raycast(chest + UP * math.max(rise, 0),
									(h2.Position + UP * Leaps.chest) - (chest + UP * math.max(rise, 0)), rp)
								local j = (clearTake and clearLand and clearLine) and locate(h2.Position, i) or nil
								if j and j ~= i then
									if not joined[math.min(i, j) .. ":" .. math.max(i, j)] then
										target, kind, land, over = j, "across", h2.Position, nil
									else
										stats.duplicate += 1
									end
								end
								break
							end
						end
					end
				end
				if target then
					if kind == "drop" then stats.dropSamples += 1 else stats.jumpSamples += 1 end
					local sample = { from = p, to = land, over = over }
					local canUp = kind == "drop" and (p.Y - land.Y) <= env.jump
					if cur and cur.target == target and cur.kind == kind
						and math.abs((land.Y - p.Y) - (cur.last.to.Y - cur.last.from.Y)) <= Leaps.groupRise then
						cur.samples[#cur.samples + 1] = sample
						cur.last = sample
						cur.canJumpUp = cur.canJumpUp and canUp
					else
						flush()
						cur = { target = target, kind = kind, samples = { sample }, last = sample, canJumpUp = canUp }
					end
				else
					flush()
				end
			end
			flush()
		end
	end

	for _, L in ipairs(out) do res.links[#res.links + 1] = L end
	stats.seconds = os.clock() - t0
	res.leaps = stats
	return stats
end

function Leaps.report(s: any): string
	return ("leaps     %d rim edges, %d samples: %d drops, %d jumps up, %d jumps across (%d drop samples, %d jump samples); %d landed off the mesh, %d already walkable  (%.1fs)")
		:format(s.rims, s.samples, s.drops, s.jumpsUp, s.jumpsAcross, s.dropSamples, s.jumpSamples, s.offMesh, s.duplicate, s.seconds)
end

return Leaps
