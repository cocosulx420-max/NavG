--!strict
--!native
-- NVGN.PathSimplify -- staircase in, straight lines and true corners out.
--
-- Two passes over an array of Vector3:
--   1. a sliding window marks CANDIDATE corners wherever the macro trajectory
--      turns. The test is a dot product against a threshold, with no acos, no
--      normalisation and therefore no square root.
--   2. Ramer-Douglas-Peucker keeps only the candidates the geometry actually
--      needs, measured as SQUARED perpendicular distance against epsilonSq.
--
-- Pass 1 is a filter, not an answer: it is cheap and deliberately over-eager,
-- and pass 2 throws away whatever it cannot justify. Nothing outside the
-- candidate set can ever become a corner, which is what keeps pass 2 cheap --
-- it searches a few hundred candidates rather than every node.
--
-- No Vector3 arithmetic in either inner loop. Components are pulled out once and
-- the maths is done on numbers, because every Vector3 operation allocates.

local PathSimplify = {}

-- TUNING
PathSimplify.windowSize = 4      -- nodes each side that form the macro trajectory
PathSimplify.dotThreshold = 0.8  -- below this the trajectory counts as turning
PathSimplify.epsilonSq = 0.0625  -- squared studs. 0.0625 is a quarter of a stud

export type Options = {
	windowSize: number?,
	dotThreshold: number?,
	epsilonSq: number?,
	closed: boolean?,   -- treat the array as a ring
	strict: boolean?,   -- default true; false skips the epsilon guarantee
}

-- Squared perpendicular distance from p to the INFINITE line through a and b.
-- The line, not the segment: RDP's anchors are always points of the path, so a
-- node between them can never project outside and clamping would only cost time.
local function distSq(px: number, py: number, pz: number,
	ax: number, ay: number, az: number,
	bx: number, by: number, bz: number): number
	local abx, aby, abz = bx - ax, by - ay, bz - az
	local apx, apy, apz = px - ax, py - ay, pz - az
	local abSq = abx * abx + aby * aby + abz * abz
	if abSq < 1e-12 then
		return apx * apx + apy * apy + apz * apz
	end
	local t = (apx * abx + apy * aby + apz * abz) / abSq
	local dx = apx - abx * t
	local dy = apy - aby * t
	local dz = apz - abz * t
	return dx * dx + dy * dy + dz * dz
end

-- PASS 1. A node is a candidate when the trajectory arriving at it and the one
-- leaving it disagree by more than dotThreshold.
--
-- SQRT-FREE. cos = d / (|u||v|), so cos < T is d < T*sqrt(lu*lv). Squaring both
-- sides is only valid while d >= 0, so a negative dot -- anything past 90
-- degrees -- is taken as a turn outright and never squared.
local function findCandidates(pts: {Vector3}, n: number, k: number, t2: number, closed: boolean)
	local out, m = {}, 0
	local first = closed and 1 or (k + 1)
	local last = closed and n or (n - k)
	for i = first, last do
		local ia, ib
		if closed then
			ia = (i - k - 1) % n + 1
			ib = (i + k - 1) % n + 1
		else
			ia = i - k
			ib = i + k
		end
		local a, b, c = pts[ia], pts[i], pts[ib]
		local ux, uy, uz = b.X - a.X, b.Y - a.Y, b.Z - a.Z
		local vx, vy, vz = c.X - b.X, c.Y - b.Y, c.Z - b.Z
		local d = ux * vx + uy * vy + uz * vz
		if d < 0 then
			m += 1; out[m] = i
		else
			local lu = ux * ux + uy * uy + uz * uz
			local lv = vx * vx + vy * vy + vz * vz
			if d * d < t2 * lu * lv then
				m += 1; out[m] = i
			end
		end
	end
	return out, m
end

-- PASS 2. Ramer-Douglas-Peucker restricted to the candidate set, iterative.
--
-- The stack holds {anchorA, anchorB, candLo, candHi}: a span of the path and the
-- slice of candidates lying inside it. A candidate that stays within epsilonSq
-- of the chord is dropped, which is what "verifying" a candidate means here.
local function reduce(pts: {Vector3}, cand: {number}, nCand: number,
	epsSq: number, keep: {[number]: boolean}, stack: {any},
	a0: number, b0: number, lo0: number, hi0: number): number
	local splits = 0
	local top = 1
	stack[1] = { a0, b0, lo0, hi0 }
	while top > 0 do
		local frame = stack[top]
		stack[top] = nil
		top -= 1
		local ai, bi, lo, hi = frame[1], frame[2], frame[3], frame[4]
		if lo <= hi then
			local A, B = pts[ai], pts[bi]
			local ax, ay, az = A.X, A.Y, A.Z
			local bx, by, bz = B.X, B.Y, B.Z
			local best, bestAt = -1, 0
			for x = lo, hi do
				local p = pts[cand[x]]
				local d = distSq(p.X, p.Y, p.Z, ax, ay, az, bx, by, bz)
				if d > best then best = d; bestAt = x end
			end
			if best > epsSq then
				local at = cand[bestAt]
				keep[at] = true
				splits += 1
				top += 1; stack[top] = { ai, at, lo, bestAt - 1 }
				top += 1; stack[top] = { at, bi, bestAt + 1, hi }
			end
		end
	end
	return splits
end

-- PASS 3. Guarantee the epsilon bound.
--
-- Passes 1 and 2 can only ever cut at a candidate, and on a gently curving path
-- the window test never fires: a 360-node circle yields ZERO candidates and
-- collapses to a single chord however small epsilon is. Measured on real
-- boundaries, 27 of 83 loops broke the bound, the worst by 7.03 studs.
--
-- So each accepted span is checked against the nodes it actually spans, not
-- against the candidate list, and split at the worst offender. This only ever
-- touches spans that failed, so the candidate filter still carries the load.
-- Set strict = false to skip it and take passes 1 and 2 on trust.
local function refine(pts: {Vector3}, n: number, keep: {[number]: boolean},
	closed: boolean, epsSq: number, stack: {any}): number
	local anchors, m = {}, 0
	for i = 1, n do
		if keep[i] then m += 1; anchors[m] = i end
	end
	if m < 2 then return 0 end
	local top = 0
	for s = 1, closed and m or (m - 1) do
		top += 1; stack[top] = { anchors[s], anchors[(s % m) + 1] }
	end
	local added = 0
	while top > 0 do
		local fr = stack[top]; stack[top] = nil; top -= 1
		local ai, bi = fr[1], fr[2]
		local A, B = pts[ai], pts[bi]
		local ax, ay, az = A.X, A.Y, A.Z
		local bx, by, bz = B.X, B.Y, B.Z
		local worst, at = -1, 0
		if closed then
			local i = (ai % n) + 1
			local guard = 0
			while i ~= bi and guard <= n do
				local p = pts[i]
				local d = distSq(p.X, p.Y, p.Z, ax, ay, az, bx, by, bz)
				if d > worst then worst = d; at = i end
				i = (i % n) + 1
				guard += 1
			end
		else
			for i = ai + 1, bi - 1 do
				local p = pts[i]
				local d = distSq(p.X, p.Y, p.Z, ax, ay, az, bx, by, bz)
				if d > worst then worst = d; at = i end
			end
		end
		if worst > epsSq and at ~= 0 then
			keep[at] = true
			added += 1
			top += 1; stack[top] = { ai, at }
			top += 1; stack[top] = { at, bi }
		end
	end
	return added
end

-- Simplify `pts`. Returns the simplified points, the indices kept from the
-- input, and a stats table.
function PathSimplify.simplify(pts: {Vector3}, opts: Options?)
	local o = opts or {}
	local k = o.windowSize or PathSimplify.windowSize
	local thr = o.dotThreshold or PathSimplify.dotThreshold
	local epsSq = o.epsilonSq or PathSimplify.epsilonSq
	local closed = o.closed == true
	local n = #pts
	local stats = { input = n, candidates = 0, kept = 0, splits = 0, refined = 0 }

	if n <= 2 or (closed and n <= 3) then
		local copy = table.create(n)
		local idx = table.create(n)
		for i = 1, n do copy[i] = pts[i]; idx[i] = i end
		stats.kept = n
		return copy, idx, stats
	end
	if k * 2 >= n then k = math.max(1, (n // 2) - 1) end

	local cand, nCand = findCandidates(pts, n, k, thr * thr, closed)
	stats.candidates = nCand

	local keep: {[number]: boolean} = {}
	local stack = {}

	if closed then
		-- A ring has no ends, so two anchors are seeded before RDP can run: the
		-- first candidate and the one furthest along the ring from it. Anything
		-- else would let a single chord span the whole loop.
		if nCand < 2 then
			local a, b = 1, (n // 2) + 1
			keep[a] = true; keep[b] = true
			stats.splits += reduce(pts, cand, nCand, epsSq, keep, stack, a, b, 1, nCand)
		else
			local half = (nCand // 2) + 1
			local a, b = cand[1], cand[half]
			keep[a] = true; keep[b] = true
			stats.splits += reduce(pts, cand, nCand, epsSq, keep, stack, a, b, 2, half - 1)
			stats.splits += reduce(pts, cand, nCand, epsSq, keep, stack, b, a, half + 1, nCand)
		end
	else
		keep[1] = true; keep[n] = true
		stats.splits += reduce(pts, cand, nCand, epsSq, keep, stack, 1, n, 1, nCand)
	end

	if o.strict ~= false then
		stats.refined = refine(pts, n, keep, closed, epsSq, stack)
	else
		stats.refined = 0
	end

	local outPts, outIdx, m = {}, {}, 0
	for i = 1, n do
		if keep[i] then
			m += 1
			outPts[m] = pts[i]
			outIdx[m] = i
		end
	end
	stats.kept = m
	return outPts, outIdx, stats
end

-- COLLINEARITY MERGE.
--
-- RDP's epsilon is absolute, and the error that matters here is angular. A 3
-- degree kink halfway along a 20 stud wall puts the midpoint 0.52 studs off the
-- chord, so RDP keeps it correctly by its own rule while the kink is navigation
-- noise. Raising epsilon globally would collapse it but would also start cutting
-- real corners on short features, which never accumulate much absolute error.
--
-- So this pass works on the SEGMENTS: dissolve a vertex whose turn is under
-- mergeAngle, smallest turn first, iterating to a fixed point. The drift it is
-- allowed to introduce scales with the length of the span being merged --
-- mergeRel of the merged chord, floored at mergeMin and capped at mergeMax --
-- which is the scale-free criterion RDP alone cannot express.
--
-- `validate` is the reason this can afford to be aggressive: the caller passes a
-- raycast, and any merge that would put the line through a wall is refused
-- whatever the numbers say.
PathSimplify.mergeAngle = 12    -- degrees; a turn under this is a candidate
PathSimplify.mergeRel = 0.05   -- allowed drift as a fraction of merged length
PathSimplify.mergeMin = 0.35   -- studs; drift floor for short spans
PathSimplify.mergeMax = 2.0    -- studs; drift ceiling however long the span

-- worst perpendicular distance of orig[from..to] from the chord, walking forward
local function spanDev(orig: {Vector3}, nOrig: number, from: number, to: number,
	closed: boolean): number
	local A, B = orig[from], orig[to]
	local ax, ay, az = A.X, A.Y, A.Z
	local bx, by, bz = B.X, B.Y, B.Z
	local worst = 0
	local i = from
	local guard = 0
	while i ~= to and guard <= nOrig do
		local p = orig[i]
		local d = distSq(p.X, p.Y, p.Z, ax, ay, az, bx, by, bz)
		if d > worst then worst = d end
		if closed then i = (i % nOrig) + 1 else i += 1; if i > nOrig then break end end
		guard += 1
	end
	return worst
end

function PathSimplify.merge(pts: {Vector3}, idx: {number}, orig: {Vector3}, opts: any?)
	local o = opts or {}
	local closed = o.closed == true
	local angleTol = o.mergeAngle or PathSimplify.mergeAngle
	local relTol = o.mergeRel or PathSimplify.mergeRel
	local minTol = o.mergeMin or PathSimplify.mergeMin
	local maxTol = o.mergeMax or PathSimplify.mergeMax
	local validate = o.validate
	local cosTol = math.cos(math.rad(angleTol))
	local cos2 = cosTol * cosTol
	local nOrig = #orig
	local m = #pts
	local stats = { input = m, merged = 0, refusedDrift = 0, refusedRay = 0 }
	if m < (closed and 4 or 3) then
		return pts, idx, stats
	end

	local alive = table.create(m, true)
	-- a vertex whose merge was refused is locked out of contention, or the sweep
	-- would keep choosing the same flattest one forever
	local locked = {}
	local prev, nxt = table.create(m), table.create(m)
	for i = 1, m do
		prev[i] = (i == 1) and (closed and m or 1) or (i - 1)
		nxt[i] = (i == m) and (closed and 1 or m) or (i + 1)
	end
	local count = m

	local changed = true
	while changed do
		changed = false
		-- smallest turn first: pick the flattest surviving vertex each sweep
		local bestAt, bestCos = nil, -2
		for v = 1, m do
			if alive[v] and not locked[v] and (closed or (v ~= 1 and v ~= m)) then
				local p, n = prev[v], nxt[v]
				local ux = pts[v].X - pts[p].X
				local uy = pts[v].Y - pts[p].Y
				local uz = pts[v].Z - pts[p].Z
				local wx = pts[n].X - pts[v].X
				local wy = pts[n].Y - pts[v].Y
				local wz = pts[n].Z - pts[v].Z
				local d = ux * wx + uy * wy + uz * wz
				if d > 0 then
					local lu = ux * ux + uy * uy + uz * uz
					local lw = wx * wx + wy * wy + wz * wz
					if d * d >= cos2 * lu * lw then
						-- flatter than the threshold; rank by how flat
						local q = (lu * lw > 0) and (d * d / (lu * lw)) or 0
						if q > bestCos then bestCos = q; bestAt = v end
					end
				end
			end
		end
		if bestAt then
			local v = bestAt
			local p, n = prev[v], nxt[v]
			local chord = (pts[n] - pts[p]).Magnitude
			local tol = math.clamp(relTol * chord, minTol, maxTol)
			local dev = spanDev(orig, nOrig, idx[p], idx[n], closed)
			local ok = dev <= tol * tol
			if not ok then
				stats.refusedDrift += 1
			elseif validate and not validate(pts[p], pts[n]) then
				ok = false
				stats.refusedRay += 1
			end
			if ok and count > (closed and 3 or 2) then
				alive[v] = false
				nxt[p] = n
				prev[n] = p
				count -= 1
				stats.merged += 1
				changed = true
				-- the two survivors have new geometry, so give them another chance
				locked[p] = nil
				locked[n] = nil
			else
				locked[v] = true
				changed = true
			end
		end
	end

	local outPts, outIdx, k = {}, {}, 0
	for i = 1, m do
		if alive[i] then k += 1; outPts[k] = pts[i]; outIdx[k] = idx[i] end
	end
	stats.kept = k
	return outPts, outIdx, stats
end

-- JOG REMOVAL.
--
-- A jog is a short link between two runs that head the SAME way but sit offset
-- sideways by a cell or so. It is not a bevel: the line steps across and then
-- carries on, so both vertices turn hard and neither can be dissolved on its
-- own. The collinearity pass therefore cannot touch them, because removing
-- either one alone swings the line by the whole offset.
--
-- They come out only as a PAIR. The test is on the triple: a short middle
-- segment whose two neighbours are near parallel and pointing the same way.
-- Both vertices go at once and the two runs join directly, which costs about
-- half the offset in drift on each side.
PathSimplify.jogMax = 1.5        -- studs; longest link that counts as a jog
PathSimplify.jogParallel = 20    -- degrees; how parallel the two runs must be

function PathSimplify.dejog(pts: {Vector3}, idx: {number}, orig: {Vector3}, opts: any?)
	local o = opts or {}
	local closed = o.closed == true
	local jogMax = o.jogMax or PathSimplify.jogMax
	local par = math.cos(math.rad(o.jogParallel or PathSimplify.jogParallel))
	local relTol = o.mergeRel or PathSimplify.mergeRel
	local minTol = o.mergeMin or PathSimplify.mergeMin
	local maxTol = o.mergeMax or PathSimplify.mergeMax
	local validate = o.validate
	local nOrig = #orig
	local m = #pts
	local stats = { input = m, removed = 0, refusedDrift = 0, refusedRay = 0, kept = m }
	if m < (closed and 5 or 4) then return pts, idx, stats end

	local alive = table.create(m, true)
	local locked = {}
	local prev, nxt = table.create(m), table.create(m)
	for i = 1, m do
		prev[i] = (i == 1) and (closed and m or 1) or (i - 1)
		nxt[i] = (i == m) and (closed and 1 or m) or (i + 1)
	end
	local count = m

	local changed = true
	while changed do
		changed = false
		-- shortest jog first
		local bestAt, bestLen = nil, math.huge
		for v = 1, m do
			if alive[v] and not locked[v] then
				local w = nxt[v]
				local a, b = prev[v], nxt[w]
				if alive[w] and a ~= w and b ~= v and a ~= b then
					local link = pts[w] - pts[v]
					local len = link.Magnitude
					if len > 1e-6 and len <= jogMax then
						local da = pts[v] - pts[a]
						local db = pts[b] - pts[w]
						if da.Magnitude > 1e-6 and db.Magnitude > 1e-6 then
							-- same heading, not merely the same axis
							if da.Unit:Dot(db.Unit) >= par and len < bestLen then
								bestLen = len; bestAt = v
							end
						end
					end
				end
			end
		end
		if bestAt then
			local v = bestAt
			local w = nxt[v]
			local a, b = prev[v], nxt[w]
			local chord = (pts[b] - pts[a]).Magnitude
			local tol = math.clamp(relTol * chord, minTol, maxTol)
			local dev = spanDev(orig, nOrig, idx[a], idx[b], closed)
			local ok = dev <= tol * tol
			if not ok then
				stats.refusedDrift += 1
			elseif validate and not validate(pts[a], pts[b]) then
				ok = false
				stats.refusedRay += 1
			end
			if ok and count - 2 >= (closed and 3 or 2) then
				alive[v] = false; alive[w] = false
				nxt[a] = b; prev[b] = a
				count -= 2
				stats.removed += 1
				locked[a] = nil; locked[b] = nil
				changed = true
			else
				locked[v] = true
				changed = true
			end
		end
	end

	local outPts, outIdx, k = {}, {}, 0
	for i = 1, m do
		if alive[i] then k += 1; outPts[k] = pts[i]; outIdx[k] = idx[i] end
	end
	stats.kept = k
	return outPts, outIdx, stats
end

local function segment(a: Vector3, b: Vector3, thick: number, colour: Color3,
	mat: Enum.Material, name: string, parent: Instance)
	local d = b - a
	local len = d.Magnitude
	if len < 1e-4 then return end
	local p = Instance.new("Part")
	p.Anchored = true; p.CanCollide = false; p.CanQuery = false; p.CanTouch = false
	p.Size = Vector3.new(thick, thick, len)
	p.CFrame = CFrame.lookAt(a + d * 0.5, b)
	p.Color = colour
	p.Material = mat
	p.Name = name
	p.Parent = parent
end

-- Render the input against the result so the parameters can be tuned by eye.
-- Original is thin and dim, simplified is thick and neon, and every kept node
-- gets a ball so a corner is countable.
function PathSimplify.visualize(original: {Vector3}, simplified: {Vector3}, opts: any?)
	local o = opts or {}
	local parent = o.parent or workspace
	local name = o.name or "PathSimplify"
	local lift = o.lift or Vector3.new(0, 0.3, 0)
	local closed = o.closed == true
	local old = parent:FindFirstChild(name)
	if old then old:Destroy() end
	local root = Instance.new("Folder"); root.Name = name; root.Parent = parent
	local rawF = Instance.new("Folder"); rawF.Name = "Original"; rawF.Parent = root
	local simF = Instance.new("Folder"); simF.Name = "Simplified"; simF.Parent = root
	local cornF = Instance.new("Folder"); cornF.Name = "Corners"; cornF.Parent = root

	local dim = o.originalColor or Color3.fromRGB(90, 95, 110)
	local bright = o.simplifiedColor or Color3.fromRGB(60, 230, 255)
	local cornerCol = o.cornerColor or Color3.fromRGB(255, 200, 40)

	local nO = #original
	for i = 1, closed and nO or (nO - 1) do
		local j = (i % nO) + 1
		segment(original[i] + lift, original[j] + lift, 0.05, dim,
			Enum.Material.SmoothPlastic, "raw", rawF)
	end
	local nS = #simplified
	for i = 1, closed and nS or (nS - 1) do
		local j = (i % nS) + 1
		segment(simplified[i] + lift, simplified[j] + lift, 0.14, bright,
			Enum.Material.Neon, "line", simF)
	end
	for i = 1, nS do
		local b = Instance.new("Part")
		b.Anchored = true; b.CanCollide = false; b.CanQuery = false; b.CanTouch = false
		b.Shape = Enum.PartType.Ball
		b.Size = Vector3.new(0.3, 0.3, 0.3)
		b.Color = cornerCol
		b.Material = Enum.Material.Neon
		b.CFrame = CFrame.new(simplified[i] + lift)
		b.Name = "corner" .. i
		b.Parent = cornF
	end
	return root, nO, nS
end

return PathSimplify
