--!strict
-- NVGN.GridPortals -- links between regions, decided on the grid and carried
-- onto the mesh.
--
-- Portals.build finds these on the finished polygons: it claims cells into
-- polygons, looks for polygon edges that face each other, walks them against
-- Severance's cell pairs, fits a gate where no edges face, and merges the
-- dashed rows that leaves. Each region's outline is simplified on its own, so by
-- then the two sides of a step no longer line up, and every stage guesses.
--
-- Here the grid decides and the mesh is told:
--
--   1. FACES   every boundary face of a traced region is labelled with what is
--              across it: a cell of region B it steps to (a Severance pair lying
--              past the face), or untraced floor that reaches B within
--              crossLimit (a bridge). Two rays, knee and chest, from above the
--              higher cell refuse a pair through a thin wall -- Severance's gate
--              has no wall test.
--   2. RUNS    on each finished loop, consecutive raw nodes whose faces open
--              into the same region are one run. A gap of up to gapFaces
--              unlabelled faces is closed unless a ray blocked it.
--   3. CARRY   `rawIdx` says which outline edge holds each raw node, the same
--              provenance edgeKinds uses, so a run lands on the outline without
--              searching. Its outline stretch is matched to the region's rim
--              polygon edges lying on it, and every overlap of an A rim edge with
--              a facing B rim edge of a matching run is one link.
--
-- Nothing upstream changes: outline, CDT and mesh are bit-identical. Shared
-- edges inside a region come from Portals as before.

local GridPortals = {}

local Portals = require(script.Parent:WaitForChild("Portals"))
local Agents = require(script.Parent:WaitForChild("Agents"))

GridPortals.rays = { 0.4 }       -- studs over the higher side the column from the lower side runs to
GridPortals.bandLo = 1.0         -- studs over the higher side the body sweep starts
GridPortals.bladeWidth = 0.2     -- studs; the swept box is a thin blade, not a body's width
GridPortals.gapFaces = 2         -- unlabelled faces a run may carry through
GridPortals.edgeTol = 0.15       -- studs a rim edge may sit off its outline edge (straighten drops 0.1)
GridPortals.facingAngle = 45     -- degrees off antiparallel the two sides may be
GridPortals.seamGap = 2.5        -- studs in plane between the two sides of a seam
GridPortals.bridgeGap = 4.0      -- and of a bridge
GridPortals.minSpan = 0.05       -- studs; shorter overlaps are not links
GridPortals.halfFace = 0.25      -- studs a run reaches past its end nodes
GridPortals.riseSlack = 0.5      -- studs over the largest step the two sides may differ in height
-- A pair opens a face when the partner lies past it: forward of the face by at
-- least this share of how far it is to the side (0.5 is about 63 degrees off
-- the face normal). At 1.0 (45 degrees) the right partner sat to the side of
-- every face of a small region and the link was lost.
GridPortals.pastRatio = 0.5
GridPortals.matchReach = 0.8   -- studs between a run's stepped-to cell and the far run's cell
-- A one-sided run's far side must come this close to a cell it stepped to: the
-- whole far rim within seamGap reached across a wall (Cocosulx's "bad").
GridPortals.fallbackReach = 1.0

local function vkey(p: Vector3): string
	return ("%.3f,%.3f,%.3f"):format(p.X, p.Y, p.Z)
end

local function hkey(x: number, y: number, z: number): number
	return x * 73856093 + y * 19349663 + z * 83492791
end

local function flatten(v: Vector3, up: Vector3): Vector3
	return v - up * v:Dot(up)
end

local function heightAt(f: any, p: Vector3): number
	local c, up = f.centre, f.up or Vector3.yAxis
	if math.abs(up.Y) < 1e-3 then return c.Y end
	return c.Y - ((p.X - c.X) * up.X + (p.Z - c.Z) * up.Z) / up.Y
end

-- Untraced floor: which traced regions each untraced cell reaches, and how far.
-- The same relaxation as Portals' bridge links, by region instead of polygon.
-- Each entry also keeps `src`, the traced cell of that region the walk started
-- from, so a bridge can be tested end to end and not only at its first step.
local function reachUntraced(data: any, snap: any, traced: { [number]: any }, limit: number): ({ [any]: { [number]: number } }, { [any]: { [number]: any } })
	local step = data.config.step
	local reachOf: { [any]: { [number]: number } } = {}
	local srcOf: { [any]: { [number]: any } } = {}
	local free = {}
	for _, g in ipairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			if cell.region and not traced[cell.region] then free[#free + 1] = cell end
		end
	end
	if #free == 0 then return reachOf, srcOf end
	local reach = step * 1.6
	local G = reach
	local bucket: { [number]: { any } } = {}
	for _, cell in ipairs(free) do
		local p = cell.pos
		local k = hkey(math.floor(p.X / G), math.floor(p.Y / G), math.floor(p.Z / G))
		local b = bucket[k]
		if not b then b = {}; bucket[k] = b end
		b[#b + 1] = cell
	end
	local queue = {}
	local function relax(cell: any, r: number, d: number, src: any)
		if d > limit then return end
		local t = reachOf[cell]
		if not t then t = {}; reachOf[cell] = t; srcOf[cell] = {} end
		if t[r] and t[r] <= d then return end
		t[r] = d
		srcOf[cell][r] = src
		queue[#queue + 1] = { cell, r }
	end
	for _, pr in ipairs(snap.pairs) do
		local ta, tb = traced[pr.a.region], traced[pr.b.region]
		if ta and not tb then relax(pr.b, pr.a.region, 0, pr.a)
		elseif tb and not ta then relax(pr.a, pr.b.region, 0, pr.b) end
	end
	local head = 1
	while head <= #queue do
		local e = queue[head]; head += 1
		local cell, r = e[1], e[2]
		local d = reachOf[cell][r]
		local p = cell.pos
		local bx, by, bz = math.floor(p.X / G), math.floor(p.Y / G), math.floor(p.Z / G)
		for dx = -1, 1 do for dy = -1, 1 do for dz = -1, 1 do
			for _, q in ipairs(bucket[hkey(bx + dx, by + dy, bz + dz)] or {}) do
				if q ~= cell then
					local dd = (q.pos - p).Magnitude
					if dd <= reach then relax(q, r, d + dd, srcOf[cell][r]) end
				end
			end
		end end end
	end
	return reachOf, srcOf
end

function GridPortals.build(mesh: any, data: any, snap: any, loops: { any }, rayExclude: { Instance }?): any
	local t0 = os.clock()
	local traced = data.boundary
	local cosFacing = math.cos(math.rad(GridPortals.facingAngle))
	local stats = {
		polys = #mesh.tris, shared = 0, seam = 0, bridge = 0,
		faces = 0, facesOpen = 0, facesBridge = 0, facesBlocked = 0, facesSideways = 0,
		runs = 0, runsMatched = 0, runsFallbackRim = 0, runsFallbackInside = 0, runsUnlinked = 0,
		gapCloses = 0, nodesUncovered = 0, piecesMissing = 0, duplicates = 0, corners = 0,
		orphans = {}, pieces = 0, unlinkedAt = {},
		tFaces = 0, tRuns = 0, tLinks = 0, seconds = 0,
		-- the fields Portals' helpers write into
		edges = 0, overshared = 0, oversharedAt = {}, crossRegionKeys = 0,
		cellsTried = 0, cellsInside = 0, cellsNearby = 0, cellsNoPolygon = 0,
	}

	local links: { any } = {}
	Portals.internal.sharedLinks(mesh, links, stats)
	local of = Portals.internal.claimCells(mesh, data, stats)

	local rp = RaycastParams.new()
	rp.FilterType = Enum.RaycastFilterType.Exclude
	rp.FilterDescendantsInstances = rayExclude or {}
	rp.IgnoreWater = true
	-- THE BODY'S BAND, SWEPT. Rays at two heights let whatever stood between
	-- them through, and every fix so far patched one more gap. A thin box as
	-- tall as the body is swept instead, from bandLo over the higher side to a
	-- crouch: anything lower than bandLo is a step walked over, anything in the
	-- band stops a body. bandLo also keeps a steep roof's own crease, 0.8 over
	-- both sides of a hip, from reading as a wall.
	-- THE BAND STOPS UNDER THE LOWER SIDE'S OWN HEADROOM. A prone-only crawl
	-- space was swept for a crouching body: the blade clipped the slab over it
	-- from outside and started inside it from under it, so alternate faces of
	-- one edge came out blocked (case5 r047, a seam in 0.5 stud dashes).
	local env = Agents.envelope()
	local crouch = (env.crouch == math.huge) and 3 or env.crouch
	local function blocked(a: Vector3, b: Vector3, head: number?): boolean
		local bandHi = math.min(crouch, head or math.huge) - 0.1
		local hi = math.max(a.Y, b.Y)
		-- UP FROM THE LOWER SIDE to where the crossing runs. A crawl space under
		-- a roof pairs with the roof's own top a step above, and a sweep over
		-- the higher side passes over the slab between (Cocosulx's "impossible").
		-- A stair riser stands beside the lower side, never over it.
		local low = (a.Y <= b.Y) and a or b
		local rise = hi + GridPortals.rays[1] - low.Y
		if rise > 0.2 and workspace:Raycast(low + Vector3.yAxis * 0.1, Vector3.yAxis * (rise - 0.1), rp) then return true end
		if bandHi - GridPortals.bandLo < 0.1 then return false end
		local bandSize = Vector3.new(GridPortals.bladeWidth, bandHi - GridPortals.bandLo, GridPortals.bladeWidth)
		local bandMid = (GridPortals.bandLo + bandHi) * 0.5
		local p1 = Vector3.new(a.X, hi + bandMid, a.Z)
		local p2 = Vector3.new(b.X, hi + bandMid, b.Z)
		local d = p2 - p1
		if d.Magnitude < 1e-3 then return false end
		return workspace:Blockcast(CFrame.new(p1), bandSize, d, rp) ~= nil
	end
	-- a gate between its two sides: the same test
	local gateBlocked = blocked

	-- ------------------------------------------------------------ 1. faces
	local partners: { [any]: { any } } = {}
	for _, pr in ipairs(snap.pairs) do
		local ta = partners[pr.a]; if not ta then ta = {}; partners[pr.a] = ta end
		ta[#ta + 1] = pr.b
		local tb = partners[pr.b]; if not tb then tb = {}; partners[pr.b] = tb end
		tb[#tb + 1] = pr.a
	end
	local limit = Portals.crossLimit or data.config.traceMinWidth or data.config.minWidth or (data.config.step * 3)
	local reachOf, srcOf = reachUntraced(data, snap, traced, limit)

	-- label[face] = { target, kind, via }; blockedFace[face] = true
	local label: { [any]: any } = {}
	local blockedFace: { [any]: boolean } = {}
	local regions = {}
	for r in pairs(traced) do regions[#regions + 1] = r end
	table.sort(regions)
	for _, r in ipairs(regions) do
		for _, f in ipairs(traced[r].faces) do
			stats.faces += 1
			local d = f.b - f.a
			if d.Magnitude < 1e-9 then continue end
			local t = d.Unit
			local up = f.up
			local o = t:Cross(up) -- outward: the floor is left of travel
			local m = (f.a + f.b) * 0.5
			local best, bd, bestU, bu = nil, math.huge, nil, math.huge
			local sideways = false
			for _, q in ipairs(partners[f.cell] or {}) do
				local v = flatten(q.pos - m, up)
				local along, side = v:Dot(o), math.abs(v:Dot(t))
				if along > 0 and along >= GridPortals.pastRatio * side then
					local dist = v.Magnitude
					if traced[q.region] then
						if q.region ~= r and dist < bd then best, bd = q, dist end
					elseif reachOf[q] and dist < bu then
						bestU, bu = q, dist
					end
				else
					sideways = true
				end
			end
			local target, kind, via = nil, nil, nil
			if best then
				target, kind, via = best.region, "seam", best
			elseif bestU then
				local bdist = math.huge
				for R, dd in pairs(reachOf[bestU]) do
					if R ~= r and (dd < bdist or (dd == bdist and R < (target :: any))) then
						target, bdist = R, dd
					end
				end
				if target then kind, via = "bridge", bestU end
			end
			if target then
				-- a bridge is tested at both ends: onto the strip, and off it onto the
				-- far region's own cell (a crawl space under a roof panel bridged to
				-- the panel's top, Cocosulx's "impossible1")
				local far = (kind == "bridge") and srcOf[via] and srcOf[via][target] or nil
				if blocked(f.cell.pos, via.pos, math.min(f.cell.clearance, via.clearance))
					or (far and blocked(via.pos, far.pos, math.min(via.clearance, far.clearance))) then
					blockedFace[f] = true
					stats.facesBlocked += 1
				else
					label[f] = { target = target, kind = kind, via = via,
						viaRegion = via.region }
					stats.facesOpen += 1
					if kind == "bridge" then stats.facesBridge += 1 end
				end
			elseif sideways then
				stats.facesSideways += 1
			end
		end
	end
	stats.tFaces = os.clock() - t0

	-- ------------------------------------------------------------- 2. runs
	local t1 = os.clock()
	local runs = {}
	local runsBy: { [string]: { any } } = {}   -- "A>B:kind" -> runs of region A facing B
	for li, L in ipairs(loops) do
		local entry = traced[L.region]
		local n = #L.poly
		local m = #L.pts
		if not entry or n < 2 or m < 2 then continue end
		local closed = L.closed

		-- where each raw node sits on the outline: edge index and fraction
		local nodeEdge, nodeT = table.create(n, 0), table.create(n, 0)
		local last = closed and m or m - 1
		for i = 1, last do
			local a, b = L.rawIdx[i], L.rawIdx[i % m + 1]
			if a and b then
				local total, j, guard = 0, a, 0
				while j ~= b and guard < n do
					local k = j % n + 1
					total += (L.poly[k] - L.poly[j]).Magnitude
					j = k; guard += 1
				end
				local cum = 0
				j, guard = a, 0
				while guard < n do
					nodeEdge[j] = i
					nodeT[j] = total > 1e-9 and cum / total or 0
					if j == b then break end
					local k = j % n + 1
					cum += (L.poly[k] - L.poly[j]).Magnitude
					j = k; guard += 1
				end
			end
		end

		local lab = table.create(n, false)
		local nodeFace = table.create(n, false)
		local isBlocked = table.create(n, false)
		for j = 1, n do
			local fi = L.faceOf[j]
			local f = fi and entry.faces[fi]
			if f then
				nodeFace[j] = f
				local lb = label[f]
				if lb then lab[j] = lb.target .. ":" .. lb.kind end
				if blockedFace[f] then isBlocked[j] = true end
			end
		end
		local function idx(j: number): number?
			if closed then return (j - 1) % n + 1 end
			if j < 1 or j > n then return nil end
			return j
		end
		-- close short gaps no ray blocked
		for j = 1, n do
			if lab[j] then
				for g = 1, GridPortals.gapFaces do
					local kg = idx(j + g)
					if not kg or lab[kg] then break end
					local k1 = idx(j + g + 1)
					if not k1 then break end
					if lab[k1] and lab[k1] ~= lab[j] then break end
					if lab[k1] == lab[j] then
						local ok = true
						for q = 1, g do if isBlocked[idx(j + q) :: number] then ok = false end end
						if ok then
							for q = 1, g do
								local kq = idx(j + q) :: number
								lab[kq] = lab[j]
								nodeFace[kq] = nodeFace[j] -- carries the neighbour's label
							end
							stats.gapCloses += 1
						end
						break
					end
				end
			end
		end

		-- runs, starting at a label change so a ring is not cut at node 1
		local start = 1
		local whole = false
		if closed then
			start = -1
			for j = 1, n do
				if lab[j] ~= lab[(j - 2) % n + 1] then start = j break end
			end
			if start == -1 then start = 1; whole = true end
		end
		local cur: any = nil
		local function flush()
			if not cur then return end
			runs[#runs + 1] = cur
			local k = cur.region .. ">" .. cur.target .. ":" .. cur.kind
			local b = runsBy[k]; if not b then b = {}; runsBy[k] = b end
			b[#b + 1] = cur
			cur = nil
		end
		local j = start
		for _ = 1, n do
			local l = lab[j]
			if cur and l ~= cur.key then flush() end
			if l then
				if not cur then
					local tgt, kind = l:match("^(%d+):(%a+)$")
					cur = { key = l, loop = L, li = li, region = L.region, target = tonumber(tgt), kind = kind,
						nodes = {}, cells = {}, partnerCells = {}, viaRegions = {}, whole = whole,
						nodeEdge = nodeEdge, nodeT = nodeT, head = math.huge }
				end
				cur.nodes[#cur.nodes + 1] = j
				local f = nodeFace[j]
				if f then
					cur.cells[f.cell] = true
					cur.head = math.min(cur.head, f.cell.clearance)
					local lb = label[f]
					if lb then
						cur.partnerCells[lb.via] = true
						cur.head = math.min(cur.head, lb.via.clearance)
						cur.viaRegions[lb.viaRegion] = true
					end
				end
				if L.rawIdx and nodeEdge[j] == 0 then stats.nodesUncovered += 1 end
			end
			j = j % n + 1
			if not closed and j == 1 then break end
		end
		flush()
	end
	stats.runs = #runs
	stats.tRuns = os.clock() - t1

	-- ------------------------------------------------------ 3. rim edges
	local t2 = os.clock()
	local rimByRegion: { [number]: { any } } = {}
	do
		local uses: { [string]: number } = {}
		local function ek(r: number, a: Vector3, b: Vector3): string
			local ka, kb = vkey(a), vkey(b)
			return r .. "#" .. ((ka < kb) and (ka .. "|" .. kb) or (kb .. "|" .. ka))
		end
		for _, f in ipairs(mesh.tris) do
			for k = 1, #f.verts do
				local key = ek(f.region, f.verts[k], f.verts[k % #f.verts + 1])
				uses[key] = (uses[key] or 0) + 1
			end
		end
		for i, f in ipairs(mesh.tris) do
			for k = 1, #f.verts do
				local a, b = f.verts[k], f.verts[k % #f.verts + 1]
				if (b - a).Magnitude > 1e-6 and uses[ek(f.region, a, b)] == 1 then
					local list = rimByRegion[f.region]
					if not list then list = {}; rimByRegion[f.region] = list end
					list[#list + 1] = { poly = i, a = a, b = b }
				end
			end
		end
	end

	-- a stretch of outline -> the rim polygon edges lying on it
	local function piecesOn(region: number, p: Vector3, q: Vector3, out: { any })
		local d = q - p
		local len = d.Magnitude
		if len < 1e-6 then return end
		local u = d / len
		local tol = GridPortals.edgeTol
		for _, E in ipairs(rimByRegion[region] or {}) do
			local dE = E.b - E.a
			if dE:Dot(u) <= 0 then continue end
			local ta, tb = (E.a - p):Dot(u), (E.b - p):Dot(u)
			local lo, hi = math.max(0, ta), math.min(len, tb)
			if hi - lo < GridPortals.minSpan then continue end
			-- the rim edge at both ends of the overlap must lie on the stretch
			local function onE(t: number): Vector3
				return E.a + dE * math.clamp((t - ta) / math.max(tb - ta, 1e-9), 0, 1)
			end
			local e1, e2 = onE(lo), onE(hi)
			if (e1 - (p + u * lo)).Magnitude <= tol and (e2 - (p + u * hi)).Magnitude <= tol then
				out[#out + 1] = { poly = E.poly, p1 = e1, p2 = e2 }
			end
		end
	end

	-- a run -> its pieces on region A's polygon edges
	local function runPieces(R: any): { any }
		if R.pieces then return R.pieces end
		local L = R.loop
		local pts = L.pts
		local m = #pts
		local out = {}
		local function P(i: number): Vector3 return pts[(i - 1) % m + 1] end
		local segs = {}
		if R.whole then
			for i = 1, (L.closed and m or m - 1) do segs[#segs + 1] = { P(i), P(i + 1) } end
		else
			local j0, j1 = R.nodes[1], R.nodes[#R.nodes]
			local e0, e1 = R.nodeEdge[j0], R.nodeEdge[j1]
			if e0 == 0 or e1 == 0 then
				stats.piecesMissing += 1
				R.pieces = out
				return out
			end
			local function edgeLen(e: number): number return (P(e + 1) - P(e)).Magnitude end
			local tS = math.max(0, R.nodeT[j0] - GridPortals.halfFace / math.max(edgeLen(e0), 1e-6))
			local tE = math.min(1, R.nodeT[j1] + GridPortals.halfFace / math.max(edgeLen(e1), 1e-6))
			-- forward from e0 to e1; a run that ends before it starts on one edge
			-- goes all the way round the ring first
			local e = e0
			for k = 1, m + 1 do
				local first = k == 1
				local ends = e == e1 and (not first or tE >= tS)
				local s = first and tS or 0
				local t = ends and tE or 1
				if t > s then
					segs[#segs + 1] = { P(e):Lerp(P(e + 1), s), P(e):Lerp(P(e + 1), t) }
				end
				if ends then break end
				e = e % m + 1
			end
		end
		for _, sg in ipairs(segs) do piecesOn(R.region, sg[1], sg[2], out) end
		if #out == 0 then stats.piecesMissing += 1 end
		R.pieces = out
		return out
	end

	-- --------------------------------------------------------- 4. links
	local seen: { [string]: boolean } = {}
	local riseMax = Agents.envelope().step + GridPortals.riseSlack
	local fails: { [string]: number } = {}
	local function fail(why: string): boolean
		fails[why] = (fails[why] or 0) + 1
		return false
	end
	local function emitPair(A: any, B: any, kind: string, gapMax: number, count: number, head: number?): boolean
		local fa = mesh.tris[A.poly]
		local up = fa.up or Vector3.yAxis
		local da = flatten(A.p2 - A.p1, up)
		local la = da.Magnitude
		if la < GridPortals.minSpan then return fail("short A") end
		local ua = da / la
		local db = flatten(B.p2 - B.p1, up)
		if db.Magnitude < 1e-6 or ua:Dot(db.Unit) > -cosFacing then return fail("not facing") end
		local o = ua:Cross(up)
		local t1, t2 = flatten(B.p1 - A.p1, up):Dot(ua), flatten(B.p2 - A.p1, up):Dot(ua)
		local lo, hi = math.max(0, math.min(t1, t2)), math.min(la, math.max(t1, t2))
		if hi - lo < GridPortals.minSpan then return fail("no overlap") end
		local function onA(t: number): Vector3 return A.p1:Lerp(A.p2, t / la) end
		local function onB(t: number): Vector3
			return B.p1:Lerp(B.p2, math.clamp((t - t1) / (t2 - t1), 0, 1))
		end
		local aL, aR = onA(lo), onA(hi)
		local bL, bR = onB(hi), onB(lo)
		local g1 = flatten(bR - aL, up):Dot(o)
		local g2 = flatten(bL - aR, up):Dot(o)
		if g1 < -0.1 or g2 < -0.1 or g1 > gapMax or g2 > gapMax then return fail("gap") end
		-- a bridge chains two crossings, so it may change height twice
		local rmax = (kind == "bridge") and 2 * riseMax or riseMax
		if math.abs((bR - aL):Dot(up)) > rmax or math.abs((bL - aR):Dot(up)) > rmax then return fail("rise") end
		local ca, cb = (aL + aR) * 0.5, (bL + bR) * 0.5
		-- the gate itself must be crossable, not only the faces behind it
		if gateBlocked(ca, cb, head) then return fail("wall") end
		local mid = (ca + cb) * 0.5
		local key = math.min(A.poly, B.poly) .. ":" .. math.max(A.poly, B.poly) .. ":"
			.. math.floor(mid.X * 2 + 0.5) .. ":" .. math.floor(mid.Y * 2 + 0.5) .. ":" .. math.floor(mid.Z * 2 + 0.5)
		if seen[key] then stats.duplicates += 1; return true end
		seen[key] = true
		links[#links + 1] = {
			kind = kind, a = A.poly, b = B.poly,
			left = aL, right = aR, bLeft = bL, bRight = bR,
			centre = ca, span = (aR - aL).Magnitude,
			drop = (ca - cb):Dot(up), gap = (g1 + g2) * 0.5,
			count = count, residual = 0, fitted = true, edge = true, source = "grid",
		}
		if kind == "bridge" then stats.bridge += 1 else stats.seam += 1 end
		return true
	end

	-- CORNER CONTACT. Two regions touching at one cell, a tread corner or a
	-- small piece, meet on edges that do not face each other, so there is no
	-- overlap to project. The gate is then A's whole piece and the nearest
	-- stretch of B's, when they come within gapMax and a step in height.
	local function segDist(p: Vector3, a: Vector3, b: Vector3, up: Vector3): (number, Vector3)
		local d = b - a
		local dd = flatten(d, up):Dot(flatten(d, up))
		local t = dd > 1e-12 and math.clamp(flatten(p - a, up):Dot(flatten(d, up)) / dd, 0, 1) or 0
		local q = a + d * t
		return flatten(q - p, up).Magnitude, q
	end
	local function emitCorner(A: any, B: any, kind: string, gapMax: number, count: number, head: number?): boolean
		local fa = mesh.tris[A.poly]
		local up = fa.up or Vector3.yAxis
		local d1, bR = segDist(A.p1, B.p1, B.p2, up)
		local d2, bL = segDist(A.p2, B.p1, B.p2, up)
		local d3 = segDist(B.p1, A.p1, A.p2, up)
		local d4 = segDist(B.p2, A.p1, A.p2, up)
		if math.min(d1, d2, d3, d4) > gapMax then return fail("corner gap") end
		local rmax = (kind == "bridge") and 2 * riseMax or riseMax
		if math.abs((bR - A.p1):Dot(up)) > rmax or math.abs((bL - A.p2):Dot(up)) > rmax then return fail("corner rise") end
		local ca, cb = (A.p1 + A.p2) * 0.5, (bL + bR) * 0.5
		if gateBlocked(ca, cb, head) then return fail("corner wall") end
		local mid = (ca + cb) * 0.5
		local key = math.min(A.poly, B.poly) .. ":" .. math.max(A.poly, B.poly) .. ":"
			.. math.floor(mid.X * 2 + 0.5) .. ":" .. math.floor(mid.Y * 2 + 0.5) .. ":" .. math.floor(mid.Z * 2 + 0.5)
		if seen[key] then stats.duplicates += 1; return true end
		seen[key] = true
		links[#links + 1] = {
			kind = kind, a = A.poly, b = B.poly,
			left = A.p1, right = A.p2, bLeft = bL, bRight = bR,
			centre = ca, span = (A.p2 - A.p1).Magnitude,
			drop = (ca - cb):Dot(up), gap = math.min(d1, d2, d3, d4),
			count = count, residual = 0, fitted = true, edge = true, source = "grid-corner",
		}
		stats.corners += 1
		if kind == "bridge" then stats.bridge += 1 else stats.seam += 1 end
		return true
	end
	-- the nearest pair of pieces, for a corner contact
	local function nearestPair(As: { any }, Bs: { any }): (any, any)
		local best, ba, bb = math.huge, nil, nil
		for _, A in ipairs(As) do
			local up = mesh.tris[A.poly].up or Vector3.yAxis
			for _, B in ipairs(Bs) do
				local d = math.min(segDist(A.p1, B.p1, B.p2, up), segDist(A.p2, B.p1, B.p2, up),
					segDist(B.p1, A.p1, A.p2, up), (segDist(B.p2, A.p1, A.p2, up)))
				if d < best then best, ba, bb = d, A, B end
			end
		end
		return ba, bb
	end

	-- match runs across each pair: seams share a cell pair, bridges an untraced region
	local matchOf: { [any]: { any } } = {}
	local function addMatch(R: any, S: any)
		local t = matchOf[R]; if not t then t = {}; matchOf[R] = t end
		for _, x in ipairs(t) do if x == S then return end end
		t[#t + 1] = S
	end
	for _, R in ipairs(runs) do
		for _, S in ipairs(runsBy[R.target .. ">" .. R.region .. ":" .. R.kind] or {}) do
			local hit = false
			if R.kind == "seam" then
				for c in pairs(R.partnerCells) do if S.cells[c] then hit = true break end end
				if not hit then for c in pairs(S.partnerCells) do if R.cells[c] then hit = true break end end end
				-- two lattices offset against each other: the cell a face steps to
				-- is then an inner cell of the far run, not one of its own face cells
				if not hit then
					for c in pairs(R.partnerCells) do
						for c2 in pairs(S.cells) do
							if (c.pos - c2.pos).Magnitude <= GridPortals.matchReach then hit = true break end
						end
						if hit then break end
					end
				end
			else
				for vr in pairs(R.viaRegions) do if S.viaRegions[vr] then hit = true break end end
			end
			if hit then addMatch(R, S); addMatch(S, R) end
		end
	end

	local function nCells(R: any): number
		local n = 0
		for _ in pairs(R.cells) do n += 1 end
		return n
	end

	for _, R in ipairs(runs) do
		local gapMax = (R.kind == "bridge") and GridPortals.bridgeGap or GridPortals.seamGap
		local ms = matchOf[R]
		if ms then
			stats.runsMatched += 1
			for _, S in ipairs(ms) do
				-- each matched pair once, from the lower region
				if R.region < S.region then
					local made = false
					for _, A in ipairs(runPieces(R)) do
						for _, B in ipairs(runPieces(S)) do
							if emitPair(A, B, R.kind, gapMax, nCells(R), math.min(R.head, S.head)) then made = true end
						end
					end
					if not made then
						local A, B = nearestPair(runPieces(R), runPieces(S))
						if A and B then made = emitCorner(A, B, R.kind, gapMax, nCells(R), math.min(R.head, S.head)) end
					end
					if made then R.made = (R.made or 0) + 1; S.made = (S.made or 0) + 1 end
				end
			end
		else
			-- ONE-SIDED: the far region has no run facing back (a filled hole, a
			-- dropped shadow piece). Match against its whole rim nearby, then
			-- failing that, through the cells the run stepped to.
			local made = false
			local pieces = runPieces(R)
			local function nearPartner(E: any): boolean
				for c in pairs(R.partnerCells) do
					if c.region == R.target or (R.kind == "bridge") then
						if (segDist(c.pos, E.a, E.b, Vector3.yAxis)) <= GridPortals.fallbackReach then return true end
					end
				end
				return false
			end
			local rimNear = {}
			for _, E in ipairs(rimByRegion[R.target] or {}) do
				if R.kind == "bridge" or nearPartner(E) then rimNear[#rimNear + 1] = E end
			end
			for _, A in ipairs(pieces) do
				local lo, hi = A.p1:Min(A.p2), A.p1:Max(A.p2)
				local pad = Vector3.one * (gapMax + 0.5)
				lo, hi = lo - pad, hi + pad
				for _, E in ipairs(rimNear) do
					local c = (E.a + E.b) * 0.5
					local ext = (E.b - E.a).Magnitude * 0.5
					if c.X >= lo.X - ext and c.X <= hi.X + ext and c.Z >= lo.Z - ext and c.Z <= hi.Z + ext
						and c.Y >= lo.Y - ext and c.Y <= hi.Y + ext then
						if emitPair(A, { poly = E.poly, p1 = E.a, p2 = E.b }, R.kind, gapMax, nCells(R), R.head) then made = true end
					end
				end
			end
			if not made then
				local near = {}
				for _, E in ipairs(rimNear) do near[#near + 1] = { poly = E.poly, p1 = E.a, p2 = E.b } end
				local A, B = nearestPair(pieces, near)
				if A and B then made = emitCorner(A, B, R.kind, gapMax, nCells(R), R.head) end
			end
			if made then
				stats.runsFallbackRim += 1
			else
				-- LAST: the cells the run's faces stepped to ARE the far side. Grouped
				-- by the polygon each was claimed into; the far bar runs through them
				-- at their own heights, so it can never sit a storey away.
				for _, A in ipairs(pieces) do
					local fa = mesh.tris[A.poly]
					local up = fa.up or Vector3.yAxis
					local da = flatten(A.p2 - A.p1, up)
					local la = da.Magnitude
					if la < GridPortals.minSpan then continue end
					local ua = da / la
					local o = ua:Cross(up)
					local byPoly: { [number]: any } = {}
					for c in pairs(R.partnerCells) do
						local pb = of[c]
						local v = flatten(c.pos - A.p1, up)
						local t, lat = v:Dot(ua), v:Dot(o)
						if pb and pb ~= A.poly and t >= -0.5 and t <= la + 0.5 and lat >= -0.1 and lat <= gapMax then
							local e = byPoly[pb]
							if not e then e = { n = 0, lo = math.huge, hi = -math.huge, lat = 0 }; byPoly[pb] = e end
							e.n += 1
							e.lo = math.min(e.lo, t); e.hi = math.max(e.hi, t)
							e.lat += lat
						end
					end
					local best, bn = nil, 0
					for pb, e in pairs(byPoly) do
						if e.n > bn or (e.n == bn and pb < (best :: any)) then best, bn = pb, e.n end
					end
					if best then
						local e = byPoly[best]
						local fb = mesh.tris[best]
						local lo = math.clamp(e.lo - 0.25, 0, la)
						local hi = math.clamp(e.hi + 0.25, 0, la)
						if hi - lo < GridPortals.minSpan then hi = math.min(la, lo + GridPortals.minSpan) end
						local g = e.lat / e.n
						local aL, aR = A.p1:Lerp(A.p2, lo / la), A.p1:Lerp(A.p2, hi / la)
						local function across(p: Vector3): Vector3
							local q = p + o * g
							return Vector3.new(q.X, heightAt(fb, q), q.Z)
						end
						local bL, bR = across(aR), across(aL)
						local rmax = (R.kind == "bridge") and 2 * riseMax or riseMax
						if math.abs((bR - aL):Dot(up)) <= rmax and math.abs((bL - aR):Dot(up)) <= rmax then
							local key = math.min(A.poly, best) .. ":" .. math.max(A.poly, best) .. ":cells:" .. vkey(aL)
							if not seen[key] then
								seen[key] = true
								local ca = (aL + aR) * 0.5
								links[#links + 1] = {
									kind = R.kind, a = A.poly, b = best,
									left = aL, right = aR, bLeft = bL, bRight = bR,
									centre = ca, span = (aR - aL).Magnitude,
									drop = (ca - (bL + bR) * 0.5):Dot(up), gap = g,
									count = e.n, residual = 0, fitted = true, edge = true,
									source = "grid-cells",
								}
								if R.kind == "bridge" then stats.bridge += 1 else stats.seam += 1 end
								made = true
							end
						else
							fail("cells rise")
						end
					end
				end
				if made then
					stats.runsFallbackInside += 1
				else
					stats.runsUnlinked += 1
					if #stats.unlinkedAt < 40 then
						local j = R.nodes[1]
						stats.unlinkedAt[#stats.unlinkedAt + 1] = ("r%03d->r%03d %s %d nodes at %s")
							:format(R.region, R.target, R.kind, #R.nodes, tostring(R.loop.poly[j]))
					end
				end
			end
		end
	end
	stats.tLinks = os.clock() - t2

	-- ------------------------------------------------------------ summary
	local degree = {}
	for _, L in ipairs(links) do degree[L.a] = true; degree[L.b] = true end
	for i = 1, stats.polys do
		if not degree[i] then
			stats.orphans[#stats.orphans + 1] =
				("f%04d r%03d %.1fsq"):format(i, mesh.tris[i].region, mesh.tris[i].area)
		end
	end
	local comp, pieces = Portals.internal.components(stats.polys, links)
	stats.pieces = pieces
	stats.seconds = os.clock() - t0
	stats.fails = fails
	return { links = links, comp = comp, polyOf = of, stats = stats, source = "grid",
		faceLabel = label, faceBlocked = blockedFace, runs = runs, matchOf = matchOf }
end

function GridPortals.report(res: any): string
	local s = res.stats
	local lines = {
		("portals   %d polys, %d links: %d shared, %d seam, %d bridge  (grid, %.2fs: faces %.2f, runs %.2f, links %.2f)")
			:format(s.polys, #res.links, s.shared, s.seam, s.bridge, s.seconds, s.tFaces, s.tRuns, s.tLinks),
		("  faces %d: %d open (%d via untraced floor), %d refused by the wall rays, %d with only a sideways pair")
			:format(s.faces, s.facesOpen, s.facesBridge, s.facesBlocked, s.facesSideways),
		("  runs %d: %d matched both sides, %d one-sided met the far rim, %d one-sided from their cells, %d unlinked; %d gaps closed")
			:format(s.runs, s.runsMatched, s.runsFallbackRim, s.runsFallbackInside, s.runsUnlinked, s.gapCloses),
		("  %d corner contacts, %d raw nodes without provenance, %d runs with no rim edge under them, %d duplicate overlaps")
			:format(s.corners, s.nodesUncovered, s.piecesMissing, s.duplicates),
		("  %d components"):format(s.pieces),
	}
	if #s.orphans > 0 then
		lines[#lines + 1] = ("  !! %d polygons with no link at all"):format(#s.orphans)
		for i = 1, math.min(#s.orphans, 8) do lines[#lines + 1] = "     " .. s.orphans[i] end
	end
	for i = 1, math.min(#s.unlinkedAt, 8) do lines[#lines + 1] = "  !  unlinked run " .. s.unlinkedAt[i] end
	return table.concat(lines, "\n")
end

return GridPortals
