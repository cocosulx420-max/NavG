--!strict
local FloorLocal = {}
local UP = Vector3.new(0, 1, 0)

local DEFAULT = {
	maxSlope = 65, clearCap = 20, minClearance = 1.5,
	minSurfaceWidth = 1.0,
	agentFit = true, agentWidth = 2, agentHeight = 5, agentStepHeight = 2, agentCylinder = true,
	nodeOcclusion = true, pointOcclusion = false,
	-- Ground probe: false = a centre ray. A number sweeps a sphere of that radius
	-- down instead, which cannot fall through a crack the ray slips into -- but a
	-- sphere contacts the FIRST thing it touches, so near a ledge it catches the
	-- edge and reports a point that is not under the candidate column.
	groundRadius = false,
	-- Cylinder ground probe. Roblox has no cylindercast, and every SWEEP (sphere
	-- or block) stops at the first thing it touches, so a curb beside the column
	-- hijacks the hit. A disc of rays keeps EVERY sample instead: the centre ray
	-- still defines the floor, and the ring only votes on whether a disc of that
	-- diameter is actually supported. discRadius is a RADIUS -- diameter halved.
	groundDisc = false, discRadius = 1.0, discRings = 1, discSamples = 8,
	discTolerance = 0.35, discMinSupport = 0,
	-- No cast at all: stand a cylinder on the node's top face and keep the spot if
	-- it is empty. Cheapest path, but it inherits the SVO's quantisation -- the
	-- true surface is anywhere within one leaf below the node top -- and it has no
	-- normal, so every spot is assumed flat and the slope filter cannot run.
	groundOverlap = false, overlapDiameter = 1.0, overlapHeight = 0.5,
	-- Recover the normal WITHOUT a cast: a node's faces are the part's own axes,
	-- so the standing face is the one the node sits flush against. Exact on boxes
	-- the node is unambiguously flush with; a node flush against two faces at once
	-- can still pick the wrong one, and on a slanted mesh this reports the
	-- bounding-box direction, not the real surface.
	normalFromNode = false,
	-- Stand the cylinder on the node's FACE, along that face's normal, instead of
	-- world-vertical from the node's world-Y top. On a slanted surface the world-Y
	-- top of a node cube is a CORNER, not the face, so the upright probe starts in
	-- the wrong place and measures the wrong direction. Note this measures
	-- clearance perpendicular to the surface, which is an exposure test -- a
	-- character still stands world-vertical, so slope decides walkability.
	overlapAlignNode = false,
	-- Second, separate check. The aligned cylinder above asks "is this face
	-- exposed", measured along the surface normal, so it should be SHORT. Whether
	-- a character fits is a different question with a different axis: it stands
	-- world-vertical however the floor is tilted. A thin upright probe answers it
	-- without re-testing the surface the cylinder already cleared.
	headroom = false, headroomHeight = 2, headroomWidth = 0.25,
	-- Debug aid: record where each candidate died, so holes in the output can be
	-- attributed to a filter instead of guessed at.
	collectRejects = false,
	maxGroundFootprint = 400, skipNonCollide = true, skipCharacters = true,
}

local function merged(cfg)
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

local function isCharacter(p: Instance): boolean
	local a = p.Parent
	while a and a ~= workspace do
		if a:IsA("Model") and a:FindFirstChildOfClass("Humanoid") then return true end
		a = a.Parent
	end
	return false
end

-- Width of the SURFACE a surfel sits on. part.Size is in the part's own frame,
-- so a rotated slab's "thickness" can be on any axis: drop the local axis most
-- aligned with the surface normal and take the smaller of the two that remain.
-- A 0.62 post cap and a 0.88 handrail fail this; a floor slab does not.
local function surfaceWidth(part: BasePart, normal: Vector3): number
	local cf, sz = part.CFrame, part.Size
	local axes = { { cf.RightVector, sz.X }, { cf.UpVector, sz.Y }, { cf.LookVector, sz.Z } }
	local ni, nbest = 1, -1
	for i, a in ipairs(axes) do
		local d = math.abs(a[1]:Dot(normal))
		if d > nbest then nbest = d; ni = i end
	end
	local e1, e2
	for i, a in ipairs(axes) do
		if i ~= ni then
			if e1 then e2 = a[2] else e1 = a[2] end
		end
	end
	return math.min(e1, e2)
end

-- Which face of the part is this node sitting against? A node in a corner
-- touches several: prefer the one it is flushest against, then the most upright.
local function nodeFaceNormal(part: BasePart, nodeCF: CFrame, edge: number): Vector3?
	local cf = part.CFrame
	local lc = cf:PointToObjectSpace(nodeCF.Position)
	local e = part.Size * 0.5
	local h = edge * 0.5
	local axes = { cf.RightVector, cf.UpVector, cf.LookVector }
	local lcv = { lc.X, lc.Y, lc.Z }
	local ev = { e.X, e.Y, e.Z }
	local best, bestScore = nil, -math.huge
	for i = 1, 3 do
		for _, sign in ipairs({ 1, -1 }) do
			local reach = sign * lcv[i] + h
			if reach >= ev[i] - 1e-3 then
				local n = axes[i] * sign
				local d = n:Dot(UP)
				if d > 0.01 then
					local score = d - math.abs(reach - ev[i]) * 2
					if score > bestScore then bestScore = score; best = n end
				end
			end
		end
	end
	return best
end

local function aabbHalf(cf: CFrame, edge: number): Vector3
	local e = edge * 0.5
	local r, u, l = cf.RightVector, cf.UpVector, cf.LookVector
	return Vector3.new(
		(math.abs(r.X) + math.abs(u.X) + math.abs(l.X)) * e,
		(math.abs(r.Y) + math.abs(u.Y) + math.abs(l.Y)) * e,
		(math.abs(r.Z) + math.abs(u.Z) + math.abs(l.Z)) * e
	)
end

function FloorLocal.extract(trees, cfg)
	local c = merged(cfg)

	local function standable(part)
		if c.skipNonCollide and not part.CanCollide then return false end
		if c.skipCharacters and isCharacter(part) then return false end
		if math.max(part.Size.X, part.Size.Z) > c.maxGroundFootprint then return false end
		return true
	end

	local solids = {}
	for part in pairs(trees) do
		if standable(part) then table.insert(solids, part) end
	end

	local rp = RaycastParams.new()
	rp.FilterType = Enum.RaycastFilterType.Include
	rp.FilterDescendantsInstances = solids

	local probe = Instance.new("Part")
	probe.Size = Vector3.new(0.05, math.max(c.minClearance - 0.1, 0.05), 0.05)
	probe.Anchored = true
	probe.CanCollide, probe.CanQuery, probe.CanTouch = false, false, false
	probe.Transparency = 1
	probe.Parent = workspace
	local op = OverlapParams.new()
	op.FilterType = Enum.RaycastFilterType.Include
	op.FilterDescendantsInstances = solids
	op.RespectCanCollide = false

	local above = Instance.new("Part")
	above.Size = Vector3.new(0.6, 0.6, 0.6)
	above.Anchored = true
	above.CanCollide, above.CanQuery, above.CanTouch = false, false, false
	above.Transparency = 1
	above.Parent = workspace

	-- Clearance is an up-RAY: it measures a vertical line, not a volume, so it
	-- cannot see a wall standing beside the surface. An agent-sized box can.
	local agent = Instance.new("Part")
	local fitH = math.max(c.agentHeight - c.agentStepHeight, 0.1)
	agent.Size = Vector3.new(c.agentWidth, fitH, c.agentWidth)
	if c.agentCylinder then
		agent.Shape = Enum.PartType.Cylinder
		-- a Cylinder part runs along its X axis; stand it up
		agent.Size = Vector3.new(fitH, c.agentWidth, c.agentWidth)
	end
	agent.Anchored = true
	agent.CanCollide, agent.CanQuery, agent.CanTouch = false, false, false
	agent.Transparency = 1
	agent.Parent = workspace
	local head = Instance.new("Part")
	head.Anchored = true
	head.CanCollide, head.CanQuery, head.CanTouch = false, false, false
	head.Transparency = 1
	head.Parent = workspace

	local cyl = Instance.new("Part")
	cyl.Shape = Enum.PartType.Cylinder
	cyl.Anchored = true
	cyl.CanCollide, cyl.CanQuery, cyl.CanTouch = false, false, false
	cyl.Transparency = 1
	cyl.Parent = workspace

	local agentOp = OverlapParams.new()
	agentOp.FilterType = Enum.RaycastFilterType.Include
	agentOp.FilterDescendantsInstances = solids
	agentOp.RespectCanCollide = false

	-- Node-level occlusion. A node whose top face lies inside ANOTHER part's node
	-- is buried, whatever a point probe would say. This matters because the probe
	-- samples the 1-stud lattice CENTRE, which is not where thin geometry lives:
	-- a 0.38-thick rail centred at x=309.19 is missed entirely by a probe at
	-- x=309.5, so the baluster underneath it read as an exposed, walkable cap.
	local nodeIndex = {}
	for part, tree in pairs(trees) do
		tree:forEachNode(function(cf: CFrame, edge: number)
			local h = aabbHalf(cf, edge)
			local p = cf.Position
			for i = math.floor(p.X - h.X), math.floor(p.X + h.X) do
				for j = math.floor(p.Z - h.Z), math.floor(p.Z + h.Z) do
					local k = i .. "_" .. j
					local b = nodeIndex[k]
					if not b then b = {}; nodeIndex[k] = b end
					table.insert(b, { cf = cf, e = edge, part = part })
				end
			end
		end)
	end
	local function buried(cf: CFrame, edge: number, part: BasePart): boolean
		local h = aabbHalf(cf, edge)
		local c = cf.Position
		-- just above the top face, on the node's OWN centre line
		local tp = Vector3.new(c.X, c.Y + h.Y + 0.08, c.Z)
		local b = nodeIndex[math.floor(tp.X) .. "_" .. math.floor(tp.Z)]
		if not b then return false end
		for _, o in ipairs(b) do
			if o.part ~= part then
				local lp = o.cf:PointToObjectSpace(tp)
				local oh = o.e * 0.5
				if math.abs(lp.X) <= oh and math.abs(lp.Y) <= oh and math.abs(lp.Z) <= oh then
					return true
				end
			end
		end
		return false
	end
	local function buriedAt(wp: Vector3, part: BasePart): boolean
		local tp = wp + Vector3.new(0, 0.08, 0)
		local b = nodeIndex[math.floor(tp.X) .. "_" .. math.floor(tp.Z)]
		if not b then return false end
		for _, o in ipairs(b) do
			if o.part ~= part then
				local lp = o.cf:PointToObjectSpace(tp)
				local oh = o.e * 0.5
				if math.abs(lp.X) <= oh and math.abs(lp.Y) <= oh and math.abs(lp.Z) <= oh then
					return true
				end
			end
		end
		return false
	end
	-- Returns a raycast-shaped result plus the fraction of ring rays that landed
	-- level with the centre. support = 1 means a full disc of floor at that height.
	local function discCast(x: number, z: number, top: number, span: number)
		local origin = Vector3.new(x, top + 1, z)
		local dir = Vector3.new(0, -(span + 2), 0)
		local centre = workspace:Raycast(origin, dir, rp)

		local ringHits, hi = {}, nil
		for ring = 1, c.discRings do
			local rr = c.discRadius * (ring / c.discRings)
			local count = c.discSamples * ring
			for i = 0, count - 1 do
				local a = (i / count) * math.pi * 2
				local o = origin + Vector3.new(math.cos(a) * rr, 0, math.sin(a) * rr)
				local h = workspace:Raycast(o, dir, rp)
				table.insert(ringHits, h or false)
				if h and (not hi or h.Position.Y > hi.Position.Y) then hi = h end
			end
		end

		-- The centre ray is the floor. Only when it slips through a crack does the
		-- highest ring hit stand in for it.
		local ref = centre or hi
		if not ref then return nil, 0 end

		local agree, total = 0, #ringHits
		for _, h in ipairs(ringHits) do
			if h and math.abs(h.Position.Y - ref.Position.Y) <= c.discTolerance then
				agree += 1
			end
		end
		local support = (total > 0) and (agree / total) or 1
		return { Position = ref.Position, Normal = ref.Normal, Instance = ref.Instance }, support
	end

	local rejBuried = 0
	local rejCovered = 0
	local rejBlocked = 0
	local blockedList = {}

	local seen, cand = {}, {}
	for part, tree in pairs(trees) do
		if standable(part) then
			tree:forEachNode(function(cf: CFrame, edge: number)
				if c.nodeOcclusion and buried(cf, edge, part) then rejBuried += 1 return end
				local h = aabbHalf(cf, edge)
				local p = cf.Position
				local top = p.Y + h.Y
				local x0, x1 = p.X - h.X, p.X + h.X
				local z0, z1 = p.Z - h.Z, p.Z + h.Z
				local i = math.floor(x0)
				while i < x1 do
					local j = math.floor(z0)
					while j < z1 do
						local key = string.format("%d_%d_%d", i, j, math.floor(top * 2 + 0.5))
						if not seen[key] then
							seen[key] = true
							table.insert(cand, { x = i + 0.5, z = j + 0.5, top = top, span = edge, part = part, cf = cf })
						end
						j += 1
					end
					i += 1
				end
			end)
		end
	end

	local surfels = {}
	local rejSolidAbove, rejNoHit, rejSlope, rejDup, rejNarrow = 0, 0, 0, 0, 0
	local rejUnsupported = 0
	local rejHeadroom = 0
	local rejects = {}
	local function note(reason: string, pos: Vector3)
		if c.collectRejects then table.insert(rejects, { reason = reason, pos = pos }) end
	end
	local landed = {}
	for _, k in ipairs(cand) do
		above.CFrame = CFrame.new(k.x, k.top + 0.5, k.z)
		if #workspace:GetPartsInPart(above, op) > 0 then
			rejSolidAbove += 1
			note("solidAbove", Vector3.new(k.x, k.top, k.z))
			continue
		end
		if c.groundOverlap then
			local on = UP
			local oslope = 0
			if c.normalFromNode or c.overlapAlignNode then
				on = nodeFaceNormal(k.part, k.cf, k.span) or UP
				oslope = math.deg(math.acos(math.clamp(on:Dot(UP), -1, 1)))
				if oslope > c.maxSlope then
					rejSlope += 1
					note("slope", Vector3.new(k.x, k.top, k.z))
					continue
				end
			end
			local d = c.overlapDiameter
			local basePos, axis
			if c.overlapAlignNode then
				-- centre of the node's own face, and the direction it looks out
				basePos = k.cf.Position + on * (k.span * 0.5)
				axis = on
			else
				basePos = Vector3.new(k.x, k.top, k.z)
				axis = UP
			end
			-- a Cylinder part runs along its X axis, so that axis becomes RightVector
			cyl.Size = Vector3.new(c.overlapHeight, d, d)
			local seed = (math.abs(axis.Y) > 0.99) and Vector3.new(0, 0, 1) or UP
			cyl.CFrame = CFrame.fromMatrix(
				basePos + axis * (0.05 + c.overlapHeight * 0.5),
				axis, axis:Cross(seed).Unit)
			if #workspace:GetPartsInPart(cyl, op) > 0 then
				rejBlocked += 1
				note("exposure", basePos)
				continue
			end
			if c.headroom then
				-- Lift clear of the surface first: an upright box centred on a TILTED
				-- face has its uphill corner below the plane, so it collides with the
				-- floor it is standing on. The part underfoot is ignored for the same
				-- reason -- it is the ground, not an obstacle.
				local tan = math.tan(math.rad(math.min(oslope, 89)))
				local lift = 0.05 + c.headroomWidth * 0.5 * tan
				head.Size = Vector3.new(c.headroomWidth, c.headroomHeight, c.headroomWidth)
				head.CFrame = CFrame.new(basePos + UP * (lift + c.headroomHeight * 0.5))
				local clear = true
				for _, hit in ipairs(workspace:GetPartsInPart(head, op)) do
					if hit ~= k.part then clear = false break end
				end
				if not clear then
					rejHeadroom += 1
					note("headroom", basePos)
					continue
				end
			end
			if surfaceWidth(k.part, on) < c.minSurfaceWidth then
				rejNarrow += 1
				note("narrow", basePos)
				continue
			end
			local ok = string.format("%d_%d_%d", math.floor(basePos.X * 2 + 0.5),
				math.floor(basePos.Y * 2 + 0.5), math.floor(basePos.Z * 2 + 0.5))
			if landed[ok] then
				rejDup += 1
				note("dup", basePos)
				continue
			end
			landed[ok] = true
			table.insert(surfels, { pos = basePos, normal = on, slope = oslope,
				clearance = c.overlapHeight, part = k.part, support = 1 })
			continue
		end

		local res
		local support = 1
		if c.groundDisc then
			res, support = discCast(k.x, k.z, k.top, k.span)
		elseif c.groundRadius then
			-- start with the whole sphere clear of the surface, or a sweep that
			-- begins already intersecting reports a degenerate zero-distance hit
			local r = c.groundRadius
			local lift = r + 1
			res = workspace:Spherecast(Vector3.new(k.x, k.top + lift, k.z), r, Vector3.new(0, -(k.span + 2 + lift), 0), rp)
		else
			res = workspace:Raycast(Vector3.new(k.x, k.top + 1, k.z), Vector3.new(0, -(k.span + 2), 0), rp)
		end
		if not res then
			rejNoHit += 1
			continue
		end
		local n = res.Normal
		local slope = math.deg(math.acos(math.clamp(n:Dot(UP), -1, 1)))
		if slope > c.maxSlope then
			rejSlope += 1
			continue
		end
		if c.pointOcclusion and buriedAt(res.Position, res.Instance) then
			rejCovered += 1
			continue
		end
		if surfaceWidth(res.Instance, n) < c.minSurfaceWidth then
			rejNarrow += 1
			continue
		end
		if c.groundDisc and support < c.discMinSupport then
			rejUnsupported += 1
			continue
		end
		if c.agentFit then
			-- start ABOVE the step height: a stair riser is climbed, not collided with
			agent.CFrame = CFrame.new(res.Position + Vector3.new(0, c.agentStepHeight + fitH * 0.5, 0))
			if c.agentCylinder then
				agent.CFrame = agent.CFrame * CFrame.Angles(0, 0, math.rad(90))
			end
			local fits = true
			for _, hit in ipairs(workspace:GetPartsInPart(agent, agentOp)) do
				if hit ~= res.Instance then fits = false break end
			end
			if not fits then
				rejBlocked += 1
				table.insert(blockedList, { pos = res.Position, normal = n, slope = slope, clearance = -1, part = res.Instance })
				continue
			end
		end
		local rk = string.format("%d_%d_%d", math.floor(res.Position.X), math.floor(res.Position.Z), math.floor(res.Position.Y * 4 + 0.5))
		if landed[rk] then
			rejDup += 1
			continue
		end
		landed[rk] = true
		local clearance
		probe.CFrame = CFrame.new(res.Position + Vector3.new(0, 0.1 + (c.minClearance - 0.1) * 0.5, 0))
		local blocked = false
		for _, hit in ipairs(workspace:GetPartsInPart(probe, op)) do
			if hit ~= res.Instance then blocked = true break end
		end
		if blocked then
			clearance = 0
		else
			local upRes = workspace:Raycast(res.Position + Vector3.new(0, 0.15, 0), Vector3.new(0, c.clearCap, 0), rp)
			clearance = upRes and upRes.Distance or c.clearCap
		end
		table.insert(surfels, { pos = res.Position, normal = n, slope = slope, clearance = clearance, part = res.Instance, support = support })
	end
	probe:Destroy()
	above:Destroy()
	agent:Destroy()
	cyl:Destroy()
	head:Destroy()

	return {
		surfels = surfels,
		rejects = rejects,
		blocked = blockedList,
		surface = (function()
			local all = table.create(#surfels + #blockedList)
			table.move(surfels, 1, #surfels, 1, all)
			table.move(blockedList, 1, #blockedList, #surfels + 1, all)
			return all
		end)(), config = c,
		stats = {
			candidates = #cand, kept = #surfels,
			rejectedSolidAbove = rejSolidAbove, rejectedNoHit = rejNoHit,
			rejectedSlope = rejSlope, rejectedDuplicate = rejDup, rejectedNarrow = rejNarrow, rejectedBuried = rejBuried, rejectedCovered = rejCovered, rejectedBlocked = rejBlocked, rejectedUnsupported = rejUnsupported, rejectedHeadroom = rejHeadroom, standableParts = #solids,
		},
	}
end

local ROOT_NAME = "NVGN_Debug"
local SECTION = "FloorLocal"

function FloorLocal.clear(name: string?)
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

function FloorLocal.draw(result, opts)
	opts = opts or {}
	local name: string = opts.name or "floor"
	local colourBy: string = opts.colourBy or "slope"
	local lift: number = opts.lift or 0.05
	local size: number = opts.size or 0.92
	local transparency: number = opts.transparency or 0

	FloorLocal.clear(name)
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

	local maxSlope = result.config.maxSlope
	local cap = result.config.clearCap
	for _, s in ipairs(result.surfels) do
		local p = Instance.new("Part")
		p.Anchored = true
		p.CanCollide, p.CanQuery, p.CanTouch = false, false, false
		p.CastShadow = false
		p.Material = Enum.Material.SmoothPlastic
		p.Size = Vector3.new(size, 0.06, size)
		local up = s.normal
		local fwd = math.abs(up.Y) > 0.99 and Vector3.new(0, 0, 1) or UP:Cross(up).Unit
		local right = up:Cross(fwd).Unit
		p.CFrame = CFrame.fromMatrix(s.pos + up * lift, right, up)
		if colourBy == "support" then
			local t = math.clamp(s.support or 1, 0, 1)
			p.Color = Color3.fromRGB(math.floor(230 * (1 - t)), math.floor(60 + 170 * t), 80)
		elseif colourBy == "clearance" then
			local t = math.clamp(s.clearance / math.max(cap, 1e-6), 0, 1)
			p.Color = (s.clearance <= 0) and Color3.fromRGB(210, 40, 40)
				or Color3.fromRGB(math.floor(255 * (1 - t)), math.floor(90 + 140 * t), 90)
		else
			local t = math.clamp(s.slope / math.max(maxSlope, 1e-6), 0, 1)
			p.Color = Color3.fromRGB(math.floor(60 + 190 * t), math.floor(220 - 130 * t), 90)
		end
		p.Transparency = transparency
		p:SetAttribute("slope", s.slope)
		p:SetAttribute("clearance", s.clearance)
		p:SetAttribute("support", s.support)
		p:SetAttribute("part", s.part.Name)
		p.Parent = folder
	end
	folder.Parent = sec
	return { drawn = #result.surfels, folder = folder }
end

return FloorLocal
