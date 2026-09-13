--!strict
-- NVGN.WallBase -- the BOTTOM LAYER of whatever stands on the walkable surface.
--
-- Only the layer where a part actually meets the floor counts. A node qualifies
-- when it is the lowest node of its column AND floor belonging to a DIFFERENT
-- part sits beside it at about the same height.
--
-- Columns are keyed on the WORLD lattice cell, not on the node's own centre.
-- Keying on the centre let a 2-stud node open a column of its own -- its centre
-- does not line up with the 1-stud nodes above and below it -- so the "bottom"
-- of that phantom column was the 2-cube itself, floating mid-wall. On case2 that
-- produced 13 false bases, 10 of them on TOP of a 5-tall wall.
--
-- The floor set passed in should be the SURFACE set, not the agent-eroded
-- walkable set: a wall base is geometry, and agent erosion pulls the floor back
-- from every wall, which punched gaps in the band.

local WallBase = {}

local DEFAULT = {
	xzTol = 0.75,
	yTol  = 1.5,
	requireSeam = false,
}

local function merged(cfg)
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
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

function WallBase.find(trees, floorResult, cfg)
	local c = merged(cfg)

	local grid = {}
	for _, s in ipairs(floorResult.surfels) do
		local key = math.floor(s.pos.X) .. "_" .. math.floor(s.pos.Z)
		local b = grid[key]
		if not b then b = {}; grid[key] = b end
		table.insert(b, { y = s.pos.Y, part = s.part })
	end

	local found = {}
	local scanned, bottoms = 0, 0

	for part, tree in pairs(trees) do
		-- Columns keyed on the node centre in the PART'S OWN frame -- the only
		-- frame in which a rotated wall has a well-defined bottom row.
		local col = {}
		tree:forEachNode(function(cf: CFrame, edge: number, isSeam: boolean)
			scanned += 1
			local lp = tree.cf:PointToObjectSpace(cf.Position)
			local key = string.format("%.2f_%.2f", lp.X, lp.Z)
			local cur = col[key]
			if not cur or lp.Y < cur.localBase then
				col[key] = {
					localBase = lp.Y,
					baseY = cf.Position.Y - aabbHalf(cf, edge).Y,
					cf = cf, edge = edge, seam = isSeam,
				}
			end
		end)

		-- ONLY THE BOTTOM LAYER. A 2-stud node sits at a different centre from the
		-- 1-stud nodes around it, so it opens a column of its own whose "bottom" is
		-- the 2-cube itself, floating mid-wall. Keep only the columns that actually
		-- reach the part's lowest level.
		local lowest = math.huge
		for _, n in pairs(col) do
			local bottom = n.localBase - n.edge * 0.5
			if bottom < lowest then lowest = bottom end
		end
		for k, n in pairs(col) do
			if (n.localBase - n.edge * 0.5) > lowest + 1e-3 then col[k] = nil end
		end

		-- one node can win several columns; keep it once
		local picked = {}
		for _, n in pairs(col) do
			bottoms += 1
			local id = string.format("%.3f_%.3f_%.3f", n.cf.Position.X, n.cf.Position.Y, n.cf.Position.Z)
			if not picked[id] then
				picked[id] = n
			end
		end

		-- Standing on the floor is a property of the PART, not of each node.
		-- Testing per node punched holes wherever two walls meet: at a junction the
		-- floor beneath is covered by both walls, so no exposed floor surfel sits
		-- beside that column and the node was dropped -- leaving a gap in a band
		-- that is otherwise continuous. Decide once for the part, then keep its
		-- whole bottom layer.
		local layer, onFloor = {}, false
		for _, n in pairs(picked) do
			local skip = c.requireSeam and not n.seam
			if not skip then
				table.insert(layer, n)
				if not onFloor then
					local h = aabbHalf(n.cf, n.edge)
					local p = n.cf.Position
					for i = math.floor(p.X - h.X - c.xzTol), math.floor(p.X + h.X + c.xzTol) do
						for j = math.floor(p.Z - h.Z - c.xzTol), math.floor(p.Z + h.Z + c.xzTol) do
							local bkt = grid[i .. "_" .. j]
							if bkt then
								for _, s in ipairs(bkt) do
									if s.part ~= part and math.abs(s.y - n.baseY) <= c.yTol then
										onFloor = true
										break
									end
								end
							end
							if onFloor then break end
						end
						if onFloor then break end
					end
				end
			end
		end
		if onFloor then
			for _, n in ipairs(layer) do
				table.insert(found, { cf = n.cf, edge = n.edge, seam = n.seam, part = part, baseY = n.baseY })
			end
		end
	end

	return { nodes = found, config = c,
		stats = { scannedNodes = scanned, columnBottoms = bottoms, kept = #found } }
end

local ROOT_NAME = "NVGN_Debug"
local SECTION = "WallBase"

function WallBase.clear(name: string?)
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

function WallBase.draw(result, opts)
	opts = opts or {}
	local name: string = opts.name or "walls"
	local inset: number = opts.inset or 0.05
	local transparency: number = opts.transparency or 0.15
	local colour: Color3 = opts.colour or Color3.fromRGB(235, 45, 40)
	local offset: Vector3 = opts.offset or Vector3.zero

	WallBase.clear(name)
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
	for _, n in ipairs(result.nodes) do
		local p = Instance.new("Part")
		p.Anchored = true
		p.CanCollide, p.CanQuery, p.CanTouch = false, false, false
		p.CastShadow = false
		p.Material = Enum.Material.SmoothPlastic
		p.Color = colour
		p.Transparency = transparency
		p.Size = Vector3.new(n.edge - inset, n.edge - inset, n.edge - inset)
		p.CFrame = n.cf + offset
		p:SetAttribute("nodeSize", n.edge)
		p:SetAttribute("baseY", n.baseY)
		p:SetAttribute("part", n.part.Name)
		p.Parent = folder
	end
	folder.Parent = sec
	return { drawn = #result.nodes, folder = folder }
end

return WallBase
