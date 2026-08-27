--!strict
-- NVGN.WallBase — the bottom nodes of whatever stands ON the walkable surface.
--
-- This is the wall side of the navmesh boundary. Memory of the rule: the
-- boundary is wall-or-dropoff, and this finds the wall half of it.
--
-- Nothing here classifies a part as "a wall". A wall is defined by the
-- RELATIONSHIP, which is what makes it robust: a node qualifies when it is the
-- lowest node of its column in its own part's local tree, and a walkable surfel
-- belonging to a DIFFERENT part sits beside it at about the same height. A part
-- standing on the floor therefore contributes its base ring; the floor slab
-- itself does not, because its own surfels are the ones being stood on.
--
-- "Lowest of its column" is measured in the PART'S OWN frame, which is the only
-- frame in which a rotated wall has a well-defined bottom row.

local WallBase = {}

local DEFAULT = {
	xzTol = 0.75,   -- how far a surfel may sit from the node footprint (studs)
	yTol  = 1.5,    -- how far below/above the node's base a surfel may sit
	requireSeam = false, -- restrict to nodes SVOLocal already marked as seam
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

-- trees: { [BasePart] = SVOLocal }   floorResult: from FloorLocal.extract
function WallBase.find(trees, floorResult, cfg)
	local c = merged(cfg)

	-- surfels bucketed on the 1-stud world lattice
	local grid: { [string]: { { y: number, part: BasePart } } } = {}
	for _, s in ipairs(floorResult.surfels) do
		local key = math.floor(s.pos.X) .. "_" .. math.floor(s.pos.Z)
		local b = grid[key]
		if not b then b = {}; grid[key] = b end
		table.insert(b, { y = s.pos.Y, part = s.part })
	end

	local found = {}
	local scanned, bottoms = 0, 0

	for part, tree in pairs(trees) do
		-- lowest node per column, in the part's OWN frame
		local col: { [string]: { ly: number, cf: CFrame, edge: number, seam: boolean } } = {}
		tree:forEachNode(function(cf: CFrame, edge: number, isSeam: boolean)
			scanned += 1
			local lp = tree.cf:PointToObjectSpace(cf.Position)
			-- quantise the column on the part's own lattice
			local key = string.format("%.2f_%.2f", lp.X, lp.Z)
			local cur = col[key]
			if not cur or lp.Y < cur.ly then
				col[key] = { ly = lp.Y, cf = cf, edge = edge, seam = isSeam }
			end
		end)

		for _, n in pairs(col) do
			bottoms += 1
			if c.requireSeam and not n.seam then continue end
			local h = aabbHalf(n.cf, n.edge)
			local p = n.cf.Position
			local baseY = p.Y - h.Y
			-- any walkable surfel from ANOTHER part beside this node's footprint?
			local hit = false
			local i0 = math.floor(p.X - h.X - c.xzTol)
			local i1 = math.floor(p.X + h.X + c.xzTol)
			local j0 = math.floor(p.Z - h.Z - c.xzTol)
			local j1 = math.floor(p.Z + h.Z + c.xzTol)
			for i = i0, i1 do
				for j = j0, j1 do
					local b = grid[i .. "_" .. j]
					if b then
						for _, s in ipairs(b) do
							if s.part ~= part and math.abs(s.y - baseY) <= c.yTol then
								hit = true
								break
							end
						end
					end
					if hit then break end
				end
				if hit then break end
			end
			if hit then
				table.insert(found, { cf = n.cf, edge = n.edge, seam = n.seam, part = part, baseY = baseY })
			end
		end
	end

	return {
		nodes = found,
		config = c,
		stats = { scannedNodes = scanned, columnBottoms = bottoms, kept = #found },
	}
end

-- ---- drawing ----------------------------------------------------------------

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
	local f = sec:FindFirstChild(name)
	if not f then return 0 end
	local n = #f:GetChildren()
	f:Destroy()
	return n
end

function WallBase.draw(result, opts)
	opts = opts or {}
	local name: string = opts.name or "walls"
	local inset: number = opts.inset or 0.05
	local transparency: number = opts.transparency or 0.15
	local colour: Color3 = opts.colour or Color3.fromRGB(255, 170, 40)

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
		p.Material = Enum.Material.Neon
		p.Color = colour
		p.Transparency = transparency
		p.Size = Vector3.new(n.edge - inset, n.edge - inset, n.edge - inset)
		p.CFrame = n.cf
		p:SetAttribute("nodeSize", n.edge)
		p:SetAttribute("seam", n.seam)
		p:SetAttribute("part", n.part.Name)
		p:SetAttribute("baseY", n.baseY)
		p.Parent = folder
	end
	folder.Parent = sec
	return { drawn = #result.nodes, folder = folder }
end

return WallBase
