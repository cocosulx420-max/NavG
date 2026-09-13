--!strict
-- NVGN.WalkLocal -- walkability by occupancy, nothing else.
--
-- For every solid node in a local SVO, stand a cube on the node's own top FACE
-- and ask whether that cube touches anything at all. Empty = you can stand
-- there. No raycast, no normal recovery, no slope filter, no agent capsule.
--
-- The cube is placed along the NODE's axes, not the world's: a node inherits
-- its part's CFrame, so "up" here is whichever of the part's six axis
-- directions points most skyward. On a level floor that is world +Y; on a
-- rotated slab it follows the slab. Flat surfaces first -- roofs/overhangs
-- (where the skyward axis is the wrong question) come later.

local WalkLocal = {}
local UP = Vector3.new(0, 1, 0)

local DEFAULT = {
	block = 1.5,   -- edge of the cube we stand on the face
	gap = 0.02,    -- lift off the face so the face itself is not a "hit"
	-- Interior nodes can never be walkable: something of their own part is
	-- directly above them. Rejecting those with a free tree walk instead of a
	-- spatial query is the difference between one probe per node and one probe
	-- per SURFACE node.
	skipBuried = true,
	skipNonCollide = true,
}

local function merged(cfg)
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

-- Which of the part's six axis directions is "up" for this node? Local frame
-- only -- world UP is used just to break the tie between the six.
local function localUp(cf: CFrame): Vector3
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

-- Every collidable part in the world except the debug draw. The fallback when
-- there is no tree set to take the filter from -- e.g. checking a node list
-- recovered from an existing debug folder.
local function worldSolids()
	local skip = workspace:FindFirstChild("NVGN_Debug")
	local out = {}
	for _, d in ipairs(workspace:GetDescendants()) do
		if d:IsA("BasePart") and d.CanCollide and not (skip and d:IsDescendantOf(skip)) then
			table.insert(out, d)
		end
	end
	return out
end

-- The test itself, over an explicit node list. Each entry needs `cf` and `edge`;
-- `part` and `tree` are optional (tree only enables the free buried reject).
-- WalkLocal.check is this with the list read out of a tree set.
function WalkLocal.checkNodes(list, cfg)
	local c = merged(cfg)
	local solids = c.solids or worldSolids()

	local op = OverlapParams.new()
	op.FilterType = Enum.RaycastFilterType.Include
	op.FilterDescendantsInstances = solids
	op.RespectCanCollide = false
	op.MaxParts = 1

	local box = Instance.new("Part")
	box.Size = Vector3.new(c.block, c.block, c.block)
	box.Anchored = true
	box.CanCollide, box.CanQuery, box.CanTouch = false, false, false
	box.Transparency = 1
	box.Parent = workspace

	-- EVERY node gets an entry and a verdict -- the walkable set is a colour on
	-- the existing node map, not a second point cloud beside it.
	local nodes = {}
	local walkable, buried, blocked, probes = 0, 0, 0, 0

	for _, n in ipairs(list) do
		local cf, edge, tree = n.cf, n.edge, n.tree
		local up = localUp(cf)
		local face = cf.Position + up * (edge * 0.5)
		local ok, why = false, "blocked"

		if c.skipBuried and tree and tree:containsPoint(face + up * 0.05) then
			buried += 1
			why = "buried"
		else
			-- Sit the cube ON the face, offset by half its own edge so it never
			-- overlaps the node it is standing on, plus a hair of clearance.
			local centre = face + up * (c.block * 0.5 + c.gap)
			local seed = (math.abs(up:Dot(cf.UpVector)) > 0.99) and cf.LookVector or cf.UpVector
			box.CFrame = CFrame.fromMatrix(centre, up:Cross(seed).Unit, up)

			probes += 1
			if #workspace:GetPartsInPart(box, op) > 0 then
				blocked += 1
			else
				ok, why = true, "walkable"
				walkable += 1
			end
		end

		table.insert(nodes, {
			cf = cf, edge = edge, ok = ok, why = why,
			pos = face, normal = up, part = n.part, source = n.source,
		})
	end

	box:Destroy()

	return {
		nodes = nodes,
		config = c,
		stats = {
			parts = #solids, nodes = #nodes, probes = probes,
			walkable = walkable, rejectedBuried = buried, rejectedBlocked = blocked,
		},
	}
end

function WalkLocal.check(trees, cfg)
	local c = merged(cfg)
	local solids, list = {}, {}
	for part, tree in pairs(trees) do
		if (not c.skipNonCollide) or part.CanCollide then
			table.insert(solids, part)
			tree:forEachNode(function(cf: CFrame, edge: number)
				table.insert(list, { cf = cf, edge = edge, part = part, tree = tree })
			end)
		end
	end
	c.solids = solids
	return WalkLocal.checkNodes(list, c)
end

-- Re-runs the test over an EXISTING debug folder of node cubes (each part's
-- CFrame is the node frame, its `edge` attribute the node size) and recolours
-- those very parts in place. No second node map, no SVO rebuild.
function WalkLocal.repaint(folder: Instance, cfg)
	local opts = (cfg and cfg.draw) or {}
	local GREEN = opts.green or Color3.fromRGB(70, 210, 100)
	local RED = opts.red or Color3.fromRGB(200, 55, 55)

	local list = {}
	for _, p in ipairs(folder:GetDescendants()) do
		if p:IsA("BasePart") then
			local edge = p:GetAttribute("edge") or p:GetAttribute("nodeSize") or p.Size.X
			table.insert(list, { cf = p.CFrame, edge = edge, source = p })
		end
	end

	local res = WalkLocal.checkNodes(list, cfg)
	for _, n in ipairs(res.nodes) do
		local p = n.source
		p.Color = n.ok and GREEN or RED
		p.Transparency = opts.transparency or 0
		p:SetAttribute("walkable", n.ok)
		p:SetAttribute("why", n.why)
	end
	return res
end

local ROOT_NAME = "NVGN_Debug"
local SECTION = "WalkLocal"

function WalkLocal.clear(name: string?)
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

-- Paints the SVO's OWN nodes: green where the block test came back empty, red
-- everywhere else. Same cubes SVODebug draws, recoloured by verdict -- no
-- separate surfel map to line up against the tree.
function WalkLocal.draw(result, opts)
	opts = opts or {}
	local name: string = opts.name or "walk"
	local inset: number = opts.inset or 0.06
	-- A node cube is INSIDE its part, so a walkable top node is coplanar with the
	-- surface it belongs to and z-fights into invisibility. Push each cube out
	-- along its own up axis so the verdict is actually readable.
	local lift: number = opts.lift or 0.08
	local transparency: number = opts.transparency or 0
	local GREEN = opts.green or Color3.fromRGB(70, 210, 100)
	local RED = opts.red or Color3.fromRGB(200, 55, 55)

	WalkLocal.clear(name)
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

	local green = 0
	for _, n in ipairs(result.nodes) do
		if opts.walkableOnly and not n.ok then continue end
		local p = Instance.new("Part")
		p.Anchored = true
		p.CanCollide, p.CanQuery, p.CanTouch = false, false, false
		p.CastShadow = false
		p.Material = Enum.Material.SmoothPlastic
		p.Size = Vector3.new(n.edge - inset, n.edge - inset, n.edge - inset)
		p.CFrame = n.cf + n.normal * lift
		p.Color = n.ok and GREEN or RED
		p.Transparency = transparency
		p:SetAttribute("walkable", n.ok)
		p:SetAttribute("why", n.why)
		p:SetAttribute("nodeSize", n.edge)
		p:SetAttribute("part", n.part.Name)
		p.Parent = folder
		if n.ok then green += 1 end
	end

	folder.Parent = sec
	return { drawn = #folder:GetChildren(), walkable = green, folder = folder }
end

return WalkLocal
