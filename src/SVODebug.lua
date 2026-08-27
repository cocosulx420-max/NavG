--!strict
-- NVGN.SVODebug — draws an SVO's solid leaves as parts so the tree can be eyeballed.
--
-- One part per solid leaf, sized to that node. Colour encodes NODE SIZE, not
-- depth, so the octree's compression reads directly off the picture: a big
-- yellow block is one node standing in for 64 unit leaves.
--
-- Everything lands under workspace.NVGN_Debug.SVO.<name>, Anchored with
-- CanCollide/CanQuery/CanTouch off. The parts are left UNLOCKED so they can be
-- clicked and inspected in the viewport; pass locked = true to protect them.

local SVODebug = {}

local ROOT_NAME = "NVGN_Debug"
local SECTION = "SVO"

-- node edge size -> colour. Sizes are powers of two studs.
local SIZE_COLOR = {
	[0.25] = Color3.fromRGB(150, 60, 200),
	[0.5]  = Color3.fromRGB(90, 130, 255),
	[1]    = Color3.fromRGB(70, 200, 190),
	[2]    = Color3.fromRGB(90, 220, 90),
	[4]    = Color3.fromRGB(230, 220, 70),
	[8]    = Color3.fromRGB(240, 150, 50),
	[16]   = Color3.fromRGB(230, 70, 60),
	[32]   = Color3.fromRGB(255, 120, 200),
}
local FALLBACK = Color3.fromRGB(255, 255, 255)

local function section(): Folder
	local root = workspace:FindFirstChild(ROOT_NAME)
	if not root then
		root = Instance.new("Folder")
		root.Name = ROOT_NAME
		root.Parent = workspace
	end
	local sec = (root :: Folder):FindFirstChild(SECTION)
	if not sec then
		sec = Instance.new("Folder")
		sec.Name = SECTION
		sec.Parent = root
	end
	return sec :: Folder
end

-- Remove one drawing by name, or every drawing when name is nil.
function SVODebug.clear(name: string?)
	local root = workspace:FindFirstChild(ROOT_NAME)
	if not root then return 0 end
	local sec = root:FindFirstChild(SECTION)
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

-- Is every face-neighbour of this node solid? Such a node is buried and can be
-- skipped, which is what makes a filled volume legible from outside.
local function isBuried(tree, c: Vector3, h: number): boolean
	local step = h + tree.leaf * 0.5
	local dirs = {
		Vector3.new(step, 0, 0), Vector3.new(-step, 0, 0),
		Vector3.new(0, step, 0), Vector3.new(0, -step, 0),
		Vector3.new(0, 0, step), Vector3.new(0, 0, -step),
	}
	for _, d in ipairs(dirs) do
		if not tree:isSolid(c + d) then return false end
	end
	return true
end

-- Draw the solid leaves of `tree`.
--   name          folder name under NVGN_Debug.SVO (default "tree")
--   surfaceOnly   skip nodes whose six face-neighbours are all solid (default true)
--   inset         shrink each box by this many studs so leaf seams stay visible (default 0.06)
--   transparency  default 0.35
--   outline       add a SelectionBox per part (default false; expensive above a few thousand)
--   maxParts      abort past this many parts rather than freezing Studio (default 20000)
function SVODebug.draw(tree, opts)
	opts = opts or {}
	local name: string = opts.name or "tree"
	local surfaceOnly: boolean = if opts.surfaceOnly == nil then true else opts.surfaceOnly
	local inset: number = opts.inset or 0.06
	local transparency: number = opts.transparency or 0.35
	local outline: boolean = opts.outline or false
	-- Unlocked by default: these are meant to be clicked, measured and deleted.
	local locked: boolean = opts.locked == true
	local maxParts: number = opts.maxParts or 20000

	SVODebug.clear(name)
	local folder = Instance.new("Folder")
	folder.Name = name

	local drawn, skipped, bySize = 0, 0, {}
	local overflow = false

	tree:forEachSolidLeaf(function(c: Vector3, h: number)
		if overflow then return end
		local size = h * 2
		if surfaceOnly and isBuried(tree, c, h) then
			skipped += 1
			return
		end
		if drawn >= maxParts then
			overflow = true
			return
		end
		local p = Instance.new("Part")
		p.Name = string.format("%g_%d_%d_%d", size, c.X, c.Y, c.Z)
		p.Anchored = true
		p.CanCollide = false
		p.CanQuery = false
		p.CanTouch = false
		p.CastShadow = false
		p.Locked = locked
		p.Material = Enum.Material.SmoothPlastic
		p.Color = SIZE_COLOR[size] or FALLBACK
		p.Transparency = transparency
		p.Size = Vector3.new(size - inset, size - inset, size - inset)
		p.CFrame = CFrame.new(c)
		p:SetAttribute("nodeSize", size)
		if outline then
			local sb = Instance.new("SelectionBox")
			sb.Adornee = p
			sb.LineThickness = 0.02
			sb.Color3 = Color3.new(0, 0, 0)
			sb.Transparency = 0.4
			sb.Parent = p
		end
		p.Parent = folder
		drawn += 1
		bySize[size] = (bySize[size] or 0) + 1
	end)

	folder.Parent = section()
	return {
		drawn = drawn,
		buriedSkipped = skipped,
		overflowed = overflow,
		bySize = bySize,
		folder = folder,
	}
end

-- Draw the root cube as a wireframe so the lattice anchor is visible.
function SVODebug.drawBounds(tree, name: string?)
	local folder = section()
	local nm = (name or "tree") .. "_bounds"
	local old = folder:FindFirstChild(nm)
	if old then old:Destroy() end
	local p = Instance.new("Part")
	p.Name = nm
	p.Anchored = true
	p.CanCollide, p.CanQuery, p.CanTouch = false, false, false
	p.CastShadow = false
	p.Locked = false
	p.Transparency = 1
	p.Size = Vector3.new(tree.half * 2, tree.half * 2, tree.half * 2)
	p.CFrame = CFrame.new(tree.center)
	local sb = Instance.new("SelectionBox")
	sb.Adornee = p
	sb.LineThickness = 0.08
	sb.Color3 = Color3.fromRGB(255, 255, 255)
	sb.Transparency = 0.2
	sb.Parent = p
	p.Parent = folder
	return p
end

return SVODebug
