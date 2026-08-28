--!strict
local SVOLocalDebug = {}

local ROOT_NAME = "NVGN_Debug"
local SECTION = "SVOLocal"

local SOLID_COLOR = {
	[0.5] = Color3.fromRGB(70, 110, 190),
	[1]   = Color3.fromRGB(70, 190, 200),
	[2]   = Color3.fromRGB(90, 210, 130),
	[4]   = Color3.fromRGB(200, 205, 90),
	[8]   = Color3.fromRGB(215, 160, 70),
}
local SOLID_FALLBACK = Color3.fromRGB(200, 200, 200)
local SEAM_COLOR = Color3.fromRGB(235, 55, 45)

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

function SVOLocalDebug.clear(name: string?)
	local root = workspace:FindFirstChild(ROOT_NAME)
	if not root then return 0 end
	local sec = root:FindFirstChild(SECTION)
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

function SVOLocalDebug.draw(trees, opts)
	opts = opts or {}
	local name: string = opts.name or "local"
	local seamsOnly: boolean = opts.seamsOnly or false
	local inset: number = opts.inset or 0.06
	local solidAlpha: number = opts.solidAlpha or 0.6
	local seamAlpha: number = opts.seamAlpha or 0.1
	local offset: Vector3 = opts.offset or Vector3.zero
	local locked: boolean = opts.locked == true
	local filter = opts.filter
	local colourFn = opts.colourFn
	local maxParts: number = opts.maxParts or 20000

	SVOLocalDebug.clear(name)
	local folder = Instance.new("Folder")
	folder.Name = name

	local drawn, seams, overflow = 0, 0, false
	for part, tree in pairs(trees) do
		local sub = Instance.new("Folder")
		sub.Name = part.Name
		tree:forEachNode(function(cf: CFrame, edge: number, isSeam: boolean)
			if overflow then return end
			if seamsOnly and not isSeam then return end
			if filter and not filter(cf, edge, isSeam, part) then return end
			if drawn >= maxParts then
				overflow = true
				return
			end
			local p = Instance.new("Part")
			p.Name = (isSeam and "seam_" or "solid_") .. tostring(edge)
			p.Anchored = true
			p.CanCollide, p.CanQuery, p.CanTouch = false, false, false
			p.CastShadow = false
			p.Locked = locked
			p.Material = Enum.Material.SmoothPlastic
			p.Color = (colourFn and colourFn(cf, edge, isSeam, part))
				or (isSeam and SEAM_COLOR or (SOLID_COLOR[edge] or SOLID_FALLBACK))
			p.Transparency = isSeam and seamAlpha or solidAlpha
			p.Size = Vector3.new(edge - inset, edge - inset, edge - inset)
			p.CFrame = cf + offset
			p:SetAttribute("nodeSize", edge)
			p:SetAttribute("seam", isSeam)
			p:SetAttribute("part", part.Name)
			p.Parent = sub
			drawn += 1
			if isSeam then seams += 1 end
		end)
		sub.Parent = folder
	end

	folder.Parent = section()
	return { drawn = drawn, seams = seams, overflowed = overflow, folder = folder }
end

function SVOLocalDebug.setSourceTransparency(model: Instance, t: number?): number
	local n = 0
	for _, d in ipairs(model:GetDescendants()) do
		if d:IsA("BasePart") then
			if t then
				if d:GetAttribute("NVGN_origT") == nil then
					d:SetAttribute("NVGN_origT", d.Transparency)
				end
				d.Transparency = math.max(t, d:GetAttribute("NVGN_origT") :: number)
			else
				local orig = d:GetAttribute("NVGN_origT")
				if orig ~= nil then
					d.Transparency = orig :: number
					d:SetAttribute("NVGN_origT", nil)
				end
			end
			n += 1
		end
	end
	return n
end

function SVOLocalDebug.setSourceHidden(model: Instance, hidden: boolean): number
	return SVOLocalDebug.setSourceTransparency(model, hidden and 1 or nil)
end

-- Cull a drawing down to the nodes that actually touch REAL geometry.
function SVOLocalDebug.keepTouching(drawName: string, model: Instance, pad: number?)
	local root = workspace:FindFirstChild(ROOT_NAME)
	local sec = root and root:FindFirstChild(SECTION)
	local folder = sec and sec:FindFirstChild(drawName)
	if not folder then return 0, 0 end

	local real = {}
	for _, d in ipairs(model:GetDescendants()) do
		if d:IsA("BasePart") then table.insert(real, d) end
	end

	local probe = Instance.new("Part")
	probe.Anchored = true
	probe.CanCollide, probe.CanQuery, probe.CanTouch = false, false, false
	probe.Transparency = 1
	probe.Parent = workspace
	local op = OverlapParams.new()
	op.FilterType = Enum.RaycastFilterType.Include
	op.FilterDescendantsInstances = real
	op.RespectCanCollide = false
	op.MaxParts = 1

	local grow = pad or 0
	local kept, removed = 0, 0
	local doomed = {}
	for _, sub in ipairs(folder:GetChildren()) do
		for _, node in ipairs(sub:GetChildren()) do
			if node:IsA("BasePart") then
				local edge = (node:GetAttribute("nodeSize") :: number) + grow
				probe.Size = Vector3.new(edge, edge, edge)
				probe.CFrame = node.CFrame
				if #workspace:GetPartsInPart(probe, op) > 0 then
					kept += 1
				else
					removed += 1
					table.insert(doomed, node)
				end
			end
		end
	end
	probe:Destroy()
	for _, d in ipairs(doomed) do d:Destroy() end
	for _, sub in ipairs(folder:GetChildren()) do
		if #sub:GetChildren() == 0 then sub:Destroy() end
	end
	return kept, removed
end

return SVOLocalDebug
