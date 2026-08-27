--!strict
-- NVGN.SVOLocalDebug — draws per-part local-axis octrees (SVOLocal).
--
-- Nodes are ORIENTED (each sits in its part's frame), so unlike SVODebug these
-- are drawn with a full CFrame rather than an axis-aligned position.
--
-- Colour carries the node CLASS first, size second:
--   seam  -> red, opaque-ish. Where this part touches or overlaps another.
--   solid -> cool palette by node size, faded back so seams read on top.

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
	local f = sec:FindFirstChild(name)
	if not f then return 0 end
	local n = #f:GetChildren()
	f:Destroy()
	return n
end

-- Draw a { [part] = tree } map as produced by SVOLocal.fromParts.
--   name        folder under NVGN_Debug.SVOLocal (default "local")
--   seamsOnly   draw only the seam nodes (default false)
--   inset       shrink each box so node seams stay visible (default 0.06)
--   solidAlpha  transparency of non-seam nodes (default 0.6)
--   seamAlpha   transparency of seam nodes (default 0.1)
--   offset      world-space Vector3 to shift the whole drawing by (default zero).
--               The tree is exactly coincident with the source geometry, so an
--               offset is the only way to see BOTH at once without hiding one.
--   maxParts    abort past this many parts (default 20000)
function SVOLocalDebug.draw(trees, opts)
	opts = opts or {}
	local name: string = opts.name or "local"
	local seamsOnly: boolean = opts.seamsOnly or false
	local inset: number = opts.inset or 0.06
	local solidAlpha: number = opts.solidAlpha or 0.6
	local seamAlpha: number = opts.seamAlpha or 0.1
	local offset: Vector3 = opts.offset or Vector3.zero
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
			if drawn >= maxParts then
				overflow = true
				return
			end
			local p = Instance.new("Part")
			p.Name = (isSeam and "seam_" or "solid_") .. tostring(edge)
			p.Anchored = true
			p.CanCollide, p.CanQuery, p.CanTouch = false, false, false
			p.CastShadow = false
			p.Locked = true
			p.Material = Enum.Material.SmoothPlastic
			p.Color = isSeam and SEAM_COLOR or (SOLID_COLOR[edge] or SOLID_FALLBACK)
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

-- The local-axis tree is exactly coincident with the real geometry, so drawn in
-- place the source parts occlude it completely. Push the source back to a given
-- transparency to see the two overlap; pass nil to restore. The original value
-- is stashed per part in the NVGN_origT attribute, so this leaves nothing
-- behind, and a part already more transparent than the target is left alone.
function SVOLocalDebug.setSourceTransparency(model: Instance, t: number?): number
	local n = 0
	for _, d in ipairs(model:GetDescendants()) do
		if d:IsA("BasePart") then
			if t then
				if d:GetAttribute("NVGN_origT") == nil then
					d:SetAttribute("NVGN_origT", d.Transparency)
				end
				-- never make a part MORE opaque than the builder made it
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

-- Fully hide / restore the source. Kept as the common case of the above.
function SVOLocalDebug.setSourceHidden(model: Instance, hidden: boolean): number
	return SVOLocalDebug.setSourceTransparency(model, hidden and 1 or nil)
end

return SVOLocalDebug
