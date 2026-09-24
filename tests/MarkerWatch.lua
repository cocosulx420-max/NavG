-- Size markers: PathStart_Normal / PathStart_Wide to PathEnd_case3, re-solved
-- whenever one moves. Posted to a resident bake worker as f(result, ctx).
-- Each line is drawn pulled tight through its portals, then the marker's own
-- body box is swept along it; every stretch where it would touch something
-- gets a red box of that body's size (NVGN_Clips_*) and the count goes on the
-- marker as BodyClips.
local r, ctx = ...
if ctx.stopWatch then ctx.stopWatch() end
local inst = ctx.ng.PathTest
local PT = assert(loadstring("local script,require=... " .. inst.Source, "=PathTest"))(inst, ctx.req)
local A = ctx.req(ctx.ng.Agents)
local STEP = A.profiles.default.step -- steps under this are walked, not bumped
local function body(radius, height)
	local p = table.clone(A.profiles.default)
	p.radius = radius
	p.height = height - 0.5
	p.crouch, p.prone = p.height, p.height
	return p
end
local specs = {
	{ part = workspace.PathStart_Normal, colour = Color3.fromRGB(40, 200, 255), prof = body(1, 5),
		line = "NVGN_Path_Normal", clips = "NVGN_Clips_Normal", lift = 0 },
	{ part = workspace.PathStart_Wide, colour = Color3.fromRGB(255, 150, 40), prof = body(2, 10),
		line = "NVGN_Path_Wide", clips = "NVGN_Clips_Wide", lift = 0.15 },
}
local goal = workspace.PathEnd_case3

-- sweeps hit the map only: not drawings, markers or characters
local rp = RaycastParams.new()
rp.FilterType = Enum.RaycastFilterType.Exclude
local function refreshFilter()
	local ex = {}
	for _, c in ipairs(workspace:GetChildren()) do
		if c.Name:sub(1, 5) == "NVGN_" or c.Name:sub(1, 9) == "PathStart" or c.Name:sub(1, 7) == "PathEnd" then
			ex[#ex + 1] = c
		end
	end
	for _, h in ipairs(workspace:GetDescendants()) do
		if h:IsA("Humanoid") and h.Parent then ex[#ex + 1] = h.Parent end
	end
	rp.FilterDescendantsInstances = ex
end
refreshFilter()

local function drawClips(name, hits, colour)
	local old = workspace:FindFirstChild(name)
	if old then old:Destroy() end
	if #hits == 0 then return end
	local f = Instance.new("Folder")
	f.Name = name
	for i, h in ipairs(hits) do
		if i > 60 then break end
		local b = Instance.new("Part")
		b.Name = ("clip_%s"):format(h.part.Name)
		b.Anchored = true; b.CanCollide = false; b.CanQuery = false; b.CanTouch = false; b.CastShadow = false
		b.Material = Enum.Material.Neon
		b.Color = Color3.fromRGB(255, 40, 40)
		b.Transparency = 0.6
		b.Size = h.box
		b.CFrame = CFrame.new(h.at)
		b:SetAttribute("Hits", h.part:GetFullName())
		b.Parent = f
	end
	f.Parent = workspace
end

local alive = true
ctx.stopWatch = function() alive = false end
task.spawn(function()
	local last = {}
	while alive do
		for _, s in ipairs(specs) do
			local p = s.part
			if p.Parent and goal.Parent then
				local key = tostring(p.Position) .. tostring(p.Size) .. tostring(goal.Position)
				if last[s.line] ~= key then
					last[s.line] = key
					local ok, err = pcall(function()
						local feet = p.Position - Vector3.new(0, p.Size.Y / 2 - 0.5, 0)
						local path, msg = PT.solve(r, feet, goal.Position, s.prof, nil, { nearest = true })
						if path then
							local _, line = PT.draw(r, path, { colour = s.colour, name = s.line, lift = s.lift, rubberBand = true })
							local hits = PT.bodyCheck(line, p.Size, STEP, rp)
							drawClips(s.clips, hits)
							p:SetAttribute("BodyClips", #hits)
						else
							for _, n in ipairs({ s.line, s.clips }) do
								local o = workspace:FindFirstChild(n)
								if o then o:Destroy() end
							end
							p:SetAttribute("BodyClips", nil)
						end
						p:SetAttribute("PathStatus", msg)
					end)
					if not ok then p:SetAttribute("PathStatus", "draw error " .. tostring(err)) end
				end
			end
		end
		task.wait(0.25)
	end
end)
task.wait(1)
return "watching"
