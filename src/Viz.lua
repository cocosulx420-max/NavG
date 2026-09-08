--!strict
-- NVGN.Viz -- the debug display, as a module instead of a throwaway script.
--
-- This drawing existed three times as ad-hoc code pasted into the command bar
-- and was lost each time. It lives here now.
--
--   * fill    -- every cell of a region, in the region's own colour
--   * borders -- the region's boundary cells, a darker shade of that colour
--   * lines   -- the contour edges, neon, lifted clear of the surface
--   * spikes  -- coincident edge pairs at <1 degree, as red balls
--
-- FILL IS CAPPED. One part per cell is fine on case3 (9k) and not on case5
-- (200k): a draw that size saturates Studio for minutes and the screen capture
-- times out well before it. Past `maxFill` the fill is skipped and the borders
-- alone carry the shape, which reads well enough at that scale.

local Contour = require(script.Parent.Contour)

local Viz = {}

local ROOT = "NavGenViz"

local DEFAULT = {
	leaf = 0.5,
	lift = 0.8,        -- how far above the surface the contour lines float
	maxFill = 40000,   -- above this many cells, borders only
	fill = true,
	borders = true,
	lines = true,
	spikes = true,
	name = "viz",
	-- The bake's own config. Contour reads far more than `leaf` from it, so
	-- passing anything else makes the picture disagree with the pipeline it is
	-- supposed to be showing: with defaults instead of the real config, case5
	-- drew 510 lines and 5 spikes where the run produced 618 and 35.
	cfg = false,
}

local function merged(o)
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if o then for k, v in pairs(o) do if c[k] ~= nil then c[k] = v end end end
	return c
end

local function hue(i: number): number
	return (i * 0.61803398875) % 1
end

local function folder(parent: Instance, name: string): Folder
	local f = parent:FindFirstChild(name)
	if not f then
		local nf = Instance.new("Folder"); nf.Name = name; nf.Parent = parent
		return nf
	end
	return f :: Folder
end

local function flat(size: Vector3, cf: CFrame, colour: Color3, mat: Enum.Material, parent: Instance, name: string)
	local p = Instance.new("Part")
	p.Anchored = true
	p.CanCollide = false
	p.CanQuery = false
	p.CanTouch = false
	p.Material = mat
	p.Color = colour
	p.Size = size
	p.CFrame = cf
	p.Name = name
	p.Parent = parent
end

function Viz.clear(name: string?)
	local root = workspace:FindFirstChild(ROOT)
	if not root then return 0 end
	if name == nil then
		local n = #root:GetDescendants()
		root:Destroy()
		return n
	end
	local f = root:FindFirstChild(name)
	if not f then return 0 end
	local n = #f:GetDescendants()
	f:Destroy()
	return n
end

-- `regions` and `walk` come straight from NodeWalk.regions / NodeWalk.run.
function Viz.draw(walk, regions, opts)
	local c = merged(opts)
	local root = folder(workspace, ROOT)
	local sec = root:FindFirstChild(c.name)
	if sec then sec:Destroy() end
	sec = folder(root, c.name)

	local fFill = c.fill and folder(sec, "Regions") or nil
	local fEdge = c.borders and folder(sec, "Borders") or nil
	local fLine = c.lines and folder(sec, "Lines") or nil
	local fSpike = c.spikes and folder(sec, "Spikes") or nil

	local total = 0
	for _, r in ipairs(regions) do total += r.size end
	local doFill = c.fill and total <= c.maxFill

	local st = {
		regions = #regions, cells = total, fill = 0, borders = 0,
		lines = 0, spikes = 0, failed = 0,
		fillSkipped = (c.fill and not doFill) or false,
		collapsing = 0, latticeCells = 0, latticeSlots = 0,
	}

	for ri, r in ipairs(regions) do
		local cells = {}
		for _, i in ipairs(r.cells) do table.insert(cells, walk[i]) end

		local ok, cr = pcall(Contour.run, cells, c.cfg or { leaf = c.leaf })
		if not ok then
			st.failed += 1
			continue
		end

		local h = hue(ri)
		local cFill = Color3.fromHSV(h, 0.45, 1.0)
		local cEdge = Color3.fromHSV(h, 1.00, 0.40)
		local cLine = Color3.fromHSV(h, 0.85, 1.0)
		local L = cr.lattice

		local slots = 0
		for _ in pairs(L.occ) do slots += 1 end
		st.latticeCells += #cells
		st.latticeSlots += slots
		if slots < #cells * 0.9 then st.collapsing += 1 end

		if doFill or fEdge then
			local sz = Vector3.new(c.leaf * 0.94, 0.08, c.leaf * 0.94)
			for k, cell in pairs(L.partAt) do
				local isEdge = cr.seed[k] ~= nil
				local target = isEdge and fEdge or (doFill and fFill or nil)
				if target then
					flat(sz, CFrame.fromMatrix(cell.face + L.up * 0.04, L.u, L.up),
						isEdge and cEdge or cFill, Enum.Material.SmoothPlastic,
						target, "r" .. ri)
					if isEdge then st.borders += 1 else st.fill += 1 end
				end
			end
		end

		if fLine then
			for _, e in ipairs(cr.edges) do
				local d = e.b - e.a
				if d.Magnitude > 1e-4 then
					flat(Vector3.new(0.24, 0.24, d.Magnitude),
						CFrame.lookAt(e.a + d / 2 + L.up * c.lift, e.b + L.up * c.lift),
						cLine, Enum.Material.Neon, fLine, ("r%d_line"):format(ri))
					st.lines += 1
				end
			end
		end

		if fSpike then
			for i = 1, #cr.edges do
				for j = i + 1, #cr.edges do
					local s, t = cr.edges[i], cr.edges[j]
					for _, wa in ipairs({ "a", "b" }) do
						for _, wb in ipairs({ "a", "b" }) do
							if (s[wa] - t[wb]).Magnitude < 1e-3 then
								local ang = math.deg(math.acos(
									math.clamp(math.abs(s.dir:Dot(t.dir)), -1, 1)))
								if ang < 1 then
									local b = Instance.new("Part")
									b.Anchored = true; b.CanCollide = false; b.CanQuery = false
									b.Shape = Enum.PartType.Ball
									b.Material = Enum.Material.Neon
									b.Color = Color3.new(1, 0, 0)
									b.Size = Vector3.new(1.3, 1.3, 1.3)
									b.Position = s[wa] + L.up * (c.lift + 0.3)
									b.Name = "spike"
									b.Parent = fSpike
									st.spikes += 1
								end
							end
						end
					end
				end
			end
		end

		-- a big bake is thousands of parts per region; let Studio breathe
		if ri % 5 == 0 then task.wait() end
	end

	return st
end

return Viz
