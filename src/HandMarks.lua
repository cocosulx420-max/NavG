--!strict
-- The 18 corner marks Cocosulx placed BY HAND on SmallMap. This is the ground
-- truth the corner stage is measured against, and there is no way to regenerate
-- it -- it is a record of human judgement about where a corner is, not an
-- output of anything.
--
-- It used to exist only as DiamondPlate material on parts inside
-- workspace.NVGN_Debug.NodeClasses, which `LocalGrid.visualizeClasses` destroys
-- and rebuilds from scratch on every call. That is exactly as fragile as it
-- sounds, and it was lost once. Now it lives in git.
--
-- Positions are world space, from the bake with root = workspace.SmallMap.
-- 12 sit on the lower level (y ~ 1.43) and 6 on the upper (y ~ 3.43); the six
-- upper ones still carry the default visualiser names rather than case names.
--
-- Duplicate names are Cocosulx's own and are deliberate -- two marks named
-- "case2", "case9" and "edge1" each mark a separate place with the same kind of
-- problem. Index them by position, never by name.

local HandMarks = {}

export type Mark = { name: string, pos: Vector3 }

local MARKS: {Mark} = {
	{ name = "case 6", pos = Vector3.new(-7.9453, 1.4319, 47.7335) },
	{ name = "case1", pos = Vector3.new(-20.6635, 1.4319, 43.4620) },
	{ name = "case2", pos = Vector3.new(-22.1944, 1.4319, 32.5690) },
	{ name = "case2", pos = Vector3.new(-32.3540, 1.4319, -39.7206) },
	{ name = "case3", pos = Vector3.new(-6.2270, 1.4319, 38.4036) },
	{ name = "case4", pos = Vector3.new(-11.7672, 1.4319, 49.2804) },
	{ name = "case5", pos = Vector3.new(-19.2718, 1.4319, 53.3646) },
	{ name = "case8", pos = Vector3.new(-8.5341, 1.4319, 57.9145) },
	{ name = "case9", pos = Vector3.new(-16.0387, 1.4319, 61.9987) },
	{ name = "case9", pos = Vector3.new(-52.6786, 1.4319, 67.1481) },
	{ name = "edge1", pos = Vector3.new(9.4032, 1.4319, 5.9121) },
	{ name = "edge1", pos = Vector3.new(-8.8392, 1.4319, 5.4464) },
	{ name = "w0_d227", pos = Vector3.new(-9.2379, 3.4319, 46.5284) },
	{ name = "w0_d248", pos = Vector3.new(-6.7293, 3.4319, 39.9933) },
	{ name = "w192_d56", pos = Vector3.new(-12.3308, 3.4319, 37.8431) },
	{ name = "w2_d60", pos = Vector3.new(-7.6629, 3.4319, 39.6349) },
	{ name = "w32_d0", pos = Vector3.new(-14.1227, 3.4319, 42.5110) },
	{ name = "w8_d0", pos = Vector3.new(-9.4548, 3.4319, 44.3028) },
}

-- A field cannot carry a type annotation in Luau, so the list is annotated as a
-- local and then attached. It is the same table, not a copy.
HandMarks.marks = MARKS

-- How many marks have a point within `radius` studs. The corner stage's recall
-- score, and the number to beat is 18.
function HandMarks.score(points: {Vector3}, radius: number?): (number, {string})
	local r = radius or 2.0
	local hit, missed = 0, {}
	for _, m in ipairs(MARKS) do
		local best = math.huge
		for _, p in ipairs(points) do
			local d = (p - m.pos).Magnitude
			if d < best then best = d end
		end
		if best <= r then
			hit += 1
		else
			missed[#missed + 1] = string.format("%s (%.2f)", m.name, best)
		end
	end
	return hit, missed
end

-- Put the marks back on the nearest visualiser node, after anything rebuilt
-- workspace.NVGN_Debug.NodeClasses.
function HandMarks.restore(folder: Instance?): number
	local nc = folder
	if not nc then
		local dbg = workspace:FindFirstChild("NVGN_Debug")
		nc = dbg and dbg:FindFirstChild("NodeClasses")
	end
	if not nc then return 0 end
	local dots = nc:GetChildren()
	local n = 0
	for _, m in ipairs(MARKS) do
		local best, bd = nil, math.huge
		for _, d in ipairs(dots) do
			if d:IsA("BasePart") then
				local dist = (d.Position - m.pos).Magnitude
				if dist < bd then bd, best = dist, d end
			end
		end
		if best and bd <= 2.0 then
			(best :: BasePart).Material = Enum.Material.DiamondPlate
			n += 1
		end
	end
	return n
end

return HandMarks
