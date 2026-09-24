-- NVGN_FollowTest -- play-mode check of the navmesh: NPCs that follow you.
--
-- Loads the exported navmesh (ServerStorage.NVGN_NavData) and, several times a
-- second, solves a path from each follower to the first player with
-- NavGenProj.PathTest (A* over polygons + funnel through the portals), FOR THAT
-- FOLLOWER'S PROFILE from NavGenProj.Agents, and walks it. One follower per
-- entry in FOLLOWERS below.
--
-- Jumps: a jump link on the path makes the follower jump as it reaches it.
-- Drops: it simply walks off. Stuck fallback: no progress for STUCK_TIME
-- seconds bans the portal it was heading for, for BAN_TIME seconds, and it
-- repaths around it.
--
-- Each follower's "PathStatus" attribute says what it is doing.

local FOLLOWERS = {
	{ name = "NVGN_Follower", profile = "default", colour = Color3.fromRGB(40, 200, 255) },
	-- lineLift: its path line sits a little higher, so it shows where both routes overlap
	{ name = "NVGN_FollowerWide", profile = "wide", colour = Color3.fromRGB(255, 150, 40), scale = 2, lineLift = 0.15 },
}
local STUCK_TIME, BAN_TIME = 1.5, 10
local LEAP_REACH, LEAP_PAST = 1.5, 1.5 -- studs: jump this close to the takeoff edge, aim this far past the landing edge

local Players = game:GetService("Players")
local SS = game:GetService("ServerStorage")
local ng = game:GetService("ServerScriptService"):WaitForChild("NavGenProj")
local PathTest = require(ng:WaitForChild("PathTest"))
local Agents = require(ng:WaitForChild("Agents"))
local raw = require(SS:WaitForChild("NVGN_NavData"))

-- a wide test profile if the game did not define one
if not Agents.profiles.wide then
	local w = table.clone(Agents.profiles.default)
	w.radius = 2
	Agents.profiles.wide = w
end

local function V(t: { number }, i: number): Vector3 return Vector3.new(t[i], t[i + 1], t[i + 2]) end
local tris = {}
for i, t in ipairs(raw.tris) do
	local verts = {}
	for k = 1, #t.v, 3 do verts[#verts + 1] = V(t.v, k) end
	tris[i] = { region = t.r, centre = V(t.c, 1), up = V(t.u, 1), area = t.a, verts = verts, n = #verts,
		headroom = t.h, headroomOpen = t.ho, width = t.w, slope = t.s }
end
local links = {}
for i, L in ipairs(raw.links) do
	links[i] = {
		kind = L.k, a = L.a, b = L.b, left = V(L.l, 1), right = V(L.r, 1), centre = V(L.c, 1),
		edge = L.e == 1, bLeft = L.bl and V(L.bl, 1) or nil, bRight = L.br and V(L.br, 1) or nil,
		gap = L.g, drop = L.d, rise = L.ri, clear = L.cl, oneWay = L.ow == 1, span = L.sp, vault = L.va,
	}
end
local result = { mesh = { tris = tris }, portals = { links = links } }
-- floor probes skip characters and the drawn paths
local groundParams = RaycastParams.new()
groundParams.FilterType = Enum.RaycastFilterType.Exclude
local function refreshGroundFilter()
	local ex = {}
	for _, pl in ipairs(Players:GetPlayers()) do if pl.Character then ex[#ex + 1] = pl.Character end end
	for _, n in ipairs({ "NVGN_Follower", "NVGN_FollowerWide", "NVGN_Path_NVGN_Follower", "NVGN_Path_NVGN_FollowerWide" }) do
		local x = workspace:FindFirstChild(n)
		if x then ex[#ex + 1] = x end
	end
	groundParams.FilterDescendantsInstances = ex
end
task.spawn(function() while true do refreshGroundFilter(); task.wait(1) end end)
print(("[NVGN_FollowTest] navmesh loaded: %d polygons, %d links"):format(#tris, #links))

local function playerRoot(): BasePart?
	local p = Players:GetPlayers()[1]
	local c = p and p.Character
	return c and c:FindFirstChild("HumanoidRootPart") :: BasePart?
end

local template = workspace:FindFirstChild("NVGN_FollowerTemplate")
	or (workspace:FindFirstChild("case6") and workspace.case6:FindFirstChild("Rig", true))
assert(template, "[NVGN_FollowTest] no Rig to copy")

local pr = playerRoot()
while not pr do task.wait(0.5); pr = playerRoot() end

local function spawnFollower(spec: any, offset: Vector3)
	local npc = template:Clone()
	npc.Name = spec.name
	if spec.scale and npc.ScaleTo then npc:ScaleTo(spec.scale) end
	for _, d in ipairs(npc:GetDescendants()) do
		if d:IsA("BasePart") then d.Anchored = false; if d.Name == "Torso" or d.Name == "UpperTorso" then d.Color = spec.colour end end
	end
	local hum = npc:FindFirstChildOfClass("Humanoid")
	local root = npc:FindFirstChild("HumanoidRootPart")
	assert(hum and root, "[NVGN_FollowTest] the Rig has no Humanoid / HumanoidRootPart")
	npc.PrimaryPart = root
	npc:PivotTo(CFrame.new(pr.Position + offset))
	npc.Parent = workspace
	root:SetNetworkOwner(nil)
	-- THE BODY THAT WAS SPAWNED, not the profile's paper height. A Humanoid
	-- cannot crouch or crawl, so every posture is its full height; the wide
	-- rig is scaled 2x and was being sent under stairs sized for a 5 stud
	-- body. HEAD_SLACK lets a head brush an eave (polygon headroom is its
	-- LOWEST cell, and the ground ring's rims sit 9.9 under the house's eaves).
	local HEAD_SLACK = 0.5
	local prof = table.clone(Agents.get(spec.profile))
	local _, rigSize = npc:GetBoundingBox()
	prof.height = rigSize.Y - HEAD_SLACK
	prof.crouch, prof.prone = prof.height, prof.height
	npc:SetAttribute("Profile", spec.profile)

	task.spawn(function()
		local banned: { [any]: number } = {}
		local lastPos, lastMoveAt = root.Position, os.clock()
		local lastDraw = 0
		while npc.Parent and hum.Health > 0 do
			local target = playerRoot()
			if target then
				local ok, err = pcall(function()
					local now = os.clock()
					for k, t in pairs(banned) do if t < now then banned[k] = nil end end
					-- IN THE AIR, KEEP GOING. A solve from mid-jump finds no floor under
					-- the feet (a leap here is 4 to 11 studs, snapHeight is 4) and the
					-- follower stopped dead mid-air.
					local st = hum:GetState()
					if st == Enum.HumanoidStateType.Freefall or st == Enum.HumanoidStateType.Jumping then
						lastPos, lastMoveAt = root.Position, now
						return
					end
					-- solve from the FEET: a scaled rig's root sits too high to find its floor
					local hip0 = (hum.RigType == Enum.HumanoidRigType.R6) and 2 * (spec.scale or 1) or hum.HipHeight
					local feetPos = root.Position - Vector3.new(0, hip0 + root.Size.Y * 0.5 - 0.5, 0)
					-- the player's FLOOR, not their root: a jumping player is in the air too
					local goal = target.Position
					local down = workspace:Raycast(goal, Vector3.new(0, -60, 0), groundParams)
					if down then goal = down.Position + Vector3.new(0, 0.5, 0) end
					-- nearest: out of reach, it waits at the closest point it can get to
					local path, msg = PathTest.solve(result, feetPos, goal, prof, banned, { nearest = true })
					local nb = 0 for _ in pairs(banned) do nb += 1 end
					npc:SetAttribute("PathStatus", (msg or "?") .. (nb > 0 and (" | %d banned"):format(nb) or ""))
					-- each follower draws its own path in its own colour
					local lineName = "NVGN_Path_" .. spec.name
					if not path then
						hum:MoveTo(root.Position)
						local stale = workspace:FindFirstChild(lineName)
						if stale then stale:Destroy() end
						return
					end
					if now - lastDraw > 0.5 then
						lastDraw = now
						PathTest.draw(result, path, { colour = spec.colour, name = lineName, lift = spec.lineLift })
					end
					local flat = Vector3.new(1, 0, 1)
					if ((target.Position - root.Position) * flat).Magnitude < 4 then
						hum:MoveTo(root.Position)
						lastPos, lastMoveAt = root.Position, now
						return
					end
					local pts = path.points
					local wp = pts[#pts]
					for k = 2, #pts do
						if ((pts[k] - root.Position) * flat).Magnitude > 1.0 then wp = pts[k] break end
					end
					hum:MoveTo(wp)
					-- the first portal ahead: jump for a jump link, or for a step taller than auto-step
					-- A LEAP IS JUMPED FORWARD, ONTO THE FAR SIDE. It used to fire within
					-- 2.5 of the gate's CENTRE with the move target still on the takeoff
					-- side, so it hopped in place and landed where it started. Now it
					-- jumps within LEAP_REACH of the takeoff EDGE and moves at the landing,
					-- LEAP_PAST beyond its edge, so it clears the lip. A drop over a rail
					-- (the stairwell's 3 stud vaults) is the same move.
					local leapt = false
					local first = path.chain[1]
					if first then
						local L = first.e.L
						local vaulting = L.kind == "drop" and (L.vault or 0) > prof.step * 0.5
						if (L.kind == "jump" or vaulting) and L.bLeft and L.bRight then
							local function onSeg(a: Vector3, b: Vector3, q: Vector3): Vector3
								local d = (b - a) * flat
								local dd = d:Dot(d)
								local t = dd > 1e-9 and math.clamp(((q - a) * flat):Dot(d) / dd, 0, 1) or 0
								return a + (b - a) * t
							end
							local take = onSeg(L.left, L.right, root.Position)
							if ((take - root.Position) * flat).Magnitude < LEAP_REACH then
								local land = onSeg(L.bLeft, L.bRight, take)
								local out = (land - take) * flat
								out = out.Magnitude > 1e-3 and out.Unit or ((land - root.Position) * flat).Unit
								hum:MoveTo(land + out * LEAP_PAST)
								hum.Jump = true
								leapt = true
							end
						end
					end
					local hip = (hum.RigType == Enum.HumanoidRigType.R6) and 2 * (spec.scale or 1) or hum.HipHeight
					local feet = root.Position.Y - (hip + root.Size.Y * 0.5)
					-- a step taller than auto-step on the way
					if not leapt and wp.Y - feet > 1.2 then hum.Jump = true end
					-- stuck fallback
					if (root.Position - lastPos).Magnitude > 1.0 then
						lastPos, lastMoveAt = root.Position, now
					elseif now - lastMoveAt > STUCK_TIME then
						if first then banned[first.e.L] = now + BAN_TIME end
						lastPos, lastMoveAt = root.Position, now
					end
				end)
				if not ok then npc:SetAttribute("PathStatus", "error " .. tostring(err)) end
			end
			task.wait(0.2)
		end
	end)
end

for i, spec in ipairs(FOLLOWERS) do
	spawnFollower(spec, Vector3.new(6 * i, 0, 0))
end
