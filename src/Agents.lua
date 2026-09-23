--!strict
-- NVGN.Agents -- what the NPCs of this game can do, in ONE place.
--
-- NavG is meant to be reused across games, and a game can have several kinds
-- of NPC: taller, shorter, wider, better jumpers, ones that survive bigger
-- falls. The navmesh is baked ONCE for all of them. So the bake does not ask
-- "can an NPC do this", it RECORDS what is there -- a step's height, a drop's
-- depth, a gap's width, a polygon's headroom and width -- and each profile
-- filters the same mesh at path time.
--
-- The bake therefore reads only the ENVELOPE: the most permissive value across
-- every profile, so nothing any NPC could use is thrown away. Change a number
-- here and rebake; add a profile that fits inside the envelope and nothing
-- needs rebaking at all.
--
-- Studs throughout.
--   radius        half the NPC's width
--   height        headroom to walk upright
--   crouch        headroom to crouch
--   prone         headroom to crawl; below this nothing fits
--   step          tallest rise it walks straight up (Roblox character: 2)
--   jump          tallest ledge it can jump onto (Cocosulx measured 8)
--   jumpDistance  widest gap it can jump across
--   drop          deepest it will drop down; math.huge for any height

local Agents = {}

Agents.profiles = {
	default = {
		radius = 1.0, height = 5, crouch = 3, prone = 1.5,
		step = 2, jump = 8, jumpDistance = 8, drop = math.huge,
	},
}

-- Map-wide, not per NPC: every NPC type walks the same slopes (Cocosulx).
Agents.maxSlope = 65

-- The most permissive value across every profile: the largest reach, the
-- smallest body. The bake reads only this.
function Agents.envelope(): any
	local e = {
		radius = math.huge, height = math.huge, crouch = math.huge, prone = math.huge,
		step = 0, jump = 0, jumpDistance = 0, drop = 0,
		maxSlope = Agents.maxSlope,
	}
	for _, p in pairs(Agents.profiles) do
		e.radius = math.min(e.radius, p.radius)
		e.height = math.min(e.height, p.height)
		e.crouch = math.min(e.crouch, p.crouch)
		e.prone = math.min(e.prone, p.prone)
		e.step = math.max(e.step, p.step)
		e.jump = math.max(e.jump, p.jump)
		e.jumpDistance = math.max(e.jumpDistance, p.jumpDistance)
		e.drop = math.max(e.drop, p.drop)
	end
	return e
end

function Agents.get(name: string?): any
	return Agents.profiles[name or "default"] or Agents.profiles.default
end

return Agents
