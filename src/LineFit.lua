--!strict
-- NavGen.LineFit — the greedy line fit stage.
--
-- Input is an ordered chain of integer grid cells. Output is the indices of the
-- cells that are vertices of a simplified polyline. Consecutive segments share
-- an endpoint, so the polyline is connected by construction.
--
-- THE WHOLE OF THE PER-SEGMENT STATE IS `startIdx` AND FOUR BOOLEANS. No major
-- axis, no minor axis, no slope, no step counting, no cadence. Every earlier
-- attempt at this stage failed by reading the STEPPING RATE of a staircase, and
-- a rate is not a fact about the ground: two identical walls half a stud apart
-- quantise differently, and an irregular staircase is still a staircase.
--
-- Two break rules, and the asymmetry between them is the point.
--
--   Rule 1, direction lock (hard, no rollback). A segment may never use two
--   opposite directions. A run that goes both up and down carries detail a
--   straight line would erase, so it never merges no matter how small the
--   deviation is. This is feature preservation, not error handling. One check
--   covers up-then-down, down-then-up and a contour doubling back, with no
--   special case for short segments.
--
--   Rule 2, corridor tolerance (soft, always rolls back). Test EVERY cell of
--   the run against the chord from the start to the candidate.
--
-- WHY EVERY CELL, AND NOT JUST THE NEW ONE. The failure appears BEHIND the
-- walker. Extending into a new slope regime rotates the chord, and the cells
-- that blow tolerance are the already-accepted ones near the start — the new
-- cell sits right on the chord it just defined. An implementation that tests
-- only cell `k` never detects this and runs straight through the corner.
--
-- WHY ROLLBACK GOES TO THE ARGMAX AND NOT TO `k-1`. The cell that failed
-- hardest is where the two slope regimes actually parted ways. `k-1` is merely
-- where belief ran out, which is somewhere inside the NEXT feature. So the
-- vertex is cut back to where the deviation began, and that cell belongs to
-- both segments as their shared vertex.
--
-- NO FLOATS IN THE DECISION PATH. The corridor test is an integer cross product
-- compared against a rational tolerance, cross-multiplied. Identical input
-- gives byte-identical output on every run, on every machine.
--
-- Coordinates are the project's grid frame, {x, z} — see Boundary.lua. The
-- stage is otherwise frame-agnostic: it reads cell coordinates and nothing else.
--
-- NOTE FOR THE CALLER: a tolerance of one cell means a fitted line can sit up to
-- a cell OUTSIDE the traced boundary on an outward-bulging run. Whatever inward
-- bias runs downstream must absorb at least `tolNum/tolDen` before any offset
-- stage, or segments will cut into geometry. Not this stage's job to fix.

local LineFit = {}

export type Cell = { x: number, z: number }

export type Config = {
	-- The corridor half-width, as a rational so the test stays in integers.
	-- One cell is the default and is this stage's main tuning knob. It is NOT a
	-- knob that moves corners around: rule 2 picks the argmax cell, and which
	-- cell deviates most is a fact about the chain, not about the threshold.
	-- The tolerance only decides WHEN that fact is acted on.
	tolNum: number?,
	tolDen: number?,

	-- Closed contours wrap; open chains have genuine endpoints. The caller knows
	-- which it traced, so it says so rather than this stage guessing.
	closed: boolean?,

	-- Where to start looking. Arbitrary by contract — see the bootstrap below.
	seed: number?,
}

local DEFAULT = {
	tolNum = 1,
	tolDen = 1,
	closed = false,
	seed = 1,
}

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

--------------------------------------------------------------------------
-- Walking
--------------------------------------------------------------------------

-- Direction slots: 1 = +x, 2 = -x, 3 = +z, 4 = -z.
--
-- Marked from the SIGN of each component independently rather than by matching
-- one of four unit steps. For the 4-connected input the contract promises, the
-- two are identical. The sign form additionally survives a chain that carries a
-- diagonal step, which is what the adversarial seed case (test I) is written in,
-- and it keeps rule 1 exactly as stated: an axis used in both directions breaks.
local function markDirs(used: { boolean }, dx: number, dz: number): boolean
	if dx > 0 then if used[2] then return false end used[1] = true
	elseif dx < 0 then if used[1] then return false end used[2] = true end
	if dz > 0 then if used[4] then return false end used[3] = true
	elseif dz < 0 then if used[3] then return false end used[4] = true end
	return true
end

-- Would this step put both signs of an axis in play? Asked BEFORE committing,
-- because rule 1 breaks at the current cell and keeps everything up to and
-- including the cell before the step.
local function dirLocked(used: { boolean }, dx: number, dz: number): boolean
	if dx > 0 and used[2] then return true end
	if dx < 0 and used[1] then return true end
	if dz > 0 and used[4] then return true end
	if dz < 0 and used[3] then return true end
	return false
end

local function advance(i: number, stepDir: number, n: number, closed: boolean): number?
	local j = i + stepDir
	if closed then
		if j > n then return 1 elseif j < 1 then return n end
		return j
	end
	if j < 1 or j > n then return nil end
	return j
end

-- The corridor test. `run` is the list of cell indices from the segment start
-- through the candidate. Returns whether the candidate is rejected, and the
-- POSITION IN `run` of the cell that deviated most.
--
-- |cross| is a monotone stand-in for perpendicular distance while the chord is
-- held fixed, so the argmax of |cross| is the argmax of the true distance. No
-- square root is needed to rank them and none is taken.
local function corridor(cells: { Cell }, run: { number }, tolNum: number, tolDen: number): (boolean, number?, number)
	local a = cells[run[1]]
	local k = cells[run[#run]]
	local dx, dz = k.x - a.x, k.z - a.z
	-- Candidate coincides with the start (a closed loop can do this). No chord
	-- exists, so there is nothing to be outside of.
	if dx == 0 and dz == 0 then return false, nil, 0 end

	local best, bestPos = -1, nil
	for p = 2, #run - 1 do
		local q = cells[run[p]]
		local cr = dx * (q.z - a.z) - dz * (q.x - a.x)
		if cr < 0 then cr = -cr end
		if cr > best then best, bestPos = cr, p end
	end
	if bestPos == nil then return false, nil, 0 end

	-- dist > tolNum/tolDen  <=>  cross^2 * tolDen^2 > tolNum^2 * |d|^2
	local dd = dx * dx + dz * dz
	local fail = (best * best * tolDen * tolDen) > (tolNum * tolNum * dd)
	return fail, bestPos, best
end

export type RunResult = {
	endIdx: number,     -- last cell belonging to this segment
	reason: string,     -- "dirlock" | "corridor" | "guard" | "end" | "lap"
	steps: number,      -- how many cells were walked before breaking
}

-- Grow one segment from `startIdx` in `stepDir` (+1 forward, -1 backward).
--
-- Every rule here is direction-agnostic. The opposite-pair check is symmetric
-- under negating all directions, and the chord test and the argmax rollback do
-- not care about travel order. So the backward bootstrap reuses this function
-- with stepDir = -1; there is no reversed variant of anything.
--
-- `cfg.stopIdx` is a hard wall, not a rule: on a closed loop the forward pass
-- must not walk past the anchor, or it laps the contour forever re-emitting the
-- same corners. Reaching it ends the segment with reason "wrap". It does not
-- affect what the rules decide, only where walking is allowed to stop.
local function growRun(cells: { Cell }, startIdx: number, stepDir: number, cfg: any): RunResult
	local n = #cells
	local closed: boolean = cfg.closed
	local stopIdx: number? = cfg.stopIdx
	local run = { startIdx }
	local used = { false, false, false, false }
	local cur = startIdx
	local steps = 0

	while true do
		local nxt = advance(cur, stepDir, n, closed)
		if nxt == nil then return { endIdx = cur, reason = "end", steps = steps } end

		local a, b = cells[cur], cells[nxt]
		local dx, dz = b.x - a.x, b.z - a.z

		-- RULE 1. Break at the current cell, no rollback.
		if dirLocked(used, dx, dz) then
			return { endIdx = cur, reason = "dirlock", steps = steps }
		end

		-- RULE 2. Test the candidate against the whole run behind it.
		table.insert(run, nxt)
		local fail, bestPos = corridor(cells, run, cfg.tolNum, cfg.tolDen)
		if fail then
			table.remove(run)
			local offset = (bestPos :: number) - 1
			-- Guard: refuse to emit a degenerate segment. STRICTLY less than 2 —
			-- an argmax exactly 2 cells along is legitimate and is the answer in
			-- the adversarial seed case, where a `<= 2` guard silently returns a
			-- vertex two cells past the true corner, inside the wrong regime.
			if offset < 2 then
				return { endIdx = cur, reason = "guard", steps = steps }
			end
			return { endIdx = run[bestPos :: number], reason = "corridor", steps = steps }
		end

		markDirs(used, dx, dz)
		cur = nxt
		steps += 1

		-- A closed loop that never broke has no corners.
		if closed and cur == startIdx then
			return { endIdx = cur, reason = "lap", steps = steps }
		end
		if stopIdx ~= nil and cur == stopIdx then
			return { endIdx = cur, reason = "wrap", steps = steps }
		end
	end
end

LineFit._growRun = growRun

--------------------------------------------------------------------------
-- Seeding
--------------------------------------------------------------------------

-- The cell handed to this stage is arbitrary. It usually lands mid-run, and a
-- naive forward-only pass would emit a false vertex there and fragment the run
-- it landed in.
--
-- BACKWARD BOOTSTRAP. Grow a segment backward from the seed under the identical
-- break rules; wherever it breaks is the anchor. Then throw that segment away
-- and run the ordinary forward pass from the anchor. Because a straight run
-- breaks at the same place no matter where inside it you start walking
-- backward, every seed within a run resolves to the same anchor — which is the
-- whole reason the output does not depend on the seed.
--
-- Corner detection falls out of this: if the backward walk breaks within a cell
-- or two of the seed, the seed was already on or beside a genuine vertex. There
-- is no separate corner test because the same two rules already answer it.
--
-- An OPEN chain needs no bootstrap. Its first and last cells are endpoints, so
-- they are vertices by definition and the contract requires the output to
-- reproduce them exactly; anchoring anywhere else would drop the head of the
-- chain. Index 1 is therefore not an arbitrary seed, it is a real vertex, and
-- seed independence on an open chain holds trivially rather than accidentally.
local function findAnchor(cells: { Cell }, cfg: any): number
	if not cfg.closed then return 1 end
	local r = growRun(cells, cfg.seed, -1, cfg)
	if r.reason == "lap" then
		-- Walked a full lap without breaking: the contour has no corners.
		return cfg.seed
	end
	return r.endIdx
end

--------------------------------------------------------------------------
-- The stage
--------------------------------------------------------------------------

export type Result = {
	vertices: { number },  -- indices into `cells`
	anchor: number,
	reasons: { string },   -- why each segment ended, parallel to the gaps
}

function LineFit.fit(cells: { Cell }, cfg: Config?): Result
	local c = merged(cfg)
	local n = #cells
	if n == 0 then return { vertices = {}, anchor = 1, reasons = {} } end
	if n == 1 then return { vertices = { 1 }, anchor = 1, reasons = {} } end

	c.seed = math.max(1, math.min(n, c.seed))
	local anchor = findAnchor(cells, c)

	local vertices = { anchor }
	local reasons = {}
	local cur = anchor
	-- The forward pass is walled at the anchor on a closed loop, so it stops
	-- there instead of lapping.
	local walk = merged(c)
	if c.closed then walk.stopIdx = anchor end

	-- Every segment consumes at least one cell, so this can only run n times.
	-- The bound is a tripwire for a rule change that stops making progress, not
	-- an expected exit.
	for _ = 1, n + 1 do
		local r = growRun(cells, cur, 1, walk)
		table.insert(reasons, r.reason)

		if r.reason == "end" then
			if r.endIdx ~= cur then table.insert(vertices, r.endIdx) end
			return { vertices = vertices, anchor = anchor, reasons = reasons }
		end

		if r.reason == "wrap" or r.reason == "lap" then
			-- CLOSED LOOP MERGE. The forward pass has come back to the anchor.
			-- Try ONCE to join the final segment to the first: grow from the last
			-- corner and see whether it reaches the first corner past the anchor
			-- without either rule firing. If it does, the anchor was never a
			-- corner and the pair becomes one segment. A single attempt is
			-- enough — when the anchor IS a genuine corner this merge fails, and
			-- it exists only to cover the case where the bootstrap found no
			-- break at all.
			if #vertices >= 2 then
				local mergeCfg = merged(c)
				mergeCfg.stopIdx = vertices[2]
				local joined = growRun(cells, vertices[#vertices], 1, mergeCfg)
				if joined.reason == "wrap" then table.remove(vertices, 1) end
			end
			return { vertices = vertices, anchor = anchor, reasons = reasons }
		end

		-- No forward progress means a rule is refusing to consume a cell. That
		-- is a bug in the rules, not a shape this stage should paper over.
		if r.endIdx == cur then
			error("LineFit: segment made no progress at cell " .. tostring(cur))
		end

		table.insert(vertices, r.endIdx)
		cur = r.endIdx
	end

	return { vertices = vertices, anchor = anchor, reasons = reasons }
end

return LineFit
