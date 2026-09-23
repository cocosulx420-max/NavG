-- Resident bake worker.
--
-- `execute_luau` is sandboxed and has no `_G`, so nothing survives between MCP
-- calls -- but a `task.spawn` coroutine keeps its own locals alive inside
-- Studio. Bake case6 ONCE in here, then answer questions about the same
-- `data` over a StringValue mailbox instead of re-baking (a case6 grid is ~6
-- minutes; a posted command is ~7 seconds).
--
-- Mailbox, all under ServerScriptService:
--   NVGN_Job  -- worker writes status: "bake ...", "ready", "run", "error ..."
--   NVGN_Cmd  -- caller writes "<id>|<luau source>"; the chunk is called as
--               f(data, ctx) and ctx carries req/ng/LocalGrid/Pipeline
--   NVGN_Out  -- worker writes "<id>|<JSON or tostring of the return>"
--
-- Only the divisible stages yield (Floor/SVO/fromFloor take `onProgress`);
-- Boundary.trace and CDT.build do not, so this worker stops at LocalGrid.build
-- and leaves the trace to a posted command.
local SSS = game:GetService("ServerScriptService")
local HttpService = game:GetService("HttpService")
local ng = SSS.NavGenProj

local function sv(name: string): StringValue
	local v = SSS:FindFirstChild(name)
	if not v then
		v = Instance.new("StringValue"); v.Name = name; v.Parent = SSS
	end
	return v :: StringValue
end

local job, cmd, out = sv("NVGN_Job"), sv("NVGN_Cmd"), sv("NVGN_Out")
job.Value = "starting"; cmd.Value = ""; out.Value = ""

-- The map to bake. One line, edited at install time.
local ROOT_NAME = "case6"
local ROOT = workspace:FindFirstChild(ROOT_NAME)
assert(ROOT, "no workspace." .. ROOT_NAME)

task.spawn(function()
	local loaded = {}
	local function req(inst)
		if loaded[inst] ~= nil then return loaded[inst] end
		local f = loadstring("local script,require=... " .. inst.Source, "=" .. inst.Name)
		local m = f(inst, req); loaded[inst] = m; return m
	end

	local ok, err = pcall(function()
		local LocalGrid = req(ng.LocalGrid)
		local Pipeline = req(ng.Pipeline)
		local t0 = os.clock()
		-- Yield on a WALL CLOCK gate, not per callback: case6 is a million
		-- columns at columnBudget 250, and a task.wait() on each would add a
		-- minute of frames to a six minute bake.
		local lastYield, lastNote = os.clock(), 0
		local cfg = {
			root = ROOT,
			onProgress = function(done, total)
				local now = os.clock()
				if now - lastNote > 1 then
					lastNote = now
					job.Value = string.format("bake %s/%s  %.0fs",
						tostring(done), tostring(total or "?"), now - t0)
				end
				if now - lastYield > 0.05 then lastYield = now; task.wait() end
			end,
		}
		local data = LocalGrid.build(cfg)
		local tBake = os.clock() - t0
		local ctx = { req = req, ng = ng, LocalGrid = LocalGrid, Pipeline = Pipeline }
		job.Value = string.format("ready  %.0fs  grids=%d regions=%s",
			tBake, #data.grids, tostring(data.stats.regions))

		local seen = ""
		while true do
			local v = cmd.Value
			if v ~= "" and v ~= seen then
				seen = v
				local id, src = v:match("^([^|]*)|(.*)$")
				job.Value = "run " .. tostring(id)
				local res
				local good, e = pcall(function()
					local f = loadstring(src, "=cmd")
					res = f(data, ctx)
				end)
				local text
				if not good then
					text = "ERROR " .. tostring(e)
				elseif typeof(res) == "table" then
					local enc, e2 = pcall(function() return HttpService:JSONEncode(res) end)
					text = enc and e2 or tostring(res)
				else
					text = tostring(res)
				end
				out.Value = id .. "|" .. text
				job.Value = "ready"
			end
			task.wait(0.2)
		end
	end)
	if not ok then job.Value = "error " .. tostring(err) end
end)
return "worker spawned"
