# NavGenProj

Baked navmesh generation for Roblox, with destructible authored environments.

A fresh start from the `nvgn` prototype. Everything up to and including the
per-part local grid is carried over unchanged, because that part works and is
validated. Nothing past it is: boundary extraction, region tracing,
polygonization and the pathfinder are all left behind deliberately.

## What is here

| Module | Role |
| --- | --- |
| `src/SVO.lua` | Sparse voxel octree of solid space. Broad phase only — which parts host walkable floor, and solid queries at runtime. Never the mesh. |
| `src/Floor.lua` | Walkable surface extraction. SVO proposes candidates, a downward raycast onto the real part gives exact height and normal, so ramps come back smooth. Max slope 65°. Headroom from an upward raycast. |
| `src/LocalGrid.lua` | One grid PER PART, aligned to that part's own axes. Cells are live or dead; a dead cell records what killed it. This is the truth about what is walkable. |
| `src/SVOLocal.lua` | One octree PER PART, in that part's own frame, so a rotated part voxelizes without an axis-aligned bounding box swelling it. Nodes touching a neighbouring part are marked as seams. Box-exact parts are solved analytically; meshes and unions fall back to `GetPartsInPart`. |
| `src/SVOLocalDebug.lua` | Draws the per-part octrees as parts, hides and restores the source geometry, and culls a drawing to the nodes touching real geometry. Visualization only. |

## What is not here, and why

The old pipeline derived boundary lines by stealing face planes from blocking
parts, then tried to close them into rings. On authored geometry those lines
mostly do not close, so they partition nothing and polygons end up covering
walls.

The local grid is good at answering *what is walkable*. What it is bad at is
*where exactly the edge is* — its outline is jagged at the 1-stud pitch, and
that staircase is the thing this project exists to kill. Solving that without
going back to stealing geometry from blocks is the open problem.
