# Global grid experiment: one world-aligned, adaptive, tiled grid, on one area of case6

## Context
Cocosulx finds the portals too messy. Root cause, agreed in discussion: the current pipeline
- splits the floor into ~3000 per-surface regions built on per-part-face grids,
- meshes each region separately,
- then rebuilds connectivity by guessing which polygon owns which cell and stitching regions back together with fitted, edge-matched, drop and jump links.

The local grid and Severance are clean; the round trip through polygons is what's messy.

The proposed replacement is a GLOBAL grid:
- One world-aligned grid, cut into tiles, each built independently, so destruction can rebuild single tiles later. The original bake is kept, and rebuilt tiles are a temporary layer on top of it.
- Adaptive top-down: big squares (16 studs) accepted wholesale where the SVO shows one flat, uninterrupted layer, refined only near walls, edges, steps and layer changes, down to 0.25.
- Connectivity decided on that one lattice by "can I step there", so stairs and kerbs sit inside one region and their portals become exact shared edges.
- Outlines straightened by Cocosulx's rubber band: tightened through the one-cell band between floor and not-floor, so they are straight but can never leave the floor or reach a wall.
- Few convex polygons per region, as today.

Goals: fewer polygons than now, portals as clean as the grid, tile-local rebuilds.

**This is an EXPERIMENT on one area, compared side by side with the current pipeline, before any decision to replace anything.** Nothing in the current pipeline is changed. Jump, drop and vault moves (the SVO arc tests) are NOT part of this experiment; they come after, on whichever mesh wins.

## Setup
- New branch **`v.c.0`** from `vba1.1` @ ebdc584. Only new files are added.
- Experiment area: a 3×3 block of 32-stud tiles (96×96 studs) around the stairs and bridge from Cocosulx's screenshots (centre about (-570, -425)), all heights. Big enough to have tile borders, stairs, a bridge over floor (two layers), walls, rails and ramps.
- Cell size and every threshold are settings in one table. The experiment runs at 0.5 AND 0.25.

## Steps

### 1. SVO column queries (`src/SVO.lua`, additive only)
Add `SVO:columnRuns(x, z, yLo, yHi)`: the solid intervals in one 1-stud column, found by walking only the octree nodes the column crosses, not by probing every voxel. It builds on the existing traversal behind `SVO:isSolid` (src/SVO.lua:178).
- From the runs: a column's walkable layers are solid tops with empty space above them at least the envelope prone height (`Agents.envelope()`).
- The experiment builds its own SVO over the area's parts: `SVO.fromParts`, leaf 1, as `Floor.build` does (src/Floor.lua:393). Characters are excluded, via Floor's `isCharacter` filter in `Floor.gatherParts`.

### 2. Adaptive global grid (new `src/GlobalGrid.lua`)
- Per tile, per walkable layer: a quadtree of square cells from 16 studs down to `minCell` (0.25 or 0.5), world-aligned.
- **Accept a square wholesale** when every 1-stud SVO column under it has the same layer structure: the same layer, tops within one leaf, the same headroom class, nothing poking up, not at a tile's outer edge. Then take its exact height and slope from ONE ray down at its centre (hit part plus normal). Plain block parts could use their top face arithmetically instead.
- **Otherwise split into four,** until uniform or `minCell`. Below 1 stud the SVO cannot see, so cells at `minCell` near walls and edges are decided by exact tests, carried over from LocalGrid:
  - a ray down per cell;
  - the kill test's narrow phase: `boxOverlap`, `wedgeOverlap`, `isBlock`, `isWedge`, `supportHalf` (src/LocalGrid.lua:379–496). Move these into a shared `src/Solid.lua` that both use, with no behaviour change;
  - a headroom probe that copes with rays starting inside a part;
  - maxSlope (65) from Agents.
- **Each cell records:** surface point, normal, slope, headroom, part, size, and posture (stand, crouch or prone, from the default profile).

### 3. Step-aware connectivity and regions
- For each cell edge, find the neighbour(s) across it on the same lattice: a quadtree neighbour walk, crossing into the adjacent tile's tree at tile borders.
- They are **connected** if the height difference at the shared edge is at most the envelope step (2.0) and the headroom is compatible.
- Otherwise the edge is a **boundary**, labelled wall (solid across, per the SVO or a ray), drop (lower floor or none), or ledge (higher than a step).
- **Regions** are flood fills of connected cells within a tile, so a staircase and its landings are ONE region. Tile borders are marked as cut edges, not boundaries.

### 4. Outline tracing
- Trace each region's boundary loops (outer rim plus holes) along the quadtree cell edges. It is one lattice, so there are no seams, no stitching and no shadow pieces.
- Tile-border cut edges stay exact straight segments on the border line, so neighbouring tiles line up.

### 5. Rubber band (new `src/RubberBand.lua`)
- For each loop, the band runs between the boundary cells' centre line (inside) and the floor's true edge (outside). That band is fed as a corridor of short portals to the same simple stupid funnel `PathTest` uses (src/PathTest.lua), and the tightened polyline becomes the outline.
- **Guaranteed:** it never leaves the band, so it is always within a cell of the real edge and never beyond it. It cannot cross another ring and cannot produce spikes or knots.
- Tile cut edges are anchors that are not moved.
- Heights: each outline point takes the height of the cell it lies on, so a staircase region's outline climbs.

### 6. Mesh, cells to polygons, portals
- Triangulate each region's banded rings with the existing `CDT.build(loops, nil)` (src/CDT.lua:1044; its `data` argument is optional) after `Rings.classify`. Polygons may rise across steps, which is fine for 2.5D. Vertex Y is kept from the input points.
- **Exact membership:** every cell is assigned to the polygon containing its centre in plan within its region, or the nearest polygon in its region for the few rim cells just outside.
- **Portals:**
  - inside a region: the shared edges (reuse `sharedLinks`' exact vertex matching, src/Portals.lua);
  - across tile borders: the overlap of the two polygons' edges on the shared border line, confirmed by real cell adjacency across the border.
  - No Severance search, no fitting, no fallbacks.
- **Output:** the same shape as today's result (`mesh.tris` with verts, centre, up, region; `portals.links` with left, right, kind), so `PathTest`, the audit and the drawing work unchanged.

### 7. Harness (`tools/globalgrid_experiment.lua`, committed)
- Runs the experiment on the area at 0.5 and at 0.25, timing every stage.
- **Draws:**
  - grid cells coloured by size and by region;
  - banded outlines;
  - grey polygons;
  - portals with the existing colours.

  All in `workspace.NVGN_GG`.
- **Collects** the same metrics from the current pipeline's held bake, clipped to the same area.

## Comparison (the deliverable)
A table, current vs global at 0.5 vs global at 0.25:
- cells;
- regions;
- polygons;
- portals by kind (target: shared and tile-border only, zero fitted);
- stage times;
- `Pipeline.auditPortals`-style checks;
- the random-pair path test (share of the path outside its own corridor);
- a projected full-map bake time.

Plus a drawing for Cocosulx to judge at the stairs and bridge. **Then he decides:** adopt, adjust, or drop.

## Known risks, stated up front
- The layer bookkeeping for a bridge over a street.
- Quadtree neighbour walks across tiles.
- The SVO's 1-stud conservatism around thin parts (handled by refining to exact tests there).
- Rubber-band corner cases at real corners.
- CDT on regions that are not planar.

Any of these may show up as a measured defect in the comparison, not a hidden one.

## Files
- **New:** `src/GlobalGrid.lua`, `src/RubberBand.lua`, `src/Solid.lua` (extracted), `tools/globalgrid_experiment.lua`.
- **Additive:** `src/SVO.lua` (`columnRuns`).
- **Reused unchanged:** `src/CDT.lua`, `src/Rings.lua`, `src/Portals.lua` (sharedLinks), `src/PathTest.lua`, `src/Agents.lua`.
- `src/LocalGrid.lua` only switches its helpers to `Solid.lua`, and must bake case6 bit-identically: fingerprint checked.

## Verification
1. The LocalGrid refactor to `Solid.lua` is proven neutral: a case6 bake keeps the saved fingerprint (loops 3164 / 15503 corners / hashes).
2. **Experiment runs:**
   - no open loops;
   - each region has exactly one outer rim;
   - no ring crosses itself;
   - every portal is a shared edge or a tile-border edge.
3. **Path test** on the experiment's mesh: 200 random pairs; found rate, and share of path length off its corridor.
4. **Cocosulx inspects** the drawing at the stairs and bridge.
