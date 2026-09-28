# Changelog

All notable changes to this project are documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `ToolOrientation.FromAxisTowardSpindle(axis)` for IJK-style pose input (shortest-rotation orientation; any finite non-zero axis is accepted).
- `SDFGrid.SyncFromVoxelGrid()` rebuilds the field from its source voxel grid (also after `UnbindFromVoxelGrid`).
- `ChunkedVoxelMeshBuilder` per-chunk API (`UpdateChunks`, `GetChunkMesh`, `GetChunkCoordinates`) for incremental rendering.

### Changed

- **Zero-size bounds are rejected** by `VoxelGrid` and `SDFGrid` (`ArgumentException`); an unconfigured `MillSimulation` stock now fails fast instead of building a degenerate grid.
- **STL export writes geometric facet normals** instead of copied vertex normals; degenerate triangles get a zero normal.
- **`CoordinateTransform.InterpolateOrientation` uses shortest-rotation slerp**, matching the simulator pose path (the midpoint of 350° → 10° is now 0°, not 180°). The returned Euler values may be an equivalent representation with different angles (e.g. 350° → -10°).
- Voxel mesh output is deterministic: the same grid always yields identical vertex arrays and STL bytes (the vertex order may differ from previous releases).
- Documented material-state and lifecycle contracts: `FromVoxelGrid` = one-time snapshot, `BindToVoxelGrid` = future changes only, `UnbindFromVoxelGrid` keeps SDF-native edits; `MillSimulation.Reset()` replaces the grid / simulator / executor (tool kept, subscriptions not migrated); executor `Reset` / `LoadCommands` do not restore stock.
- Documented accuracy and collision scope in `docs/Accuracy.md`: voxel-center sampling, pose-sampling error, narrow band clamping, and single-pose, cutting-edge-only collision checks (no holder / fixture / target part).
- CI builds now use `-warnaserror`, matching the documented zero-warning policy.

### Fixed

- `ToolpathExecutor.ExecuteNextSteps` no longer skips a command that throws: the cursor advances only after success and a failed last command is not reported as completed.
- Viewer G-code parser: inconsistent I/J arcs are ignored without moving the tool, valid arcs far from the origin are no longer misjudged (the radius check uses double precision), and the last chord lands exactly on the commanded end (previously the tool position and the emitted arc could jump apart).
- Viewer G-code parser: a block with an unsupported G code (G28, canned cycles, work offsets, G18/G19, ...) no longer runs as the previous modal motion (e.g. `G28 X0` after `G1 X10` cut back to the origin); such blocks emit no motion and keep the position, while their modal unit / distance words still apply.
- `VoxelGrid` rejects work areas whose voxel count exceeds `int.MaxValue` at construction (previously the public counts overflowed, e.g. 200 mm at 0.1 mm resolution); grids up to the `int` boundary keep working.
- Viewer step mode requests the initial step-state mesh when it is enabled, and mesh regeneration requests that arrive while a build is running are queued and re-run afterwards, so consecutive steps or mode switches cannot leave a stale display.
- `ToolOrientation.FromAxisTowardSpindle` now returns the actual shortest rotation (the previous construction added an unnecessary roll, e.g. 98.4° instead of 90°), only the exact `-Z` direction is pinned to the 180° X flip (near-antipodal axes keep their direction), and tool axes are normalized robustly: tiny (but non-zero) and very large finite axes are accepted by `FromAxisTowardSpindle` and `ToolCollisionDetector.IntersectsMaterial` as documented.

## [0.2.1] - 2026-09-22

### Changed

- Grid dimensions are rounded up to whole voxels. `VoxelGrid.Bounds` and `SDFGrid.Bounds` now report the effective voxelized extent (`Min + Dimensions * Resolution`), so non-divisible requested sizes may expand by less than one voxel per axis.
- Adaptive pose sampling now derives a conservative rotation radius from `IToolGeometry.LocalBounds`, so rotation-only moves of flat, bull-nose and tapered tools are subdivided by the chord-error criterion (previously only ball tools were refined).
- `SDFGrid.BindToVoxelGrid()` / `UpdateRegionFromVoxelGrid()` now throw `ArgumentException` when the supplied voxel grid has mismatched dimensions, resolution or bounds.
- Public numeric inputs are validated: `SimulationSettings` step/chord values must be finite and positive and `MinimumSteps` must be at least 1, `ToolpathExecutor.StepSize` must be at least 1, tool and geometry dimensions must be finite, `BoundingBox` coordinates must be finite and grid resolutions must be finite.
- `ToolCollisionDetector.IntersectsMaterial(...)` normalizes the tool axis and throws `ArgumentException` for zero or non-finite axes.

### Fixed

- Dual contouring used face diagonals as cube edges, degrading QEF vertex placement; the edge table now uses the true 12 cube edges
- `VoxelGrid.GetOccupiedVoxels()` reported the SVO power-of-two padding region as occupied voxels
- A normal `G1Move` after a 5-axis move cut with the default pose instead of the current orientation
- `ToolpathExecutor.Reset()` / `LoadCommands()` did not restore the initial constructor tool
- `SetVoxel` / `SetVoxelAtWorld` / `Clear` raised no `VoxelsChanged` event, leaving bound SDF grids stale
- Bull-nose geometry bounds now contain the full toroidal corner even when the cutting length is shorter than the corner diameter
- Mesh vertex comparers now satisfy the equality/hash contract (near-equal vertices could previously fail to merge)
- G-code R-format arcs: positive R selects the minor arc and negative R the major arc; an arc without I/J/R or with an impossible radius is ignored instead of being emitted as a linear move
- `VoxelGrid.RemoveVoxelsInSphere` / `RemoveVoxelsInCylinder` now use the strict `signedDistance < 0` convention (a voxel center exactly on the surface is preserved)
- `VoxelGrid.RemoveVoxelsInCylinder(start, start, radius, flatEnds: true)` no longer removes a sphere for a zero-length cylinder (matches the SDF backend: a flat cylinder with no length has no volume)
- `ToolPoseMath.GetWorldBounds()` now covers geometries whose local bounds extend below the physical tip (`LocalBounds.Min.Z < 0`)
- `SDFGrid.RemoveFiniteCylinder` now uses the exact capped-cylinder distance (the previous `max()` of half-space distances under-reported distances outside the end-face corners)
- `TaperedEndMillGeometry` now returns the exact capped-frustum signed distance (the previous lateral approximation was off by up to ~1.1 mm near the corners)

## [0.2.0] - 2026-09-21

### Added

- Tool geometry abstraction (`IToolGeometry`) and new tool types: bull-nose (`BullNoseEndMill`) and tapered (`TaperEndMill`)
- Collision / gouge detection (`ToolCollisionDetector`), tool changes (`ToolChange`) and cancellable execution with progress and estimated time
- `ToolPose` and quaternion orientation math (`Slerp`, `FromQuaternion`, `AngularDistanceDegrees`)
- Adaptive pose sampling via `SimulationSettings` (`MaxLinearStep`, `MaxAngularStep`, `MinimumSteps`, `MaxChordError`)
- `ChunkedVoxelMeshBuilder` for incremental voxel remeshing, and OBJ / PLY exporters
- Viewer: example G-code parser, middle-drag camera pan and an end mill display model (`M`)

### Changed

- **Breaking**: SDF core reworked to the standard convention (negative = material, positive = air, values in mm); exact EDT replaces `FastSweepingSDF` / `OctreeSDF`
- **Breaking**: all toolpath positions now refer to the physical tool tip; ball compensation via `Tool.BallCenterOffsetFromTip`
- Dual contouring quality: winding from the field sign, regularized mass-point QEF, zero-level-set projection and CSG junction repair
- Deterministic material removal and parallelised SDF carving; sample 05 is ~8× faster with unchanged output
- Samples and docs updated to the current API; CI enforces formatting and zero-warning builds

### Fixed

- Ball end mill cutting center on tilted moves
- SDF mesh shells: holes at grid borders, inverted winding and normals
- QEF and CSG junction artifacts on cross cuts

### Removed

- Unused Marching Cubes table, core-library G-code parser and the unused viewer voxel renderer

## [0.2.0-beta] - 2025-12-22

### Added

- 5-axis machining: `ToolOrientation` (A/B/C), `G0Move5Axis` / `G1Move5Axis` and orientation interpolation
- Five-axis sample and machine configuration

## [0.1.0] - 2025-12-05

- Initial public release: voxel-based 3-axis simulation, SDF mesh conversion, STL export, viewer and samples.

[Unreleased]: https://github.com/nyarurato/MillSimSharp/compare/0.2.1...HEAD
[0.2.0]: https://github.com/nyarurato/MillSimSharp/compare/0.2.0-beta...0.2.0
[0.2.1]: https://github.com/nyarurato/MillSimSharp/compare/0.2.0...0.2.1
[0.2.0-beta]: https://github.com/nyarurato/MillSimSharp/compare/0.1.0...0.2.0-beta
[0.1.0]: https://github.com/nyarurato/MillSimSharp/releases/tag/0.1.0
