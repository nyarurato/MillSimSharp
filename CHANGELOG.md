# Changelog

All notable changes to this project are documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- Grid dimensions are rounded up to whole voxels. `VoxelGrid.Bounds` and `SDFGrid.Bounds` now report the effective voxelized extent (`Min + Dimensions * Resolution`), so non-divisible requested sizes may expand by less than one voxel per axis.

### Fixed

- Dual contouring used face diagonals as cube edges, degrading QEF vertex placement; the edge table now uses the true 12 cube edges
- `VoxelGrid.GetOccupiedVoxels()` reported the SVO power-of-two padding region as occupied voxels
- A normal `G1Move` after a 5-axis move cut with the default pose instead of the current orientation
- `ToolpathExecutor.Reset()` / `LoadCommands()` did not restore the initial constructor tool
- `SetVoxel` / `SetVoxelAtWorld` / `Clear` raised no `VoxelsChanged` event, leaving bound SDF grids stale

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

[0.2.0]: https://github.com/nyarurato/MillSimSharp/compare/0.2.0-beta...0.2.0
[0.2.0-beta]: https://github.com/nyarurato/MillSimSharp/compare/0.1.0...0.2.0-beta
[0.1.0]: https://github.com/nyarurato/MillSimSharp/releases/tag/0.1.0
