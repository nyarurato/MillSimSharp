# MillSimSharp Samples

This directory contains sample projects demonstrating how to use the MillSimSharp library.

## Available Samples

### 01-BasicToolpath
A basic sample of toolpath simulation using VoxelGrid.

**Features**:
- Creating a VoxelGrid
- Defining an EndMill tool
- Executing a simple toolpath (square pocket)
- Exporting to STL

**Output**: `output/basic_toolpath.stl`

**How to Run**:
```bash
cd 01-BasicToolpath
dotnet run
```

---

### 02-SDFMeshGeneration
Sample demonstrating high-quality mesh generation using Signed Distance Fields (SDF).

**Features**:
- Directly operating on an SDFGrid
- Executing toolpaths (circular pockets) with `SDFCutterSimulator`
- Generating high-quality clean meshes using Dual Contouring
- Exporting to STL

**Output**: `output/sdf_mesh.stl`

**How to Run**:
```bash
cd 02-SDFMeshGeneration
dotnet run
```

**Note**: You can observe the superior surface smoothness compared to direct voxel exports.

---

### 03-CustomShapes
Advanced sample demonstrating custom shape machining with multiple tools.

**Features**:
- Using multiple tools (Roughing and Finishing)
- Complex toolpaths including a spherical spiral finishing pass
- Simulation using `SDFGrid` and `G0`/`G1` commands
- Exporting the final result to STL

**Output**: `output/custom_shape.stl`

**How to Run**:
```bash
cd 03-CustomShapes
dotnet run
```

**Key Concept**: Demonstrates a realistic multi-stage machining workflow.

---

### 04-StepByStep
Sample demonstrating incremental step-by-step execution.

**Features**:
- Executing toolpaths incrementally using `ToolpathExecutor`
- Saving intermediate simulation results as STL files
- Using `MeshConverter` to generate meshes from SDF at each step

**Output**: `output/step_01.stl`, `output/step_02.stl`, ..., `output/step_final.stl`

**How to Run**:
```bash
cd 04-StepByStep
dotnet run
```

**Tip**: Load the generated STL files sequentially in a 3D viewer to animate the machining process.

---

## Viewing Outputs

Each sample generates STL files in an `output/` folder. You can view them with any standard 3D viewer.


---

## Requirements

- .NET 8.0 SDK
- MillSimSharp library
