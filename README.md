# MillSimSharp

MillSimSharp is a milling simulation library for .NET focused on voxel-based and SDF (Signed Distance Field) workflows with support for 3-axis and 5-axis machining.  
It provides fast SDF generation, robust voxel-based simulation for milling operations, and high-quality mesh export.  
The repository also contains a lightweight viewer app for visualization and demos.

## Overview

MillSimSharp simulates CNC milling operations using both voxel-based representations and SDFs. It is designed for accurate material removal simulation and for producing high-quality meshes from the resulting geometry. It provides:

- **3-axis and 5-axis machining support** with tool orientation control
- **Voxel-based material representation** for accurate, conservative milling simulation (fast incremental operations)
- **Signed Distance Field (SDF) generation** (Fast Sweeping algorithm) for high-quality mesh conversion and fast distance queries
- **High-quality mesh export** using Dual Contouring and Marching Cubes
- **Flexible stock origin configuration** (center or corner-based)
- **G-code parser independence** - bring your own parser (gsGCode is used for the viewer demo)
- **Flexible resolution** - adjust voxel size based on your needs
- **Simple API** for toolpath execution

**Default Configuration:**
- Voxel resolution: 0.5mm
- Work area: 100×100×100mm  

## Features

### 3-Axis Machining
Standard CNC milling with XYZ motion and vertical tool orientation.

### 5-Axis Machining
Full 5-axis support with A, B, C rotational axes:
- **Tool orientation control** using Euler angles (A: X-axis, B: Y-axis, C: Z-axis rotation)
- **Automatic orientation interpolation** for smooth transitions
- **Right-hand coordinate system** with configurable rotation order (C → B → A)
- **Tool tip reference point** for all positioning

### Stock Configuration
Configure stock origin placement:
- **`StockOriginType.MinCorner`**: Origin at minimum corner (X-, Y-, Z-), stock extends in positive directions
- **`StockOriginType.Center`**: Origin at center, stock extends equally in all directions  

## Installation

The library is published to NuGet via CI.   
you can install it with:

```bash
dotnet add package MillSimSharp
```

## Quick Start (Core library)

### Basic 3-Axis Toolpath Simulation

```csharp
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;
using MillSimSharp.IO;
using System.Numerics;

// 1. Create a work area (100×100×100mm)
var workArea = BoundingBox.FromCenterAndSize(
    Vector3.Zero,
    new Vector3(100, 100, 100)
);

// 2. Initialize voxel grid with 1.0mm resolution
var voxelGrid = new VoxelGrid(workArea, resolution: 1.0f);

// 3. Define a tool (10mm diameter ball-end mill)
var tool = new EndMill(diameter: 10.0f, length: 50.0f, isBallEnd: true);

// 4. Create simulator and executor
var simulator = new CutterSimulator(voxelGrid);
var startPosition = new Vector3(0, 0, 50);
var executor = new ToolpathExecutor(simulator, tool, startPosition);

// 5. Execute toolpath commands
var commands = new List<IToolpathCommand>
{
    new G0Move(new Vector3(0, 0, 10)),      // Rapid move to start
    new G1Move(new Vector3(20, 0, 10), 100), // Linear cut
    new G1Move(new Vector3(20, 20, 10), 100) // Linear cut
};
executor.ExecuteCommands(commands);

// 6. Export to STL (direct from voxel grid)
StlExporter.Export(voxelGrid, "output.stl");
```

### 5-Axis Machining with Tool Orientation

```csharp
using MillSimSharp.Config;
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;
using MillSimSharp.IO;
using System.Numerics;

// 1. Configure stock with center origin
var stockConfig = new StockConfiguration
{
    WorkOrigin = new Vector3Data(0, 0, 0),
    WorkSize = new Vector3Data(100, 100, 50),
    OriginType = StockOriginType.Center  // Origin at center
};

// 2. Create SDF grid for high-quality 5-axis machining
var bbox = stockConfig.GetBoundingBox();
var sdfGrid = new SDFGrid(bbox, resolution: 0.5f, narrowBandWidth: 5);
var simulator = new SDFCutterSimulator(sdfGrid);

// 3. Define tool and create executor
var tool = new EndMill(diameter: 10.0f, length: 100.0f, isBallEnd: true);
var executor = new ToolpathExecutor(simulator, tool, Vector3.Zero);

// 4. Execute 5-axis toolpath with orientation
var commands = new List<IToolpathCommand>
{
    // Tilted cutting pass (30° on A-axis)
    new G0Move5Axis(new Vector3(-10, 0, 10), new ToolOrientation(a_deg: 30)),
    new G1Move5Axis(new Vector3(50, 0, 10), new ToolOrientation(a_deg: 30), feedRate: 200f),
    
    // Cone-shaped path with varying orientation
    // (Tool shaft passes through fixed point while tip moves in circle)
};
executor.ExecuteCommands(commands);

// 5. Generate mesh and export
var mesh = MeshConverter.ConvertToMeshFromSDF(sdfGrid);
StlExporter.Export(mesh, "five_axis_output.stl");
```

### SDF-Native Workflow (Direct SDF Manipulation)

For SDF-based workflows without voxels, use `SDFGrid` directly:

```csharp
using MillSimSharp.Geometry;
using MillSimSharp.IO;
using System.Numerics;

// 1. Create an SDF grid directly (all material initially)
var workArea = BoundingBox.FromCenterAndSize(
    Vector3.Zero,
    new Vector3(100, 100, 100)
);
var sdfGrid = new SDFGrid(workArea, resolution: 0.5f, narrowBandWidth: 10);

// 2. Remove material using SDF operations
sdfGrid.RemoveSphere(new Vector3(0, 0, 0), radius: 15.0f);
sdfGrid.RemoveSphere(new Vector3(20, 0, 0), radius: 10.0f);

// 3. Generate high-quality mesh using Dual Contouring
var mesh = sdfGrid.GenerateMesh();

// 4. Export to STL
StlExporter.Export(mesh, "output_sdf.stl");
```

### Converting Voxel Simulation to SDF

You can also convert a voxel grid (after simulation) to an SDF for mesh export:

```csharp
using MillSimSharp.Geometry;

// After voxel simulation (see first example)...
var sdfGrid = SDFGrid.FromVoxelGrid(
    voxelGrid, 
    narrowBandWidth: 2,
    useSparse: true
);

var mesh = sdfGrid.GenerateMesh();
StlExporter.Export(mesh, "output_from_voxel.stl");
```

## Viewer and Samples (Repository Only)

> **Note:** The viewer app and sample projects are included in the **source repository** but are **not part of the NuGet package**. The NuGet package contains only the core `MillSimSharp` library.

The `MillSimSharp.Viewer` project is a lightweight sample application and visualizer to demonstrate library usage. It is a demo tool and not intended to be a full GUI for production.

### Sample Projects

The repository includes several sample projects in the `samples/` directory:

1. **01-BasicToolpath**: Simple 3-axis toolpath execution
2. **02-SDFMeshGeneration**: SDF-based mesh generation and export
3. **03-CustomShapes**: Custom shape creation using SDF operations
4. **04-StepByStep**: Step-by-step toolpath execution with intermediate results
5. **05-FiveAxisMachining**: 5-axis machining with tool orientation control

To run a sample:

```powershell
cd samples/05-FiveAxisMachining
dotnet run
```

### Viewer Application

<img width="1282" height="752" alt="image" src="https://github.com/user-attachments/assets/54a82f2c-3519-4e7c-b18a-20732c760441" />

![Animation](https://github.com/user-attachments/assets/dfda64cc-c0e9-40aa-bfe1-dba323b3b4ef)

To run the viewer locally:

```powershell
dotnet run --project src\MillSimSharp.Viewer
```

If you have a G-code file at `src/MillSimSharp.Viewer/gcodes/test.nc`, the viewer will load and simulate it; otherwise it will run the demo scene.

## Build and Test (Repository)

> **Note:** This section applies to the source repository, not the NuGet package.

To build and run tests locally:

```powershell
dotnet build
dotnet test
```

## Requirements

- .NET 8.0 or .NET Standard 2.1 or higher

## API Documentation

### 5-Axis Tool Orientation

The `ToolOrientation` struct defines tool rotation using Euler angles:

- **A-axis**: Rotation around X-axis (degrees)
- **B-axis**: Rotation around Y-axis (degrees)
- **C-axis**: Rotation around Z-axis (degrees)
- **Rotation order**: C → B → A (ZYX Euler angles)
- **Default tool direction**: (0, 0, -1) pointing downward along Z-axis

```csharp
// Create orientation with 30° tilt on A-axis
var orientation = new ToolOrientation(a_deg: 30, b_deg: 0, c_deg: 0);

// Get tool direction vector
Vector3 direction = orientation.GetToolDirection();
```

### Stock Origin Configuration

Configure where the work origin (0,0,0) is located on the stock:

```csharp
var stockConfig = new StockConfiguration
{
    WorkOrigin = new Vector3Data(0, 0, 0),
    WorkSize = new Vector3Data(100, 100, 50),
    OriginType = StockOriginType.Center  // or StockOriginType.MinCorner
};
```

- **`MinCorner`**: Origin at (X-, Y-, Z-) corner, stock extends in positive directions (legacy behavior)
- **`Center`**: Origin at center of stock, extends equally in all directions

## Performance Optimization

For 5-axis machining, the library automatically optimizes interpolation step size to balance quality and performance:
- **3-axis**: One step per voxel resolution
- **5-axis**: One step per 2.5× voxel resolution (configurable)

This provides smooth orientation changes while maintaining reasonable computation times.

## License

MIT

## Contributing

Contributions are welcome!
