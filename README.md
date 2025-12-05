# Mill Sim Sharp

A voxel-based milling simulation dependency-free library for .NET — the core library provides a voxel-based representation, SDF generation, and mesh exporting utilities. The project contains a small viewer sample app for demonstrations.

## Overview

MillSimSharp is a library designed to simulate CNC milling operations using a voxel-based approach. It provides:

- **Voxel-based material representation** for accurate simulation
- **STL mesh export** using the Marching Cubes algorithm
- **G-code parser independence** - bring your own parser (gsGCode recommended for the viewer)
- **Flexible resolution** - adjust voxel size based on your needs
- **Simple API** for toolpath execution

**Default Configuration:**
- Voxel resolution: 0.5mm
- Work area: 100×100×100mm  

## Installation

The library is published to NuGet (GitHub Packages) via CI.   
you can install it with:

```bash
dotnet add package MillSimSharp
```

## Quick Start (Core library)

### Basic Toolpath Simulation

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

### Advanced: Using SDF for High-Quality Mesh

For large grids or high-quality mesh output, use `SDFGrid`:

```csharp
using MillSimSharp.Geometry;

// After voxel simulation...
// Generate SDF with narrow band optimization
var sdfGrid = SDFGrid.FromVoxelGrid(
    voxelGrid, 
    narrowBandWidth: 2,  // Optimize for speed
    useSparse: true      // Use sparse storage for large grids
);

// Generate high-quality mesh using Dual Contouring
var mesh = sdfGrid.GenerateMesh();

// Export mesh to STL
StlExporter.Export(mesh, "output_hq.stl");
```

## Viewer (sample app)

The `MillSimSharp.Viewer` project is a lightweight sample application and visualizer to demonstrate library usage. It is a demo tool and not intended to be a full GUI for production.
<img width="1282" height="752" alt="image" src="https://github.com/user-attachments/assets/54a82f2c-3519-4e7c-b18a-20732c760441" />

![Animation](https://github.com/user-attachments/assets/dfda64cc-c0e9-40aa-bfe1-dba323b3b4ef)

To run the viewer locally:

```powershell
dotnet run --project src\MillSimSharp.Viewer
```

If you have a G-code file at `src/MillSimSharp.Viewer/gcodes/test.nc`, the viewer will load and simulate it; otherwise it will run the demo scene.

## Build and Test (local)

To build and run tests locally

```powershell
dotnet build
dotnet test
```

## Requirements

- .NET 8.0 or .NET Standard 2.1

## License

MIT

## Contributing

Contributions are welcome!
