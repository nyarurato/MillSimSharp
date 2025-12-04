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

```csharp
using MillSimSharp.Geometry;

// Create a work area (100×100×100mm)
var workArea = BoundingBox.FromCenterAndSize(
    Vector3.Zero,
    new Vector3(100, 100, 100)
);

// Initialize voxel grid with 0.5mm resolution
var grid = new VoxelGrid(workArea, resolution: 0.5);

// Simulate tool cutting (sphere removal)
var toolPosition = new Vector3(0, 0, 10);
var toolRadius = 3.0; // 6mm diameter tool
grid.RemoveVoxelsInSphere(toolPosition, toolRadius);

// Simulate linear tool movement (cylinder removal)
var start = new Vector3(-10, 0, 5);
var end = new Vector3(10, 0, 5);
grid.RemoveVoxelsInCylinder(start, end, toolRadius);

// Export to STL
StlExporter.Export(grid, "output.stl");
```

## Viewer (sample app)

The `MillSimSharp.Viewer` project is a lightweight sample application and visualizer to demonstrate library usage. It is a demo tool and not intended to be a full GUI for production.

To run the viewer locally:

```powershell
dotnet run --project src\MillSimSharp.Viewer
```

If you have a G-code file at `src/MillSimSharp.Viewer/gcodes/test.nc`, the viewer will load and simulate it; otherwise it will run the demo scene.

For more detailed developer information and debugging tips, see `docs/SDF.md` and `docs/DEVELOPER.md`.

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
- Open an issue to discuss larger feature changes before coding.

Please include unit tests for any behavioral changes.