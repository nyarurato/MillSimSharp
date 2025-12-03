# MillSimSharp

A voxel-based milling simulation library for .NET.

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
- Memory usage: ~1MB for default settings

## Installation

```bash
dotnet add package MillSimSharp
```

## Quick Start

```csharp
using Mill SimSharp.Geometry;

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

## Viewer

The `MillSimSharp.Viewer` application automatically looks for a G-code file under `src/MillSimSharp.Viewer/gcodes/test.nc` at startup and, if present, parses it and simulates the toolpath on the voxel grid. Otherwise the viewer falls back to the demo scene.
```

## Requirements

- .NET 8.0 or .NET Standard 2.1

## License

MIT

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.