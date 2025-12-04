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

## Installation

```bash
dotnet add package MillSimSharp
```

## Quick Start

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


## Viewer（Sample App）

`MillSimSharp.Viewer` is provided as a sample application that demonstrates the library usage and offers a visual debugging surface. The core MillSimSharp library is the main project; the viewer is intended for demos and development/testing only.

How to use:

- The viewer looks for a G-code file at `src/MillSimSharp.Viewer/gcodes/test.nc` and, if present, will load and simulate it. Otherwise a demo scene is shown.

For more detailed developer information and debugging tips, see `docs/SDF.md` and `docs/DEVELOPER.md`.
```

## Requirements

- .NET 8.0 or .NET Standard 2.1

## License

MIT

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.