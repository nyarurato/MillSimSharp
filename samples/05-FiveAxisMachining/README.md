# 5-Axis Machining Example (SDF-based)

This sample demonstrates the 5-axis machining capabilities of MillSimSharp using **SDF (Signed Distance Field)** for high-quality mesh generation.

## Coordinate System Reference

### XYZ Coordinates (Linear Axes)

**All XYZ coordinates represent the tool tip (cutting edge) position.**

- **End Mill**: Center of the bottom cutting edge
- **Ball End Mill**: Center of the spherical tip (lowest point of the sphere)

```
      Spindle
         |
         | ← Tool shank
         |
    +----+----+
    |  Tool   |  ← Length
    |  Body   |
    +----+----+
         |
      ---+---  ← Cutting edge
         |
         * ← Reference point (X, Y, Z) = Tool Tip
```

### Rotation Angles (A, B, C Axes)

**All angles are defined using the right-hand coordinate system.**

#### Default Orientation (A=0, B=0, C=0)
```
     Z↑
      |
      |    / Y
      |  /
      |/_____ X
      
Tool direction: (0, 0, -1)  ← Negative Z (pointing down)
```

#### A-axis (Rotation around X)
- Positive rotation: Y-axis toward Z-axis
- Example: `A=+30°` tilts the tool in the YZ plane

#### B-axis (Rotation around Y)
- Positive rotation: Z-axis toward X-axis
- Example: `B=+30°` tilts the tool in the ZX plane

#### C-axis (Rotation around Z)
- Positive rotation: X-axis toward Y-axis
- Example: `C=+45°` rotates the tool around vertical axis

#### Rotation Order
Rotations are applied in **C → B → A** order (ZYX Euler angles).

For `ToolOrientation(30, 20, 45)`:
1. Rotate 45° around Z-axis (C)
2. Rotate 20° around Y-axis (B)
3. Rotate 30° around X-axis (A)

### Machine Configuration Independence

The `ToolOrientation` class represents **machine-independent tool direction**.

Real 5-axis machines have various configurations:
- **Head-tilt type**: Spindle rotates
- **Table-tilt type**: Workpiece rotates
- **Mixed type**: Both rotate

The specified orientation defines the final tool direction vector. Conversion to actual machine axes (inverse kinematics) must be done by a post-processor.

## Overview

5-axis machining adds two rotational axes (typically A and C) to the traditional 3-axis (X, Y, Z) setup, allowing the tool to approach the workpiece from virtually any angle.

## Features Demonstrated

1. **Tilted Cutting Pass**: Simple linear cutting with a tilted tool orientation
2. **5-Axis Spiral**: Complex toolpath with continuously changing position and orientation
3. **Curved Surface Machining**: Simulating surface following with normal-controlled tool orientation

## Key Classes

### ToolOrientation
Represents the rotational state of the tool:
- `A`: Rotation around X-axis (degrees)
- `B`: Rotation around Y-axis (degrees)  
- `C`: Rotation around Z-axis (degrees)

### 5-Axis Commands
- `G0Move5Axis`: Rapid positioning with orientation
- `G1Move5Axis`: Linear interpolation with orientation

### CoordinateTransform Utilities
- `TransformPoint()`: Transform points with rotation
- `Get5AxisTransform()`: Get transformation matrix
- `ToolTipToSpindlePosition()`: Convert tool tip to spindle position
- `InterpolateOrientation()`: Smooth orientation transitions

## Configuration

The sample uses a 5-axis machine configuration defined in `configs/five_axis_machine.xml`:
- Linear axes: X (0-500mm), Y (0-400mm), Z (-200-300mm)
- Rotary axes: A (-120° to 120°), C (-360° to 360°)

## Running the Sample

```bash
cd samples/05-FiveAxisMachining
dotnet run
```

The program will:
1. Create a 100x100x50mm stock represented as an SDF
2. Execute three different 5-axis toolpath examples
3. Generate a high-quality mesh from the SDF using Dual Contouring
4. Export the result to `five_axis_result.stl`

**Advantages of SDF-based simulation:**
- High-quality smooth surfaces
- Better representation of curved tool paths
- More accurate material removal visualization
- Clean mesh output suitable for rendering

## Technical Details

### SDF-based Implementation

This sample uses `SDFGrid` and `SDFCutterSimulator` instead of voxel-based simulation:

```csharp
// Create SDF grid with narrow band for efficient computation
var sdfGrid = new SDFGrid(bbox, resolution: 0.5f, narrowBandWidth: 5);
var simulator = new SDFCutterSimulator(sdfGrid);
var executor = new ToolpathExecutor(simulator, tool, Vector3.Zero);

// Execute toolpath
executor.ExecuteCommands(commands);

// Generate high-quality mesh
var mesh = MeshConverter.ConvertToMeshFromSDF(sdfGrid);
StlExporter.Export(mesh, "output.stl");
```

The SDF approach provides smoother surfaces compared to direct voxel export, especially for curved toolpaths and tilted cutting operations.

### Tool Tip vs Spindle Position

**Important**: All position commands (`G0Move5Axis`, `G1Move5Axis`) specify the **tool tip position**, not the spindle position.

```
Spindle Position (calculated)
         ↓
    [=======]  ← Spindle/Tool holder
         |
    [---+---]  ← Tool body (Length)
         |
         |
         * ← Tool Tip Position (specified in commands)
```

To get the spindle position from tool tip:
```csharp
Vector3 spindlePos = CoordinateTransform.ToolTipToSpindlePosition(
    toolTipPosition,  // Specified in command
    orientation,      // Tool orientation
    tool.Length       // Tool length
);
```

### Tool Orientation

The tool orientation uses Euler angles in the order Z-Y-X (C-B-A):
1. Rotate around Z-axis (C)
2. Rotate around Y-axis (B)
3. Rotate around X-axis (A)

The default tool direction is along the negative Z-axis (0, 0, -1).

### Interpolation

The `G1Move5Axis` command interpolates both position and orientation to create smooth transitions. The `interpolationSteps` parameter controls the granularity.

### Coordinate Systems

- **Work Coordinates**: The coordinate system where parts are designed
- **Machine Coordinates**: The physical machine coordinate system
- **Tool Coordinates**: Coordinate system aligned with the tool axis

## Advanced Usage

### Custom Surface Machining

```csharp
// Define surface point and normal
var surfacePoint = new Vector3(x, y, z);
var surfaceNormal = CalculateSurfaceNormal(surfacePoint);

// Convert normal to tool orientation
var orientation = NormalToOrientation(surfaceNormal);

// Create cutting move
var command = new G1Move5Axis(surfacePoint, orientation, feedRate: 150f);
```

### Collision Avoidance

For collision avoidance, calculate the spindle position:

```csharp
var spindlePos = CoordinateTransform.ToolTipToSpindlePosition(
    toolTipPosition, orientation, tool.Length);
    
// Check if spindle position collides with fixtures
if (CheckCollision(spindlePos))
{
    // Adjust orientation or retract
}
```

## Limitations

1. The current implementation simulates material removal at the tool tip position
2. Full tool geometry rotation and swept volume calculation is not yet implemented
3. Collision detection between tool holder and workpiece is not included

## Next Steps

- Implement full rotary axis kinematics
- Add tool holder geometry and collision detection
- Support for simultaneous 5-axis interpolation
- Automatic tool axis optimization for surface machining
