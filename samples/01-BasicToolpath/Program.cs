using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;
using MillSimSharp.IO;
using System.Numerics;

Console.WriteLine("=== MillSimSharp Sample: Basic Toolpath ===\n");

// Create output directory
var outputDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "output");
Directory.CreateDirectory(outputDir);

// 1. Create a work area (100×100×50mm stock)
Console.WriteLine("Creating voxel grid (100×100×50mm, 1mm resolution)...");
var workArea = BoundingBox.FromCenterAndSize(
    new Vector3(0, 0, 25),  // Center at Z=25 so top is at Z=50
    new Vector3(100, 100, 50)
);
var voxelGrid = new VoxelGrid(workArea, resolution: 1.0f);
Console.WriteLine($"  Grid created: {voxelGrid.Dimensions.X}×{voxelGrid.Dimensions.Y}×{voxelGrid.Dimensions.Z} voxels");
Console.WriteLine($"  Initial voxels: {voxelGrid.CountMaterialVoxels():N0}\n");

// 2. Define a tool (10mm diameter ball-end mill)
Console.WriteLine("Setting up tool and simulator...");
var tool = new EndMill(diameter: 10.0f, length: 50.0f, isBallEnd: true);
Console.WriteLine($"  Tool: {tool.Diameter}mm diameter, {(tool.Type == MillSimSharp.Simulation.ToolType.Ball ? "ball-end" : "flat-end")} mill\n");

// 3. Create simulator and executor
var simulator = new CutterSimulator(voxelGrid);
var startPosition = new Vector3(0, 0, 60);  // Start above the stock
var executor = new ToolpathExecutor(simulator, tool, startPosition);

// 4. Create a simple toolpath - cut a square pocket
Console.WriteLine("Executing toolpath (square pocket)...");
var commands = new List<IToolpathCommand>
{
    // Move to start position
    new G0Move(new Vector3(-20, -20, 45)),
    
    // Cut outer square at Z=45
    new G1Move(new Vector3(20, -20, 45), 500),
    new G1Move(new Vector3(20, 20, 45), 500),
    new G1Move(new Vector3(-20, 20, 45), 500),
    new G1Move(new Vector3(-20, -20, 45), 500),
    
    // Move down to Z=40
    new G1Move(new Vector3(-20, -20, 40), 200),
    
    // Cut inner square at Z=40
    new G1Move(new Vector3(-10, -10, 40), 500),
    new G1Move(new Vector3(10, -10, 40), 500),
    new G1Move(new Vector3(10, 10, 40), 500),
    new G1Move(new Vector3(-10, 10, 40), 500),
    new G1Move(new Vector3(-10, -10, 40), 500),
    
    // Retract
    new G0Move(new Vector3(0, 0, 60))
};

var sw = System.Diagnostics.Stopwatch.StartNew();
executor.ExecuteCommands(commands);
sw.Stop();

Console.WriteLine($"  Executed {commands.Count} commands in {sw.ElapsedMilliseconds}ms");
Console.WriteLine($"  Remaining voxels: {voxelGrid.CountMaterialVoxels():N0}\n");

// 5. Export to STL
Console.WriteLine("Exporting to STL...");
var outputPath = Path.Combine(outputDir, "basic_toolpath.stl");
sw.Restart();
StlExporter.Export(voxelGrid, outputPath);
sw.Stop();

Console.WriteLine($"  Exported to: {outputPath}");
Console.WriteLine($"  Export time: {sw.ElapsedMilliseconds}ms\n");

Console.WriteLine("=== Sample completed successfully! ===");
Console.WriteLine($"Output file: {outputPath}");
