using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;
using MillSimSharp.IO;
using System.Numerics;

Console.WriteLine("=== MillSimSharp Sample: Custom Shapes ===\n");

// Create output directory
var outputDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "output");
Directory.CreateDirectory(outputDir);

// 1. Create a work area (120×120×80mm stock)
Console.WriteLine("Creating voxel grid (120×120×80mm, 1mm resolution)...");
var workArea = BoundingBox.FromCenterAndSize(
    new Vector3(0, 0, 40),
    new Vector3(120, 120, 80)
);
var voxelGrid = new VoxelGrid(workArea, resolution: 1.0f);
Console.WriteLine($"  Grid created: {voxelGrid.Dimensions.X}×{voxelGrid.Dimensions.Y}×{voxelGrid.Dimensions.Z} voxels\n");

// 2. Define tools
Console.WriteLine("Setting up tools...");
var roughingTool = new EndMill(diameter: 12.0f, length: 60.0f, isBallEnd: false);
var finishingTool = new EndMill(diameter: 6.0f, length: 50.0f, isBallEnd: true);
Console.WriteLine($"  Roughing tool: {roughingTool.Diameter}mm flat-end mill");
Console.WriteLine($"  Finishing tool: {finishingTool.Diameter}mm ball-end mill\n");

// 3. Roughing pass - create circular pocket with layers
Console.WriteLine("Executing roughing pass (circular pocket, 3 layers)...");
var simulator = new CutterSimulator(voxelGrid);
var executor = new ToolpathExecutor(simulator, roughingTool, new Vector3(0, 0, 90));

var roughingCommands = new List<IToolpathCommand>();
var sw = System.Diagnostics.Stopwatch.StartNew();

// Create circular pockets at different Z heights
for (int layer = 0; layer < 3; layer++)
{
    float z = 70 - layer * 10;  // Z=70, 60, 50
    float radius = 30 - layer * 5;  // Decreasing radius
    
    // Move to start of circle
    roughingCommands.Add(new G0Move(new Vector3(radius, 0, z)));
    
    // Cut circle (approximate with line segments)
    int segments = 36;
    for (int i = 0; i <= segments; i++)
    {
        float angle = (float)(i * 2 * Math.PI / segments);
        float x = radius * MathF.Cos(angle);
        float y = radius * MathF.Sin(angle);
        roughingCommands.Add(new G1Move(new Vector3(x, y, z), 800));
    }
}

executor.ExecuteCommands(roughingCommands);
sw.Stop();
Console.WriteLine($"  Roughing completed in {sw.ElapsedMilliseconds}ms");
Console.WriteLine($"  Commands executed: {roughingCommands.Count}");
Console.WriteLine($"  Remaining voxels: {voxelGrid.CountMaterialVoxels():N0}\n");

// 4. Finishing pass - smooth the walls
Console.WriteLine("Executing finishing pass (spiral path)...");
executor = new ToolpathExecutor(simulator, finishingTool, new Vector3(0, 0, 90));

var finishingCommands = new List<IToolpathCommand>();
sw.Restart();

// Spiral finishing path
int spiralTurns = 5;
int pointsPerTurn = 20;
float startRadius = 25;
float endRadius = 15;
float startZ = 65;
float endZ = 45;

finishingCommands.Add(new G0Move(new Vector3(startRadius, 0, startZ)));

for (int i = 0; i <= spiralTurns * pointsPerTurn; i++)
{
    float t = (float)i / (spiralTurns * pointsPerTurn);
    float angle = (float)(i * 2 * Math.PI / pointsPerTurn);
    float radius = startRadius + (endRadius - startRadius) * t;
    float z = startZ + (endZ - startZ) * t;
    
    float x = radius * MathF.Cos(angle);
    float y = radius * MathF.Sin(angle);
    
    finishingCommands.Add(new G1Move(new Vector3(x, y, z), 600));
}

executor.ExecuteCommands(finishingCommands);
sw.Stop();
Console.WriteLine($"  Finishing completed in {sw.ElapsedMilliseconds}ms");
Console.WriteLine($"  Commands executed: {finishingCommands.Count}");
Console.WriteLine($"  Final voxels: {voxelGrid.CountMaterialVoxels():N0}\n");

// 5. Export to STL
Console.WriteLine("Exporting to STL...");
var outputPath = Path.Combine(outputDir, "custom_shape.stl");
sw.Restart();
StlExporter.Export(voxelGrid, outputPath);
sw.Stop();

Console.WriteLine($"  Exported to: {outputPath}");
Console.WriteLine($"  Export time: {sw.ElapsedMilliseconds}ms\n");

Console.WriteLine("=== Sample completed successfully! ===");
Console.WriteLine($"Output file: {outputPath}");
Console.WriteLine($"\nTotal commands executed: {roughingCommands.Count + finishingCommands.Count}");
