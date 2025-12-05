using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;
using MillSimSharp.IO;
using System.Numerics;

Console.WriteLine("=== MillSimSharp Sample: SDF Mesh Generation ===\n");

// Create output directory
var outputDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "output");
Directory.CreateDirectory(outputDir);

// 1. Create a work area and voxel grid (80×80×60mm stock)
Console.WriteLine("Creating voxel grid (80×80×60mm, 0.5mm resolution)...");
var workArea = BoundingBox.FromCenterAndSize(
    new Vector3(0, 0, 30),
    new Vector3(80, 80, 60)
);
var voxelGrid = new VoxelGrid(workArea, resolution: 0.5f);
Console.WriteLine($"  Grid created: {voxelGrid.Dimensions.X}×{voxelGrid.Dimensions.Y}×{voxelGrid.Dimensions.Z} voxels");
Console.WriteLine($"  Initial voxels: {voxelGrid.CountMaterialVoxels():N0}\n");

// 2. Define a tool (10mm diameter ball-end mill)
Console.WriteLine("Setting up tool and simulator...");
var tool = new EndMill(diameter: 10.0f, length: 50.0f, isBallEnd: true);
Console.WriteLine($"  Tool: {tool.Diameter}mm diameter, {(tool.Type == MillSimSharp.Simulation.ToolType.Ball ? "ball-end" : "flat-end")} mill\n");

// 3. Create simulator and executor
var simulator = new CutterSimulator(voxelGrid);
var startPosition = new Vector3(0, 0, 65);  // Start above the stock
var executor = new ToolpathExecutor(simulator, tool, startPosition);

// 4. Create a complex toolpath - circular pockets at different depths
Console.WriteLine("Executing toolpath (circular pockets)...");
var commands = new List<IToolpathCommand>();
var sw = System.Diagnostics.Stopwatch.StartNew();

// Create circular pockets at different Z heights
for (int layer = 0; layer < 3; layer++)
{
    float z = 55 - layer * 10;  // Z=55, 45, 35
    float radius = 25 - layer * 5;  // Decreasing radius
    
    // Move to start of circle
    commands.Add(new G0Move(new Vector3(radius, 0, z)));
    
    // Cut circle (approximate with line segments)
    int segments = 36;
    for (int i = 0; i <= segments; i++)
    {
        float angle = (float)(i * 2 * Math.PI / segments);
        float x = radius * MathF.Cos(angle);
        float y = radius * MathF.Sin(angle);
        commands.Add(new G1Move(new Vector3(x, y, z), 500));
    }
}

// Add some crossing cuts
commands.Add(new G0Move(new Vector3(-25, 0, 50)));
commands.Add(new G1Move(new Vector3(25, 0, 50), 500));
commands.Add(new G0Move(new Vector3(0, -25, 50)));
commands.Add(new G1Move(new Vector3(0, 25, 50), 500));

// Retract
commands.Add(new G0Move(new Vector3(0, 0, 65)));

executor.ExecuteCommands(commands);
sw.Stop();

Console.WriteLine($"  Executed {commands.Count} commands in {sw.ElapsedMilliseconds}ms");
Console.WriteLine($"  Remaining voxels: {voxelGrid.CountMaterialVoxels():N0}\n");

// 5. Generate SDF with narrow band optimization
Console.WriteLine("Generating SDF (narrow band width: 2, sparse storage)...");
sw.Restart();
var sdfGrid = SDFGrid.FromVoxelGrid(
    voxelGrid,
    narrowBandWidth: 2,
    useSparse: true,
    fastMode: false  // Use accurate mode for high quality
);
sw.Stop();
Console.WriteLine($"  SDF generated in {sw.ElapsedMilliseconds}ms");
Console.WriteLine($"  SDF dimensions: {sdfGrid.Dimensions.X}×{sdfGrid.Dimensions.Y}×{sdfGrid.Dimensions.Z}");
Console.WriteLine($"  Resolution: {sdfGrid.Resolution}mm\n");

// 6. Generate high-quality mesh using Dual Contouring
Console.WriteLine("Generating mesh with Dual Contouring...");
sw.Restart();
var mesh = MeshConverter.ConvertToMeshFromSDF(sdfGrid);
sw.Stop();
Console.WriteLine($"  Mesh generated in {sw.ElapsedMilliseconds}ms");
Console.WriteLine($"  Vertices: {mesh.Vertices.Length:N0}");
Console.WriteLine($"  Triangles: {mesh.Indices.Length / 3:N0}\n");

// 7. Export to STL
Console.WriteLine("Exporting to STL...");
var outputPath = Path.Combine(outputDir, "sdf_mesh.stl");
sw.Restart();
StlExporter.Export(mesh, outputPath);
sw.Stop();

Console.WriteLine($"  Exported to: {outputPath}");
Console.WriteLine($"  Export time: {sw.ElapsedMilliseconds}ms\n");

Console.WriteLine("=== Sample completed successfully! ===");
Console.WriteLine($"Output file: {outputPath}");
Console.WriteLine("\nTip: Compare this high-quality SDF mesh with direct voxel export");
Console.WriteLine("     to see the difference in surface smoothness!");
