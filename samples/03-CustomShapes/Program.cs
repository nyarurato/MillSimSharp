using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;
using MillSimSharp.IO;
using System.Numerics;

Console.WriteLine("=== MillSimSharp Sample: Custom Shapes (SDF) ===\n");

// Create output directory
var outputDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "output");
Directory.CreateDirectory(outputDir);

// 1. Create a work area (120×120×80mm stock)
Console.WriteLine("Creating SDF grid (120×120×80mm, 1mm resolution)...");
var workArea = BoundingBox.FromCenterAndSize(
    new Vector3(0, 0, 40),
    new Vector3(120, 120, 80)
);
var sdfGrid = new SDFGrid(workArea, resolution: 1.0f, narrowBandWidth: 2, useSparse: true);
Console.WriteLine($"  Grid created: {sdfGrid.Resolution}mm resolution\n");

// 2. Define tools
Console.WriteLine("Setting up tools...");
var roughingTool = new EndMill(diameter: 12.0f, length: 60.0f, isBallEnd: false);
var finishingTool = new EndMill(diameter: 6.0f, length: 50.0f, isBallEnd: true);
Console.WriteLine($"  Roughing tool: {roughingTool.Diameter}mm flat-end mill");
Console.WriteLine($"  Finishing tool: {finishingTool.Diameter}mm ball-end mill\n");

// 3. Roughing pass - create circular pocket with layers
Console.WriteLine("Executing roughing pass (circular pocket, 3 layers)...");
var simulator = new SDFCutterSimulator(sdfGrid);
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
Console.WriteLine($"  Commands executed: {roughingCommands.Count}\n");

// 4. Finishing pass - smooth the walls with a spherical spiral
Console.WriteLine("Executing finishing pass (spherical spiral path)...");
executor = new ToolpathExecutor(simulator, finishingTool, new Vector3(0, 0, 90));

var finishingCommands = new List<IToolpathCommand>();
sw.Restart();

// Spherical spiral finishing path
// Cutting a hemisphere-like shape
float centerX = 0;
float centerY = 0;
float sphereRadius = 30; // Matches the roughing radius at top
float zStart = 60; // Start height (top of hemisphere)
float zEnd = 30;   // End height (bottom of hemisphere cut)
int spiralTurns = 10;
int pointsPerTurn = 36;

// Move to start
finishingCommands.Add(new G0Move(new Vector3(sphereRadius + 5, 0, zStart))); // Safe approach
finishingCommands.Add(new G1Move(new Vector3(sphereRadius, 0, zStart), 800)); // Engage

for (int i = 0; i <= spiralTurns * pointsPerTurn; i++)
{
    float t = (float)i / (spiralTurns * pointsPerTurn); // 0 to 1
    
    // Calculate angle
    float angle = (float)(i * 2 * Math.PI / pointsPerTurn);  
    float currentZ = 70 - (t * 30); // 70 -> 40    
    float sphereR = 35.0f;
    float dz = 70.0f - currentZ; // 0 to 30
    float currentRadius = (float)Math.Sqrt(Math.Max(0, sphereR*sphereR - dz*dz));   
    float x = currentRadius * MathF.Cos(angle);
    float y = currentRadius * MathF.Sin(angle);
    
    finishingCommands.Add(new G1Move(new Vector3(x, y, currentZ), 600));
}

// Retract
finishingCommands.Add(new G0Move(new Vector3(0, 0, 90)));

executor.ExecuteCommands(finishingCommands);
sw.Stop();
Console.WriteLine($"  Finishing completed in {sw.ElapsedMilliseconds}ms");
Console.WriteLine($"  Commands executed: {finishingCommands.Count}\n");

// 5. Export to STL
Console.WriteLine("Generating mesh and exporting to STL...");
var outputPath = Path.Combine(outputDir, "custom_shape.stl");
sw.Restart();
var mesh = MeshConverter.ConvertToMeshFromSDF(sdfGrid);
StlExporter.Export(mesh, outputPath);
sw.Stop();

Console.WriteLine($"  Exported to: {outputPath}");
Console.WriteLine($"  Export time: {sw.ElapsedMilliseconds}ms\n");

Console.WriteLine("=== Sample completed successfully! ===");
Console.WriteLine($"Output file: {outputPath}");
Console.WriteLine($"\nTotal commands executed: {roughingCommands.Count + finishingCommands.Count}");
