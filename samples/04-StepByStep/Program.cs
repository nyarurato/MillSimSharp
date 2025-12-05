using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;
using MillSimSharp.IO;
using System.Numerics;

Console.WriteLine("=== MillSimSharp Sample: Step-by-Step Execution ===\n");

// Create output directory
var outputDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "output");
Directory.CreateDirectory(outputDir);

// 1. Create a work area (60×60×40mm stock)
Console.WriteLine("Creating voxel grid (60×60×40mm, 1mm resolution)...");
var workArea = BoundingBox.FromCenterAndSize(
    new Vector3(0, 0, 20),
    new Vector3(60, 60, 40)
);
var voxelGrid = new VoxelGrid(workArea, resolution: 1.0f);
Console.WriteLine($"  Grid created: {voxelGrid.Dimensions.X}×{voxelGrid.Dimensions.Y}×{voxelGrid.Dimensions.Z} voxels\n");

// 2. Define tool and create simulator
Console.WriteLine("Setting up tool and simulator...");
var tool = new EndMill(diameter: 8.0f, length: 40.0f, isBallEnd: true);
var simulator = new CutterSimulator(voxelGrid);
var executor = new ToolpathExecutor(simulator, tool, new Vector3(0, 0, 50));
Console.WriteLine($"  Tool: {tool.Diameter}mm ball-end mill\n");

// 3. Create a simple cross-shaped toolpath
Console.WriteLine("Creating toolpath (cross pattern)...");
var commands = new List<IToolpathCommand>
{
    // Horizontal line
    new G0Move(new Vector3(-20, 0, 35)),
    new G1Move(new Vector3(-20, 0, 30), 500),
    new G1Move(new Vector3(20, 0, 30), 500),
    new G0Move(new Vector3(20, 0, 35)),
    
    // Vertical line
    new G0Move(new Vector3(0, -20, 35)),
    new G1Move(new Vector3(0, -20, 30), 500),
    new G1Move(new Vector3(0, 20, 30), 500),
    new G0Move(new Vector3(0, 20, 35)),
};

executor.LoadCommands(commands);
Console.WriteLine($"  Loaded {commands.Count} commands\n");

// 4. Execute step-by-step and save intermediate results
Console.WriteLine("Executing step-by-step (saving intermediate STL files)...");
executor.StepSize = 2;  // Execute 2 commands at a time

int stepCount = 0;
var sw = System.Diagnostics.Stopwatch.StartNew();

while (executor.CurrentCommandIndex < executor.TotalCommands)
{
    long voxelsBefore = voxelGrid.CountMaterialVoxels();
    int executed = executor.ExecuteNextSteps();
    long voxelsAfter = voxelGrid.CountMaterialVoxels();
    
    if (executed > 0)
    {
        stepCount++;
        Console.WriteLine($"  Step {stepCount}: Executed commands {executor.CurrentCommandIndex - executed + 1}-{executor.CurrentCommandIndex}");
        Console.WriteLine($"    Voxels removed: {voxelsBefore - voxelsAfter:N0}");
        
        // Save intermediate result
        var stepOutputPath = Path.Combine(outputDir, $"step_{stepCount:D2}.stl");
        StlExporter.Export(voxelGrid, stepOutputPath);
        Console.WriteLine($"    Saved: step_{stepCount:D2}.stl\n");
    }
}

sw.Stop();
Console.WriteLine($"Total execution time: {sw.ElapsedMilliseconds}ms");
Console.WriteLine($"Total steps: {stepCount}");
Console.WriteLine($"Final voxels: {voxelGrid.CountMaterialVoxels():N0}\n");

// 5. Export final result
Console.WriteLine("Exporting final result...");
var finalOutputPath = Path.Combine(outputDir, "step_final.stl");
StlExporter.Export(voxelGrid, finalOutputPath);
Console.WriteLine($"  Exported to: {finalOutputPath}\n");

Console.WriteLine("=== Sample completed successfully! ===");
Console.WriteLine($"Output directory: {outputDir}");
Console.WriteLine($"Generated {stepCount + 1} STL files (step_01.stl to step_{stepCount:D2}.stl + step_final.stl)");
Console.WriteLine("\nTip: Load these files sequentially in a 3D viewer to see");
Console.WriteLine("     the machining process step-by-step!");
