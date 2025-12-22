using System;
using System.Collections.Generic;
using System.Numerics;
using MillSimSharp.Config;
using MillSimSharp.Geometry;
using MillSimSharp.IO;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;
using MillSimSharp.Util;

namespace FiveAxisMachining
{
    /// <summary>
    /// Demonstrates 5-axis machining capabilities using SDF (Signed Distance Field).
    /// </summary>
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("=== MillSimSharp Sample: 5-Axis Machining (SDF) ===\n");

            // Create output directory
            var outputDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "output");
            Directory.CreateDirectory(outputDir);

            // Create stock configuration (100x100x50mm block)
            // Origin at center (0,0,0), so stock extends from (-50,-50,-25) to (50,50,25)
            var stockConfig = new StockConfiguration
            {
                WorkOrigin = new Vector3Data(0, 0, 0),
                WorkSize = new Vector3Data(100, 100, 50),
                OriginType = StockOriginType.Center
            };

            // Create tool configuration (20mm ball end mill)
            var toolConfig = new ToolConfiguration
            {
                Diameter = 10f,
                Length = 100f,
                IsBallEnd = true
            };

            var tool = toolConfig.CreateTool();

            // Create SDF grid with 0.5mm resolution for high-quality results
            Console.WriteLine("Creating SDF grid (100×100×50mm, 0.5mm resolution)...");
            var bbox = stockConfig.GetBoundingBox();
            var sdfGrid = new SDFGrid(bbox, resolution: 0.5f, narrowBandWidth: 5);
            var simulator = new SDFCutterSimulator(sdfGrid);
            var executor = new ToolpathExecutor(simulator, tool, Vector3.Zero);
            var size = bbox.Max - bbox.Min;
            Console.WriteLine($"  Stock: {size.X}x{size.Y}x{size.Z}mm");
            Console.WriteLine($"  Tool: {toolConfig.Diameter}mm ball-end mill");
            Console.WriteLine($"  Resolution: {sdfGrid.Resolution}mm\n");

            // Example 1: Simple tilted cutting pass
            Console.WriteLine("Example 1: Tilted cutting pass with A-axis rotation...");
            SimpleTiltedPass(executor);

            // Example 2: Cone-shaped toolpath with continuously changing orientation
            Console.WriteLine("\nExample 2: 5-axis cone toolpath...");
            FiveAxisCone(executor);

            // Generate high-quality mesh from SDF
            Console.WriteLine("\nGenerating mesh from SDF...");
            var mesh = MeshConverter.ConvertToMeshFromSDF(sdfGrid);
            Console.WriteLine($"  Generated: {mesh.Vertices.Length} vertices, {mesh.Indices.Length / 3} triangles\n");

            // Export result
            string outputFile = "five_axis_result.stl";
            string outputPath = Path.Combine(outputDir, outputFile);
            StlExporter.Export(mesh, outputPath);
            Console.WriteLine($"Saved: {outputPath}");
        }

        /// <summary>
        /// Example 1: Simple pass with tool tilted at 30 degrees.
        /// </summary>
        static void SimpleTiltedPass(ToolpathExecutor executor)
        {
            var commands = new List<IToolpathCommand>();

            // Start position - above the stock
            var startPos = new Vector3(-10, -30, 10);
            
            // Tilted orientation (30 degrees on A-axis)
            var orientation = new ToolOrientation(a_deg: 30, b_deg: -20, c_deg: 20);

            // Rapid move to start position with orientation
            commands.Add(new G0Move5Axis(startPos, orientation));

            // Cut across the stock with tilted tool
            for (float x = -10; x <= 55; x += 2)
            {
                var pos = new Vector3(x, -30, 10);
                commands.Add(new G1Move5Axis(pos, orientation, feedRate: 200f));
            }

            // Retract
            commands.Add(new G0Move5Axis(new Vector3(90, 50, 60), orientation));

            executor.ExecuteCommands(commands);
            Console.WriteLine($"  Completed {commands.Count} commands");
        }

        /// <summary>
        /// Example 2: Cone-shaped toolpath with continuously changing orientation.
        /// </summary>
        static void FiveAxisCone(ToolpathExecutor executor)
        {
            var commands = new List<IToolpathCommand>();
            var tool_fixed_point = new Vector3(0, 0, 10); // Fixed point on tool shaft (above origin)
            int divN = 10; // Number of divisions
            float cone_base_z = -15f; // Cone base Z coordinate (below origin)
            float radius = 20f; // Base radius
            
            // Retract to safe position
            commands.Add(new G0Move5Axis(new Vector3(0, 0, 50), ToolOrientation.Default));

            for(int i = 0; i <= divN; i++)
            {
                float angle = i * 2.0f * MathF.PI / divN;
                float x = radius * MathF.Cos(angle);  // Circle centered at origin
                float y = radius * MathF.Sin(angle);
                float z = cone_base_z; // Cone base height
                
                // Tool tip position (on cone base circumference)
                var targetPos = new Vector3(x, y, z);

                // Tool axis direction = from fixed point to tool tip (tip pointing down, same as default (0,0,-1))
                Vector3 tool_direction = Vector3.Normalize(targetPos - tool_fixed_point);
                
                // Calculate orientation angles from tool direction
                // tool_direction is the direction tool points (default is (0,0,-1) downward)
                // Rotation order: C → B → A (ZYX Euler angles)
                
                // B-axis: Rotation around Y-axis (tilt in XZ plane)
                // Negate to match Matrix4x4.CreateRotationY definition
                float b_deg = -MathF.Atan2(tool_direction.X, -tool_direction.Z) * 180f / MathF.PI;
                
                // A-axis: Rotation around X-axis (tilt in YZ plane)
                float projectionXZ = MathF.Sqrt(tool_direction.X * tool_direction.X + tool_direction.Z * tool_direction.Z);
                float a_deg = MathF.Atan2(tool_direction.Y, projectionXZ) * 180f / MathF.PI;
                
                var orientation = new ToolOrientation(a_deg, b_deg, 0);
                
                // Cutting move (start cutting from first point)
                commands.Add(new G1Move5Axis(targetPos, orientation, feedRate: 150f));
            }
            
            // Retract
            commands.Add(new G0Move5Axis(new Vector3(50, 50, 80), ToolOrientation.Default));
            
            executor.ExecuteCommands(commands);
            Console.WriteLine($"  Completed {commands.Count} commands");
        }

    }
}
