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
            Console.WriteLine("5-Axis Machining Simulation Demo (SDF-based)");
            Console.WriteLine("=============================================\n");

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
            var bbox = stockConfig.GetBoundingBox();
            var sdfGrid = new SDFGrid(bbox, resolution: 0.5f, narrowBandWidth: 5);
            var simulator = new SDFCutterSimulator(sdfGrid);
            var executor = new ToolpathExecutor(simulator, tool, Vector3.Zero);

            var size = bbox.Max - bbox.Min;
            Console.WriteLine($"Stock: {size.X}x{size.Y}x{size.Z}mm");
            Console.WriteLine($"Tool: Ball End Mill, Diameter: {toolConfig.Diameter}mm");
            Console.WriteLine($"Resolution: {sdfGrid.Resolution}mm (SDF-based)");
            Console.WriteLine($"Narrow band width: {sdfGrid.NarrowBandWidth} voxels\n");

            // Example 1: Simple tilted cutting pass
            Console.WriteLine("Example 1: Tilted cutting pass with A-axis rotation");
            SimpleTiltedPass(executor);

            // Example 2: Cone-shaped toolpath with continuously changing orientation
            Console.WriteLine("Example 2: 5-axis cone toolpath");
            FiveAxisCone(executor);

            // Generate high-quality mesh from SDF
            Console.WriteLine("\nGenerating mesh from SDF...");
            var mesh = MeshConverter.ConvertToMeshFromSDF(sdfGrid);
            Console.WriteLine($"Mesh generated: {mesh.Vertices.Length} vertices, {mesh.Indices.Length / 3} triangles");

            // Export result
            string outputFile = "five_axis_result.stl";
            StlExporter.Export(mesh, outputFile);
            Console.WriteLine($"\nExported result to: {outputFile}");
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
            var tool_fixed_point = new Vector3(0, 0, 10); //工具のシャフトが通る固定点（原点より上）
            int divN = 10; //分割数
            float cone_base_z = -15f; //円錐底面のZ座標（原点より下）
            float radius = 20f; //底面の半径
            
            // 安全な退避位置
            commands.Add(new G0Move5Axis(new Vector3(0, 0, 50), ToolOrientation.Default));

            Console.WriteLine($"  Cone: center=(0,0,{cone_base_z}), radius={radius}mm, apex=(0,0,{tool_fixed_point.Z})");

            for(int i = 0; i <= divN; i++)
            {
                float angle = i * 2.0f * MathF.PI / divN;
                float x = radius * MathF.Cos(angle);  // 原点(0,0)中心の円
                float y = radius * MathF.Sin(angle);
                float z = cone_base_z; // 円錐底面の高さ
                
                // 工具先端位置（円錐の底面円周上）
                var targetPos = new Vector3(x, y, z);

                // 工具軸方向 = 固定点から工具先端への方向（工具先端が下、デフォルト(0,0,-1)と同じ向き）
                Vector3 tool_direction = Vector3.Normalize(targetPos - tool_fixed_point);
                
                // 工具方向から姿勢角度を計算
                // tool_direction は工具が指す方向（デフォルトは(0,0,-1)が下向き）
                // 回転順序: C → B → A (ZYX Euler angles)
                
                // B軸: Y軸周りの回転（XZ平面での傾き）
                // 符号を反転（Matrix4x4.CreateRotationYの定義と合わせる）
                float b_deg = -MathF.Atan2(tool_direction.X, -tool_direction.Z) * 180f / MathF.PI;
                
                // A軸: X軸周りの回転（YZ平面での傾き）
                float projectionXZ = MathF.Sqrt(tool_direction.X * tool_direction.X + tool_direction.Z * tool_direction.Z);
                float a_deg = MathF.Atan2(tool_direction.Y, projectionXZ) * 180f / MathF.PI;
                
                var orientation = new ToolOrientation(a_deg, b_deg, 0);
                
                // 切削移動（最初のポイントから切削開始）
                commands.Add(new G1Move5Axis(targetPos, orientation, feedRate: 150f));
            }
            
            // Retract
            commands.Add(new G0Move5Axis(new Vector3(50, 50, 80), ToolOrientation.Default));
            
            executor.ExecuteCommands(commands);
            Console.WriteLine($"  Completed {commands.Count} commands");
        }

    }
}
