using NUnit.Framework;
using MillSimSharp;
using MillSimSharp.Config;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using System;

namespace MillSimSharp.Tests.Integration
{
    [TestFixture]
    public class IntegrationTest
    {
        private string _configPath;

        [SetUp]
        public void Setup()
        {
            _configPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "configs");
        }

        [Test]
        public void TestSimpleMillingOperation()
        {
            // Create simulation with 10x10x10mm stock, 1mm resolution
            var stockConfig = new StockConfiguration
            {
                WorkOrigin = new Vector3Data(0, 0, 0),
                WorkSize = new Vector3Data(10, 10, 10)
            };
            var toolConfig = new ToolConfiguration
            {
                Diameter = 2.0f,
                Length = 10.0f,
                IsBallEnd = false
            };

            var sim = new MillSimulation(stockConfig, toolConfig, resolution: 1.0f);

            // Initial state: all voxels are material
            int initialCount = sim.GetMaterialVoxelCount();
            Assert.That(initialCount, Is.GreaterThan(0));

            // Execute a simple cut
            var commands = new List<IToolpathCommand>
            {
                new G0Move(new Vector3(-3, 0, 0)),
                new G1Move(new Vector3(3, 0, 0))
            };
            sim.ExecuteToolpath(commands);

            // Material should be removed
            int afterCut = sim.GetMaterialVoxelCount();
            Assert.That(afterCut, Is.LessThan(initialCount));
        }

        [Test]
        public void TestLoadConfigAndSimulate()
        {
            var toolPath = Path.Combine(_configPath, "default_tool.xml");
            var stockPath = Path.Combine(_configPath, "default_stock.xml");

            var sim = MillSimulation.FromConfigFiles(toolPath, stockPath, resolution: 2.0f);

            Assert.That(sim.Grid, Is.Not.Null);
            Assert.That(sim.Tool, Is.Not.Null);
            Assert.That(sim.Tool.Diameter, Is.EqualTo(10.0f));

            // Execute simple toolpath
            var commands = new List<IToolpathCommand>
            {
                new G0Move(new Vector3(0, 0, 10)),
                new G0Move(new Vector3(-20, 0, 0)),
                new G1Move(new Vector3(20, 0, 0))
            };
            sim.ExecuteToolpath(commands);

            // Check that material was removed
            Assert.That(sim.GetMaterialVoxelCount(), Is.LessThan(sim.Grid.Dimensions.X * sim.Grid.Dimensions.Y * sim.Grid.Dimensions.Z));
        }

        [Test]
        public void TestExportToStl()
        {
            var stockConfig = new StockConfiguration
            {
                WorkOrigin = new Vector3Data(0, 0, 0),
                WorkSize = new Vector3Data(20, 20, 20)
            };
            var toolConfig = new ToolConfiguration
            {
                Diameter = 5.0f,
                Length = 20.0f,
                IsBallEnd = false
            };

            var sim = new MillSimulation(stockConfig, toolConfig, resolution: 2.0f);

            // Cut a simple path
            var commands = new List<IToolpathCommand>
            {
                new G1Move(new Vector3(10, 0, 0))
            };
            sim.ExecuteToolpath(commands);

            // Export to STL
            var outputPath = Path.Combine(Path.GetTempPath(), "test_output.stl");
            sim.ExportToStl(outputPath);

            // Verify file was created
            Assert.That(File.Exists(outputPath), Is.True);

            // Cleanup
            if (File.Exists(outputPath))
                File.Delete(outputPath);
        }

        [Test]
        public void TestReset()
        {
            var stockConfig = new StockConfiguration
            {
                WorkOrigin = new Vector3Data(0, 0, 0),
                WorkSize = new Vector3Data(10, 10, 10)
            };
            var toolConfig = new ToolConfiguration
            {
                Diameter = 2.0f,
                Length = 10.0f,
                IsBallEnd = false
            };

            var sim = new MillSimulation(stockConfig, toolConfig, resolution: 1.0f);

            int initialCount = sim.GetMaterialVoxelCount();

            // Cut
            sim.ExecuteToolpath(new List<IToolpathCommand> { new G1Move(new Vector3(5, 0, 0)) });
            Assert.That(sim.GetMaterialVoxelCount(), Is.LessThan(initialCount));

            // Reset
            sim.Reset();
            Assert.That(sim.GetMaterialVoxelCount(), Is.EqualTo(initialCount));
        }

        [Test]
        public void TestToolChange()
        {
            var stockConfig = new StockConfiguration
            {
                WorkOrigin = new Vector3Data(0, 0, 0),
                WorkSize = new Vector3Data(10, 10, 10)
            };
            var toolConfig = new ToolConfiguration
            {
                Diameter = 2.0f,
                Length = 10.0f,
                IsBallEnd = false
            };

            var sim = new MillSimulation(stockConfig, toolConfig, resolution: 1.0f);

            Assert.That(sim.Tool.Diameter, Is.EqualTo(2.0f));

            // Change to larger tool
            var newTool = new EndMill(5.0f, 20.0f, false);
            sim.ChangeTool(newTool);

            Assert.That(sim.Tool.Diameter, Is.EqualTo(5.0f));
            
            // Execute with new tool
            sim.ExecuteToolpath(new List<IToolpathCommand> { new G1Move(new Vector3(5, 0, 0)) });
            
            // Should succeed without error
            Assert.That(sim.Tool.Diameter, Is.EqualTo(5.0f));
        }
    }
}
