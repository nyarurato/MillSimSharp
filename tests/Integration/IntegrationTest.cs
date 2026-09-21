using NUnit.Framework;
using MillSimSharp;
using MillSimSharp.Config;
using MillSimSharp.Tests.Config;
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

            var toolConfig = ConfigurationLoader.LoadToolConfig(toolPath);
            var stockConfig = ConfigurationLoader.LoadStockConfig(stockPath);
            var sim = new MillSimulation(stockConfig, toolConfig, resolution: 2.0f);

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

        [Test]
        public void TestChangeToolPreservesPose()
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
            sim.Executor.ExecuteCommand(new G0Move5Axis(new Vector3(1, 2, 3), new ToolOrientation(30, 10, 0)));
            sim.Executor.StepSize = 7;

            sim.ChangeTool(new EndMill(5.0f, 20.0f, false));

            Assert.That(sim.Executor.CurrentPosition, Is.EqualTo(new Vector3(1, 2, 3)));
            Assert.That(sim.Executor.CurrentOrientation.A, Is.EqualTo(30f).Within(1e-5f));
            Assert.That(sim.Executor.CurrentOrientation.B, Is.EqualTo(10f).Within(1e-5f));
            Assert.That(sim.Executor.StepSize, Is.EqualTo(7));
        }

        // ---------------------------------------------------------------------
        // V1 audit: tool state across the facade / executor
        // ---------------------------------------------------------------------

        private static MillSimulation CreateSimulation()
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
            return new MillSimulation(stockConfig, toolConfig, resolution: 1.0f);
        }

        [Test]
        public void MillSimulation_ToolChangeCommand_SynchronizesFacadeTool()
        {
            var sim = CreateSimulation();
            var newTool = new EndMill(5.0f, 20.0f, false);

            sim.Executor.ExecuteCommand(new ToolChange(newTool));

            Assert.That(sim.Executor.CurrentTool, Is.SameAs(newTool));
            Assert.That(sim.Tool, Is.SameAs(newTool),
                "The facade Tool must reflect a ToolChange command executed by the executor");
        }

        [Test]
        public void MillSimulation_ToolSetter_SynchronizesExecutor()
        {
            var sim = CreateSimulation();
            var newTool = new EndMill(5.0f, 20.0f, false);

            sim.Tool = newTool;

            Assert.That(sim.Executor.CurrentTool, Is.SameAs(newTool),
                "Setting the facade Tool must update the executor tool");
        }

        [Test]
        public void ChangeTool_PreservesRapidFeedRate()
        {
            var sim = CreateSimulation();
            sim.Executor.RapidFeedRate = 1234f;

            sim.ChangeTool(new EndMill(5.0f, 20.0f, false));

            Assert.That(sim.Executor.RapidFeedRate, Is.EqualTo(1234f),
                "ChangeTool must preserve RapidFeedRate");
        }

        [Test]
        public void ChangeTool_PreservesLoadedCommandsAndCurrentIndex()
        {
            var sim = CreateSimulation();
            var commands = new List<IToolpathCommand>
            {
                new G0Move(new Vector3(0, 0, 1)),
                new G0Move(new Vector3(0, 0, 2)),
                new G0Move(new Vector3(0, 0, 3)),
            };
            sim.Executor.LoadCommands(commands);
            sim.Executor.ExecuteNextSteps(1);
            Assert.That(sim.Executor.CurrentCommandIndex, Is.EqualTo(0));

            sim.ChangeTool(new EndMill(5.0f, 20.0f, false));

            Assert.That(sim.Executor.TotalCommands, Is.EqualTo(3), "ChangeTool must keep the loaded commands");
            Assert.That(sim.Executor.CurrentCommandIndex, Is.EqualTo(0), "ChangeTool must keep the command index");
        }

        [Test]
        public void ChangeTool_PreservesProgressSubscribers()
        {
            var sim = CreateSimulation();
            int events = 0;
            sim.Executor.ProgressChanged += (done, total) => events++;

            sim.ChangeTool(new EndMill(5.0f, 20.0f, false));
            sim.Executor.ExecuteCommands(new List<IToolpathCommand>
            {
                new G0Move(new Vector3(1, 0, 0)),
                new G0Move(new Vector3(2, 0, 0)),
            });

            Assert.That(events, Is.EqualTo(2), "Progress subscribers must survive ChangeTool");
        }

        [Test]
        public void ChangeTool_DoesNotUnexpectedlyResetEstimatedTime()
        {
            var sim = CreateSimulation();
            sim.Executor.ExecuteCommands(new List<IToolpathCommand>
            {
                new G1Move(new Vector3(10, 0, 0), 60f), // 10 mm at 60 mm/min = 10 s
            });
            Assert.That(sim.Executor.EstimatedTimeSeconds, Is.EqualTo(10.0).Within(1e-3));

            sim.ChangeTool(new EndMill(5.0f, 20.0f, false));

            Assert.That(sim.Executor.EstimatedTimeSeconds, Is.EqualTo(10.0).Within(1e-3),
                "ChangeTool must not reset accumulated estimated time");
        }

        [Test]
        public void MillSimulation_Reset_KeepsToolConsistent()
        {
            var sim = CreateSimulation();
            var newTool = new EndMill(5.0f, 20.0f, false);
            sim.ChangeTool(newTool);

            sim.Reset();

            Assert.That(sim.Tool, Is.SameAs(newTool), "Grid reset must keep the current tool");
            Assert.That(sim.Executor.CurrentTool, Is.SameAs(newTool));
        }

        [Test]
        public void TestExportToStlViaSdf()
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
                IsBallEnd = true
            };

            var sim = new MillSimulation(stockConfig, toolConfig, resolution: 2.0f);
            sim.ExecuteToolpath(new List<IToolpathCommand>
            {
                new G0Move(new Vector3(0, 0, 10)),
                new G1Move(new Vector3(10, 10, 10))
            });

            var outputPath = Path.Combine(Path.GetTempPath(), "test_output_sdf.stl");
            sim.ExportToStlViaSdf(outputPath, narrowBandWidth: 5);

            Assert.That(File.Exists(outputPath), Is.True);
            Assert.That(new FileInfo(outputPath).Length, Is.GreaterThan(84)); // header + triangle count

            if (File.Exists(outputPath))
                File.Delete(outputPath);
        }
    }
}
