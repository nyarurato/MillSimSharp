using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading;
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;
using NUnit.Framework;

namespace MillSimSharp.Tests.Toolpath
{
    /// <summary>
    /// Tests for executor features: time estimation, tool changes, progress and cancellation.
    /// </summary>
    [TestFixture]
    public class ExecutorFeaturesTest
    {
        private static VoxelGrid CreateGrid()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));
            return new VoxelGrid(bbox, 1.0f);
        }

        [Test]
        public void EstimatedTime_AccumulatesFeedAndRapidMoves()
        {
            var executor = new ToolpathExecutor(new CutterSimulator(CreateGrid()), new EndMill(2f, 10f, false), Vector3.Zero)
            {
                RapidFeedRate = 600f
            };

            executor.ExecuteCommands(new List<IToolpathCommand>
            {
                new G1Move(new Vector3(10, 0, 0), feedRate: 60f),  // 10mm at 60mm/min = 10s
                new G0Move(new Vector3(20, 0, 0)),                  // 10mm at 600mm/min = 1s
            });

            Assert.That(executor.EstimatedTimeSeconds, Is.EqualTo(11.0).Within(1e-3));
        }

        [Test]
        public void Reset_And_LoadCommands_ResetEstimatedTime()
        {
            var executor = new ToolpathExecutor(new CutterSimulator(CreateGrid()), new EndMill(2f, 10f, false), Vector3.Zero);
            executor.ExecuteCommands(new List<IToolpathCommand> { new G1Move(new Vector3(10, 0, 0), 60f) });
            Assert.That(executor.EstimatedTimeSeconds, Is.EqualTo(10.0).Within(1e-3));

            executor.Reset();
            Assert.That(executor.EstimatedTimeSeconds, Is.EqualTo(0.0));

            executor.LoadCommands(new List<IToolpathCommand> { new G1Move(new Vector3(10, 0, 0), 60f) });
            Assert.That(executor.EstimatedTimeSeconds, Is.EqualTo(0.0));
        }

        [Test]
        public void ToolChange_SwitchesExecutingTool()
        {
            var grid = CreateGrid();
            var executor = new ToolpathExecutor(new CutterSimulator(grid), new EndMill(2f, 10f, false), new Vector3(0, 0, 5));

            executor.ExecuteCommands(new List<IToolpathCommand>
            {
                new G1Move(new Vector3(-5, 0, 0)),
                new ToolChange(new EndMill(10f, 20f, false)),
                new G1Move(new Vector3(5, 0, 0)),
            });

            Assert.That(executor.CurrentTool.Diameter, Is.EqualTo(10f));

            // The second cut uses the larger tool: 3mm off the path is removed
            Assert.That(grid.GetVoxelAtWorld(new Vector3(0, 3, 0)), Is.False);
        }

        [Test]
        public void ProgressChanged_ReportsEachCommand()
        {
            var executor = new ToolpathExecutor(new CutterSimulator(CreateGrid()), new EndMill(2f, 10f, false), Vector3.Zero);
            var events = new List<(int done, int total)>();
            executor.ProgressChanged += (done, total) => events.Add((done, total));

            executor.ExecuteCommands(new List<IToolpathCommand>
            {
                new G0Move(new Vector3(1, 0, 0)),
                new G0Move(new Vector3(2, 0, 0)),
                new G0Move(new Vector3(3, 0, 0)),
            });

            Assert.That(events.Count, Is.EqualTo(3));
            Assert.That(events[^1], Is.EqualTo((3, 3)));
        }

        [Test]
        public void Cancellation_StopsExecution()
        {
            var executor = new ToolpathExecutor(new CutterSimulator(CreateGrid()), new EndMill(2f, 10f, false), Vector3.Zero);
            using var cts = new CancellationTokenSource();
            int executed = 0;
            executor.ProgressChanged += (done, total) =>
            {
                executed = done;
                if (done == 2) cts.Cancel();
            };

            var commands = new List<IToolpathCommand>
            {
                new G0Move(new Vector3(1, 0, 0)),
                new G0Move(new Vector3(2, 0, 0)),
                new G0Move(new Vector3(3, 0, 0)),
                new G0Move(new Vector3(4, 0, 0)),
            };

            Assert.Throws<OperationCanceledException>(() => executor.ExecuteCommands(commands, cts.Token));
            Assert.That(executed, Is.EqualTo(2), "Execution must stop after cancellation");
        }
    }
}
