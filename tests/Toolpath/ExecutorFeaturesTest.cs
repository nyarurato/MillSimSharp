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

        private static void AssertVoxelGridsEqual(VoxelGrid expected, VoxelGrid actual)
        {
            var (sx, sy, sz) = expected.Dimensions;
            int differences = 0;
            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                    {
                        if (expected.GetVoxel(x, y, z) != actual.GetVoxel(x, y, z)) differences++;
                    }

            Assert.That(differences, Is.EqualTo(0),
                "The executor cut must match a cut performed with the current orientation held fixed");
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
        public void Reset_RestoresInitialTool()
        {
            var initialTool = new EndMill(2f, 10f, isBallEnd: false);
            var executor = new ToolpathExecutor(new CutterSimulator(CreateGrid()), initialTool, Vector3.Zero);

            executor.ExecuteCommand(new ToolChange(new EndMill(10f, 20f, isBallEnd: false)));
            Assert.That(executor.CurrentTool.Diameter, Is.EqualTo(10f), "ToolChange must switch the tool");

            executor.Reset();
            Assert.That(executor.CurrentTool, Is.SameAs(initialTool),
                "Reset must restore the initial (constructor) tool");
        }

        [Test]
        public void LoadCommands_RestoresInitialTool()
        {
            var initialTool = new EndMill(2f, 10f, isBallEnd: false);
            var executor = new ToolpathExecutor(new CutterSimulator(CreateGrid()), initialTool, Vector3.Zero);

            executor.ExecuteCommand(new ToolChange(new EndMill(10f, 20f, isBallEnd: false)));

            executor.LoadCommands(new List<IToolpathCommand> { new G0Move(Vector3.Zero) });
            Assert.That(executor.CurrentTool, Is.SameAs(initialTool),
                "LoadCommands must restore the initial (constructor) tool");
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
        public void Executor_G1MoveAfterFiveAxisPose_UsesCurrentOrientation()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));
            var tool = new EndMill(10f, 30f, isBallEnd: false);
            var start = new Vector3(0, 0, 0);
            var target = new Vector3(8, 0, 0);
            var tilted = new ToolOrientation(30, 0, 0);

            // Reference cut: the current pose is held fixed (start = end = tilted).
            var referenceGrid = new VoxelGrid(bbox, 1.0f);
            new CutterSimulator(referenceGrid).CutLinearWithOrientation(start, target, tool, tilted, tilted);

            // Executor: a normal G1 after a 5-axis pose must keep the current orientation,
            // not silently fall back to the default 3-axis pose.
            var executorGrid = new VoxelGrid(bbox, 1.0f);
            var executor = new ToolpathExecutor(new CutterSimulator(executorGrid), tool, start);
            executor.ExecuteCommands(new List<IToolpathCommand>
            {
                new G0Move5Axis(start, tilted),
                new G1Move(target),
            });

            AssertVoxelGridsEqual(referenceGrid, executorGrid);
        }

        [Test]
        public void Executor_G1MoveAfterFiveAxisPose_UsesCurrentOrientation_Sdf()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));
            var tool = new EndMill(10f, 30f, isBallEnd: false);
            var start = new Vector3(0, 0, 0);
            var target = new Vector3(8, 0, 0);
            var tilted = new ToolOrientation(30, 0, 0);

            var reference = new SDFGrid(bbox, 1.0f, narrowBandWidth: 6);
            new SDFCutterSimulator(reference).CutLinearWithOrientation(start, target, tool, tilted, tilted);

            var actual = new SDFGrid(bbox, 1.0f, narrowBandWidth: 6);
            var executor = new ToolpathExecutor(new SDFCutterSimulator(actual), tool, start);
            executor.ExecuteCommands(new List<IToolpathCommand>
            {
                new G0Move5Axis(start, tilted),
                new G1Move(target),
            });

            var (sx, sy, sz) = reference.Dimensions;
            int signDifferences = 0;
            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                    {
                        float expected = reference.GetDistance(x, y, z);
                        float value = actual.GetDistance(x, y, z);
                        if ((expected < 0f) != (value < 0f)) signDifferences++;
                    }

            Assert.That(signDifferences, Is.EqualTo(0),
                "The executor SDF cut must match the fixed-orientation cut (sign differences found)");
        }

        [Test]
        public void Executor_G0MoveAfterFiveAxisPose_KeepsOrientation()
        {
            var executor = new ToolpathExecutor(new CutterSimulator(CreateGrid()), new EndMill(2f, 10f, false), new Vector3(0, 0, 5));
            var tilted = new ToolOrientation(30, 10, -5);

            executor.ExecuteCommands(new List<IToolpathCommand>
            {
                new G0Move5Axis(new Vector3(0, 0, 5), tilted),
                new G0Move(new Vector3(5, 0, 5)),
            });

            Assert.That(executor.CurrentOrientation.A, Is.EqualTo(30f).Within(1e-5f));
            Assert.That(executor.CurrentOrientation.B, Is.EqualTo(10f).Within(1e-5f));
            Assert.That(executor.CurrentOrientation.C, Is.EqualTo(-5f).Within(1e-5f));
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

        [Test]
        public void StepSize_BelowOne_Throws()
        {
            var executor = new ToolpathExecutor(new CutterSimulator(CreateGrid()), new EndMill(2f, 10f, false), Vector3.Zero);

            Assert.Throws<ArgumentOutOfRangeException>(() => executor.StepSize = 0);
            Assert.Throws<ArgumentOutOfRangeException>(() => executor.StepSize = -3);

            executor.StepSize = 1;
            Assert.That(executor.StepSize, Is.EqualTo(1), "the smallest valid step size must be accepted");
        }
    }
}
