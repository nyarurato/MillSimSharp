using System;
using System.Collections.Generic;
using System.Numerics;
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;
using NUnit.Framework;

namespace MillSimSharp.Tests.Toolpath
{
    /// <summary>
    /// Combined executor state-machine tests: position, orientation, tool, command index and
    /// estimated time across long mixed sequences, and full vs step execution equivalence.
    /// </summary>
    [TestFixture]
    public class ExecutorSequenceTest
    {
        private static BoundingBox StockBounds =>
            BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));

        private static List<IToolpathCommand> BuildMixedSequence(Tool changedTool)
        {
            return new List<IToolpathCommand>
            {
                new G0Move5Axis(new Vector3(0, 0, 5), new ToolOrientation(30, 0, 0)),
                new G1Move(new Vector3(3, 0, 5), feedRate: 60f),
                new ToolChange(changedTool),
                new G1Move5Axis(new Vector3(6, 0, 5), new ToolOrientation(30, 20, 0), feedRate: 120f),
                new G0Move(new Vector3(6, 0, 10)),
            };
        }

        [Test]
        public void Executor_LongMixedSequence_KeepsStateConsistent()
        {
            var initialTool = new EndMill(4f, 20f, isBallEnd: false);
            var newTool = new EndMill(8f, 20f, isBallEnd: true);
            var startPosition = Vector3.Zero;
            var initialOrientation = new ToolOrientation(10, 0, 0);

            var grid = new VoxelGrid(StockBounds, 1.0f);
            var executor = new ToolpathExecutor(new CutterSimulator(grid), initialTool, startPosition, initialOrientation);
            executor.LoadCommands(BuildMixedSequence(newTool));

            // Step 1: G0Move5Axis (5mm rapid at the default 5000 mm/min = 0.06 s).
            Assert.That(executor.ExecuteNextSteps(1), Is.EqualTo(1));
            Assert.That(executor.CurrentCommandIndex, Is.EqualTo(0));
            Assert.That(executor.CurrentPosition, Is.EqualTo(new Vector3(0, 0, 5)));
            Assert.That(executor.CurrentOrientation.A, Is.EqualTo(30f).Within(1e-5f));
            Assert.That(executor.CurrentTool, Is.SameAs(initialTool));
            Assert.That(executor.EstimatedTimeSeconds, Is.EqualTo(0.06).Within(1e-3));

            // Steps 2-3: normal G1 keeps the 5-axis pose; ToolChange switches the tool.
            Assert.That(executor.ExecuteNextSteps(2), Is.EqualTo(2));
            Assert.That(executor.CurrentCommandIndex, Is.EqualTo(2));
            Assert.That(executor.CurrentTool, Is.SameAs(newTool));
            Assert.That(executor.CurrentPosition, Is.EqualTo(new Vector3(3, 0, 5)));
            Assert.That(executor.CurrentOrientation.A, Is.EqualTo(30f).Within(1e-5f));
            Assert.That(executor.EstimatedTimeSeconds, Is.EqualTo(0.06 + 3.0).Within(1e-3)); // 3mm at 60 mm/min

            // Finish: 5-axis move (3mm at 120 = 1.5 s) and rapid (5mm = 0.06 s).
            Assert.That(executor.ExecuteNextSteps(2), Is.EqualTo(2));
            Assert.That(executor.IsCompleted, Is.True);
            Assert.That(executor.CurrentPosition, Is.EqualTo(new Vector3(6, 0, 10)));
            Assert.That(executor.CurrentOrientation.A, Is.EqualTo(30f).Within(1e-5f));
            Assert.That(executor.CurrentOrientation.B, Is.EqualTo(20f).Within(1e-5f));
            Assert.That(executor.CurrentTool, Is.SameAs(newTool));
            Assert.That(executor.EstimatedTimeSeconds, Is.EqualTo(0.06 + 3.0 + 1.5 + 0.06).Within(1e-3));

            // Reset returns to the constructor baseline.
            executor.Reset();
            Assert.That(executor.CurrentCommandIndex, Is.EqualTo(-1));
            Assert.That(executor.CurrentPosition, Is.EqualTo(startPosition));
            Assert.That(executor.CurrentOrientation.A, Is.EqualTo(10f).Within(1e-5f));
            Assert.That(executor.CurrentTool, Is.SameAs(initialTool));
            Assert.That(executor.EstimatedTimeSeconds, Is.EqualTo(0.0));

            // Load again: the tool baseline is restored and the sequence can be re-run step by step.
            executor.LoadCommands(BuildMixedSequence(newTool));
            Assert.That(executor.CurrentTool, Is.SameAs(initialTool));
            while (executor.ExecuteNextSteps(1) > 0)
            {
            }

            Assert.That(executor.IsCompleted, Is.True);
            Assert.That(executor.CurrentTool, Is.SameAs(newTool));
            Assert.That(executor.CurrentPosition, Is.EqualTo(new Vector3(6, 0, 10)));
        }

        [Test]
        public void ToolChange_FullVsStepExecution_ProducesSameState()
        {
            var initialTool = new EndMill(4f, 20f, isBallEnd: false);
            var changedTool = new EndMill(8f, 20f, isBallEnd: true);
            var startPosition = Vector3.Zero;

            var batchGrid = new VoxelGrid(StockBounds, 1.0f);
            var batch = new ToolpathExecutor(new CutterSimulator(batchGrid), initialTool, startPosition);
            batch.ExecuteCommands(BuildMixedSequence(changedTool));

            var stepGrid = new VoxelGrid(StockBounds, 1.0f);
            var step = new ToolpathExecutor(new CutterSimulator(stepGrid), initialTool, startPosition);
            step.LoadCommands(BuildMixedSequence(changedTool));
            while (step.ExecuteNextSteps(1) > 0)
            {
            }

            Assert.That(step.CurrentPosition, Is.EqualTo(batch.CurrentPosition));
            Assert.That(step.CurrentOrientation.A, Is.EqualTo(batch.CurrentOrientation.A).Within(1e-5f));
            Assert.That(step.CurrentOrientation.B, Is.EqualTo(batch.CurrentOrientation.B).Within(1e-5f));
            Assert.That(step.CurrentOrientation.C, Is.EqualTo(batch.CurrentOrientation.C).Within(1e-5f));
            Assert.That(step.CurrentTool.Diameter, Is.EqualTo(batch.CurrentTool.Diameter));
            Assert.That(step.CurrentTool.Type, Is.EqualTo(batch.CurrentTool.Type));
            Assert.That(step.EstimatedTimeSeconds, Is.EqualTo(batch.EstimatedTimeSeconds).Within(1e-9));
            // Note: ExecuteCommands does not advance CurrentCommandIndex (it is a step-execution
            // cursor), so the index is not part of this comparison.

            var (sx, sy, sz) = batchGrid.Dimensions;
            int differences = 0;
            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                        if (batchGrid.GetVoxel(x, y, z) != stepGrid.GetVoxel(x, y, z)) differences++;

            Assert.That(differences, Is.EqualTo(0), "Full and step execution must remove the same voxels");
        }
    }
}
