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
    /// Regression tests for the executor cursor contract on command failure: a command that throws
    /// must not advance the cursor, must be retried by the next call and must not mark the executor
    /// as completed. Material removal is not transactional (cursor rollback is not an undo).
    /// </summary>
    [TestFixture]
    public class ExecutorFailureTest
    {
        private const string FailureMessage = "injected command failure";

        private static BoundingBox StockBounds =>
            BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));

        private static ToolpathExecutor CreateExecutor()
        {
            return new ToolpathExecutor(
                new CutterSimulator(new VoxelGrid(StockBounds, 1.0f)),
                new EndMill(4f, 20f, isBallEnd: false),
                Vector3.Zero);
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
                "batch and step execution must remove the same voxels");
        }

        [Test]
        public void ExecuteNextSteps_ThrowingCommand_DoesNotAdvanceIndex()
        {
            var throwing = new ThrowingCommand();
            var executor = CreateExecutor();
            executor.LoadCommands(new List<IToolpathCommand>
            {
                throwing,
                new G0Move(new Vector3(5, 0, 0)),
            });

            Assert.Throws<InvalidOperationException>(() => executor.ExecuteNextSteps(1));

            Assert.That(throwing.ExecuteCalls, Is.EqualTo(1));
            Assert.That(executor.CurrentCommandIndex, Is.EqualTo(-1),
                "a throwing command must keep the cursor before it");
            Assert.That(executor.IsCompleted, Is.False);
            Assert.That(executor.CurrentPosition, Is.EqualTo(Vector3.Zero),
                "the position must not advance for a failed command");
        }

        [Test]
        public void ExecuteNextSteps_ThrowingCommand_IsRetriedOnNextCall()
        {
            var flaky = new FailOnceCommand(new Vector3(2, 0, 0));
            var executor = CreateExecutor();
            executor.LoadCommands(new List<IToolpathCommand>
            {
                flaky,
                new G0Move(new Vector3(4, 0, 0)),
            });

            Assert.Throws<InvalidOperationException>(() => executor.ExecuteNextSteps(1));
            Assert.That(executor.CurrentCommandIndex, Is.EqualTo(-1));

            Assert.That(executor.ExecuteNextSteps(1), Is.EqualTo(1),
                "the failed command must be retried, not skipped");
            Assert.That(flaky.ExecuteCalls, Is.EqualTo(2), "the same command must be re-invoked");
            Assert.That(executor.CurrentCommandIndex, Is.EqualTo(0));
            Assert.That(executor.CurrentPosition, Is.EqualTo(new Vector3(2, 0, 0)));

            Assert.That(executor.ExecuteNextSteps(1), Is.EqualTo(1));
            Assert.That(executor.CurrentPosition, Is.EqualTo(new Vector3(4, 0, 0)));
            Assert.That(executor.IsCompleted, Is.True);
        }

        [Test]
        public void ExecuteNextSteps_LastCommandFailure_IsNotCompleted()
        {
            var executor = CreateExecutor();
            executor.LoadCommands(new List<IToolpathCommand>
            {
                new G0Move(new Vector3(1, 0, 0)),
                new ThrowingCommand(),
            });

            Assert.That(executor.ExecuteNextSteps(1), Is.EqualTo(1));
            Assert.Throws<InvalidOperationException>(() => executor.ExecuteNextSteps(1));

            Assert.That(executor.CurrentCommandIndex, Is.EqualTo(0),
                "the failed final command must not be marked as executed");
            Assert.That(executor.IsCompleted, Is.False,
                "a failed final command must not complete the executor");
        }

        [Test]
        public void ExecuteNextSteps_SuccessfulCommand_ProgressFiresWithCommittedIndex()
        {
            var executor = CreateExecutor();
            var events = new List<(int done, int total, int indexAtEvent)>();
            executor.ProgressChanged += (done, total) =>
                events.Add((done, total, executor.CurrentCommandIndex));

            executor.LoadCommands(new List<IToolpathCommand>
            {
                new FailOnceCommand(new Vector3(1, 0, 0)),
                new G0Move(new Vector3(2, 0, 0)),
            });

            Assert.Throws<InvalidOperationException>(() => executor.ExecuteNextSteps(1));
            Assert.That(events, Is.Empty, "no progress must be reported for a failed command");

            Assert.That(executor.ExecuteNextSteps(1), Is.EqualTo(1));
            Assert.That(events, Has.Count.EqualTo(1));
            Assert.That(events[0], Is.EqualTo((1, 2, 0)),
                "progress must fire after the cursor is committed");
        }

        [Test]
        public void ExecuteNextSteps_CustomCommand_BatchAndStepBehaviorIsPreserved()
        {
            var tool = new EndMill(6f, 20f, isBallEnd: false);

            var batchGrid = new VoxelGrid(StockBounds, 1.0f);
            var batch = new ToolpathExecutor(new CutterSimulator(batchGrid), tool, Vector3.Zero);
            batch.ExecuteCommands(BuildCustomSequence());

            var stepGrid = new VoxelGrid(StockBounds, 1.0f);
            var step = new ToolpathExecutor(new CutterSimulator(stepGrid), tool, Vector3.Zero);
            step.LoadCommands(BuildCustomSequence());
            while (step.ExecuteNextSteps(1) > 0)
            {
            }

            Assert.That(step.CurrentPosition, Is.EqualTo(batch.CurrentPosition));
            Assert.That(step.CurrentOrientation.A, Is.EqualTo(batch.CurrentOrientation.A).Within(1e-5f));
            Assert.That(step.CurrentOrientation.B, Is.EqualTo(batch.CurrentOrientation.B).Within(1e-5f));
            Assert.That(step.CurrentOrientation.C, Is.EqualTo(batch.CurrentOrientation.C).Within(1e-5f));
            Assert.That(step.EstimatedTimeSeconds, Is.EqualTo(batch.EstimatedTimeSeconds).Within(1e-9));
            Assert.That(step.IsCompleted, Is.True);
            AssertVoxelGridsEqual(batchGrid, stepGrid);
        }

        private static List<IToolpathCommand> BuildCustomSequence()
        {
            return new List<IToolpathCommand>
            {
                new CustomCut(new Vector3(5, 0, 0)),
                new G0Move(new Vector3(5, 0, 5)),
                new CustomCut(new Vector3(10, 0, 5)),
            };
        }

        /// <summary>Command that always throws when executed.</summary>
        private sealed class ThrowingCommand : IToolpathCommand
        {
            public int ExecuteCalls { get; private set; }

            public void Execute(ICutterSimulator simulator, Tool tool, ref Vector3 currentPosition)
            {
                ExecuteCalls++;
                throw new InvalidOperationException(FailureMessage);
            }
        }

        /// <summary>Command that throws on the first call, then behaves like a rapid move.</summary>
        private sealed class FailOnceCommand : IToolpathCommand
        {
            public FailOnceCommand(Vector3 target)
            {
                Target = target;
            }

            public Vector3 Target { get; }
            public int ExecuteCalls { get; private set; }

            public void Execute(ICutterSimulator simulator, Tool tool, ref Vector3 currentPosition)
            {
                ExecuteCalls++;
                if (ExecuteCalls == 1) throw new InvalidOperationException(FailureMessage);
                currentPosition = Target;
            }
        }

        /// <summary>
        /// Custom command type that hits the generic <c>command.Execute(...)</c> path of the
        /// executor (unlike G0 / G1 / 5-axis commands, which are special-cased).
        /// </summary>
        private sealed class CustomCut : IToolpathCommand
        {
            public CustomCut(Vector3 target)
            {
                Target = target;
            }

            public Vector3 Target { get; }

            public void Execute(ICutterSimulator simulator, Tool tool, ref Vector3 currentPosition)
            {
                simulator.CutLinear(currentPosition, Target, tool);
                currentPosition = Target;
            }
        }
    }
}
