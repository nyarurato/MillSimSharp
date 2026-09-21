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
    /// Tests for 5-axis pose interpolation and sweep behavior:
    /// quaternion orientation, angular sampling, rotation-only moves, and
    /// batch/step execution consistency.
    /// </summary>
    [TestFixture]
    public class FiveAxisPoseTest
    {
        private static Vector3 Snap(BoundingBox bbox, float resolution, Vector3 world)
        {
            Vector3 local = world - bbox.Min;
            int ix = (int)MathF.Floor(local.X / resolution);
            int iy = (int)MathF.Floor(local.Y / resolution);
            int iz = (int)MathF.Floor(local.Z / resolution);
            return bbox.Min + new Vector3((ix + 0.5f) * resolution, (iy + 0.5f) * resolution, (iz + 0.5f) * resolution);
        }

        [Test]
        public void ToolOrientation_Quaternion_MatchesAxisDirections()
        {
            var samples = new[]
            {
                (a: 30f, b: -20f, c: 20f),
                (a: 15f, b: 25f, c: -35f),
                (a: 0f, b: 45f, c: 0f),
                (a: 0f, b: 0f, c: 60f),
                (a: 90f, b: 0f, c: 0f),
                (a: 350f, b: 0f, c: 0f),
            };

            foreach (var (a, b, c) in samples)
            {
                var orientation = new ToolOrientation(a, b, c);
                Quaternion q = orientation.GetQuaternion();

                Vector3 towardFromQ = Vector3.Normalize(Vector3.Transform(Vector3.UnitZ, q));
                Vector3 toward = orientation.GetAxisTowardSpindle();
                Assert.That(towardFromQ.X, Is.EqualTo(toward.X).Within(1e-5f), $"toward X A={a} B={b} C={c}");
                Assert.That(towardFromQ.Y, Is.EqualTo(toward.Y).Within(1e-5f), $"toward Y A={a} B={b} C={c}");
                Assert.That(towardFromQ.Z, Is.EqualTo(toward.Z).Within(1e-5f), $"toward Z A={a} B={b} C={c}");

                Vector3 cuttingFromQ = Vector3.Normalize(Vector3.Transform(new Vector3(0, 0, -1), q));
                Vector3 cutting = orientation.GetCuttingAxisDirection();
                Assert.That(cuttingFromQ.X, Is.EqualTo(cutting.X).Within(1e-5f), $"cutting X A={a} B={b} C={c}");
                Assert.That(cuttingFromQ.Y, Is.EqualTo(cutting.Y).Within(1e-5f), $"cutting Y A={a} B={b} C={c}");
                Assert.That(cuttingFromQ.Z, Is.EqualTo(cutting.Z).Within(1e-5f), $"cutting Z A={a} B={b} C={c}");
            }
        }

        [Test]
        public void ToolOrientation_AngularDistance_UsesShortestRotation()
        {
            Assert.That(ToolOrientation.AngularDistanceDegrees(new ToolOrientation(0, 0, 0), new ToolOrientation(90, 0, 0)),
                Is.EqualTo(90f).Within(0.01f));
            Assert.That(ToolOrientation.AngularDistanceDegrees(new ToolOrientation(350, 0, 0), new ToolOrientation(10, 0, 0)),
                Is.EqualTo(20f).Within(0.01f));
            Assert.That(ToolOrientation.AngularDistanceDegrees(new ToolOrientation(170, 0, 0), new ToolOrientation(-170, 0, 0)),
                Is.EqualTo(20f).Within(0.01f));
            Assert.That(ToolOrientation.AngularDistanceDegrees(new ToolOrientation(0, 0, 0), new ToolOrientation(180, 0, 0)),
                Is.EqualTo(180f).Within(0.01f));
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void FiveAxis_RotationOnly_PerformsSweep(float resolution)
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(30, 30, 30));
            var grid = new VoxelGrid(bbox, resolution);
            var simulator = new CutterSimulator(grid);
            var flat = new EndMill(10f, 30f, isBallEnd: false);

            simulator.CutLinearWithOrientation(
                Vector3.Zero, Vector3.Zero, flat,
                new ToolOrientation(0, 0, 0), new ToolOrientation(90, 0, 0));

            // A point covered only at an intermediate tilt angle must be removed.
            Vector3 intermediate = Snap(bbox, resolution, new Vector3(0, -6, 20));
            Assert.That(grid.GetVoxelAtWorld(intermediate), Is.False,
                "Rotation-only move must sweep the tool through intermediate orientations");

            // Below the tip stays material at every intermediate orientation.
            Vector3 below = Snap(bbox, resolution, new Vector3(0, 0, -2));
            Assert.That(grid.GetVoxelAtWorld(below), Is.True);
        }

        [TestCase(1.0f)]
        public void FiveAxis_ShortLinearLargeAngularMove_IsSubdivided(float resolution)
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(30, 30, 30));
            var grid = new VoxelGrid(bbox, resolution);
            var simulator = new CutterSimulator(grid);
            var flat = new EndMill(10f, 30f, isBallEnd: false);

            simulator.CutLinearWithOrientation(
                Vector3.Zero, new Vector3(0.05f, 0, 0), flat,
                new ToolOrientation(0, 0, 0), new ToolOrientation(90, 0, 0));

            Vector3 intermediate = Snap(bbox, resolution, new Vector3(0, -6, 20));
            Assert.That(grid.GetVoxelAtWorld(intermediate), Is.False,
                "Large angular motion must be subdivided even when linear motion is negligible");

            Vector3 below = Snap(bbox, resolution, new Vector3(0, 0, -1));
            Assert.That(grid.GetVoxelAtWorld(below), Is.True);
        }

        [TestCase(1.0f, true)]
        [TestCase(1.0f, false)]
        public void FiveAxis_DefaultOrientation_MatchesThreeAxisGeometry(float resolution, bool isBallEnd)
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(30, 30, 30));
            var tool = new EndMill(10f, 30f, isBallEnd);
            var start = new Vector3(-5, 0, 0);
            var end = new Vector3(5, 0, 0);

            var grid3 = new VoxelGrid(bbox, resolution);
            new CutterSimulator(grid3).CutLinear(start, end, tool);

            var grid5 = new VoxelGrid(bbox, resolution);
            new CutterSimulator(grid5).CutLinearWithOrientation(start, end, tool, ToolOrientation.Default, ToolOrientation.Default);

            var (sx, sy, sz) = grid3.Dimensions;
            for (int x = 0; x < sx; x++)
            for (int y = 0; y < sy; y++)
            for (int z = 0; z < sz; z++)
            {
                Assert.That(grid5.GetVoxel(x, y, z), Is.EqualTo(grid3.GetVoxel(x, y, z)),
                    $"voxel ({x},{y},{z}) differs between 3-axis and default-orientation sweep");
            }
        }

        [Test]
        public void FullExecution_vs_StepExecution_SameFinalPoseAndState()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));
            var tool = new EndMill(10f, 30f, isBallEnd: true);
            var startPos = new Vector3(0, 0, 0);

            var commands = new List<IToolpathCommand>
            {
                new G0Move5Axis(new Vector3(0, 0, 0), new ToolOrientation(30, 0, 0)),
                new G1Move5Axis(new Vector3(4, 0, 0), new ToolOrientation(30, 20, 0), feedRate: 200f),
                new G1Move5Axis(new Vector3(8, 0, 0), new ToolOrientation(45, 20, 10), feedRate: 200f),
                new G0Move5Axis(new Vector3(10, 0, 5), ToolOrientation.Default),
            };

            var gridBatch = new VoxelGrid(bbox, 1.0f);
            var executorBatch = new ToolpathExecutor(new CutterSimulator(gridBatch), tool, startPos);
            executorBatch.ExecuteCommands(commands);

            var gridStep = new VoxelGrid(bbox, 1.0f);
            var executorStep = new ToolpathExecutor(new CutterSimulator(gridStep), tool, startPos);
            executorStep.LoadCommands(commands);
            while (executorStep.CurrentCommandIndex < executorStep.TotalCommands)
            {
                if (executorStep.ExecuteNextSteps(1) == 0) break;
            }

            Assert.That(executorStep.CurrentPosition.X, Is.EqualTo(executorBatch.CurrentPosition.X).Within(1e-5f));
            Assert.That(executorStep.CurrentPosition.Y, Is.EqualTo(executorBatch.CurrentPosition.Y).Within(1e-5f));
            Assert.That(executorStep.CurrentPosition.Z, Is.EqualTo(executorBatch.CurrentPosition.Z).Within(1e-5f));

            Assert.That(executorStep.CurrentOrientation.A, Is.EqualTo(executorBatch.CurrentOrientation.A).Within(1e-5f));
            Assert.That(executorStep.CurrentOrientation.B, Is.EqualTo(executorBatch.CurrentOrientation.B).Within(1e-5f));
            Assert.That(executorStep.CurrentOrientation.C, Is.EqualTo(executorBatch.CurrentOrientation.C).Within(1e-5f));

            var (sx, sy, sz) = gridBatch.Dimensions;
            int differences = 0;
            for (int x = 0; x < sx; x++)
            for (int y = 0; y < sy; y++)
            for (int z = 0; z < sz; z++)
            {
                if (gridBatch.GetVoxel(x, y, z) != gridStep.GetVoxel(x, y, z)) differences++;
            }
            Assert.That(differences, Is.EqualTo(0), "Batch and step execution must produce identical voxel states");
        }
    }
}
