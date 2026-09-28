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

        private static float RotationAngleDegrees(ToolOrientation orientation)
        {
            Quaternion q = orientation.GetQuaternion();
            return 2f * MathF.Acos(Math.Clamp(MathF.Abs(q.W), -1f, 1f)) * 180f / MathF.PI;
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

        // ---------------------------------------------------------------------
        // Tool axis (IJK-style) pose input
        // ---------------------------------------------------------------------

        [Test]
        public void Orientation_FromAxis_ZeroOrNonFinite_Throws()
        {
            Assert.Throws<ArgumentException>(() => ToolOrientation.FromAxisTowardSpindle(Vector3.Zero));
            Assert.Throws<ArgumentException>(() =>
                ToolOrientation.FromAxisTowardSpindle(new Vector3(float.NaN, 0, 1)));
            Assert.Throws<ArgumentException>(() =>
                ToolOrientation.FromAxisTowardSpindle(new Vector3(0, 0, float.PositiveInfinity)));
        }

        [Test]
        public void Orientation_FromAxis_MatchesEulerRoundTrip()
        {
            var axes = new[]
            {
                Vector3.UnitZ,
                Vector3.UnitX,
                -Vector3.UnitX,
                new Vector3(0, 1, 0),
                Vector3.Normalize(new Vector3(1, 1, 1)),
                Vector3.Normalize(new Vector3(-2, 0.5f, 3)),
            };

            foreach (Vector3 axis in axes)
            {
                ToolOrientation orientation = ToolOrientation.FromAxisTowardSpindle(axis);
                Vector3 roundTrip = orientation.GetAxisTowardSpindle();

                Assert.That(roundTrip.X, Is.EqualTo(axis.X).Within(1e-5f), $"X for {axis}");
                Assert.That(roundTrip.Y, Is.EqualTo(axis.Y).Within(1e-5f), $"Y for {axis}");
                Assert.That(roundTrip.Z, Is.EqualTo(axis.Z).Within(1e-5f), $"Z for {axis}");

                // The rotation must be the shortest one: angle = acos(u . +Z).
                float expectedAngle = MathF.Acos(Math.Clamp(axis.Z, -1f, 1f)) * 180f / MathF.PI;
                Assert.That(RotationAngleDegrees(orientation), Is.EqualTo(expectedAngle).Within(0.05f),
                    $"shortest rotation angle for {axis}");
            }

            // Hand-derived case: a 45 degree tilt around X has the canonical A = 45, B = C = 0.
            var tiltedAxis = new Vector3(0, -MathF.Sqrt(0.5f), MathF.Sqrt(0.5f));
            ToolOrientation tilted = ToolOrientation.FromAxisTowardSpindle(tiltedAxis);
            Assert.That(tilted.A, Is.EqualTo(45f).Within(1e-3f));
            Assert.That(tilted.B, Is.EqualTo(0f).Within(1e-3f));
            Assert.That(tilted.C, Is.EqualTo(0f).Within(1e-3f));
        }

        [Test]
        public void Orientation_FromAxis_UsesShortestRotation()
        {
            // Review regression: (sqrt(1/2), -sqrt(1/2), 0) returned the right axis but with a
            // 98.4 degree rotation; the shortest rotation is 90 degrees.
            Vector3 axis = Vector3.Normalize(new Vector3(1, -1, 0));

            ToolOrientation orientation = ToolOrientation.FromAxisTowardSpindle(axis);

            Assert.That(RotationAngleDegrees(orientation), Is.EqualTo(90f).Within(0.01f));

            Vector3 roundTrip = orientation.GetAxisTowardSpindle();
            Assert.That(roundTrip.X, Is.EqualTo(axis.X).Within(1e-5f));
            Assert.That(roundTrip.Y, Is.EqualTo(axis.Y).Within(1e-5f));
            Assert.That(roundTrip.Z, Is.EqualTo(axis.Z).Within(1e-5f));
        }

        [Test]
        public void Orientation_FromAxis_TinyAndHugeAxes_AreNormalized()
        {
            // Review regression: finite non-zero axes must be accepted. The previous float
            // normalization rejected tiny axes and overflowed for huge components.
            var axes = new[]
            {
                new Vector3(1e-7f, 0, 0),
                new Vector3(1e-45f, 0, 0), // subnormal but non-zero
                new Vector3(1e20f, 0, 0),
                new Vector3(0, 1e20f, 0),
            };

            foreach (Vector3 axis in axes)
            {
                ToolOrientation orientation = ToolOrientation.FromAxisTowardSpindle(axis);
                Vector3 roundTrip = orientation.GetAxisTowardSpindle();
                var expected = Vector3.Normalize(
                    new Vector3(MathF.Sign(axis.X), MathF.Sign(axis.Y), MathF.Sign(axis.Z)));

                Assert.That(roundTrip.X, Is.EqualTo(expected.X).Within(1e-5f), $"X for {axis}");
                Assert.That(roundTrip.Y, Is.EqualTo(expected.Y).Within(1e-5f), $"Y for {axis}");
                Assert.That(roundTrip.Z, Is.EqualTo(expected.Z).Within(1e-5f), $"Z for {axis}");
            }
        }

        [Test]
        public void Orientation_FromAxis_NearAntipodal_PreservesDirection()
        {
            // Review regression: the input Z component rounds to -1f, which previously matched the
            // pinned antipodal case and collapsed the direction (the X component was lost). Only the
            // exact antipodal axis may be pinned; near-antipodal axes keep their direction.
            var axis = new Vector3(0.0001f, 0f, -MathF.Sqrt(1f - 0.0001f * 0.0001f));
            Vector3 expected = Vector3.Normalize(axis);

            ToolOrientation orientation = ToolOrientation.FromAxisTowardSpindle(axis);
            Vector3 roundTrip = orientation.GetAxisTowardSpindle();

            Assert.That(roundTrip.X, Is.EqualTo(expected.X).Within(1e-5f));
            Assert.That(roundTrip.Y, Is.EqualTo(expected.Y).Within(1e-5f));
            Assert.That(roundTrip.Z, Is.EqualTo(expected.Z).Within(1e-5f));

            // The near-antipodal rotation must still be the shortest one.
            float expectedAngle = MathF.Acos(Math.Clamp(expected.Z, -1f, 1f)) * 180f / MathF.PI;
            Assert.That(RotationAngleDegrees(orientation), Is.EqualTo(expectedAngle).Within(0.01f));
        }

        [Test]
        public void Orientation_180DegreeFlip_IsDeterministic()
        {
            // The antipodal direction has no unique shortest rotation: pin the construction
            // (180 degree X flip) and require deterministic, repeatable results.
            ToolOrientation flipped = ToolOrientation.FromAxisTowardSpindle(-Vector3.UnitZ);
            var canonical = new ToolOrientation(180, 0, 0);

            Assert.That(MathF.Abs(Quaternion.Dot(flipped.GetQuaternion(), canonical.GetQuaternion())),
                Is.GreaterThan(0.99999f), "the antipodal pose must be the 180 degree X flip");

            ToolOrientation midFirst = ToolOrientation.Slerp(ToolOrientation.Default, canonical, 0.5f);
            ToolOrientation midSecond = ToolOrientation.Slerp(ToolOrientation.Default, canonical, 0.5f);

            Assert.That(MathF.Abs(Quaternion.Dot(midFirst.GetQuaternion(), midSecond.GetQuaternion())),
                Is.GreaterThan(0.99999f), "180 degree slerp must be deterministic");

            // Both +/-90 degrees around X are equally short half-way poses; the quaternion sign picks
            // one of them, so only the sign-agnostic property (a horizontal pose) is pinned here.
            Vector3 midAxis = midFirst.GetAxisTowardSpindle();
            Assert.That(midAxis.X, Is.EqualTo(0f).Within(1e-4f));
            Assert.That(MathF.Abs(midAxis.Y), Is.EqualTo(1f).Within(1e-4f));
            Assert.That(midAxis.Z, Is.EqualTo(0f).Within(1e-4f));
        }

        [Test]
        public void Orientation_RollOnly_DoesNotChangeCut()
        {
            // With A = B = 0 the C angle is a pure roll about the tool axis. Cutting geometries are
            // solids of revolution, so the removal must be identical for both backends.
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(30, 30, 30));
            var tool = new EndMill(10f, 30f, isBallEnd: false);
            var start = new Vector3(0, 0, 10);
            var end = new Vector3(8, 0, 10);

            var reference = new VoxelGrid(bbox, 1.0f);
            new CutterSimulator(reference).CutLinearWithOrientation(start, end, tool,
                new ToolOrientation(0, 0, 0), new ToolOrientation(0, 0, 0));

            var rolled = new VoxelGrid(bbox, 1.0f);
            new CutterSimulator(rolled).CutLinearWithOrientation(start, end, tool,
                new ToolOrientation(0, 0, 45), new ToolOrientation(0, 0, 90));

            var (sx, sy, sz) = reference.Dimensions;
            int differences = 0;
            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                        if (reference.GetVoxel(x, y, z) != rolled.GetVoxel(x, y, z)) differences++;

            Assert.That(differences, Is.EqualTo(0),
                "roll-only orientation changes must not change the removed voxels");

            var referenceSdf = new SDFGrid(bbox, 1.0f, narrowBandWidth: 4);
            new SDFCutterSimulator(referenceSdf).CutLinearWithOrientation(start, end, tool,
                new ToolOrientation(0, 0, 0), new ToolOrientation(0, 0, 0));

            var rolledSdf = new SDFGrid(bbox, 1.0f, narrowBandWidth: 4);
            new SDFCutterSimulator(rolledSdf).CutLinearWithOrientation(start, end, tool,
                new ToolOrientation(0, 0, 45), new ToolOrientation(0, 0, 90));

            int signDifferences = 0;
            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                        if ((referenceSdf.GetDistance(x, y, z) < 0f) != (rolledSdf.GetDistance(x, y, z) < 0f))
                            signDifferences++;

            Assert.That(signDifferences, Is.EqualTo(0),
                "roll-only orientation changes must not change the SDF sign pattern");
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

        [Test]
        public void FiveAxis_SmallOrientationChange_RemovesMaterialAtTheEndPose()
        {
            // Review regression: a 0.1 degree orientation change used to take the exact-swept-solid
            // fast path, which swept only the start pose. With a 100 mm tool the top of the tool
            // moves about 0.17 mm, so material covered only by the end pose was missed.
            const float resolution = 0.1f;
            var tool = new EndMill(6f, 100f, isBallEnd: false);
            var start = new Vector3(0, 0, 0);
            var end = new Vector3(10, 0, 0);

            // Voxel center (9.9, -3.1, 99.0): outside the start-pose swept solid (radius 3 around
            // y = 0) but inside the end pose rotated by A = 0.1 degrees (the top shifts toward -Y).
            var bounds = new BoundingBox(new Vector3(9.85f, -3.15f, 98.95f), new Vector3(11.05f, -2.75f, 99.15f));
            var probe = new Vector3(9.9f, -3.1f, 99.0f);

            var tiltedGrid = new VoxelGrid(bounds, resolution);
            new CutterSimulator(tiltedGrid).CutLinearWithOrientation(
                start, end, tool, ToolOrientation.Default, new ToolOrientation(0.1f, 0, 0));

            Assert.That(tiltedGrid.GetVoxelAtWorld(probe), Is.False,
                "material covered only by the end pose must be removed");

            // Negative control: without the tilt the probe is outside the swept solid.
            var straightGrid = new VoxelGrid(bounds, resolution);
            new CutterSimulator(straightGrid).CutLinearWithOrientation(
                start, end, tool, ToolOrientation.Default, ToolOrientation.Default);

            Assert.That(straightGrid.GetVoxelAtWorld(probe), Is.True,
                "the probe must remain material without the orientation change");
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

        [Test]
        public void ToolOrientation_QuaternionRoundTrip_PreservesOrientation()
        {
            var samples = new[]
            {
                (a: 30f, b: -20f, c: 20f),
                (a: 15f, b: 25f, c: -35f),
                (a: 0f, b: 45f, c: 0f),
                (a: 0f, b: 0f, c: 60f),
                (a: 90f, b: 0f, c: 0f),
            };

            foreach (var (a, b, c) in samples)
            {
                var orientation = new ToolOrientation(a, b, c);
                var roundTrip = ToolOrientation.FromQuaternion(orientation.GetQuaternion());

                Vector3 expected = orientation.GetAxisTowardSpindle();
                Vector3 actual = roundTrip.GetAxisTowardSpindle();
                Assert.That(actual.X, Is.EqualTo(expected.X).Within(1e-4f), $"A={a} B={b} C={c}");
                Assert.That(actual.Y, Is.EqualTo(expected.Y).Within(1e-4f), $"A={a} B={b} C={c}");
                Assert.That(actual.Z, Is.EqualTo(expected.Z).Within(1e-4f), $"A={a} B={b} C={c}");
            }
        }

        [Test]
        public void ToolOrientation_Slerp_UsesShortestArc()
        {
            var start = new ToolOrientation(350, 0, 0);
            var end = new ToolOrientation(10, 0, 0);

            var midpoint = ToolOrientation.Slerp(start, end, 0.5f);
            Vector3 axis = midpoint.GetAxisTowardSpindle();

            // The shortest arc goes through A = 0 (vertical tool)
            Assert.That(axis.X, Is.EqualTo(0f).Within(1e-4f));
            Assert.That(axis.Y, Is.EqualTo(0f).Within(1e-4f));
            Assert.That(axis.Z, Is.EqualTo(1f).Within(1e-4f));
        }

        [Test]
        public void FiveAxis_LongFlatTool_AdaptiveRefinement_SubdividesRotationOnlyMove()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(60, 60, 60));
            var grid = new VoxelGrid(bbox, 1.0f);
            var simulator = new CutterSimulator(grid);
            var flat = new EndMill(8f, 30f, isBallEnd: false); // radius 4, length 30

            // Without adaptive refinement (radius 0 for flat tools) these settings would sample
            // only the two endpoint poses (angular step 180 degrees).
            simulator.Settings.MaxLinearStep = 1000f;
            simulator.Settings.MaxAngularStep = 180f;
            simulator.Settings.MaxChordError = 0.05f;
            simulator.Settings.EnableAdaptiveSampling = true;

            // Without the rotation sweep radius the move would use a single step.
            Assert.That(simulator.Settings.ComputeSteps(0f, 90f, 0f), Is.EqualTo(1));

            simulator.CutLinearWithOrientation(
                Vector3.Zero, Vector3.Zero, flat,
                new ToolOrientation(0, 0, 0), new ToolOrientation(90, 0, 0));

            // This point lies on the tool axis at 45 degrees and is only covered when the
            // rotation-only move is subdivided into intermediate poses.
            Vector3 intermediate = new Vector3(0, -14.5f, 14.5f);
            Assert.That(grid.GetVoxelAtWorld(intermediate), Is.False,
                "A long flat tool must refine rotation-only moves adaptively");

            // Below the tip stays material at every sampled orientation.
            Assert.That(grid.GetVoxelAtWorld(new Vector3(0, 0, -2.5f)), Is.True);
        }

        [Test]
        public void StraightSweep_IsExact_EvenWithLargeLinearStep()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(30, 30, 30));
            var grid = new VoxelGrid(bbox, 1.0f);
            var simulator = new CutterSimulator(grid);
            simulator.Settings.MaxLinearStep = 1000f; // discrete sampling would only use the endpoints

            var flat = new EndMill(10f, 30f, isBallEnd: false);
            simulator.CutLinear(new Vector3(-5, 0, 0), new Vector3(5, 0, 0), flat);

            foreach (float x in new[] { -2.5f, 0f, 2.5f })
            {
                Assert.That(grid.GetVoxelAtWorld(new Vector3(x, 0, 0)), Is.False,
                    $"x={x} must be removed by the exact swept solid");
            }
        }

        [Test]
        public void SdfStraightSweep_IsExact_EvenWithLargeLinearStep()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(30, 30, 30));
            var sdf = new SDFGrid(bbox, 1.0f, narrowBandWidth: 10);
            var simulator = new SDFCutterSimulator(sdf);
            simulator.Settings.MaxLinearStep = 1000f;

            var flat = new EndMill(10f, 30f, isBallEnd: false);
            simulator.CutLinear(new Vector3(-5, 0, 0), new Vector3(5, 0, 0), flat);

            foreach (float x in new[] { -2.5f, 0f, 2.5f })
            {
                Assert.That(sdf.GetDistance(new Vector3(x, 0, 0.5f)), Is.GreaterThan(0f),
                    $"x={x} must be removed by the exact swept solid");
                Assert.That(sdf.GetDistance(new Vector3(x, 0, -1f)), Is.LessThan(0f),
                    $"x={x} below the tool must remain material");
            }
        }
    }
}
