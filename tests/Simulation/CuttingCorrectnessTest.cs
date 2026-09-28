using System;
using System.Collections.Generic;
using System.Numerics;
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using MillSimSharp.Tests.Reference;
using MillSimSharp.Toolpath;
using NUnit.Framework;

namespace MillSimSharp.Tests.Simulation
{
    /// <summary>
    /// Cutting-shape correctness tests. These verify not only what is removed but also what must
    /// be preserved, using an independent test-side oracle (<see cref="ReferenceCutEvaluator"/>).
    /// The canonical input reference point is the physical tool tip.
    /// </summary>
    [TestFixture]
    public class CuttingCorrectnessTest
    {
        private const float Radius = 5f;
        private const float ToolLength = 30f;

        private static BoundingBox StockBounds => BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));

        private static EndMill BallTool => new EndMill(Radius * 2f, ToolLength, isBallEnd: true);

        private static EndMill FlatTool => new EndMill(Radius * 2f, ToolLength, isBallEnd: false);

        private static Vector3 VoxelCenter(BoundingBox bbox, float resolution, int ix, int iy, int iz)
        {
            return bbox.Min + new Vector3((ix + 0.5f) * resolution, (iy + 0.5f) * resolution, (iz + 0.5f) * resolution);
        }

        private static Vector3 SnapToVoxelCenter(BoundingBox bbox, float resolution, Vector3 world)
        {
            Vector3 local = world - bbox.Min;
            int ix = (int)MathF.Floor(local.X / resolution);
            int iy = (int)MathF.Floor(local.Y / resolution);
            int iz = (int)MathF.Floor(local.Z / resolution);
            return VoxelCenter(bbox, resolution, ix, iy, iz);
        }

        private static List<string> ScanMismatches(
            VoxelGrid grid, BoundingBox bbox, float resolution,
            Vector3 regionMin, Vector3 regionMax,
            Func<Vector3, bool> expectedRemoved)
        {
            var mismatches = new List<string>();
            var dense = grid.ToDenseArray();
            var (sx, sy, sz) = grid.Dimensions;

            for (int z = 0; z < sz; z++)
                for (int y = 0; y < sy; y++)
                    for (int x = 0; x < sx; x++)
                    {
                        Vector3 center = VoxelCenter(bbox, resolution, x, y, z);
                        if (center.X < regionMin.X || center.X > regionMax.X ||
                            center.Y < regionMin.Y || center.Y > regionMax.Y ||
                            center.Z < regionMin.Z || center.Z > regionMax.Z)
                        {
                            continue;
                        }

                        bool expected = expectedRemoved(center);
                        bool actual = !dense[x][y][z];
                        if (expected != actual && mismatches.Count < 10)
                        {
                            mismatches.Add($"center=({center.X:F3},{center.Y:F3},{center.Z:F3}) expectedRemoved={expected} actualRemoved={actual}");
                        }
                    }

            return mismatches;
        }

        private static float MinRemovedZ(VoxelGrid grid)
        {
            var removed = ReferenceCutEvaluator.CollectRemovedVoxelCenters(grid);
            Assert.That(removed, Is.Not.Empty, "The cut should have removed material");
            float minZ = float.MaxValue;
            foreach (var p in removed)
            {
                if (p.Z < minZ) minZ = p.Z;
            }
            return minZ;
        }

        // ---------------------------------------------------------------------
        // Ball end mill - static pose
        // ---------------------------------------------------------------------

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        [TestCase(0.25f)]
        public void BallEndMill_StaticPose_RemovesExpectedSphereRegion(float resolution)
        {
            var bbox = StockBounds;
            var grid = new VoxelGrid(bbox, resolution);
            new CutterSimulator(grid).CutPoint(Vector3.Zero, BallTool);

            var mismatches = ScanMismatches(
                grid, bbox, resolution,
                new Vector3(-6, -6, -1), new Vector3(6, 6, 11),
                p => ReferenceCutEvaluator.IsInsideBallCutPointTool(p, Vector3.Zero, Radius, ToolLength));

            Assert.That(mismatches, Is.Empty, string.Join(Environment.NewLine, mismatches));
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        [TestCase(0.25f)]
        public void BallEndMill_StaticPose_DoesNotCutBelowPhysicalTip(float resolution)
        {
            var bbox = StockBounds;
            var grid = new VoxelGrid(bbox, resolution);
            new CutterSimulator(grid).CutPoint(Vector3.Zero, BallTool);

            // Points below the physical tip (or beyond the ball) must stay material.
            var probes = new[]
            {
                new Vector3(0, 0, -4.5f),
                new Vector3(0, 0, -1f),
                new Vector3(3, 0, -2f),
                new Vector3(6.5f, 0, 5f),
            };

            foreach (var probe in probes)
            {
                Vector3 center = SnapToVoxelCenter(bbox, resolution, probe);
                Assert.That(ReferenceCutEvaluator.IsInsideBallCutPointTool(center, Vector3.Zero, Radius, ToolLength),
                    Is.False, $"probe {probe} should be outside the reference tool solid");

                Assert.That(grid.GetVoxelAtWorld(center), Is.True,
                    $"Regression (A12): material at {probe} must be preserved; the ball center is not the physical tip");
            }
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        [TestCase(0.25f)]
        public void BallEndMill_StaticPose_RemovedBoundsDoNotExtendBelowTip(float resolution)
        {
            var bbox = StockBounds;
            var grid = new VoxelGrid(bbox, resolution);
            new CutterSimulator(grid).CutPoint(Vector3.Zero, BallTool);

            float minRemovedZ = MinRemovedZ(grid);
            Assert.That(minRemovedZ, Is.GreaterThanOrEqualTo(-0.5f * resolution),
                "Removed region must not extend below the physical tip");
        }

        [Test]
        public void BallEndMill_StaticPose_UsesCorrectBallCenter()
        {
            var bbox = StockBounds;
            var grid = new VoxelGrid(bbox, 1.0f);
            new CutterSimulator(grid).CutPoint(Vector3.Zero, BallTool);

            // Below the tip: removed by the old (buggy) tip-centered sphere, preserved by the correct ball.
            Assert.That(grid.GetVoxelAtWorld(new Vector3(0, 0, -4.5f)), Is.True);

            // Top of the correct ball must be removed (verified via the reference ball center).
            Vector3 ballCenter = ReferenceCutEvaluator.ExpectedBallCenter(Vector3.Zero, 0, 0, 0, Radius);
            Vector3 nearTop = SnapToVoxelCenter(bbox, 1.0f, ballCenter + new Vector3(0, 0, Radius - 1f));
            Assert.That(grid.GetVoxelAtWorld(nearTop), Is.False);
        }

        // ---------------------------------------------------------------------
        // Ball end mill - 3-axis linear sweep
        // ---------------------------------------------------------------------

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void BallEndMill_LinearSweep_RemovesExpectedRegion(float resolution)
        {
            var bbox = StockBounds;
            var grid = new VoxelGrid(bbox, resolution);
            var start = new Vector3(0, 0, 0);
            var end = new Vector3(10, 0, 0);
            new CutterSimulator(grid).CutLinear(start, end, BallTool);

            Vector3 centerStart = start + new Vector3(0, 0, Radius);
            Vector3 centerEnd = end + new Vector3(0, 0, Radius);

            var mismatches = new List<string>();
            var dense = grid.ToDenseArray();
            var (sx, sy, sz) = grid.Dimensions;
            int removedInside = 0;

            for (int z = 0; z < sz; z++)
                for (int y = 0; y < sy; y++)
                    for (int x = 0; x < sx; x++)
                    {
                        Vector3 center = VoxelCenter(bbox, resolution, x, y, z);
                        if (center.X < -1f || center.X > 11f || center.Y < -6f || center.Y > 6f ||
                            center.Z < -1f || center.Z > 11f)
                        {
                            continue;
                        }

                        bool actualRemoved = !dense[x][y][z];
                        bool insideBallSweep = ReferenceCutEvaluator.IsInsideCapsule(center, centerStart, centerEnd, Radius);

                        if (insideBallSweep)
                        {
                            if (!actualRemoved && mismatches.Count < 10)
                                mismatches.Add($"expectedRemoved center=({center.X:F3},{center.Y:F3},{center.Z:F3})");
                            if (actualRemoved) removedInside++;
                        }
                        else if (center.Z < -0.5f * resolution && actualRemoved && mismatches.Count < 10)
                        {
                            mismatches.Add($"expectedPreserved center=({center.X:F3},{center.Y:F3},{center.Z:F3})");
                        }
                    }

            Assert.That(removedInside, Is.GreaterThan(50), "The ball sweep should remove a substantial region");
            Assert.That(mismatches, Is.Empty, string.Join(Environment.NewLine, mismatches));
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        [TestCase(0.25f)]
        public void BallEndMill_LinearSweep_DoesNotCutBelowPhysicalTipEnvelope(float resolution)
        {
            var bbox = StockBounds;
            var grid = new VoxelGrid(bbox, resolution);
            new CutterSimulator(grid).CutLinear(new Vector3(0, 0, 0), new Vector3(10, 0, 0), BallTool);

            float minRemovedZ = MinRemovedZ(grid);
            Assert.That(minRemovedZ, Is.GreaterThanOrEqualTo(-0.5f * resolution),
                "The swept ball must not remove material below the physical tip envelope");
        }

        // ---------------------------------------------------------------------
        // Flat end mill - static pose
        // ---------------------------------------------------------------------

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        [TestCase(0.25f)]
        public void FlatEndMill_StaticPose_RemovesInsideRadius(float resolution)
        {
            var bbox = StockBounds;
            var grid = new VoxelGrid(bbox, resolution);
            new CutterSimulator(grid).CutPoint(Vector3.Zero, FlatTool);

            var mismatches = ScanMismatches(
                grid, bbox, resolution,
                new Vector3(-6, -6, -1), new Vector3(6, 6, 11),
                p => ReferenceCutEvaluator.IsInsideFlatCutPointTool(p, Vector3.Zero, Radius, ToolLength));

            Assert.That(mismatches, Is.Empty, string.Join(Environment.NewLine, mismatches));
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void FlatEndMill_StaticPose_PreservesOutsideRadius(float resolution)
        {
            var bbox = StockBounds;
            var grid = new VoxelGrid(bbox, resolution);
            new CutterSimulator(grid).CutPoint(Vector3.Zero, FlatTool);

            var probes = new[]
            {
                new Vector3(5.5f, 0, 0.5f),  // outside radius
                new Vector3(0, 0, -1f),      // below the flat bottom
                new Vector3(3, 0, -2f),      // outside radius and below the bottom
            };

            foreach (var probe in probes)
            {
                Vector3 center = SnapToVoxelCenter(bbox, resolution, probe);
                Assert.That(ReferenceCutEvaluator.IsInsideFlatCutPointTool(center, Vector3.Zero, Radius, ToolLength),
                    Is.False, $"probe {probe} should be outside the reference tool solid");
                Assert.That(grid.GetVoxelAtWorld(center), Is.True, $"material at {probe} must be preserved");
            }
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void FlatEndMill_StaticPose_DoesNotUseSphereApproximation(float resolution)
        {
            var bbox = StockBounds;
            var grid = new VoxelGrid(bbox, resolution);
            new CutterSimulator(grid).CutPoint(Vector3.Zero, FlatTool);

            // This point lies within a tip-centered sphere of radius R but outside the flat cylinder
            // (below the flat bottom). It must be preserved.
            Vector3 center = SnapToVoxelCenter(bbox, resolution, new Vector3(3, 0, -2f));
            Assert.That(Vector3.Distance(center, Vector3.Zero), Is.LessThan(Radius),
                "probe must lie inside the legacy sphere approximation to be a regression check");
            Assert.That(grid.GetVoxelAtWorld(center), Is.True,
                "A flat end mill must not be approximated by a sphere centered at the tip");
        }

        // ---------------------------------------------------------------------
        // SDF backend - static pose
        // ---------------------------------------------------------------------

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        [TestCase(0.25f)]
        public void SDFBallEndMill_StaticPose_RemovesExpectedSphereRegion(float resolution)
        {
            var bbox = StockBounds;
            var sdf = new SDFGrid(bbox, resolution, narrowBandWidth: 10);
            new SDFCutterSimulator(sdf).CutPoint(Vector3.Zero, BallTool);

            Vector3 ballCenter = ReferenceCutEvaluator.ExpectedBallCenter(Vector3.Zero, 0, 0, 0, Radius);
            Vector3 top = Vector3.Zero + new Vector3(0, 0, Math.Max(ToolLength, Radius));

            int checkedInside = 0;
            int checkedBelowTip = 0;
            var (sx, sy, sz) = sdf.Dimensions;
            int stride = resolution >= 1f ? 1 : resolution >= 0.5f ? 2 : 4;

            for (int z = 0; z < sz; z += stride)
                for (int y = 0; y < sy; y += stride)
                    for (int x = 0; x < sx; x += stride)
                    {
                        Vector3 center = VoxelCenter(bbox, resolution, x, y, z);
                        if (center.X < -6f || center.X > 6f || center.Y < -6f || center.Y > 6f ||
                            center.Z < -3f || center.Z > 11f)
                        {
                            continue;
                        }

                        float distanceToBallCenter = Vector3.Distance(center, ballCenter);

                        if (distanceToBallCenter < Radius - 0.5f * resolution)
                        {
                            Assert.That(sdf.GetDistance(center), Is.GreaterThan(0f),
                                $"inside ball center={center} must be empty (positive)");
                            checkedInside++;
                        }
                        else if (center.Z < -0.5f * resolution && !ReferenceCutEvaluator.IsInsideCylinder(center, ballCenter, top, Radius))
                        {
                            // Below the physical tip: must remain material even though the old bug removed it.
                            Assert.That(Vector3.Distance(center, ballCenter), Is.GreaterThan(Radius));
                            Assert.That(sdf.GetDistance(center), Is.LessThan(0f),
                                $"below physical tip center={center} must remain material (negative)");
                            checkedBelowTip++;
                        }
                    }

            Assert.That(checkedInside, Is.GreaterThan(20), "Not enough inside-ball samples were checked");
            Assert.That(checkedBelowTip, Is.GreaterThan(5), "Not enough below-tip samples were checked");
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        [TestCase(0.25f)]
        public void SDFBallEndMill_StaticPose_PreservesOutsideSphereRegion(float resolution)
        {
            var bbox = StockBounds;
            var sdf = new SDFGrid(bbox, resolution, narrowBandWidth: 10);
            new SDFCutterSimulator(sdf).CutPoint(Vector3.Zero, BallTool);

            // Regression points for A12: these would be positive (empty) if the ball center were
            // mistaken for the physical tip.
            var probes = new[]
            {
                new Vector3(0, 0, -4.5f),
                new Vector3(4, 0, 0),
                new Vector3(3, 0, -2f),
            };

            foreach (var probe in probes)
            {
                Assert.That(ReferenceCutEvaluator.IsInsideBall(probe, Vector3.Zero, Radius), Is.False,
                    $"probe {probe} must be outside the correct ball");
                Assert.That(sdf.GetDistance(probe), Is.LessThan(0f),
                    $"material at {probe} must be preserved (negative)");
            }
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void SDFFlatEndMill_StaticPose_HasFlatBottom(float resolution)
        {
            var bbox = StockBounds;
            var sdf = new SDFGrid(bbox, resolution, narrowBandWidth: 10);
            new SDFCutterSimulator(sdf).CutPoint(Vector3.Zero, FlatTool);

            Assert.That(sdf.GetDistance(new Vector3(0, 0, -1f)), Is.LessThan(0f),
                "Material below the flat bottom must be preserved");
            Assert.That(sdf.GetDistance(new Vector3(0, 0, 1f)), Is.GreaterThan(0f),
                "Material inside the tool must be removed");
            Assert.That(sdf.GetDistance(new Vector3(4, 0, 2f)), Is.GreaterThan(0f),
                "Material inside the tool radius must be removed");
            Assert.That(sdf.GetDistance(new Vector3(6, 0, 2f)), Is.LessThan(0f),
                "Material outside the tool radius must be preserved");
        }

        // ---------------------------------------------------------------------
        // Reference evaluator self-check
        // ---------------------------------------------------------------------

        [Test]
        public void ReferenceSurfaceSample_SignedDistanceIsZero_IsNotInside()
        {
            Assert.That(ReferenceCutEvaluator.ReferenceSignedDistanceBall(new Vector3(5, 0, 0), Vector3.Zero, 5f),
                Is.EqualTo(0f).Within(1e-5f));
            Assert.That(ReferenceCutEvaluator.IsInsideBall(new Vector3(5, 0, 5), Vector3.Zero, 5f), Is.False,
                "Surface samples must not be treated as inside (signedDistance < 0 convention)");

            var start = new Vector3(0, 0, 0);
            var end = new Vector3(0, 0, 10);

            Assert.That(ReferenceCutEvaluator.IsInsideCylinder(new Vector3(5, 0, 5), start, end, 5f), Is.False,
                "Cylinder side surface");
            Assert.That(ReferenceCutEvaluator.IsInsideCylinder(new Vector3(2, 0, 0), start, end, 5f), Is.False,
                "Cylinder cap surface");
            Assert.That(ReferenceCutEvaluator.IsInsideCapsule(new Vector3(0, 0, -5), start, end, 5f), Is.False,
                "Capsule end-cap surface");

            Assert.That(ReferenceCutEvaluator.IsInsideBall(new Vector3(4.9f, 0, 5f), Vector3.Zero, 5f), Is.True);
            Assert.That(ReferenceCutEvaluator.IsInsideCylinder(new Vector3(4.9f, 0, 5), start, end, 5f), Is.True);
        }

        [Test]
        public void ReferenceSignedDistance_MatchesInsidePredicates()
        {
            var start = new Vector3(0, 0, 0);
            var end = new Vector3(0, 0, 10);

            for (float x = -6f; x <= 6f; x += 1.3f)
                for (float y = -6f; y <= 6f; y += 1.3f)
                    for (float z = -6f; z <= 14f; z += 1.3f)
                    {
                        var p = new Vector3(x, y, z);

                        Assert.That(ReferenceCutEvaluator.IsInsideCylinder(p, start, end, 3f),
                            Is.EqualTo(ReferenceCutEvaluator.ReferenceSignedDistanceCylinder(p, start, end, 3f) < 0f));
                        Assert.That(ReferenceCutEvaluator.IsInsideCapsule(p, start, end, 3f),
                            Is.EqualTo(ReferenceCutEvaluator.ReferenceSignedDistanceCapsule(p, start, end, 3f) < 0f));
                        Assert.That(ReferenceCutEvaluator.IsInsideBall(p, Vector3.Zero, 3f),
                            Is.EqualTo(ReferenceCutEvaluator.ReferenceSignedDistanceBall(p, new Vector3(0, 0, 3f), 3f) < 0f));
                        Assert.That(ReferenceCutEvaluator.IsInsideBallCutPointTool(p, Vector3.Zero, 3f, 10f),
                            Is.EqualTo(ReferenceCutEvaluator.ReferenceSignedDistanceBallTool(p, Vector3.Zero, 0f, 0f, 0f, 3f, 10f) < 0f));
                        Assert.That(ReferenceCutEvaluator.IsInsideFlatCutPointTool(p, Vector3.Zero, 3f, 10f),
                            Is.EqualTo(ReferenceCutEvaluator.ReferenceSignedDistanceFlatTool(p, Vector3.Zero, 0f, 0f, 0f, 3f, 10f) < 0f));
                    }
        }

        [Test]
        public void ReferenceEvaluator_AxisDirections_MatchHandDerivedValues()
        {
            var def = ReferenceCutEvaluator.ExpectedCuttingAxisDirection(0, 0, 0);
            Assert.That(def.X, Is.EqualTo(0f).Within(1e-5f));
            Assert.That(def.Y, Is.EqualTo(0f).Within(1e-5f));
            Assert.That(def.Z, Is.EqualTo(-1f).Within(1e-5f));

            var a90 = ReferenceCutEvaluator.ExpectedCuttingAxisDirection(90, 0, 0);
            Assert.That(a90.X, Is.EqualTo(0f).Within(1e-5f));
            Assert.That(a90.Y, Is.EqualTo(1f).Within(1e-5f));
            Assert.That(a90.Z, Is.EqualTo(0f).Within(1e-5f));

            var b90 = ReferenceCutEvaluator.ExpectedCuttingAxisDirection(0, 90, 0);
            Assert.That(b90.X, Is.EqualTo(-1f).Within(1e-5f));
            Assert.That(b90.Y, Is.EqualTo(0f).Within(1e-5f));
            Assert.That(b90.Z, Is.EqualTo(0f).Within(1e-5f));

            // Cross-check the independent implementation against the production orientation type.
            var samples = new[]
            {
                (a: 30f, b: -20f, c: 20f),
                (a: 15f, b: 25f, c: -35f),
                (a: 0f, b: 45f, c: 0f),
                (a: 0f, b: 0f, c: 60f),
            };

            foreach (var (a, b, c) in samples)
            {
                Vector3 expected = ReferenceCutEvaluator.ExpectedCuttingAxisDirection(a, b, c);
                Vector3 actual = new MillSimSharp.Toolpath.ToolOrientation(a, b, c).GetCuttingAxisDirection();
                Assert.That(actual.X, Is.EqualTo(expected.X).Within(1e-5f), $"A={a} B={b} C={c}");
                Assert.That(actual.Y, Is.EqualTo(expected.Y).Within(1e-5f), $"A={a} B={b} C={c}");
                Assert.That(actual.Z, Is.EqualTo(expected.Z).Within(1e-5f), $"A={a} B={b} C={c}");
            }
        }

        // ---------------------------------------------------------------------
        // D4: accuracy model - resolution refinement and adaptive sampling
        // ---------------------------------------------------------------------

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void Accuracy_ResolutionRefinement_IsMeasuredAndBounded(float resolution)
        {
            // Static flat-end-mill cut with an analytically known solid (capped cylinder).
            const float radius = 2f;
            const float length = 10f;
            var tool = new EndMill(radius * 2f, length, isBallEnd: false);

            var bounds = BoundingBox.FromCenterAndSize(
                new Vector3(0f, 0f, length * 0.5f),
                new Vector3(2f * radius + 4f, 2f * radius + 4f, length + 4f));
            var grid = new VoxelGrid(bounds, resolution);

            new CutterSimulator(grid).CutPoint(Vector3.Zero, tool);

            var (sx, sy, sz) = grid.Dimensions;
            int removed = 0;
            int disagreements = 0;

            for (int z = 0; z < sz; z++)
                for (int y = 0; y < sy; y++)
                    for (int x = 0; x < sx; x++)
                    {
                        Vector3 center = VoxelCenter(bounds, resolution, x, y, z);
                        bool expectedRemoved = ReferenceCutEvaluator.IsInsideFlatCutPointTool(
                            center, Vector3.Zero, radius, length);
                        bool actualRemoved = !grid.GetVoxel(x, y, z);

                        if (actualRemoved) removed++;
                        if (expectedRemoved != actualRemoved) disagreements++;
                    }

            // Material is removed exactly where the analytic solid contains the voxel center.
            Assert.That(disagreements, Is.EqualTo(0),
                "voxel centers must be classified against the analytic tool solid");

            // The removed volume differs from the analytic volume only by the surface layer.
            // The measured error must stay below the surface-area sampling bound.
            double analyticVolume = Math.PI * radius * radius * length;
            double removedVolume = removed * (double)resolution * resolution * resolution;
            double volumeError = Math.Abs(removedVolume - analyticVolume);
            double surfaceArea = 2.0 * Math.PI * radius * radius + 2.0 * Math.PI * radius * length;
            double bound = surfaceArea * resolution * 2.0;

            Assert.That(volumeError, Is.LessThan(bound),
                $"resolution {resolution}: volume error {volumeError:F4} mm^3 must stay below {bound:F4} mm^3");
        }

        [Test]
        public void Accuracy_AdaptiveSampling_ChangesOnlyNearSurface()
        {
            // Rotation-only move (A: 0 -> 90 degrees) of a ball-only test tool, so the swept solid
            // has an exact test-side reference: the union of balls whose centers follow the arc.
            const float ballRadius = 3f;
            const float resolution = 0.5f;
            var tool = new BallOnlyTool(ballRadius);
            var bounds = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(16, 16, 16));
            var startOrientation = new ToolOrientation(0, 0, 0);
            var endOrientation = new ToolOrientation(90, 0, 0);

            var coarseGrid = new VoxelGrid(bounds, resolution);
            var coarse = new CutterSimulator(coarseGrid);
            coarse.Settings.MaxAngularStep = 45f; // coarse: poses at 0 / 45 / 90 degrees
            coarse.Settings.EnableAdaptiveSampling = false;
            coarse.CutLinearWithOrientation(Vector3.Zero, Vector3.Zero, tool, startOrientation, endOrientation);

            var adaptiveGrid = new VoxelGrid(bounds, resolution);
            var adaptive = new CutterSimulator(adaptiveGrid);
            adaptive.Settings.MaxAngularStep = 45f;
            adaptive.Settings.EnableAdaptiveSampling = true;
            adaptive.Settings.MaxChordError = 0.01f; // refine to ~5.6 degree poses
            adaptive.CutLinearWithOrientation(Vector3.Zero, Vector3.Zero, tool, startOrientation, endOrientation);

            // Dense independent oracle: ball centers along the swept arc, evaluated without
            // production helpers.
            const int densePoses = 360;
            var denseCenters = new Vector3[densePoses + 1];
            for (int i = 0; i <= densePoses; i++)
            {
                float aDeg = 90f * i / densePoses;
                denseCenters[i] = ReferenceCutEvaluator.ExpectedBallCenter(
                    Vector3.Zero, aDeg, 0f, 0f, ballRadius);
            }

            float ReferenceDistance(Vector3 point)
            {
                float best = float.PositiveInfinity;
                foreach (Vector3 center in denseCenters)
                {
                    float distance = Vector3.Distance(point, center) - ballRadius;
                    if (distance < best) best = distance;
                }
                return best;
            }

            var (sx, sy, sz) = adaptiveGrid.Dimensions;
            int differences = 0;
            int coarseMismatch = 0;
            int adaptiveMismatch = 0;
            float maxDifferenceDepth = 0f;

            for (int z = 0; z < sz; z++)
                for (int y = 0; y < sy; y++)
                    for (int x = 0; x < sx; x++)
                    {
                        Vector3 center = VoxelCenter(adaptiveGrid.Bounds, resolution, x, y, z);
                        float reference = ReferenceDistance(center);
                        bool insideReference = reference < 0f;
                        bool removedCoarse = !coarseGrid.GetVoxel(x, y, z);
                        bool removedAdaptive = !adaptiveGrid.GetVoxel(x, y, z);

                        if (removedCoarse != insideReference) coarseMismatch++;
                        if (removedAdaptive != insideReference) adaptiveMismatch++;

                        if (removedCoarse != removedAdaptive)
                        {
                            differences++;
                            maxDifferenceDepth = MathF.Max(maxDifferenceDepth, MathF.Abs(reference));
                        }
                    }

            Assert.That(differences, Is.GreaterThan(0),
                "adaptive sampling must actually change the swept result");
            Assert.That(maxDifferenceDepth, Is.LessThan(2.4f),
                $"adaptive changes must stay within the coarse pose chord of the true surface "
                + $"(max depth {maxDifferenceDepth:F3} mm)");
            Assert.That(adaptiveMismatch, Is.LessThan(coarseMismatch),
                "adaptive sampling must approximate the dense sweep better than coarse sampling");
        }

        // ---------------------------------------------------------------------
        // D4: swept-volume accuracy (voxel and SDF backends)
        // ---------------------------------------------------------------------

        private const float SweepRadius = 3f;
        private const float SweepToolLength = 10f;
        private const float SweepDistance = 8f;

        private static BoundingBox SweptBounds(Vector3 start, Vector3 end, float maxRadius, float toolLength)
        {
            // The tool extends from the tip toward the spindle (+Z), so the bounds must cover
            // [tip, tip + length], not just the sweep path.
            Vector3 center = (start + end) * 0.5f;
            center.Z += 0.5f * toolLength;
            return BoundingBox.FromCenterAndSize(center, new Vector3(
                Vector3.Distance(start, end) + 2f * maxRadius + 2f,
                2f * maxRadius + 2f,
                toolLength + 2f));
        }

        private static double BoundingBoxSurfaceArea(BoundingBox bounds)
        {
            Vector3 size = bounds.Size;
            return 2.0 * (size.X * size.Y + size.Y * size.Z + size.Z * size.X);
        }

        /// <summary>
        /// Volume-error bound for a center-sampled solid: the smaller of the theoretical surface-layer
        /// bound (surfaceArea * resolution) and a relative bound that drops quadratically with the
        /// resolution. Measured errors are at most ~7% at resolution 1 and ~0.6% at 0.5, so
        /// 12% * resolution^2 keeps a margin of at least ~1.6x while rejecting gross under-cutting
        /// (for example 30%) and wrong tool profiles (a flat/ball mix-up differs by about 6-8%),
        /// which the surface-layer bound alone would let through.
        /// </summary>
        private static double VolumeErrorBound(double surfaceArea, float resolution, double analyticVolume)
        {
            double surfaceBound = surfaceArea * resolution;
            double relativeBound = 0.12 * resolution * resolution * analyticVolume;
            return Math.Min(surfaceBound, relativeBound);
        }

        // Center sampling can misclassify at most about surfaceArea / resolution surface-layer cells,
        // each contributing one cell volume (resolution^3), so the removed volume error is bounded by
        // surfaceArea * resolution for an exactly-known solid.

        /// <summary>
        /// Analytic volume and surface area of a flat tool swept perpendicular to its axis: the tool
        /// cross-section (a disk of radius R) sweeps into a stadium, so V = (2*R*d + pi*R^2) * L.
        /// </summary>
        private static (double Volume, double SurfaceArea) FlatSweepSolid()
        {
            double stadium = 2.0 * SweepRadius * SweepDistance + Math.PI * SweepRadius * SweepRadius;
            double perimeter = 2.0 * SweepDistance + 2.0 * Math.PI * SweepRadius;
            return (stadium * SweepToolLength, 2.0 * stadium + perimeter * SweepToolLength);
        }

        /// <summary>
        /// Analytic volume of a ball tool swept perpendicular to its axis. The profile radius by
        /// height is r(z) = sqrt(R^2 - (z - R)^2) for z in [0, R] and R for z in [R, L]; the swept
        /// solid at height z is that profile disk swept along the motion (stadium of area
        /// 2*d*r + pi*r^2), so V = pi*R^2*d/2 + (2/3)*pi*R^3 + (L - R)*(2*R*d + pi*R^2).
        /// </summary>
        private static double BallSweepSolidVolume()
        {
            double lower = Math.PI * SweepRadius * SweepRadius * SweepDistance / 2.0
                + (2.0 / 3.0) * Math.PI * SweepRadius * SweepRadius * SweepRadius;
            double flute = (SweepToolLength - SweepRadius)
                * (2.0 * SweepRadius * SweepDistance + Math.PI * SweepRadius * SweepRadius);
            return lower + flute;
        }

        private static double MeasureRemovedVolume(VoxelGrid grid)
        {
            return grid.GetRemovedVolume();
        }

        private static double MeasureRemovedVolume(SDFGrid sdf)
        {
            return sdf.GetRemovedVolume();
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void Accuracy_FlatSweep_RemovedVolumeMatchesAnalytic(float resolution)
        {
            var tool = new EndMill(SweepRadius * 2f, SweepToolLength, isBallEnd: false);
            var start = new Vector3(0, 0, 0);
            var end = new Vector3(SweepDistance, 0, 0);
            var (analyticVolume, surfaceArea) = FlatSweepSolid();

            var grid = new VoxelGrid(SweptBounds(start, end, SweepRadius, SweepToolLength), resolution);
            new CutterSimulator(grid).CutLinear(start, end, tool);

            double removedVolume = MeasureRemovedVolume(grid);
            double error = Math.Abs(removedVolume - analyticVolume);
            double bound = VolumeErrorBound(surfaceArea, resolution, analyticVolume);
            TestContext.Progress.WriteLine(
                $"voxel flat sweep res={resolution}: removed={removedVolume:F2}, analytic={analyticVolume:F2}, error={error:F3}, bound={bound:F3}");

            Assert.That(removedVolume, Is.GreaterThan(0.5 * analyticVolume),
                "positive control: the sweep must remove a substantial volume");
            Assert.That(error, Is.LessThan(bound),
                $"resolution {resolution}: voxel volume error {error:F3} mm^3 must stay below {bound:F3} mm^3");
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void Accuracy_BallSweep_RemovedVolumeMatchesAnalytic(float resolution)
        {
            var tool = new EndMill(SweepRadius * 2f, SweepToolLength, isBallEnd: true);
            var start = new Vector3(0, 0, 0);
            var end = new Vector3(SweepDistance, 0, 0);
            double analyticVolume = BallSweepSolidVolume();
            var bounds = SweptBounds(start, end, SweepRadius, SweepToolLength);
            // Conservative bound: the swept solid lies inside its bounding box and a shape's surface
            // area does not exceed its bounding box surface area.
            double surfaceArea = BoundingBoxSurfaceArea(bounds);

            var grid = new VoxelGrid(bounds, resolution);
            new CutterSimulator(grid).CutLinear(start, end, tool);

            double removedVolume = MeasureRemovedVolume(grid);
            double error = Math.Abs(removedVolume - analyticVolume);
            double bound = VolumeErrorBound(surfaceArea, resolution, analyticVolume);
            TestContext.Progress.WriteLine(
                $"voxel ball sweep res={resolution}: removed={removedVolume:F2}, analytic={analyticVolume:F2}, error={error:F3}, bound={bound:F3}");

            Assert.That(removedVolume, Is.GreaterThan(0.5 * analyticVolume),
                "positive control: the sweep must remove a substantial volume");
            Assert.That(error, Is.LessThan(bound),
                $"resolution {resolution}: voxel volume error {error:F3} mm^3 must stay below {bound:F3} mm^3");
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void Accuracy_FlatSweep_RemovedVolumeMatchesAnalytic_Sdf(float resolution)
        {
            var tool = new EndMill(SweepRadius * 2f, SweepToolLength, isBallEnd: false);
            var start = new Vector3(0, 0, 0);
            var end = new Vector3(SweepDistance, 0, 0);
            var (analyticVolume, surfaceArea) = FlatSweepSolid();

            var sdf = new SDFGrid(SweptBounds(start, end, SweepRadius, SweepToolLength), resolution, narrowBandWidth: 4);
            new SDFCutterSimulator(sdf).CutLinear(start, end, tool);

            double removedVolume = MeasureRemovedVolume(sdf);
            double error = Math.Abs(removedVolume - analyticVolume);
            double bound = VolumeErrorBound(surfaceArea, resolution, analyticVolume);
            TestContext.Progress.WriteLine(
                $"sdf flat sweep res={resolution}: removed={removedVolume:F2}, analytic={analyticVolume:F2}, error={error:F3}, bound={bound:F3}");

            Assert.That(removedVolume, Is.GreaterThan(0.5 * analyticVolume),
                "positive control: the sweep must remove a substantial volume");
            Assert.That(error, Is.LessThan(bound),
                $"resolution {resolution}: SDF volume error {error:F3} mm^3 must stay below {bound:F3} mm^3");
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void Accuracy_BallSweep_RemovedVolumeMatchesAnalytic_Sdf(float resolution)
        {
            var tool = new EndMill(SweepRadius * 2f, SweepToolLength, isBallEnd: true);
            var start = new Vector3(0, 0, 0);
            var end = new Vector3(SweepDistance, 0, 0);
            double analyticVolume = BallSweepSolidVolume();
            var bounds = SweptBounds(start, end, SweepRadius, SweepToolLength);
            double surfaceArea = BoundingBoxSurfaceArea(bounds);

            var sdf = new SDFGrid(bounds, resolution, narrowBandWidth: 4);
            new SDFCutterSimulator(sdf).CutLinear(start, end, tool);

            double removedVolume = MeasureRemovedVolume(sdf);
            double error = Math.Abs(removedVolume - analyticVolume);
            double bound = VolumeErrorBound(surfaceArea, resolution, analyticVolume);
            TestContext.Progress.WriteLine(
                $"sdf ball sweep res={resolution}: removed={removedVolume:F2}, analytic={analyticVolume:F2}, error={error:F3}, bound={bound:F3}");

            Assert.That(removedVolume, Is.GreaterThan(0.5 * analyticVolume),
                "positive control: the sweep must remove a substantial volume");
            Assert.That(error, Is.LessThan(bound),
                $"resolution {resolution}: SDF volume error {error:F3} mm^3 must stay below {bound:F3} mm^3");
        }

        // ---------------------------------------------------------------------
        // D4: circular-path swept-volume accuracy (full circle executed as chords)
        // ---------------------------------------------------------------------

        private const float CirclePathRadius = 8f;
        private const int CircleChordCount = 720; // 0.5 degrees per chord; sagitta ~7.6e-5 mm

        private static BoundingBox CircularSweepBounds(Tool tool)
        {
            float xy = 2f * (CirclePathRadius + SweepRadius + 1f);
            return BoundingBox.FromCenterAndSize(
                new Vector3(0f, 0f, 0.5f * tool.Length),
                new Vector3(xy, xy, tool.Length + 2f));
        }

        /// <summary>
        /// Builds a full circle as G1 chords (the same representation the viewer G-code parser emits
        /// for G2/G3). The first chord ends one segment after the start point.
        /// </summary>
        private static List<IToolpathCommand> BuildCircleChords(Vector3 center, float radius, float z)
        {
            var commands = new List<IToolpathCommand>(CircleChordCount);
            for (int i = 1; i <= CircleChordCount; i++)
            {
                double angle = 2.0 * Math.PI * i / CircleChordCount;
                commands.Add(new G1Move(new Vector3(
                    center.X + (float)(radius * Math.Cos(angle)),
                    center.Y + (float)(radius * Math.Sin(angle)),
                    z), feedRate: 300f));
            }

            return commands;
        }

        private static double ExecuteCircle(VoxelGrid grid, Tool tool, Vector3 start)
        {
            var executor = new ToolpathExecutor(new CutterSimulator(grid), tool, start);
            executor.ExecuteCommands(BuildCircleChords(Vector3.Zero, CirclePathRadius, start.Z));
            return MeasureRemovedVolume(grid);
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void Accuracy_FlatCircleSweep_RemovedVolumeMatchesAnalytic(float resolution)
        {
            // A flat tool following a full circle (rho > R) sweeps an annulus: V = 4*pi*rho*R*L.
            var tool = new EndMill(SweepRadius * 2f, SweepToolLength, isBallEnd: false);
            var start = new Vector3(CirclePathRadius, 0, 0);
            double analyticVolume = 4.0 * Math.PI * CirclePathRadius * SweepRadius * SweepToolLength;

            var grid = new VoxelGrid(CircularSweepBounds(tool), resolution);
            double removedVolume = ExecuteCircle(grid, tool, start);
            double error = Math.Abs(removedVolume - analyticVolume);

            double surfaceArea = 8.0 * Math.PI * CirclePathRadius * SweepRadius
                + 4.0 * Math.PI * CirclePathRadius * SweepToolLength;
            double bound = VolumeErrorBound(surfaceArea, resolution, analyticVolume);
            TestContext.Progress.WriteLine(
                $"voxel flat circle res={resolution}: removed={removedVolume:F2}, analytic={analyticVolume:F2}, error={error:F3}, bound={bound:F3}");

            Assert.That(removedVolume, Is.GreaterThan(0.5 * analyticVolume),
                "positive control: the circle must remove a substantial volume");
            Assert.That(error, Is.LessThan(bound),
                $"resolution {resolution}: voxel volume error {error:F3} mm^3 must stay below {bound:F3} mm^3");
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void Accuracy_BallCircleSweep_RemovedVolumeMatchesAnalytic(float resolution)
        {
            // A ball tool following a full circle (rho > R) sweeps a torus below the ball center plus
            // the flute annulus above it: V = pi^2*rho*R^2 + 4*pi*rho*R*(L - R).
            var tool = new EndMill(SweepRadius * 2f, SweepToolLength, isBallEnd: true);
            var start = new Vector3(CirclePathRadius, 0, 0);
            double analyticVolume = Math.PI * Math.PI * CirclePathRadius * SweepRadius * SweepRadius
                + 4.0 * Math.PI * CirclePathRadius * SweepRadius * (SweepToolLength - SweepRadius);

            var bounds = CircularSweepBounds(tool);
            var grid = new VoxelGrid(bounds, resolution);
            double removedVolume = ExecuteCircle(grid, tool, start);
            double error = Math.Abs(removedVolume - analyticVolume);

            double surfaceArea = BoundingBoxSurfaceArea(bounds);
            double bound = VolumeErrorBound(surfaceArea, resolution, analyticVolume);
            TestContext.Progress.WriteLine(
                $"voxel ball circle res={resolution}: removed={removedVolume:F2}, analytic={analyticVolume:F2}, error={error:F3}, bound={bound:F3}");

            Assert.That(removedVolume, Is.GreaterThan(0.5 * analyticVolume),
                "positive control: the circle must remove a substantial volume");
            Assert.That(error, Is.LessThan(bound),
                $"resolution {resolution}: voxel volume error {error:F3} mm^3 must stay below {bound:F3} mm^3");
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void Accuracy_FlatCircleSweep_RemovedVolumeMatchesAnalytic_Sdf(float resolution)
        {
            var tool = new EndMill(SweepRadius * 2f, SweepToolLength, isBallEnd: false);
            var start = new Vector3(CirclePathRadius, 0, 0);
            double analyticVolume = 4.0 * Math.PI * CirclePathRadius * SweepRadius * SweepToolLength;

            var sdf = new SDFGrid(CircularSweepBounds(tool), resolution, narrowBandWidth: 4);
            var executor = new ToolpathExecutor(new SDFCutterSimulator(sdf), tool, start);
            executor.ExecuteCommands(BuildCircleChords(Vector3.Zero, CirclePathRadius, start.Z));
            double removedVolume = MeasureRemovedVolume(sdf);
            double error = Math.Abs(removedVolume - analyticVolume);

            double surfaceArea = 8.0 * Math.PI * CirclePathRadius * SweepRadius
                + 4.0 * Math.PI * CirclePathRadius * SweepToolLength;
            double bound = VolumeErrorBound(surfaceArea, resolution, analyticVolume);
            TestContext.Progress.WriteLine(
                $"sdf flat circle res={resolution}: removed={removedVolume:F2}, analytic={analyticVolume:F2}, error={error:F3}, bound={bound:F3}");

            Assert.That(removedVolume, Is.GreaterThan(0.5 * analyticVolume),
                "positive control: the circle must remove a substantial volume");
            Assert.That(error, Is.LessThan(bound),
                $"resolution {resolution}: SDF volume error {error:F3} mm^3 must stay below {bound:F3} mm^3");
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void Accuracy_BallCircleSweep_RemovedVolumeMatchesAnalytic_Sdf(float resolution)
        {
            var tool = new EndMill(SweepRadius * 2f, SweepToolLength, isBallEnd: true);
            var start = new Vector3(CirclePathRadius, 0, 0);
            double analyticVolume = Math.PI * Math.PI * CirclePathRadius * SweepRadius * SweepRadius
                + 4.0 * Math.PI * CirclePathRadius * SweepRadius * (SweepToolLength - SweepRadius);

            var bounds = CircularSweepBounds(tool);
            var sdf = new SDFGrid(bounds, resolution, narrowBandWidth: 4);
            var executor = new ToolpathExecutor(new SDFCutterSimulator(sdf), tool, start);
            executor.ExecuteCommands(BuildCircleChords(Vector3.Zero, CirclePathRadius, start.Z));
            double removedVolume = MeasureRemovedVolume(sdf);
            double error = Math.Abs(removedVolume - analyticVolume);

            double surfaceArea = BoundingBoxSurfaceArea(bounds);
            double bound = VolumeErrorBound(surfaceArea, resolution, analyticVolume);
            TestContext.Progress.WriteLine(
                $"sdf ball circle res={resolution}: removed={removedVolume:F2}, analytic={analyticVolume:F2}, error={error:F3}, bound={bound:F3}");

            Assert.That(removedVolume, Is.GreaterThan(0.5 * analyticVolume),
                "positive control: the circle must remove a substantial volume");
            Assert.That(error, Is.LessThan(bound),
                $"resolution {resolution}: SDF volume error {error:F3} mm^3 must stay below {bound:F3} mm^3");
        }

        // ---------------------------------------------------------------------
        // D4: bull-nose / taper / tilted 5-axis sweep volumes
        // ---------------------------------------------------------------------

        private const float BullNoseCornerRadius = 1.5f;
        private const float TaperTipRadius = 2f;
        private const float TaperAngleDegrees = 10f;

        /// <summary>
        /// Analytic volume of a bull-nose tool swept perpendicular to its axis. Profile radius:
        /// R - rc for the flat core (z in [0, rc]), then the torus corner
        /// (R - rc) + sqrt(rc^2 - (rc - z)^2), then the full radius R up to L. With
        /// I1 = integral r dz and I2 = integral r^2 dz, V = 2*d*I1 + pi*I2.
        /// </summary>
        private static double BullNoseSweptVolume()
        {
            double a = SweepRadius - BullNoseCornerRadius;
            double integralR = a * BullNoseCornerRadius
                + Math.PI * BullNoseCornerRadius * BullNoseCornerRadius / 4.0
                + SweepRadius * (SweepToolLength - BullNoseCornerRadius);
            double integralR2 = a * a * BullNoseCornerRadius
                + a * Math.PI * BullNoseCornerRadius * BullNoseCornerRadius / 2.0
                + 2.0 / 3.0 * BullNoseCornerRadius * BullNoseCornerRadius * BullNoseCornerRadius
                + SweepRadius * SweepRadius * (SweepToolLength - BullNoseCornerRadius);
            return 2.0 * SweepDistance * integralR + Math.PI * integralR2;
        }

        /// <summary>
        /// Analytic volume of a tapered tool swept perpendicular to its axis. The profile is
        /// r(z) = tipRadius + z*tan(angle), so with I1 = integral r dz and I2 = integral r^2 dz,
        /// V = 2*d*I1 + pi*I2.
        /// </summary>
        private static double TaperSweptVolume()
        {
            double t = Math.Tan(TaperAngleDegrees * Math.PI / 180.0);
            double topRadius = TaperTipRadius + SweepToolLength * t;
            double integralR = TaperTipRadius * SweepToolLength + t * SweepToolLength * SweepToolLength / 2.0;
            double integralR2 = (topRadius * topRadius * topRadius - TaperTipRadius * TaperTipRadius * TaperTipRadius)
                / (3.0 * t);
            return 2.0 * SweepDistance * integralR + Math.PI * integralR2;
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void Accuracy_BullNoseSweep_RemovedVolumeMatchesAnalytic(float resolution)
        {
            var tool = new BullNoseEndMill(SweepRadius * 2f, SweepToolLength, cornerRadius: BullNoseCornerRadius);
            var start = new Vector3(0, 0, 0);
            var end = new Vector3(SweepDistance, 0, 0);
            double analyticVolume = BullNoseSweptVolume();

            var bounds = SweptBounds(start, end, SweepRadius, SweepToolLength);
            var grid = new VoxelGrid(bounds, resolution);
            new CutterSimulator(grid).CutLinear(start, end, tool);

            double removedVolume = MeasureRemovedVolume(grid);
            double error = Math.Abs(removedVolume - analyticVolume);
            double bound = VolumeErrorBound(BoundingBoxSurfaceArea(bounds), resolution, analyticVolume);
            TestContext.Progress.WriteLine(
                $"voxel bull-nose sweep res={resolution}: removed={removedVolume:F2}, analytic={analyticVolume:F2}, error={error:F3}, bound={bound:F3}");

            Assert.That(removedVolume, Is.GreaterThan(0.5 * analyticVolume),
                "positive control: the sweep must remove a substantial volume");
            Assert.That(error, Is.LessThan(bound),
                $"resolution {resolution}: voxel volume error {error:F3} mm^3 must stay below {bound:F3} mm^3");
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void Accuracy_BullNoseSweep_RemovedVolumeMatchesAnalytic_Sdf(float resolution)
        {
            var tool = new BullNoseEndMill(SweepRadius * 2f, SweepToolLength, cornerRadius: BullNoseCornerRadius);
            var start = new Vector3(0, 0, 0);
            var end = new Vector3(SweepDistance, 0, 0);
            double analyticVolume = BullNoseSweptVolume();

            var bounds = SweptBounds(start, end, SweepRadius, SweepToolLength);
            var sdf = new SDFGrid(bounds, resolution, narrowBandWidth: 4);
            new SDFCutterSimulator(sdf).CutLinear(start, end, tool);

            double removedVolume = MeasureRemovedVolume(sdf);
            double error = Math.Abs(removedVolume - analyticVolume);
            double bound = VolumeErrorBound(BoundingBoxSurfaceArea(bounds), resolution, analyticVolume);
            TestContext.Progress.WriteLine(
                $"sdf bull-nose sweep res={resolution}: removed={removedVolume:F2}, analytic={analyticVolume:F2}, error={error:F3}, bound={bound:F3}");

            Assert.That(removedVolume, Is.GreaterThan(0.5 * analyticVolume),
                "positive control: the sweep must remove a substantial volume");
            Assert.That(error, Is.LessThan(bound),
                $"resolution {resolution}: SDF volume error {error:F3} mm^3 must stay below {bound:F3} mm^3");
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void Accuracy_TaperSweep_RemovedVolumeMatchesAnalytic(float resolution)
        {
            var tool = new TaperEndMill(TaperTipRadius * 2f, SweepToolLength, TaperAngleDegrees);
            var start = new Vector3(0, 0, 0);
            var end = new Vector3(SweepDistance, 0, 0);
            double analyticVolume = TaperSweptVolume();
            float maxRadius = TaperTipRadius + SweepToolLength * MathF.Tan(TaperAngleDegrees * MathF.PI / 180f);

            var bounds = SweptBounds(start, end, maxRadius, SweepToolLength);
            var grid = new VoxelGrid(bounds, resolution);
            new CutterSimulator(grid).CutLinear(start, end, tool);

            double removedVolume = MeasureRemovedVolume(grid);
            double error = Math.Abs(removedVolume - analyticVolume);
            double bound = VolumeErrorBound(BoundingBoxSurfaceArea(bounds), resolution, analyticVolume);
            TestContext.Progress.WriteLine(
                $"voxel taper sweep res={resolution}: removed={removedVolume:F2}, analytic={analyticVolume:F2}, error={error:F3}, bound={bound:F3}");

            Assert.That(removedVolume, Is.GreaterThan(0.5 * analyticVolume),
                "positive control: the sweep must remove a substantial volume");
            Assert.That(error, Is.LessThan(bound),
                $"resolution {resolution}: voxel volume error {error:F3} mm^3 must stay below {bound:F3} mm^3");
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void Accuracy_TaperSweep_RemovedVolumeMatchesAnalytic_Sdf(float resolution)
        {
            var tool = new TaperEndMill(TaperTipRadius * 2f, SweepToolLength, TaperAngleDegrees);
            var start = new Vector3(0, 0, 0);
            var end = new Vector3(SweepDistance, 0, 0);
            double analyticVolume = TaperSweptVolume();
            float maxRadius = TaperTipRadius + SweepToolLength * MathF.Tan(TaperAngleDegrees * MathF.PI / 180f);

            var bounds = SweptBounds(start, end, maxRadius, SweepToolLength);
            var sdf = new SDFGrid(bounds, resolution, narrowBandWidth: 4);
            new SDFCutterSimulator(sdf).CutLinear(start, end, tool);

            double removedVolume = MeasureRemovedVolume(sdf);
            double error = Math.Abs(removedVolume - analyticVolume);
            double bound = VolumeErrorBound(BoundingBoxSurfaceArea(bounds), resolution, analyticVolume);
            TestContext.Progress.WriteLine(
                $"sdf taper sweep res={resolution}: removed={removedVolume:F2}, analytic={analyticVolume:F2}, error={error:F3}, bound={bound:F3}");

            Assert.That(removedVolume, Is.GreaterThan(0.5 * analyticVolume),
                "positive control: the sweep must remove a substantial volume");
            Assert.That(error, Is.LessThan(bound),
                $"resolution {resolution}: SDF volume error {error:F3} mm^3 must stay below {bound:F3} mm^3");
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void Accuracy_FiveAxisTiltedSweep_RemovedVolumeMatchesCapsule(float resolution)
        {
            // A tilted (A = 30 degrees) ball-only tool swept along a straight move that is not
            // perpendicular to the tool axis: the ball centers follow the tip path, so the swept
            // solid is a capsule of the tip-path length. The flute is excluded because a tilted
            // flute sweep has no closed-form volume; the tilted pose exercises the 5-axis path.
            const float tiltDegrees = 30f;
            var tool = new BallOnlyTool(SweepRadius);
            var start = new Vector3(0, 0, 0);
            var end = new Vector3(SweepDistance, SweepDistance, 0);
            double pathLength = Vector3.Distance(start, end);
            double analyticVolume = Math.PI * SweepRadius * SweepRadius * pathLength
                + 4.0 / 3.0 * Math.PI * SweepRadius * SweepRadius * SweepRadius;

            // The ball center is offset by R along the tilted axis, so the solid reaches up to 2*R
            // from the tip path (plus a margin), not just R.
            var bounds = BoundingBox.FromCenterAndSize(
                new Vector3(0.5f * SweepDistance, 0.5f * SweepDistance, 0f),
                new Vector3(
                    SweepDistance + 4f * SweepRadius + 2f,
                    SweepDistance + 4f * SweepRadius + 2f,
                    4f * SweepRadius + 2f));

            var grid = new VoxelGrid(bounds, resolution);
            new CutterSimulator(grid).CutLinearWithOrientation(
                start, end, tool, new ToolOrientation(tiltDegrees, 0, 0), new ToolOrientation(tiltDegrees, 0, 0));

            double removedVolume = MeasureRemovedVolume(grid);
            double error = Math.Abs(removedVolume - analyticVolume);
            double bound = VolumeErrorBound(BoundingBoxSurfaceArea(bounds), resolution, analyticVolume);
            TestContext.Progress.WriteLine(
                $"voxel 5-axis tilted sweep res={resolution}: removed={removedVolume:F2}, analytic={analyticVolume:F2}, error={error:F3}, bound={bound:F3}");

            Assert.That(removedVolume, Is.GreaterThan(0.5 * analyticVolume),
                "positive control: the sweep must remove a substantial volume");
            Assert.That(error, Is.LessThan(bound),
                $"resolution {resolution}: voxel volume error {error:F3} mm^3 must stay below {bound:F3} mm^3");
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void Accuracy_FiveAxisTiltedSweep_RemovedVolumeMatchesCapsule_Sdf(float resolution)
        {
            const float tiltDegrees = 30f;
            var tool = new BallOnlyTool(SweepRadius);
            var start = new Vector3(0, 0, 0);
            var end = new Vector3(SweepDistance, SweepDistance, 0);
            double pathLength = Vector3.Distance(start, end);
            double analyticVolume = Math.PI * SweepRadius * SweepRadius * pathLength
                + 4.0 / 3.0 * Math.PI * SweepRadius * SweepRadius * SweepRadius;

            var bounds = BoundingBox.FromCenterAndSize(
                new Vector3(0.5f * SweepDistance, 0.5f * SweepDistance, 0f),
                new Vector3(
                    SweepDistance + 4f * SweepRadius + 2f,
                    SweepDistance + 4f * SweepRadius + 2f,
                    4f * SweepRadius + 2f));

            var sdf = new SDFGrid(bounds, resolution, narrowBandWidth: 4);
            new SDFCutterSimulator(sdf).CutLinearWithOrientation(
                start, end, tool, new ToolOrientation(tiltDegrees, 0, 0), new ToolOrientation(tiltDegrees, 0, 0));

            double removedVolume = MeasureRemovedVolume(sdf);
            double error = Math.Abs(removedVolume - analyticVolume);
            double bound = VolumeErrorBound(BoundingBoxSurfaceArea(bounds), resolution, analyticVolume);
            TestContext.Progress.WriteLine(
                $"sdf 5-axis tilted sweep res={resolution}: removed={removedVolume:F2}, analytic={analyticVolume:F2}, error={error:F3}, bound={bound:F3}");

            Assert.That(removedVolume, Is.GreaterThan(0.5 * analyticVolume),
                "positive control: the sweep must remove a substantial volume");
            Assert.That(error, Is.LessThan(bound),
                $"resolution {resolution}: SDF volume error {error:F3} mm^3 must stay below {bound:F3} mm^3");
        }

        /// <summary>Ball-only tool (no flute) for the adaptive-sampling accuracy test.</summary>
        private sealed class BallOnlyTool : Tool
        {
            private readonly BallOnlyGeometry _geometry;

            public BallOnlyTool(float radius)
                : base(radius * 2f, 2f * radius, ToolType.Ball)
            {
                _geometry = new BallOnlyGeometry(radius);
            }

            public override float BallCenterOffsetFromTip => _geometry.Radius;

            public override IToolGeometry GetCuttingGeometry() => _geometry;
        }

        private sealed class BallOnlyGeometry : IToolGeometry
        {
            public BallOnlyGeometry(float radius)
            {
                Radius = radius;
            }

            public float Radius { get; }

            public BoundingBox LocalBounds => new BoundingBox(
                new Vector3(-Radius, -Radius, 0f),
                new Vector3(Radius, Radius, 2f * Radius));

            public float CuttingCenterOffset => Radius;

            public float SignedDistance(Vector3 localPoint) =>
                Vector3.Distance(localPoint, new Vector3(0f, 0f, Radius)) - Radius;
        }
    }
}
