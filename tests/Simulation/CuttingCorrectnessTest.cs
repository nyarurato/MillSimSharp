using System;
using System.Collections.Generic;
using System.Numerics;
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using MillSimSharp.Tests.Reference;
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
    }
}
