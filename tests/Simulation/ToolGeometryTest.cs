using System;
using System.Numerics;
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;
using NUnit.Framework;

namespace MillSimSharp.Tests.Simulation
{
    /// <summary>
    /// Tests for the tool geometry abstraction: exact signed distances, local frame conventions,
    /// and voxel/SDF backend consistency for the same tool pose.
    /// </summary>
    [TestFixture]
    public class ToolGeometryTest
    {
        private static BoundingBox StockBounds => BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(30, 30, 30));

        /// <summary>
        /// Independent test-side local-frame conversion (radial distance, 0, axial distance).
        /// </summary>
        private static Vector3 ToLocal(Vector3 point, Vector3 tip, Vector3 axisTowardSpindle)
        {
            Vector3 relative = point - tip;
            float axial = Vector3.Dot(relative, axisTowardSpindle);
            float radial = (relative - axisTowardSpindle * axial).Length();
            return new Vector3(radial, 0f, axial);
        }

        private static Vector3 VoxelCenter(BoundingBox bbox, float resolution, int ix, int iy, int iz)
        {
            return bbox.Min + new Vector3((ix + 0.5f) * resolution, (iy + 0.5f) * resolution, (iz + 0.5f) * resolution);
        }

        [Test]
        public void EndMill_GetCuttingGeometry_ReturnsExpectedTypeAndOffset()
        {
            var flat = new EndMill(10f, 30f, isBallEnd: false);
            var flatGeometry = flat.GetCuttingGeometry();
            Assert.That(flatGeometry, Is.InstanceOf<FlatEndMillGeometry>());
            Assert.That(flatGeometry.CuttingCenterOffset, Is.EqualTo(0f));

            var ball = new EndMill(10f, 30f, isBallEnd: true);
            var ballGeometry = ball.GetCuttingGeometry();
            Assert.That(ballGeometry, Is.InstanceOf<BallEndMillGeometry>());
            Assert.That(ballGeometry.CuttingCenterOffset, Is.EqualTo(5f));
        }

        [Test]
        public void FlatGeometry_SignedDistance_IsExact()
        {
            var geometry = new FlatEndMillGeometry(5f, 30f);

            Assert.That(geometry.SignedDistance(new Vector3(0, 0, 15f)), Is.EqualTo(-5f).Within(1e-5f), "center");
            Assert.That(geometry.SignedDistance(new Vector3(0, 0, 0f)), Is.EqualTo(0f).Within(1e-5f), "bottom face");
            Assert.That(geometry.SignedDistance(new Vector3(0, 0, 30f)), Is.EqualTo(0f).Within(1e-5f), "top face");
            Assert.That(geometry.SignedDistance(new Vector3(6, 0, 15f)), Is.EqualTo(1f).Within(1e-5f), "side");
            Assert.That(geometry.SignedDistance(new Vector3(0, 0, -1f)), Is.EqualTo(1f).Within(1e-5f), "below tip");

            // Exact corner distance (the legacy max() formula returned 1 here).
            Assert.That(geometry.SignedDistance(new Vector3(6, 0, -1f)),
                Is.EqualTo(MathF.Sqrt(2f)).Within(1e-5f), "bottom corner");
        }

        [Test]
        public void BallGeometry_SignedDistance_IsExact()
        {
            var geometry = new BallEndMillGeometry(5f, 30f);

            Assert.That(geometry.SignedDistance(new Vector3(0, 0, 0f)), Is.EqualTo(0f).Within(1e-5f), "physical tip");
            Assert.That(geometry.SignedDistance(new Vector3(0, 0, -1f)), Is.EqualTo(1f).Within(1e-5f), "below tip");
            Assert.That(geometry.SignedDistance(new Vector3(0, 0, 5f)), Is.EqualTo(-5f).Within(1e-5f), "ball center");
            Assert.That(geometry.SignedDistance(new Vector3(5, 0, 5f)), Is.EqualTo(0f).Within(1e-5f), "ball equator");
        }

        [TestCase(1.0f, true)]
        [TestCase(1.0f, false)]
        [TestCase(0.5f, true)]
        [TestCase(0.5f, false)]
        public void VoxelBackend_CutPoint_MatchesToolGeometry(float resolution, bool isBallEnd)
        {
            var bbox = StockBounds;
            var grid = new VoxelGrid(bbox, resolution);
            var tool = new EndMill(10f, 30f, isBallEnd);
            new CutterSimulator(grid).CutPoint(Vector3.Zero, tool);

            IToolGeometry geometry = tool.GetCuttingGeometry();
            var (sx, sy, sz) = grid.Dimensions;

            int mismatches = 0;
            for (int x = 0; x < sx; x++)
            for (int y = 0; y < sy; y++)
            for (int z = 0; z < sz; z++)
            {
                Vector3 center = VoxelCenter(bbox, resolution, x, y, z);
                float distance = geometry.SignedDistance(ToLocal(center, Vector3.Zero, Vector3.UnitZ));
                bool expectedMaterial = distance >= 0f;
                if (grid.GetVoxel(x, y, z) != expectedMaterial) mismatches++;
            }

            Assert.That(mismatches, Is.EqualTo(0), "Voxel occupancy must match the tool geometry exactly");
        }

        [TestCase(1.0f, true)]
        [TestCase(1.0f, false)]
        [TestCase(0.5f, true)]
        [TestCase(0.5f, false)]
        public void SdfBackend_CutPoint_MatchesToolGeometry(float resolution, bool isBallEnd)
        {
            var bbox = StockBounds;
            var sdf = new SDFGrid(bbox, resolution, narrowBandWidth: 10);
            var tool = new EndMill(10f, 30f, isBallEnd);
            new SDFCutterSimulator(sdf).CutPoint(Vector3.Zero, tool);

            IToolGeometry geometry = tool.GetCuttingGeometry();
            var (sx, sy, sz) = sdf.Dimensions;

            int mismatches = 0;
            for (int x = 0; x < sx; x++)
            for (int y = 0; y < sy; y++)
            for (int z = 0; z < sz; z++)
            {
                Vector3 center = VoxelCenter(bbox, resolution, x, y, z);
                float distance = geometry.SignedDistance(ToLocal(center, Vector3.Zero, Vector3.UnitZ));
                bool expectedMaterial = distance >= 0f;
                bool actualMaterial = sdf.GetDistance(x, y, z) < 0f;
                if (actualMaterial != expectedMaterial) mismatches++;
            }

            Assert.That(mismatches, Is.EqualTo(0), "SDF sign must match the tool geometry exactly");
        }

        [TestCase(1.0f, true)]
        [TestCase(1.0f, false)]
        [TestCase(0.5f, true)]
        public void VoxelAndSdfBackends_ProduceSameOccupancy_LinearSweep(float resolution, bool isBallEnd)
        {
            var bbox = StockBounds;
            var tool = new EndMill(10f, 30f, isBallEnd);
            var start = new Vector3(-5, 0, 0);
            var end = new Vector3(5, 0, 0);

            var grid = new VoxelGrid(bbox, resolution);
            new CutterSimulator(grid).CutLinear(start, end, tool);

            var sdf = new SDFGrid(bbox, resolution, narrowBandWidth: 10);
            new SDFCutterSimulator(sdf).CutLinear(start, end, tool);

            var (sx, sy, sz) = grid.Dimensions;
            int mismatches = 0;
            for (int x = 0; x < sx; x++)
            for (int y = 0; y < sy; y++)
            for (int z = 0; z < sz; z++)
            {
                bool voxelMaterial = grid.GetVoxel(x, y, z);
                bool sdfMaterial = sdf.GetDistance(x, y, z) < 0f;
                if (voxelMaterial != sdfMaterial) mismatches++;
            }

            Assert.That(mismatches, Is.EqualTo(0), "Voxel and SDF backends must match for the same tool sweep");
        }

        [Test]
        public void FiveAxis_SamePoseProducesSameVoxelAndSdfResult()
        {
            var bbox = StockBounds;
            var tool = new EndMill(10f, 30f, isBallEnd: true);
            var start = new Vector3(-5, 0, 0);
            var end = new Vector3(5, 0, 0);
            var startOrientation = new ToolOrientation(30, 10, 0);
            var endOrientation = new ToolOrientation(45, 20, 10);

            var grid = new VoxelGrid(bbox, 1.0f);
            new CutterSimulator(grid).CutLinearWithOrientation(start, end, tool, startOrientation, endOrientation);

            var sdf = new SDFGrid(bbox, 1.0f, narrowBandWidth: 10);
            new SDFCutterSimulator(sdf).CutLinearWithOrientation(start, end, tool, startOrientation, endOrientation);

            var (sx, sy, sz) = grid.Dimensions;
            int mismatches = 0;
            for (int x = 0; x < sx; x++)
            for (int y = 0; y < sy; y++)
            for (int z = 0; z < sz; z++)
            {
                bool voxelMaterial = grid.GetVoxel(x, y, z);
                bool sdfMaterial = sdf.GetDistance(x, y, z) < 0f;
                if (voxelMaterial != sdfMaterial) mismatches++;
            }

            Assert.That(mismatches, Is.EqualTo(0), "Voxel and SDF backends must match for the same 5-axis pose sweep");
        }
    }
}
