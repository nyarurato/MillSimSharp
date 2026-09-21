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

        [Test]
        public void BullNoseAndTaperTools_ExposeTheirGeometry()
        {
            var bullNose = new BullNoseEndMill(10f, 30f, cornerRadius: 2f);
            Assert.That(bullNose.Type, Is.EqualTo(ToolType.BullNose));
            Assert.That(bullNose.BallCenterOffsetFromTip, Is.EqualTo(0f));
            var bullGeometry = bullNose.GetCuttingGeometry();
            Assert.That(bullGeometry, Is.InstanceOf<BullNoseEndMillGeometry>());
            Assert.That(bullGeometry.CuttingCenterOffset, Is.EqualTo(0f));

            var taper = new TaperEndMill(tipDiameter: 4f, length: 20f, taperAngleDegrees: 10f);
            Assert.That(taper.Type, Is.EqualTo(ToolType.Taper));
            var taperGeometry = taper.GetCuttingGeometry();
            Assert.That(taperGeometry, Is.InstanceOf<TaperedEndMillGeometry>());
            Assert.That(taperGeometry.CuttingCenterOffset, Is.EqualTo(0f));
        }

        [Test]
        public void BullNoseGeometry_SignedDistance_MatchesShape()
        {
            var geometry = new BullNoseEndMillGeometry(5f, 2f, 30f);

            Assert.That(geometry.SignedDistance(new Vector3(0, 0, 0f)), Is.EqualTo(0f).Within(1e-5f), "tip");
            Assert.That(geometry.SignedDistance(new Vector3(0, 0, -1f)), Is.EqualTo(1f).Within(1e-5f), "below tip");
            Assert.That(geometry.SignedDistance(new Vector3(6, 0, 15f)), Is.EqualTo(1f).Within(1e-5f), "side");
            Assert.That(geometry.SignedDistance(new Vector3(5, 0, 2f)), Is.EqualTo(0f).Within(1e-5f), "corner surface");
            Assert.That(geometry.SignedDistance(new Vector3(3, 0, 2f)), Is.EqualTo(-2f).Within(1e-5f), "torus center");
            Assert.That(geometry.SignedDistance(new Vector3(4, 0, 1f)),
                Is.EqualTo(MathF.Sqrt(2f) - 2f).Within(1e-5f), "corner interior");
        }

        [Test]
        public void TaperGeometry_SignedDistance_MatchesShape()
        {
            var geometry = new TaperedEndMillGeometry(2f, 10f, 20f);

            float topRadius = 2f + 20f * MathF.Tan(10f * MathF.PI / 180f);
            Assert.That(geometry.TopRadius, Is.EqualTo(topRadius).Within(1e-4f));

            float radiusAt10 = 2f + 10f * MathF.Tan(10f * MathF.PI / 180f);
            float expectedLateral = MathF.Cos(10f * MathF.PI / 180f);
            Assert.That(geometry.SignedDistance(new Vector3(radiusAt10 + 1f, 0, 10f)),
                Is.EqualTo(expectedLateral).Within(1e-3f), "lateral surface");
            Assert.That(geometry.SignedDistance(new Vector3(0, 0, 10f)), Is.LessThan(0f), "interior");
            Assert.That(geometry.SignedDistance(new Vector3(0, 0, -1f)), Is.EqualTo(1f).Within(1e-5f), "below tip");
            Assert.That(geometry.SignedDistance(new Vector3(0, 0, 20f)), Is.EqualTo(0f).Within(1e-4f), "top face");
        }

        [TestCase(1.0f, true)]
        [TestCase(1.0f, false)]
        [TestCase(0.5f, true)]
        public void VoxelBackend_CutPoint_MatchesVariableRadiusGeometry(float resolution, bool useBullNose)
        {
            var bbox = StockBounds;
            var grid = new VoxelGrid(bbox, resolution);
            Tool tool = useBullNose ? new BullNoseEndMill(10f, 30f, 2f) : new TaperEndMill(6f, 30f, 15f);
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
        public void SdfBackend_CutPoint_MatchesVariableRadiusGeometry(float resolution, bool useBullNose)
        {
            var bbox = StockBounds;
            var sdf = new SDFGrid(bbox, resolution, narrowBandWidth: 10);
            Tool tool = useBullNose ? new BullNoseEndMill(10f, 30f, 2f) : new TaperEndMill(6f, 30f, 15f);
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

        [Test]
        public void ToolGeometry_Solid_StaysWithinLocalBounds()
        {
            IToolGeometry[] geometries =
            {
                new FlatEndMillGeometry(5f, 30f),
                new BallEndMillGeometry(5f, 30f),
                new BallEndMillGeometry(5f, 6f),          // cutting length < 2 * radius
                new BullNoseEndMillGeometry(5f, 2f, 30f),
                new BullNoseEndMillGeometry(5f, 3f, 4f),  // cutting length < 2 * corner radius
                new BullNoseEndMillGeometry(5f, 4f, 3f),  // corner radius > cutting length
                new TaperedEndMillGeometry(2f, 10f, 20f),
            };

            foreach (IToolGeometry geometry in geometries)
            {
                BoundingBox bounds = geometry.LocalBounds;
                const float margin = 2f;

                for (float z = bounds.Min.Z - margin; z <= bounds.Max.Z + margin; z += 0.5f)
                    for (float y = bounds.Min.Y - margin; y <= bounds.Max.Y + margin; y += 0.5f)
                        for (float x = bounds.Min.X - margin; x <= bounds.Max.X + margin; x += 0.5f)
                        {
                            var point = new Vector3(x, y, z);
                            if (geometry.SignedDistance(point) >= 0f) continue;

                            Assert.That(bounds.Contains(point), Is.True,
                                $"{geometry.GetType().Name}: inside point {point} is outside LocalBounds {bounds}");
                        }
            }
        }

        [Test]
        public void NonAxisymmetricGeometry_LosesYExtent()
        {
            // Documented limitation: the production path maps world points to (radial, 0, axial),
            // so a non-axisymmetric solid cannot be represented. An elliptical geometry
            // (X radius 4, Y radius 1) is treated as a solid of revolution with radius 4.
            var geometry = new TestEllipticalGeometry(xRadius: 4f, yRadius: 1f, length: 10f);
            var tool = new TestGeometryTool(geometry, diameter: 8f, length: 10f);

            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(25, 25, 25));
            var grid = new VoxelGrid(bbox, 1.0f);
            new CutterSimulator(grid).CutPoint(Vector3.Zero, tool);

            // The true ellipse keeps (0, 2, 5) outside (Y radius is 1)...
            Assert.That(geometry.SignedDistance(new Vector3(0f, 2f, 5f)), Is.GreaterThan(0f));
            // ...but the production cut removes it anyway (Y information is discarded).
            Assert.That(grid.GetVoxelAtWorld(new Vector3(0f, 2f, 5f)), Is.False,
                "Known limitation: non-axisymmetric geometry is treated as a solid of revolution");

            // The X extent is honoured because it matches the collapsed radial direction.
            Assert.That(geometry.SignedDistance(new Vector3(2f, 0f, 5f)), Is.LessThan(0f));
            Assert.That(grid.GetVoxelAtWorld(new Vector3(2f, 0f, 5f)), Is.False);
        }
    }
}
