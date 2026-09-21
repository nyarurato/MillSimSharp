using System;
using System.Numerics;
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;
using NUnit.Framework;

namespace MillSimSharp.Tests.Simulation
{
    /// <summary>
    /// Differential / metamorphic tests: the same physical removal executed through different
    /// production paths (segmented cuts, reversed paths, translated setups, voxel vs SDF backend)
    /// must produce the same result. Expected values are never computed with the production
    /// helpers under test.
    /// </summary>
    [TestFixture]
    public class CutInvarianceTest
    {
        private static BoundingBox StockBounds =>
            BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));

        private static Tool CreateTool(string name)
        {
            return name switch
            {
                "Flat" => new EndMill(10f, 30f, isBallEnd: false),
                "Ball" => new EndMill(10f, 30f, isBallEnd: true),
                "BullNose" => new BullNoseEndMill(10f, 30f, cornerRadius: 2f),
                "Taper" => new TaperEndMill(tipDiameter: 4f, length: 30f, taperAngleDegrees: 10f),
                _ => throw new ArgumentException($"Unknown tool {name}", nameof(name)),
            };
        }

        private static void AssertSameOccupancy(VoxelGrid expected, VoxelGrid actual, string message)
        {
            var (sx, sy, sz) = expected.Dimensions;
            int differences = 0;

            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                        if (expected.GetVoxel(x, y, z) != actual.GetVoxel(x, y, z)) differences++;

            Assert.That(differences, Is.EqualTo(0), message);
        }

        private static int CountRemoved(VoxelGrid grid)
        {
            var (sx, sy, sz) = grid.Dimensions;
            return sx * sy * sz - grid.CountMaterialVoxels();
        }

        // ---------------------------------------------------------------------
        // T1: linear cut split invariance
        // ---------------------------------------------------------------------

        [TestCase("Flat")]
        [TestCase("Ball")]
        [TestCase("BullNose")]
        [TestCase("Taper")]
        public void LinearCut_SplitVsSingle_ProducesSameOccupancy(string toolName)
        {
            const float resolution = 1.0f;
            Tool tool = CreateTool(toolName);
            var start = new Vector3(-6, 0, 0);
            var end = new Vector3(6, 0, 0);

            var single = new VoxelGrid(StockBounds, resolution);
            new CutterSimulator(single).CutLinear(start, end, tool);

            var split = new VoxelGrid(StockBounds, resolution);
            var simulator = new CutterSimulator(split);
            for (int i = 0; i < 4; i++)
            {
                simulator.CutLinear(
                    Vector3.Lerp(start, end, i / 4f),
                    Vector3.Lerp(start, end, (i + 1) / 4f),
                    tool);
            }

            Assert.That(CountRemoved(single), Is.GreaterThan(100), "positive control: the cut removes material");
            AssertSameOccupancy(single, split,
                "Splitting an axis-perpendicular straight cut must not change the removed voxels");
        }

        [Test]
        public void LinearCut_SplitVsSingle_ProducesSameOccupancy_ResolutionHalf()
        {
            const float resolution = 0.5f;
            Tool tool = CreateTool("Flat");
            var start = new Vector3(-6, 0, 0);
            var end = new Vector3(6, 0, 0);

            var single = new VoxelGrid(StockBounds, resolution);
            new CutterSimulator(single).CutLinear(start, end, tool);

            var split = new VoxelGrid(StockBounds, resolution);
            var simulator = new CutterSimulator(split);
            for (int i = 0; i < 8; i++)
            {
                simulator.CutLinear(
                    Vector3.Lerp(start, end, i / 8f),
                    Vector3.Lerp(start, end, (i + 1) / 8f),
                    tool);
            }

            AssertSameOccupancy(single, split, "Split invariance must also hold at resolution 0.5");
        }

        // ---------------------------------------------------------------------
        // T2: path reversal invariance
        // ---------------------------------------------------------------------

        [TestCase("Flat")]
        [TestCase("Ball")]
        [TestCase("BullNose")]
        [TestCase("Taper")]
        public void LinearCut_ReversedPath_ProducesSameOccupancy(string toolName)
        {
            const float resolution = 1.0f;
            Tool tool = CreateTool(toolName);
            var a = new Vector3(-5, 0, 0);
            var b = new Vector3(5, 0, 0);

            var forward = new VoxelGrid(StockBounds, resolution);
            new CutterSimulator(forward).CutLinear(a, b, tool);

            var backward = new VoxelGrid(StockBounds, resolution);
            new CutterSimulator(backward).CutLinear(b, a, tool);

            Assert.That(CountRemoved(forward), Is.GreaterThan(100));
            AssertSameOccupancy(forward, backward, "A->B and B->A must remove the same voxels");
        }

        [Test]
        public void LinearCut_ReversedPath_ProducesSameSigns_Sdf()
        {
            Tool tool = CreateTool("Ball");
            var a = new Vector3(-5, 0, 0);
            var b = new Vector3(5, 0, 0);

            var forward = new SDFGrid(StockBounds, 1.0f, narrowBandWidth: 8);
            new SDFCutterSimulator(forward).CutLinear(a, b, tool);

            var backward = new SDFGrid(StockBounds, 1.0f, narrowBandWidth: 8);
            new SDFCutterSimulator(backward).CutLinear(b, a, tool);

            var (sx, sy, sz) = forward.Dimensions;
            int signDifferences = 0;
            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                        if ((forward.GetDistance(x, y, z) < 0f) != (backward.GetDistance(x, y, z) < 0f))
                            signDifferences++;

            Assert.That(signDifferences, Is.EqualTo(0), "SDF sign pattern must be path-direction independent");
        }

        // ---------------------------------------------------------------------
        // T3: translation invariance
        // ---------------------------------------------------------------------

        [Test]
        public void LinearCut_TranslationInvariance_Holds()
        {
            const float resolution = 0.5f;
            Tool tool = CreateTool("Flat");

            // Translation is a multiple of the resolution so voxel phases are preserved; the
            // original bounds have a non-zero Min.
            var translation = new Vector3(7.5f, -4.0f, 2.5f);
            var originalBounds = new BoundingBox(new Vector3(-5, -5, -5), new Vector3(35, 35, 35));
            var translatedBounds = new BoundingBox(originalBounds.Min + translation, originalBounds.Max + translation);

            var start = new Vector3(0, 0, 0);
            var end = new Vector3(10, 0, 0);

            var original = new VoxelGrid(originalBounds, resolution);
            new CutterSimulator(original).CutLinear(start, end, tool);

            var translated = new VoxelGrid(translatedBounds, resolution);
            new CutterSimulator(translated).CutLinear(start + translation, end + translation, tool);

            Assert.That(translated.Dimensions, Is.EqualTo(original.Dimensions));
            Assert.That(CountRemoved(original), Is.GreaterThan(100));

            var (sx, sy, sz) = original.Dimensions;
            int differences = 0;
            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                        if (original.GetVoxel(x, y, z) != translated.GetVoxel(x, y, z)) differences++;

            Assert.That(differences, Is.EqualTo(0),
                "A translated setup (non-zero Min, resolution 0.5) must produce the same voxel pattern");
        }

        // ---------------------------------------------------------------------
        // T4: axisymmetric tool-local roll invariance
        // ---------------------------------------------------------------------

        [TestCase("Flat")]
        [TestCase("Ball")]
        [TestCase("BullNose")]
        [TestCase("Taper")]
        public void RotationOnly_ToolLocalRoll_RemovesSameVoxels(string toolName)
        {
            const float resolution = 1.0f;
            Tool tool = CreateTool(toolName);
            var pose = new ToolOrientation(25, -15, 0);
            var rolled = new ToolOrientation(25, -15, 90); // same tool axis direction, roll about it

            var noRoll = new VoxelGrid(StockBounds, resolution);
            var simulatorA = new CutterSimulator(noRoll);
            ConfigureSingleStepSampling(simulatorA);
            simulatorA.CutLinearWithOrientation(Vector3.Zero, Vector3.Zero, tool, pose, pose);

            var withRoll = new VoxelGrid(StockBounds, resolution);
            var simulatorB = new CutterSimulator(withRoll);
            ConfigureSingleStepSampling(simulatorB);
            simulatorB.CutLinearWithOrientation(Vector3.Zero, Vector3.Zero, tool, pose, rolled);

            Assert.That(CountRemoved(noRoll), Is.GreaterThan(20), "positive control: the static tool removes material");
            AssertSameOccupancy(noRoll, withRoll,
                "A tool-local roll must not change the removal of an axisymmetric tool");
        }

        private static void ConfigureSingleStepSampling(ICutterSimulator simulator)
        {
            // Both setups sample the same pose set (t = 0 and t = 1) regardless of the roll angle.
            simulator.Settings.EnableAdaptiveSampling = false;
            simulator.Settings.MaxLinearStep = 1000f;
            simulator.Settings.MaxAngularStep = 90f;
        }

        // ---------------------------------------------------------------------
        // T5: voxel / SDF backend differential
        // ---------------------------------------------------------------------

        private readonly struct BackendDifference
        {
            public BackendDifference(int deepMismatches, int surfaceSamples, float maxMismatchDistance)
            {
                DeepMismatches = deepMismatches;
                SurfaceSamples = surfaceSamples;
                MaxMismatchDistance = maxMismatchDistance;
            }

            public int DeepMismatches { get; }
            public int SurfaceSamples { get; }
            public float MaxMismatchDistance { get; }

            public override string ToString() =>
                $"deep={DeepMismatches} surface={SurfaceSamples} maxMismatchDistance={MaxMismatchDistance:F4}";
        }

        /// <summary>
        /// Compares the voxel occupancy with the SDF sign at every voxel center. Samples within
        /// <paramref name="margin"/> of the surface are counted separately: the SDF is rebuilt by
        /// CSG + EDT repair, so sign flips within roughly one voxel of the surface are expected.
        /// </summary>
        private static BackendDifference CompareBackends(VoxelGrid grid, SDFGrid sdf, float margin)
        {
            var (sx, sy, sz) = grid.Dimensions;
            int deepMismatches = 0;
            int surfaceSamples = 0;
            float maxMismatchDistance = 0;

            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                    {
                        float distance = sdf.GetDistance(x, y, z);
                        if (MathF.Abs(distance) <= margin)
                        {
                            surfaceSamples++;
                            continue;
                        }

                        bool voxelMaterial = grid.GetVoxel(x, y, z);
                        bool sdfMaterial = distance < 0f;
                        if (voxelMaterial != sdfMaterial)
                        {
                            deepMismatches++;
                            if (MathF.Abs(distance) > maxMismatchDistance)
                                maxMismatchDistance = MathF.Abs(distance);
                        }
                    }

            return new BackendDifference(deepMismatches, surfaceSamples, maxMismatchDistance);
        }

        [TestCase("Flat")]
        [TestCase("Ball")]
        [TestCase("BullNose")]
        [TestCase("Taper")]
        public void VoxelAndSdfBackends_LinearCut_AgreeOutsideSurfaceMargin(string toolName)
        {
            const float resolution = 1.0f;
            const float margin = 1.0f * resolution; // CSG + EDT repair can flip signs within ~1 voxel
            Tool tool = CreateTool(toolName);
            var start = new Vector3(-5, 0, 0);
            var end = new Vector3(5, 0, 0);

            var grid = new VoxelGrid(StockBounds, resolution);
            new CutterSimulator(grid).CutLinear(start, end, tool);

            var sdf = new SDFGrid(StockBounds, resolution, narrowBandWidth: 8);
            new SDFCutterSimulator(sdf).CutLinear(start, end, tool);

            var difference = CompareBackends(grid, sdf, margin);
            TestContext.Out.WriteLine($"{toolName} linear cut: {difference}");

            Assert.That(difference.DeepMismatches, Is.EqualTo(0),
                "Outside the surface margin the backends must agree on material/air");
            Assert.That(difference.SurfaceSamples, Is.GreaterThan(0), "the surface band must not be empty");
        }

        [Test]
        public void VoxelAndSdfBackends_RotationOnly_AgreeOutsideSurfaceMargin()
        {
            const float resolution = 1.0f;
            Tool tool = CreateTool("Flat");

            var grid = new VoxelGrid(StockBounds, resolution);
            new CutterSimulator(grid).CutLinearWithOrientation(
                Vector3.Zero, Vector3.Zero, tool, new ToolOrientation(0, 0, 0), new ToolOrientation(90, 0, 0));

            var sdf = new SDFGrid(StockBounds, resolution, narrowBandWidth: 8);
            new SDFCutterSimulator(sdf).CutLinearWithOrientation(
                Vector3.Zero, Vector3.Zero, tool, new ToolOrientation(0, 0, 0), new ToolOrientation(90, 0, 0));

            var difference = CompareBackends(grid, sdf, 1.0f * resolution);
            TestContext.Out.WriteLine($"rotation-only: {difference}");

            Assert.That(difference.DeepMismatches, Is.EqualTo(0));
            Assert.That(difference.SurfaceSamples, Is.GreaterThan(0));
        }

        [Test]
        public void VoxelAndSdfBackends_ShortLinearLargeRotation_AgreeOutsideSurfaceMargin()
        {
            const float resolution = 0.5f;
            Tool tool = CreateTool("BullNose");
            var start = new Vector3(0, 0, 0);
            var end = new Vector3(0.5f, 0, 0);

            var grid = new VoxelGrid(StockBounds, resolution);
            new CutterSimulator(grid).CutLinearWithOrientation(
                start, end, tool, new ToolOrientation(0, 0, 0), new ToolOrientation(75, 0, 0));

            var sdf = new SDFGrid(StockBounds, resolution, narrowBandWidth: 8);
            new SDFCutterSimulator(sdf).CutLinearWithOrientation(
                start, end, tool, new ToolOrientation(0, 0, 0), new ToolOrientation(75, 0, 0));

            var difference = CompareBackends(grid, sdf, 1.0f * resolution);
            TestContext.Out.WriteLine($"short linear + large rotation: {difference}");

            Assert.That(difference.DeepMismatches, Is.EqualTo(0));
            Assert.That(difference.SurfaceSamples, Is.GreaterThan(0));
        }
    }
}
