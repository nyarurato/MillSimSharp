using System;
using System.Numerics;
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using NUnit.Framework;

namespace MillSimSharp.Tests.Geometry
{
    /// <summary>
    /// Differential tests for incremental SDF updates and idempotence: a sequence of edits must
    /// end in the same field as a full rebuild, and repeating the same removal must not change
    /// the result.
    /// </summary>
    [TestFixture]
    public class SdfSequenceTest
    {
        private static BoundingBox StockBounds =>
            BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));

        private static float MaxDistanceDifference(SDFGrid a, SDFGrid b)
        {
            var (sx, sy, sz) = a.Dimensions;
            float maxDifference = 0;
            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                    {
                        float difference = MathF.Abs(a.GetDistance(x, y, z) - b.GetDistance(x, y, z));
                        if (difference > maxDifference) maxDifference = difference;
                    }

            return maxDifference;
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

        [Test]
        public void BoundSdf_MixedEditSequence_MatchesFullRebuild()
        {
            const int narrowBand = 10;
            var grid = new VoxelGrid(StockBounds, 1.0f);
            var incremental = SDFGrid.FromVoxelGrid(grid, narrowBand);
            incremental.BindToVoxelGrid(grid);

            grid.RemoveVoxelsInSphere(new Vector3(-10, 0, 0), 3f);
            grid.RemoveVoxelsInCylinder(new Vector3(5, -8, 0), new Vector3(5, 8, 0), 2f);
            grid.SetVoxel(30, 20, 20, false);
            grid.SetVoxelAtWorld(new Vector3(-5.5f, 0.5f, 0.5f), false);
            new CutterSimulator(grid).CutLinear(new Vector3(0, 0, 0), new Vector3(8, 0, 0), new EndMill(6f, 20f, false));

            var full = SDFGrid.FromVoxelGrid(grid, narrowBand);

            Assert.That(MaxDistanceDifference(incremental, full), Is.LessThanOrEqualTo(1e-3f),
                "A mixed sequence of incremental edits must match a full rebuild");
        }

        [Test]
        public void BoundSdf_DisjointOperations_OrderIndependent()
        {
            const int narrowBand = 10;
            var bounds = StockBounds;

            var forwardGrid = new VoxelGrid(bounds, 1.0f);
            var forward = SDFGrid.FromVoxelGrid(forwardGrid, narrowBand);
            forward.BindToVoxelGrid(forwardGrid);
            forwardGrid.RemoveVoxelsInSphere(new Vector3(-12, 0, 0), 3f);
            forwardGrid.RemoveVoxelsInSphere(new Vector3(12, 0, 0), 3f);

            var backwardGrid = new VoxelGrid(bounds, 1.0f);
            var backward = SDFGrid.FromVoxelGrid(backwardGrid, narrowBand);
            backward.BindToVoxelGrid(backwardGrid);
            backwardGrid.RemoveVoxelsInSphere(new Vector3(12, 0, 0), 3f);
            backwardGrid.RemoveVoxelsInSphere(new Vector3(-12, 0, 0), 3f);

            AssertSameOccupancy(forwardGrid, backwardGrid, "disjoint voxel edits commute");
            Assert.That(MaxDistanceDifference(forward, backward), Is.LessThanOrEqualTo(1e-3f),
                "Disjoint incremental updates must be order independent");
        }

        [Test]
        public void VoxelRemoval_Repeated_IsIdempotent()
        {
            var once = new VoxelGrid(StockBounds, 1.0f);
            once.RemoveVoxelsInSphere(Vector3.Zero, 5f);
            new CutterSimulator(once).CutLinear(new Vector3(-5, 0, 0), new Vector3(5, 0, 0), new EndMill(8f, 20f, false));

            var twice = new VoxelGrid(StockBounds, 1.0f);
            twice.RemoveVoxelsInSphere(Vector3.Zero, 5f);
            twice.RemoveVoxelsInSphere(Vector3.Zero, 5f);
            var simulator = new CutterSimulator(twice);
            simulator.CutLinear(new Vector3(-5, 0, 0), new Vector3(5, 0, 0), new EndMill(8f, 20f, false));
            simulator.CutLinear(new Vector3(-5, 0, 0), new Vector3(5, 0, 0), new EndMill(8f, 20f, false));

            AssertSameOccupancy(once, twice, "Repeating the same removal must not change the voxel state");
        }

        [Test]
        public void SdfRemoval_Repeated_IsIdempotent()
        {
            var once = new SDFGrid(StockBounds, 1.0f, narrowBandWidth: 8);
            once.RemoveSphere(Vector3.Zero, 5f);
            new SDFCutterSimulator(once).CutLinear(new Vector3(-5, 0, 0), new Vector3(5, 0, 0), new EndMill(8f, 20f, false));

            var twice = new SDFGrid(StockBounds, 1.0f, narrowBandWidth: 8);
            twice.RemoveSphere(Vector3.Zero, 5f);
            twice.RemoveSphere(Vector3.Zero, 5f);
            var simulator = new SDFCutterSimulator(twice);
            simulator.CutLinear(new Vector3(-5, 0, 0), new Vector3(5, 0, 0), new EndMill(8f, 20f, false));
            simulator.CutLinear(new Vector3(-5, 0, 0), new Vector3(5, 0, 0), new EndMill(8f, 20f, false));

            Assert.That(MaxDistanceDifference(once, twice), Is.LessThanOrEqualTo(1e-4f),
                "Repeating the same SDF removal must not change the field (RepairDistances regression)");
        }
    }
}
