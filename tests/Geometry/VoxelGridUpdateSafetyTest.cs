using System;
using System.Collections.Generic;
using System.Numerics;
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using NUnit.Framework;

namespace MillSimSharp.Tests.Geometry
{
    /// <summary>
    /// Tests for voxel update safety: deterministic parallel/serial removal and aggregated
    /// dirty-bounds reporting via BeginEdit / EndEdit.
    /// </summary>
    [TestFixture]
    public class VoxelGridUpdateSafetyTest
    {
        private static (int minX, int minY, int minZ, int maxX, int maxY, int maxZ) RemovedBounds(VoxelGrid grid)
        {
            var dense = grid.ToDenseArray();
            var (sx, sy, sz) = grid.Dimensions;

            int minX = int.MaxValue, minY = int.MaxValue, minZ = int.MaxValue;
            int maxX = int.MinValue, maxY = int.MinValue, maxZ = int.MinValue;

            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                    {
                        if (dense[x][y][z]) continue;
                        if (x < minX) minX = x;
                        if (y < minY) minY = y;
                        if (z < minZ) minZ = z;
                        if (x > maxX) maxX = x;
                        if (y > maxY) maxY = y;
                        if (z > maxZ) maxZ = z;
                    }

            return (minX, minY, minZ, maxX, maxY, maxZ);
        }

        [Test]
        public void VoxelGrid_ParallelAndSerialRemoval_ProduceSameState()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(60, 60, 60));
            var parallel = new VoxelGrid(bbox, 0.5f) { UseParallelRemoval = true };
            var serial = new VoxelGrid(bbox, 0.5f) { UseParallelRemoval = false };

            // These volumes are well above the parallel threshold.
            parallel.RemoveVoxelsInSphere(new Vector3(-5, 0, 0), 10f);
            serial.RemoveVoxelsInSphere(new Vector3(-5, 0, 0), 10f);
            parallel.RemoveVoxelsInCylinder(new Vector3(5, -20, 0), new Vector3(5, 20, 0), 6f);
            serial.RemoveVoxelsInCylinder(new Vector3(5, -20, 0), new Vector3(5, 20, 0), 6f);

            var denseP = parallel.ToDenseArray();
            var denseS = serial.ToDenseArray();
            var (sx, sy, sz) = parallel.Dimensions;

            int differences = 0;
            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                    {
                        if (denseP[x][y][z] != denseS[x][y][z]) differences++;
                    }

            Assert.That(differences, Is.EqualTo(0),
                "Parallel and serial removal must produce identical voxel states");
        }

        [Test]
        public void VoxelsChanged_FiresOncePerEdit_WithAggregatedBounds()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));
            var grid = new VoxelGrid(bbox, 1.0f);
            var events = new List<(int minX, int minY, int minZ, int maxX, int maxY, int maxZ)>();
            grid.VoxelsChanged += (minX, minY, minZ, maxX, maxY, maxZ) =>
                events.Add((minX, minY, minZ, maxX, maxY, maxZ));

            grid.BeginEdit();
            grid.RemoveVoxelsInSphere(new Vector3(-10, 0, 0), 2f);
            grid.RemoveVoxelsInSphere(new Vector3(10, 0, 0), 2f);
            Assert.That(events, Is.Empty, "VoxelsChanged must not fire during a batch edit");
            grid.EndEdit();

            Assert.That(events.Count, Is.EqualTo(1), "VoxelsChanged must fire exactly once per batch edit");

            var expected = RemovedBounds(grid);
            var actual = events[0];
            Assert.That(actual.minX, Is.EqualTo(expected.minX));
            Assert.That(actual.minY, Is.EqualTo(expected.minY));
            Assert.That(actual.minZ, Is.EqualTo(expected.minZ));
            Assert.That(actual.maxX, Is.EqualTo(expected.maxX));
            Assert.That(actual.maxY, Is.EqualTo(expected.maxY));
            Assert.That(actual.maxZ, Is.EqualTo(expected.maxZ));

            // The aggregated bounds must cover both distant cuts (not just the last one).
            Assert.That(actual.minX, Is.LessThan(15));
            Assert.That(actual.maxX, Is.GreaterThan(25));
        }

        [Test]
        public void VoxelsChanged_DoesNotFire_WhenEditHasNoChanges()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));
            var grid = new VoxelGrid(bbox, 1.0f);
            grid.RemoveVoxelsInSphere(Vector3.Zero, 3f);

            var events = new List<(int, int, int, int, int, int)>();
            grid.VoxelsChanged += (a, b, c, d, e, f) => events.Add((a, b, c, d, e, f));

            grid.BeginEdit();
            grid.RemoveVoxelsInSphere(Vector3.Zero, 3f); // already empty, no change
            grid.EndEdit();

            Assert.That(events, Is.Empty, "No change must produce no VoxelsChanged event");
        }

        [Test]
        public void EndEdit_WithoutBeginEdit_Throws()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var grid = new VoxelGrid(bbox, 1.0f);

            Assert.Throws<InvalidOperationException>(() => grid.EndEdit());
        }

        [Test]
        public void NestedEdits_FireOnlyOnce()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));
            var grid = new VoxelGrid(bbox, 1.0f);
            var events = new List<(int, int, int, int, int, int)>();
            grid.VoxelsChanged += (a, b, c, d, e, f) => events.Add((a, b, c, d, e, f));

            grid.BeginEdit();
            grid.RemoveVoxelsInSphere(new Vector3(-5, 0, 0), 2f);
            grid.BeginEdit();
            grid.RemoveVoxelsInSphere(new Vector3(5, 0, 0), 2f);
            grid.EndEdit();
            Assert.That(events, Is.Empty);
            grid.EndEdit();

            Assert.That(events.Count, Is.EqualTo(1), "Nested edits must report once at the outermost EndEdit");
        }

        [Test]
        public void CutterSimulator_CutLinear_RaisesSingleAggregatedEvent()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));
            var grid = new VoxelGrid(bbox, 1.0f);
            int eventCount = 0;
            grid.VoxelsChanged += (a, b, c, d, e, f) => eventCount++;

            var simulator = new CutterSimulator(grid);
            var tool = new EndMill(10f, 30f, isBallEnd: false);
            simulator.CutLinear(new Vector3(-5, 0, 0), new Vector3(5, 0, 0), tool);

            Assert.That(eventCount, Is.EqualTo(1),
                "A cutting move must report a single aggregated dirty region");
        }

        [Test]
        public void CutterSimulator_CutPoint_RaisesSingleAggregatedEvent()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));
            var grid = new VoxelGrid(bbox, 1.0f);
            int eventCount = 0;
            grid.VoxelsChanged += (a, b, c, d, e, f) => eventCount++;

            var simulator = new CutterSimulator(grid);
            var tool = new EndMill(10f, 30f, isBallEnd: true);
            simulator.CutPoint(Vector3.Zero, tool);

            Assert.That(eventCount, Is.EqualTo(1),
                "A point cut must report a single aggregated dirty region");
        }
    }
}
