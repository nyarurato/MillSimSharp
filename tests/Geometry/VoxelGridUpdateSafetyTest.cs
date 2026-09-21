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
        public void VoxelGrid_ParallelAndSerialRemoval_MixedSequenceProducesSameState()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(60, 60, 60));
            var parallel = new VoxelGrid(bbox, 0.5f) { UseParallelRemoval = true };
            var serial = new VoxelGrid(bbox, 0.5f) { UseParallelRemoval = false };

            foreach (VoxelGrid grid in new[] { parallel, serial })
            {
                grid.RemoveVoxelsInSphere(new Vector3(-10, -5, 0), 8f);
                grid.RemoveVoxelsInCylinder(new Vector3(10, -20, -5), new Vector3(10, 20, 5), 5f, flatEnds: true);
                grid.RemoveVoxelsInCylinder(new Vector3(-15, 0, 0), new Vector3(15, 0, 0), 4f, flatEnds: false);
                grid.SetVoxel(5, 5, 5, false);
                grid.SetVoxelAtWorld(new Vector3(20.5f, -20.5f, 10.5f), false);
                grid.RemoveVoxelsInSphere(new Vector3(5, 5, 5), 6f);
            }

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
                "A mixed removal sequence must produce identical voxel states for both modes");
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

        // ---------------------------------------------------------------------
        // V2 audit / P5: public SetVoxel / SetVoxelAtWorld / Clear notifications
        // ---------------------------------------------------------------------

        [Test]
        public void PublicSetVoxel_ChangedValue_FiresOnce()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var grid = new VoxelGrid(bbox, 1.0f);
            int eventCount = 0;
            grid.VoxelsChanged += (a, b, c, d, e, f) => eventCount++;

            grid.SetVoxel(3, 4, 5, false);

            Assert.That(eventCount, Is.EqualTo(1), "A changed SetVoxel must notify exactly once");
        }

        [Test]
        public void PublicSetVoxel_SameValue_DoesNotFire()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var grid = new VoxelGrid(bbox, 1.0f);

            grid.SetVoxel(3, 4, 5, false); // first change
            int eventCount = 0;
            grid.VoxelsChanged += (a, b, c, d, e, f) => eventCount++;

            grid.SetVoxel(3, 4, 5, false); // same value: no change
            grid.SetVoxel(0, 0, 0, true);  // already material: no change

            Assert.That(eventCount, Is.EqualTo(0), "Setting the same value must not notify");
        }

        [Test]
        public void PublicSetVoxelAtWorld_ChangedValue_FiresOnce()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var grid = new VoxelGrid(bbox, 1.0f);
            int eventCount = 0;
            grid.VoxelsChanged += (a, b, c, d, e, f) => eventCount++;

            grid.SetVoxelAtWorld(new Vector3(0.5f, 1.5f, 2.5f), false);

            Assert.That(eventCount, Is.EqualTo(1), "A changed SetVoxelAtWorld must notify exactly once");
        }

        [Test]
        public void BulkSphereRemoval_FiresExactlyOnce()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));
            var grid = new VoxelGrid(bbox, 0.5f);
            int eventCount = 0;
            grid.VoxelsChanged += (a, b, c, d, e, f) => eventCount++;

            grid.RemoveVoxelsInSphere(new Vector3(3, 0, 0), 4f);

            Assert.That(eventCount, Is.EqualTo(1),
                "Bulk sphere removal must report exactly one aggregated region");
        }

        [Test]
        public void BulkCylinderRemoval_FiresExactlyOnce()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));
            var grid = new VoxelGrid(bbox, 0.5f);
            int eventCount = 0;
            grid.VoxelsChanged += (a, b, c, d, e, f) => eventCount++;

            grid.RemoveVoxelsInCylinder(new Vector3(-5, 0, 0), new Vector3(5, 0, 0), 3f);

            Assert.That(eventCount, Is.EqualTo(1),
                "Bulk cylinder removal must report exactly one aggregated region");
        }

        [Test]
        public void Clear_WhenAlreadyMaterial_DoesNotFire()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var grid = new VoxelGrid(bbox, 1.0f);
            int eventCount = 0;
            grid.VoxelsChanged += (a, b, c, d, e, f) => eventCount++;

            grid.Clear();

            Assert.That(eventCount, Is.EqualTo(0), "Clearing an all-material grid must not notify");
        }

        [Test]
        public void Clear_AfterRemoval_FiresExactlyOnce()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var grid = new VoxelGrid(bbox, 1.0f);
            grid.RemoveVoxelsInSphere(Vector3.Zero, 2f);
            int eventCount = 0;
            grid.VoxelsChanged += (a, b, c, d, e, f) => eventCount++;

            grid.Clear();

            Assert.That(eventCount, Is.EqualTo(1), "Clearing a modified grid must notify exactly once");
        }
    }
}
