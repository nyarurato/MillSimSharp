using System;
using System.Numerics;
using MillSimSharp.Geometry;
using NUnit.Framework;

namespace MillSimSharp.Tests.Geometry
{
    /// <summary>
    /// Contract tests for material state ownership between VoxelGrid and SDFGrid:
    /// FromVoxelGrid = one-time snapshot, BindToVoxelGrid = subscription for future changes only,
    /// SyncFromVoxelGrid = full rebuild from the source grid, UnbindFromVoxelGrid = SDF-native mode.
    /// </summary>
    [TestFixture]
    public class SDFStateContractTest
    {
        private static BoundingBox StockBounds =>
            BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));

        private static VoxelGrid CreateGrid() => new VoxelGrid(StockBounds, 1.0f);

        private static void AssertAllDistancesEqual(SDFGrid expected, SDFGrid actual)
        {
            var (sx, sy, sz) = expected.Dimensions;
            Assert.That(actual.Dimensions, Is.EqualTo(expected.Dimensions));

            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                        Assert.That(actual.GetDistance(x, y, z), Is.EqualTo(expected.GetDistance(x, y, z)),
                            $"distance differs at ({x},{y},{z})");
        }

        [Test]
        public void FromVoxelGrid_IsSnapshot_NotLive()
        {
            var grid = CreateGrid();
            var sdf = SDFGrid.FromVoxelGrid(grid);
            float centerBefore = sdf.GetDistance(Vector3.Zero);
            Assert.That(centerBefore, Is.LessThan(0f), "the stock starts as material");

            grid.RemoveVoxelsInSphere(Vector3.Zero, 3f);

            Assert.That(sdf.GetDistance(Vector3.Zero), Is.EqualTo(centerBefore),
                "voxel edits after FromVoxelGrid must not be observed (snapshot)");

            var rebuilt = SDFGrid.FromVoxelGrid(grid);
            Assert.That(rebuilt.GetDistance(Vector3.Zero), Is.GreaterThan(0f),
                "a fresh rebuild sees the edit (negative control)");
        }

        [Test]
        public void BindToVoxelGrid_DoesNotSyncExistingValues()
        {
            var grid = CreateGrid();
            var sdf = SDFGrid.FromVoxelGrid(grid);

            grid.RemoveVoxelsInSphere(Vector3.Zero, 3f);
            float stale = sdf.GetDistance(Vector3.Zero);

            sdf.BindToVoxelGrid(grid);

            Assert.That(sdf.GetDistance(Vector3.Zero), Is.EqualTo(stale),
                "binding must not synchronize edits made before the call");

            grid.RemoveVoxelsInSphere(new Vector3(6, 0, 0), 2f);

            Assert.That(sdf.GetDistance(new Vector3(6, 0, 0)), Is.GreaterThan(0f),
                "edits made after binding must be applied incrementally");
        }

        [Test]
        public void SyncFromVoxelGrid_MatchesFullRebuild()
        {
            var grid = CreateGrid();
            var sdf = SDFGrid.FromVoxelGrid(grid);
            sdf.BindToVoxelGrid(grid);

            sdf.UnbindFromVoxelGrid();
            grid.RemoveVoxelsInSphere(Vector3.Zero, 4f);
            grid.SetVoxelAtWorld(new Vector3(5, 5, 0), false);

            sdf.BindToVoxelGrid(grid);
            sdf.SyncFromVoxelGrid();

            AssertAllDistancesEqual(SDFGrid.FromVoxelGrid(grid), sdf);
        }

        [Test]
        public void BoundSdf_DirectCarve_ThenVoxelEdit_IsOverwritten()
        {
            var grid = CreateGrid();
            var sdf = SDFGrid.FromVoxelGrid(grid);
            sdf.BindToVoxelGrid(grid);

            sdf.RemoveSphere(Vector3.Zero, 3f);
            Assert.That(sdf.GetDistance(Vector3.Zero), Is.GreaterThan(0f),
                "the direct carve is visible while no voxel edit has touched the region");

            // A voxel edit inside the same rebuild region discards the direct carve.
            grid.RemoveVoxelsInSphere(new Vector3(2, 0, 0), 1f);

            Assert.That(sdf.GetDistance(Vector3.Zero), Is.LessThan(0f),
                "the voxel-driven rebuild must overwrite the direct carve in its region");
        }

        [Test]
        public void UnboundSdf_DirectCarve_IsPreserved()
        {
            var grid = CreateGrid();
            var sdf = SDFGrid.FromVoxelGrid(grid);
            sdf.BindToVoxelGrid(grid);
            sdf.UnbindFromVoxelGrid();

            sdf.RemoveSphere(Vector3.Zero, 3f);
            grid.RemoveVoxelsInSphere(Vector3.Zero, 1f); // must not be observed while unbound

            Assert.That(sdf.GetDistance(Vector3.Zero), Is.GreaterThan(0f),
                "SDF-native carving must be preserved while unbound");
        }

        [Test]
        public void UnbindRebind_WithoutSync_KeepsValues()
        {
            var grid = CreateGrid();
            var sdf = SDFGrid.FromVoxelGrid(grid);
            sdf.BindToVoxelGrid(grid);

            grid.RemoveVoxelsInSphere(new Vector3(4, 0, 0), 2f);
            float synced = sdf.GetDistance(new Vector3(4, 0, 0));
            Assert.That(synced, Is.GreaterThan(0f));

            sdf.UnbindFromVoxelGrid();
            grid.RemoveVoxelsInSphere(Vector3.Zero, 3f);
            sdf.BindToVoxelGrid(grid);

            Assert.That(sdf.GetDistance(new Vector3(4, 0, 0)), Is.EqualTo(synced),
                "rebinding must not resynchronize existing values");
            Assert.That(sdf.GetDistance(Vector3.Zero), Is.LessThan(0f),
                "the edit made while unbound must still not be applied");
        }

        [Test]
        public void UnbindRebind_AfterSync_MatchesFullRebuild()
        {
            var grid = CreateGrid();
            var sdf = SDFGrid.FromVoxelGrid(grid);
            sdf.BindToVoxelGrid(grid);

            sdf.UnbindFromVoxelGrid();
            grid.RemoveVoxelsInSphere(Vector3.Zero, 3f);
            sdf.BindToVoxelGrid(grid);

            sdf.SyncFromVoxelGrid();

            AssertAllDistancesEqual(SDFGrid.FromVoxelGrid(grid), sdf);
        }

        [Test]
        public void SyncFromVoxelGrid_WithoutSource_Throws()
        {
            var native = new SDFGrid(StockBounds, 1.0f);

            Assert.Throws<InvalidOperationException>(() => native.SyncFromVoxelGrid());
        }
    }
}
