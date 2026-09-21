using NUnit.Framework;
using MillSimSharp.Geometry;
using System.Numerics;

namespace MillSimSharp.Tests.Geometry
{
    [TestFixture]
    public class VoxelGridTest
    {
        [Test]
        public void TestConstruction()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var grid = new VoxelGrid(bbox, 1.0f);

            Assert.That(grid.Resolution, Is.EqualTo(1.0f));
            var (x, y, z) = grid.Dimensions;
            Assert.That(x, Is.EqualTo(10));
            Assert.That(y, Is.EqualTo(10));
            Assert.That(z, Is.EqualTo(10));
        }

        [Test]
        public void TestInitialState()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var grid = new VoxelGrid(bbox, 1.0f);

            // All voxels should be initialized as material (true)
            for (int z = 0; z < 10; z++)
            {
                for (int y = 0; y < 10; y++)
                {
                    for (int x = 0; x < 10; x++)
                    {
                        Assert.That(grid.GetVoxel(x, y, z), Is.True, $"Voxel at ({x},{y},{z}) should be material");
                    }
                }
            }
        }

        [Test]
        public void TestSetAndGetVoxel()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var grid = new VoxelGrid(bbox, 1.0f);

            grid.SetVoxel(5, 5, 5, false);
            Assert.That(grid.GetVoxel(5, 5, 5), Is.False);
            Assert.That(grid.GetVoxel(4, 5, 5), Is.True);
        }

        [Test]
        public void TestOutOfBounds()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var grid = new VoxelGrid(bbox, 1.0f);

            // Out of bounds should return false
            Assert.That(grid.GetVoxel(-1, 5, 5), Is.False);
            Assert.That(grid.GetVoxel(100, 5, 5), Is.False);
        }

        [Test]
        public void TestRemoveVoxelsInSphere()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var grid = new VoxelGrid(bbox, 0.5f);

            // Remove sphere at center with radius 1.0
            grid.RemoveVoxelsInSphere(Vector3.Zero, 1.0f);

            // Check that voxels near center are removed
            Assert.That(grid.GetVoxelAtWorld(Vector3.Zero), Is.False);
            Assert.That(grid.GetVoxelAtWorld(new Vector3(0.5f, 0, 0)), Is.False);

            // Check that voxels far from center remain
            Assert.That(grid.GetVoxelAtWorld(new Vector3(3, 3, 3)), Is.True);
        }

        [Test]
        public void TestRemoveVoxelsInCylinder()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var grid = new VoxelGrid(bbox, 0.5f);

            // Remove cylinder along X-axis
            var start = new Vector3(-2, 0, 0);
            var end = new Vector3(2, 0, 0);
            grid.RemoveVoxelsInCylinder(start, end, 0.5f);

            // Check that voxels along axis are removed
            Assert.That(grid.GetVoxelAtWorld(Vector3.Zero), Is.False);
            Assert.That(grid.GetVoxelAtWorld(new Vector3(1, 0, 0)), Is.False);
            Assert.That(grid.GetVoxelAtWorld(new Vector3(-1, 0, 0)), Is.False);

            // Check that voxels away from axis remain
            Assert.That(grid.GetVoxelAtWorld(new Vector3(0, 2, 0)), Is.True);
        }

        [Test]
        public void TestCountMaterialVoxels()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var grid = new VoxelGrid(bbox, 1.0f);

            int initialCount = grid.CountMaterialVoxels();
            Assert.That(initialCount, Is.EqualTo(1000)); // 10x10x10

            // Remove one voxel
            grid.SetVoxel(0, 0, 0, false);
            Assert.That(grid.CountMaterialVoxels(), Is.EqualTo(999));
        }

        [Test]
        public void TestClear()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var grid = new VoxelGrid(bbox, 1.0f);

            // Remove some voxels
            grid.SetVoxel(0, 0, 0, false);
            grid.SetVoxel(5, 5, 5, false);

            // Clear should restore all to material
            grid.Clear();
            Assert.That(grid.CountMaterialVoxels(), Is.EqualTo(1000));
        }

        [Test]
        public void GetOccupiedVoxels_NonPowerOfTwoDimensions_StaysWithinGrid()
        {
            // The SVO is padded to the next power of two (10x7x5 -> 16x16x16), so the padding
            // region must never be listed as occupied voxels.
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 7, 5));
            var grid = new VoxelGrid(bbox, 1.0f);

            // Force the SVO root to exist so the traversal path (not the empty-grid shortcut) is used.
            grid.SetVoxel(0, 0, 0, false);
            grid.SetVoxel(9, 6, 4, false);

            var occupied = grid.GetOccupiedVoxels();

            foreach (var (x, y, z) in occupied)
            {
                Assert.That(x, Is.InRange(0, 9), $"occupied voxel X out of range: ({x},{y},{z})");
                Assert.That(y, Is.InRange(0, 6), $"occupied voxel Y out of range: ({x},{y},{z})");
                Assert.That(z, Is.InRange(0, 4), $"occupied voxel Z out of range: ({x},{y},{z})");
            }

            Assert.That(occupied.Count, Is.EqualTo(grid.CountMaterialVoxels()),
                "GetOccupiedVoxels must count exactly the material voxels inside the real grid");
        }

        [Test]
        public void GetOccupiedVoxels_MatchesMaterialCount_ForPowerOfTwoDimensions()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(8, 8, 8));
            var grid = new VoxelGrid(bbox, 1.0f);
            grid.RemoveVoxelsInSphere(Vector3.Zero, 2f);

            Assert.That(grid.GetOccupiedVoxels().Count, Is.EqualTo(grid.CountMaterialVoxels()));
        }
    }
}
