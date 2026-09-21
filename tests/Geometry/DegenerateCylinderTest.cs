using System;
using System.Numerics;
using MillSimSharp.Geometry;
using NUnit.Framework;

namespace MillSimSharp.Tests.Geometry
{
    /// <summary>
    /// Backend consistency for degenerate (zero-length) cylinders:
    /// a capsule degenerates to a sphere, a flat-ended cylinder has no volume.
    /// </summary>
    [TestFixture]
    public class DegenerateCylinderTest
    {
        private static BoundingBox StockBounds => BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));

        private static void AssertGridsEqual(VoxelGrid expected, VoxelGrid actual)
        {
            var (sx, sy, sz) = expected.Dimensions;
            int differences = 0;
            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                        if (expected.GetVoxel(x, y, z) != actual.GetVoxel(x, y, z)) differences++;

            Assert.That(differences, Is.EqualTo(0), "Voxel states must match");
        }

        [Test]
        public void DegenerateCapsule_BecomesSphere()
        {
            var bbox = StockBounds;

            var capsuleGrid = new VoxelGrid(bbox, 1.0f);
            capsuleGrid.RemoveVoxelsInCylinder(Vector3.Zero, Vector3.Zero, 3f, flatEnds: false);
            var sphereGrid = new VoxelGrid(bbox, 1.0f);
            sphereGrid.RemoveVoxelsInSphere(Vector3.Zero, 3f);
            AssertGridsEqual(sphereGrid, capsuleGrid);

            var capsuleSdf = new SDFGrid(bbox, 1.0f, narrowBandWidth: 6);
            capsuleSdf.RemoveCapsule(Vector3.Zero, Vector3.Zero, 3f);
            var sphereSdf = new SDFGrid(bbox, 1.0f, narrowBandWidth: 6);
            sphereSdf.RemoveSphere(Vector3.Zero, 3f);

            var (sx, sy, sz) = capsuleSdf.Dimensions;
            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                    {
                        Assert.That(capsuleSdf.GetDistance(x, y, z),
                            Is.EqualTo(sphereSdf.GetDistance(x, y, z)).Within(1e-4f),
                            $"SDF mismatch at ({x},{y},{z})");
                    }
        }

        [Test]
        public void DegenerateFlatCylinder_VoxelAndSdfHaveSameSemantics()
        {
            var bbox = StockBounds;

            // A zero-length flat cylinder has no volume: nothing may be removed.
            var grid = new VoxelGrid(bbox, 1.0f);
            var (sx, sy, sz) = grid.Dimensions;
            grid.RemoveVoxelsInCylinder(Vector3.Zero, Vector3.Zero, 3f, flatEnds: true);
            Assert.That(grid.CountMaterialVoxels(), Is.EqualTo(sx * sy * sz),
                "A zero-length flat cylinder must not remove any voxels");

            var sdf = new SDFGrid(bbox, 1.0f, narrowBandWidth: 6);
            sdf.RemoveFiniteCylinder(Vector3.Zero, Vector3.Zero, 3f);
            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                        Assert.That(sdf.GetDistance(x, y, z), Is.LessThan(0f),
                            $"A zero-length flat cylinder must keep material at ({x},{y},{z})");
        }
    }
}
