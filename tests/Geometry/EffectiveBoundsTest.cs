using System;
using System.Numerics;
using MillSimSharp.Geometry;
using NUnit.Framework;

namespace MillSimSharp.Tests.Geometry
{
    /// <summary>
    /// Tests for effective (voxelized) bounds. Requested bounds whose size is not divisible by the
    /// resolution are rounded up to whole voxels: Min is preserved and Max expands by less than one
    /// voxel per axis. Voxel centers, voxel faces, world-index mapping, mesh output and SDF bounds
    /// must all agree with this effective extent.
    /// </summary>
    [TestFixture]
    public class EffectiveBoundsTest
    {
        private const float Resolution = 1.0f;
        private static readonly Vector3 RequestedSize = new Vector3(10.3f, 7.7f, 3.2f);

        private static BoundingBox RequestedBounds => new BoundingBox(Vector3.Zero, RequestedSize);

        [Test]
        public void NonDivisibleBounds_EffectiveBoundsExpandToVoxelExtent()
        {
            var grid = new VoxelGrid(RequestedBounds, Resolution);

            Assert.That(grid.Dimensions, Is.EqualTo((11, 8, 4)));

            Assert.That(grid.Bounds.Min.X, Is.EqualTo(0f).Within(1e-6f));
            Assert.That(grid.Bounds.Min.Y, Is.EqualTo(0f).Within(1e-6f));
            Assert.That(grid.Bounds.Min.Z, Is.EqualTo(0f).Within(1e-6f));

            Assert.That(grid.Bounds.Max.X, Is.EqualTo(11f).Within(1e-6f));
            Assert.That(grid.Bounds.Max.Y, Is.EqualTo(8f).Within(1e-6f));
            Assert.That(grid.Bounds.Max.Z, Is.EqualTo(4f).Within(1e-6f));

            // Max side only, and strictly less than one voxel of expansion per axis.
            Assert.That(grid.Bounds.Max.X, Is.GreaterThanOrEqualTo(RequestedSize.X));
            Assert.That(grid.Bounds.Max.Y, Is.GreaterThanOrEqualTo(RequestedSize.Y));
            Assert.That(grid.Bounds.Max.Z, Is.GreaterThanOrEqualTo(RequestedSize.Z));
            Assert.That(grid.Bounds.Max.X - RequestedSize.X, Is.LessThan(Resolution));
            Assert.That(grid.Bounds.Max.Y - RequestedSize.Y, Is.LessThan(Resolution));
            Assert.That(grid.Bounds.Max.Z - RequestedSize.Z, Is.LessThan(Resolution));
        }

        [Test]
        public void NonDivisibleBounds_LastVoxelCenterIsInsideEffectiveBounds()
        {
            var grid = new VoxelGrid(RequestedBounds, Resolution);
            var (sx, sy, sz) = grid.Dimensions;

            Vector3 lastCenter = grid.Bounds.Min + new Vector3(
                (sx - 0.5f) * Resolution,
                (sy - 0.5f) * Resolution,
                (sz - 0.5f) * Resolution);

            Assert.That(lastCenter.X, Is.LessThan(grid.Bounds.Max.X));
            Assert.That(lastCenter.Y, Is.LessThan(grid.Bounds.Max.Y));
            Assert.That(lastCenter.Z, Is.LessThan(grid.Bounds.Max.Z));

            Assert.That(grid.GetVoxelAtWorld(lastCenter), Is.True, "The last voxel center must be inside the grid");
        }

        [Test]
        public void NonDivisibleBounds_PointBetweenRequestedAndEffectiveMaxMapsToLastVoxel()
        {
            var grid = new VoxelGrid(RequestedBounds, Resolution);
            var (sx, sy, sz) = grid.Dimensions;

            Vector3 betweenX = new Vector3((RequestedSize.X + grid.Bounds.Max.X) * 0.5f, 0.5f, 0.5f);
            Vector3 betweenY = new Vector3(0.5f, (RequestedSize.Y + grid.Bounds.Max.Y) * 0.5f, 0.5f);
            Vector3 betweenZ = new Vector3(0.5f, 0.5f, (RequestedSize.Z + grid.Bounds.Max.Z) * 0.5f);

            Assert.That(betweenX.X, Is.GreaterThan(RequestedSize.X));
            Assert.That(betweenX.X, Is.LessThan(grid.Bounds.Max.X));
            Assert.That(betweenY.Y, Is.GreaterThan(RequestedSize.Y));
            Assert.That(betweenY.Y, Is.LessThan(grid.Bounds.Max.Y));
            Assert.That(betweenZ.Z, Is.GreaterThan(RequestedSize.Z));
            Assert.That(betweenZ.Z, Is.LessThan(grid.Bounds.Max.Z));

            grid.SetVoxelAtWorld(betweenX, false);
            grid.SetVoxelAtWorld(betweenY, false);
            grid.SetVoxelAtWorld(betweenZ, false);

            Assert.That(grid.GetVoxel(sx - 1, 0, 0), Is.False, "X: point must map to the last voxel");
            Assert.That(grid.GetVoxel(0, sy - 1, 0), Is.False, "Y: point must map to the last voxel");
            Assert.That(grid.GetVoxel(0, 0, sz - 1), Is.False, "Z: point must map to the last voxel");
        }

        [Test]
        public void NonDivisibleBounds_PointAtOrBeyondEffectiveMaxIsOutside()
        {
            var grid = new VoxelGrid(RequestedBounds, Resolution);
            var (sx, sy, sz) = grid.Dimensions;

            grid.SetVoxelAtWorld(new Vector3(grid.Bounds.Max.X, 0.5f, 0.5f), false);
            Assert.That(grid.GetVoxel(sx - 1, 0, 0), Is.True, "A write exactly at effective Max must be ignored");

            grid.SetVoxelAtWorld(new Vector3(grid.Bounds.Max.X - 1e-3f, 0.5f, 0.5f), false);
            Assert.That(grid.GetVoxel(sx - 1, 0, 0), Is.False, "A point just inside effective Max belongs to the last voxel");

            grid.SetVoxelAtWorld(new Vector3(0.5f, grid.Bounds.Max.Y, 0.5f), false);
            Assert.That(grid.GetVoxel(0, sy - 1, 0), Is.True, "A write exactly at effective Max must be ignored (Y)");

            grid.SetVoxelAtWorld(new Vector3(0.5f, grid.Bounds.Max.Y - 1e-3f, 0.5f), false);
            Assert.That(grid.GetVoxel(0, sy - 1, 0), Is.False, "A point just inside effective Max belongs to the last voxel (Y)");

            grid.SetVoxelAtWorld(new Vector3(0.5f, 0.5f, grid.Bounds.Max.Z), false);
            Assert.That(grid.GetVoxel(0, 0, sz - 1), Is.True, "A write exactly at effective Max must be ignored (Z)");

            grid.SetVoxelAtWorld(new Vector3(0.5f, 0.5f, grid.Bounds.Max.Z - 1e-3f), false);
            Assert.That(grid.GetVoxel(0, 0, sz - 1), Is.False, "A point just inside effective Max belongs to the last voxel (Z)");
        }

        [Test]
        public void NonDivisibleBounds_VoxelMeshDoesNotExtendBeyondEffectiveBounds()
        {
            var grid = new VoxelGrid(RequestedBounds, Resolution);
            Mesh mesh = MeshConverter.ConvertToMesh(grid);

            bool reachesMaxX = false, reachesMaxY = false, reachesMaxZ = false;
            foreach (Vector3 v in mesh.Vertices)
            {
                Assert.That(v.X, Is.GreaterThanOrEqualTo(grid.Bounds.Min.X - 1e-4f));
                Assert.That(v.X, Is.LessThanOrEqualTo(grid.Bounds.Max.X + 1e-4f));
                Assert.That(v.Y, Is.GreaterThanOrEqualTo(grid.Bounds.Min.Y - 1e-4f));
                Assert.That(v.Y, Is.LessThanOrEqualTo(grid.Bounds.Max.Y + 1e-4f));
                Assert.That(v.Z, Is.GreaterThanOrEqualTo(grid.Bounds.Min.Z - 1e-4f));
                Assert.That(v.Z, Is.LessThanOrEqualTo(grid.Bounds.Max.Z + 1e-4f));

                if (MathF.Abs(v.X - grid.Bounds.Max.X) < 1e-4f) reachesMaxX = true;
                if (MathF.Abs(v.Y - grid.Bounds.Max.Y) < 1e-4f) reachesMaxY = true;
                if (MathF.Abs(v.Z - grid.Bounds.Max.Z) < 1e-4f) reachesMaxZ = true;
            }

            Assert.That(reachesMaxX && reachesMaxY && reachesMaxZ, Is.True,
                "The material block must reach the effective max faces");
        }

        [Test]
        public void NonDivisibleBounds_FromVoxelGridSdfUsesSameEffectiveBounds()
        {
            var grid = new VoxelGrid(RequestedBounds, Resolution);
            var sdf = SDFGrid.FromVoxelGrid(grid);

            Assert.That(sdf.Dimensions, Is.EqualTo(grid.Dimensions));
            Assert.That(sdf.Bounds.Min.X, Is.EqualTo(grid.Bounds.Min.X).Within(1e-6f));
            Assert.That(sdf.Bounds.Min.Y, Is.EqualTo(grid.Bounds.Min.Y).Within(1e-6f));
            Assert.That(sdf.Bounds.Min.Z, Is.EqualTo(grid.Bounds.Min.Z).Within(1e-6f));
            Assert.That(sdf.Bounds.Max.X, Is.EqualTo(grid.Bounds.Max.X).Within(1e-6f));
            Assert.That(sdf.Bounds.Max.Y, Is.EqualTo(grid.Bounds.Max.Y).Within(1e-6f));
            Assert.That(sdf.Bounds.Max.Z, Is.EqualTo(grid.Bounds.Max.Z).Within(1e-6f));

            // The native SDF constructor follows the same convention as VoxelGrid.
            var native = new SDFGrid(RequestedBounds, Resolution);
            Assert.That(native.Dimensions, Is.EqualTo((11, 8, 4)));
            Assert.That(native.Bounds.Max.X, Is.EqualTo(grid.Bounds.Max.X).Within(1e-6f));
            Assert.That(native.Bounds.Max.Y, Is.EqualTo(grid.Bounds.Max.Y).Within(1e-6f));
            Assert.That(native.Bounds.Max.Z, Is.EqualTo(grid.Bounds.Max.Z).Within(1e-6f));
        }

        [Test]
        public void NonDivisibleBounds_NonZeroMinWorks()
        {
            var min = new Vector3(-2.3f, 5.2f, -1.7f);
            var size = new Vector3(10.3f, 7.7f, 3.2f);
            var bbox = new BoundingBox(min, min + size);
            var grid = new VoxelGrid(bbox, 1.0f);

            Assert.That(grid.Dimensions, Is.EqualTo((11, 8, 4)));

            Vector3 effectiveMax = min + new Vector3(11f, 8f, 4f);

            Assert.That(grid.Bounds.Min.X, Is.EqualTo(min.X).Within(1e-6f));
            Assert.That(grid.Bounds.Min.Y, Is.EqualTo(min.Y).Within(1e-6f));
            Assert.That(grid.Bounds.Min.Z, Is.EqualTo(min.Z).Within(1e-6f));
            Assert.That(grid.Bounds.Max.X, Is.EqualTo(effectiveMax.X).Within(1e-6f));
            Assert.That(grid.Bounds.Max.Y, Is.EqualTo(effectiveMax.Y).Within(1e-6f));
            Assert.That(grid.Bounds.Max.Z, Is.EqualTo(effectiveMax.Z).Within(1e-6f));

            var (sx, sy, sz) = grid.Dimensions;

            // Points between the requested max and the effective max must belong to the last voxel.
            Vector3 requestedMax = min + size;
            var betweenX = new Vector3((requestedMax.X + effectiveMax.X) * 0.5f, min.Y + 0.5f, min.Z + 0.5f);
            var betweenY = new Vector3(min.X + 0.5f, (requestedMax.Y + effectiveMax.Y) * 0.5f, min.Z + 0.5f);
            var betweenZ = new Vector3(min.X + 0.5f, min.Y + 0.5f, (requestedMax.Z + effectiveMax.Z) * 0.5f);

            grid.SetVoxelAtWorld(betweenX, false);
            grid.SetVoxelAtWorld(betweenY, false);
            grid.SetVoxelAtWorld(betweenZ, false);

            Assert.That(grid.GetVoxel(sx - 1, 0, 0), Is.False);
            Assert.That(grid.GetVoxel(0, sy - 1, 0), Is.False);
            Assert.That(grid.GetVoxel(0, 0, sz - 1), Is.False);
        }
    }
}
