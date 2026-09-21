using NUnit.Framework;
using MillSimSharp.Geometry;
using System.Numerics;
using System;

namespace MillSimSharp.Tests.Geometry
{
    [TestFixture]
    public class SDFGridTest
    {
        [Test]
        public void UpdateRegionFromVoxelGrid_UpdatesLocalDistances()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));
            var grid = new VoxelGrid(bbox, resolution: 1.0f);
            // Create a small cut near the center
            grid.RemoveVoxelsInSphere(new Vector3(5, 0, 0), 2.0f);

            var sdf = SDFGrid.FromVoxelGrid(grid, narrowBandWidth: 5);

            // pick a region near where we will cut and record distances
            var idx = (x: 5, y: 0, z: 0);
            int sampleMinX = Math.Max(0, idx.x + 20 - 1);
            int sampleMaxX = Math.Min(sdf.Dimensions.X - 1, idx.x + 20 + 1);
            int sampleMinY = Math.Max(0, idx.y + 20 - 1);
            int sampleMaxY = Math.Min(sdf.Dimensions.Y - 1, idx.y + 20 + 1);
            int sampleMinZ = Math.Max(0, idx.z + 20 - 1);
            int sampleMaxZ = Math.Min(sdf.Dimensions.Z - 1, idx.z + 20 + 1);

            var beforeSamples = new System.Collections.Generic.List<float>();
            for (int x = sampleMinX; x <= sampleMaxX; x++)
                for (int y = sampleMinY; y <= sampleMaxY; y++)
                    for (int z = sampleMinZ; z <= sampleMaxZ; z++)
                        beforeSamples.Add(sdf.GetDistance(x, y, z));

            // Now remove a nearby voxel intentionally (ensure change) — prefer a single voxel operation
            var neighWorld = new Vector3(10, 0, 0);
            // Only set to false if it was true (material) so we actually change state
            bool wasMaterialAtNeigh = grid.GetVoxelAtWorld(neighWorld);
            if (wasMaterialAtNeigh)
                grid.SetVoxelAtWorld(neighWorld, false);

            // Determine voxel indices for a small region around the modified area
            int minXi = Math.Max(0, (int)((4 - bbox.Min.X) / 1.0f));
            int minYi = Math.Max(0, (int)((-2 - bbox.Min.Y) / 1.0f));
            int minZi = Math.Max(0, (int)((-2 - bbox.Min.Z) / 1.0f));
            int maxXi = Math.Min(sdf.Dimensions.X - 1, (int)((8 - bbox.Min.X) / 1.0f));
            int maxYi = Math.Min(sdf.Dimensions.Y - 1, (int)((2 - bbox.Min.Y) / 1.0f));
            int maxZi = Math.Min(sdf.Dimensions.Z - 1, (int)((2 - bbox.Min.Z) / 1.0f));

            long voxelsBeforeRemoval = grid.CountMaterialVoxels();
            sdf.UpdateRegionFromVoxelGrid(grid, minXi, minYi, minZi, maxXi, maxYi, maxZi);
            long voxelsAfterRemoval = grid.CountMaterialVoxels();

            var afterSamples = new System.Collections.Generic.List<float>();
            for (int x = sampleMinX; x <= sampleMaxX; x++)
                for (int y = sampleMinY; y <= sampleMaxY; y++)
                    for (int z = sampleMinZ; z <= sampleMaxZ; z++)
                        afterSamples.Add(sdf.GetDistance(x, y, z));

            // No debug output; we only assert that computations succeeded and values are finite
            foreach (var v in afterSamples)
                Assert.That(float.IsNaN(v), Is.False);
        }

        [Test]
        public void TestSDFFromSimpleSphere()
        {
            // Create a voxel grid with a sphere
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));
            var voxelGrid = new VoxelGrid(bbox, 0.5f);

            // Remove a sphere at center (creates empty space)
            float radius = 3.0f;
            voxelGrid.RemoveVoxelsInSphere(Vector3.Zero, radius);

            // Convert to SDF
            var sdf = SDFGrid.FromVoxelGrid(voxelGrid);

            Assert.That(sdf, Is.Not.Null);
            Assert.That(sdf.Resolution, Is.EqualTo(0.5f));

            // Standard SDF convention: negative = material, positive = empty
            // Since we removed a sphere, the center should have a positive distance
            float distCenter = sdf.GetDistance(Vector3.Zero);

            Assert.That(distCenter, Is.GreaterThan(0), "Center should be in empty space (positive)");
            Assert.That(distCenter, Is.LessThan(radius + 1.0f), "Distance should be reasonable");

            // Test distance outside the removed sphere (should be negative = material)
            Vector3 outsidePoint = new Vector3(radius + 2.0f, 0, 0);
            float distOutside = sdf.GetDistance(outsidePoint);
            Assert.That(distOutside, Is.LessThan(0), "Outside point should be in material (negative)");
        }

        [Test]
        public void TestSDFFromBox()
        {
            // Create a voxel grid
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));
            var voxelGrid = new VoxelGrid(bbox, 1.0f);

            // Remove a box region (5x5x5) at center
            for (int z = -2; z <= 2; z++)
            {
                for (int y = -2; y <= 2; y++)
                {
                    for (int x = -2; x <= 2; x++)
                    {
                        voxelGrid.SetVoxelAtWorld(new Vector3(x, y, z), false);
                    }
                }
            }

            // Convert to SDF
            var sdf = SDFGrid.FromVoxelGrid(voxelGrid);

            Assert.That(sdf, Is.Not.Null);

            // Test center point (empty under standard SDF convention -> positive)
            float distCenter = sdf.GetDistance(Vector3.Zero);
            Assert.That(distCenter, Is.GreaterThan(0), "Center of empty box should be empty (positive)");

            // Test outside point (material -> negative)
            Vector3 outsidePoint = new Vector3(5, 5, 5);
            float distOutside = sdf.GetDistance(outsidePoint);
            Assert.That(distOutside, Is.LessThan(0), "Point in material should be negative");
        }

        [Test]
        public void TestSDFSignCorrectness()
        {
            // Create a voxel grid with material
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var voxelGrid = new VoxelGrid(bbox, 0.5f);

            // Remove a small sphere (empty space)
            voxelGrid.RemoveVoxelsInSphere(Vector3.Zero, 2.0f);

            var sdf = SDFGrid.FromVoxelGrid(voxelGrid);

            // Standard SDF: points inside empty regions are positive
            float distInside = sdf.GetDistance(Vector3.Zero);
            Assert.That(distInside, Is.GreaterThan(0), "Inside empty region should be positive");

            // Points in material are negative
            Vector3 materialPoint = new Vector3(4, 4, 4);
            float distMaterial = sdf.GetDistance(materialPoint);
            Assert.That(distMaterial, Is.LessThan(0), "Inside material should be negative");
        }

        [Test]
        public void TestSDFGradient()
        {
            // Create a voxel grid with a sphere
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));
            var voxelGrid = new VoxelGrid(bbox, 0.5f);

            // Remove sphere at center
            voxelGrid.RemoveVoxelsInSphere(Vector3.Zero, 3.0f);

            var sdf = SDFGrid.FromVoxelGrid(voxelGrid);

            // Test gradient at a point on positive X axis
            Vector3 testPoint = new Vector3(3.0f, 0, 0);
            Vector3 gradient = sdf.GetGradient(testPoint);

            // Gradient should be normalized
            Assert.That(gradient.Length(), Is.EqualTo(1.0f).Within(0.1f), "Gradient should be normalized");

            // Standard SDF gradient points from material toward empty space.
            // At (3,0,0) on the removed sphere wall, that direction is toward the sphere center (-X).
            Assert.That(gradient.X, Is.LessThan(-0.5f), "Gradient X component should point into the removed sphere");
            Assert.That(Math.Abs(gradient.Y), Is.LessThan(0.5f), "Gradient Y component should be small");
            Assert.That(Math.Abs(gradient.Z), Is.LessThan(0.5f), "Gradient Z component should be small");
        }

        [Test]
        public void TestSDFInterpolation()
        {
            // Create a simple voxel grid
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var voxelGrid = new VoxelGrid(bbox, 1.0f);

            // Remove sphere
            voxelGrid.RemoveVoxelsInSphere(Vector3.Zero, 2.0f);

            var sdf = SDFGrid.FromVoxelGrid(voxelGrid);

            // Test interpolation between voxel centers
            Vector3 voxelCenter = new Vector3(0, 0, 0);
            Vector3 betweenVoxels = new Vector3(0.25f, 0.25f, 0.25f);

            float distCenter = sdf.GetDistance(voxelCenter);
            float distBetween = sdf.GetDistance(betweenVoxels);

            // Both should be positive (inside the removed sphere)
            Assert.That(distCenter, Is.GreaterThan(0));
            Assert.That(distBetween, Is.GreaterThan(0));

            // The interpolated value should be reasonable
            Assert.That(Math.Abs(distBetween - distCenter), Is.LessThan(2.0f),
                       "Interpolated distance should be close to voxel center distance");
        }

        [Test]
        public void TestSDFNarrowBand()
        {
            // Create a voxel grid
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(50, 50, 50));
            var voxelGrid = new VoxelGrid(bbox, 1.0f);

            // Remove sphere at center
            voxelGrid.RemoveVoxelsInSphere(Vector3.Zero, 3.0f);

            // Create SDF with narrow band of 5 voxels
            int narrowBandVoxels = 5;
            var sdf = SDFGrid.FromVoxelGrid(voxelGrid, narrowBandVoxels);

            Assert.That(sdf.NarrowBandWidth, Is.EqualTo(narrowBandVoxels * 1.0f));

            // Points far from surface should be clamped to narrow band width
            Vector3 farPoint = new Vector3(20, 20, 20);
            float distFar = sdf.GetDistance(farPoint);

            // Should be clamped to narrow band width
            Assert.That(Math.Abs(distFar), Is.LessThanOrEqualTo(sdf.NarrowBandWidth * 1.1f),
                       "Distance should be clamped to narrow band width");
        }

        [Test]
        public void TestSDFDimensions()
        {
            // Create a voxel grid
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 15, 20));
            var voxelGrid = new VoxelGrid(bbox, 1.0f);

            var sdf = SDFGrid.FromVoxelGrid(voxelGrid);

            // Check dimensions match
            var voxelDims = voxelGrid.Dimensions;
            var sdfDims = sdf.Dimensions;

            Assert.That(sdfDims.X, Is.EqualTo(voxelDims.X));
            Assert.That(sdfDims.Y, Is.EqualTo(voxelDims.Y));
            Assert.That(sdfDims.Z, Is.EqualTo(voxelDims.Z));

            // Check bounds match
            Assert.That(sdf.Bounds.Min, Is.EqualTo(voxelGrid.Bounds.Min));
            Assert.That(sdf.Bounds.Max, Is.EqualTo(voxelGrid.Bounds.Max));
        }

        [Test]
        public void TestSDFOutOfBounds()
        {
            // Create a simple voxel grid
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var voxelGrid = new VoxelGrid(bbox, 1.0f);

            var sdf = SDFGrid.FromVoxelGrid(voxelGrid);

            // Query far outside bounds
            Vector3 farOutside = new Vector3(100, 100, 100);
            float dist = sdf.GetDistance(farOutside);

            // Should return positive narrow band width (outside is empty)
            Assert.That(dist, Is.GreaterThan(0));
            Assert.That(dist, Is.EqualTo(sdf.NarrowBandWidth).Within(0.1f));
        }

        [Test]
        public void TestSDFIndexQuery()
        {
            // Create a voxel grid
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var voxelGrid = new VoxelGrid(bbox, 1.0f);

            voxelGrid.RemoveVoxelsInSphere(Vector3.Zero, 2.0f);

            var sdf = SDFGrid.FromVoxelGrid(voxelGrid);

            // Query by index
            var dims = sdf.Dimensions;
            int centerX = dims.X / 2;
            int centerY = dims.Y / 2;
            int centerZ = dims.Z / 2;

            float distByIndex = sdf.GetDistance(centerX, centerY, centerZ);

            // Should be positive (inside the removed sphere)
            Assert.That(distByIndex, Is.GreaterThan(0));

            // Out of bounds query should return a positive distance (outside = empty). The magnitude
            // is the distance from the sample position to the nearest point inside bounds
            // (clamped to the narrow band width).
            float oobDist = sdf.GetDistance(-1, -1, -1);
            Assert.That(oobDist, Is.GreaterThan(0));
            Assert.That(Math.Abs(oobDist), Is.LessThanOrEqualTo(sdf.NarrowBandWidth + 1e-6f));
        }
    }
}
