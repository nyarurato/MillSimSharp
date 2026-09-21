using NUnit.Framework;
using MillSimSharp.Geometry;
using System;
using System.Numerics;

namespace MillSimSharp.Tests.Geometry
{
    /// <summary>
    /// Numerical accuracy tests comparing the generated SDF against analytic distances.
    /// Convention under test: negative = material, positive = empty, units = millimeters.
    /// </summary>
    [TestFixture]
    public class SDFAccuracyTest
    {
        private static SDFGrid BuildPlaneSdf(float resolution)
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));
            var grid = new VoxelGrid(bbox, resolution);
            var (sx, sy, sz) = grid.Dimensions;
            for (int ix = 0; ix < sx; ix++)
            {
                float centerX = bbox.Min.X + (ix + 0.5f) * resolution;
                if (centerX < 0) continue;
                for (int iy = 0; iy < sy; iy++)
                    for (int iz = 0; iz < sz; iz++)
                    {
                        grid.SetVoxel(ix, iy, iz, false);
                    }
            }
            return SDFGrid.FromVoxelGrid(grid, narrowBandWidth: 10);
        }

        private static SDFGrid BuildSphereSdf(float resolution, float radius, out BoundingBox bbox)
        {
            bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(24, 24, 24));
            var grid = new VoxelGrid(bbox, resolution);
            grid.RemoveVoxelsInSphere(Vector3.Zero, radius);
            return SDFGrid.FromVoxelGrid(grid, narrowBandWidth: 20);
        }

        private static float MaxDistanceDifference(SDFGrid a, SDFGrid b)
        {
            var (sx, sy, sz) = a.Dimensions;
            float maxDiff = 0;
            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                    {
                        float diff = Math.Abs(a.GetDistance(x, y, z) - b.GetDistance(x, y, z));
                        if (diff > maxDiff) maxDiff = diff;
                    }
            return maxDiff;
        }

        [Test]
        public void SDF_Plane_MatchesAnalyticDistance()
        {
            var sdf = BuildPlaneSdf(1.0f);

            // Plane at world x = 0. Material side (x < 0) -> analytic distance = x.
            foreach (float x in new[] { -0.5f, -1.5f, -3.0f, -6.5f, -9.0f })
            {
                float actual = sdf.GetDistance(new Vector3(x, 0.0f, 0.0f));
                Assert.That(actual, Is.EqualTo(x).Within(0.05f), $"material sample x={x}");
            }

            // Empty side (x > 0) -> analytic distance = x.
            foreach (float x in new[] { 0.5f, 1.5f, 3.0f, 6.5f, 9.0f })
            {
                float actual = sdf.GetDistance(new Vector3(x, 0.0f, 0.0f));
                Assert.That(actual, Is.EqualTo(x).Within(0.05f), $"empty sample x={x}");
            }

            // Off-axis samples: the field is linear in x, so trilinear interpolation stays exact.
            Assert.That(sdf.GetDistance(new Vector3(-3.3f, 1.7f, -2.1f)), Is.EqualTo(-3.3f).Within(0.05f));
            Assert.That(sdf.GetDistance(new Vector3(4.4f, -2.3f, 1.9f)), Is.EqualTo(4.4f).Within(0.05f));
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        [TestCase(0.25f)]
        public void SDF_Sphere_MatchesAnalyticDistance(float resolution)
        {
            const float radius = 5.0f;
            var sdf = BuildSphereSdf(resolution, radius, out _);
            float tolerance = 2.0f * resolution;

            // Removed sphere = empty region -> positive distance (radius - d).
            foreach (float d in new[] { 1.0f, 2.5f, 4.0f })
            {
                float actual = sdf.GetDistance(new Vector3(d, 0, 0));
                Assert.That(actual, Is.EqualTo(radius - d).Within(tolerance), $"inside d={d} res={resolution}");
            }

            // Outside = material -> negative distance (radius - d).
            // Samples stay at least one narrow band away from the grid boundary (which is also air).
            foreach (float d in new[] { 6.0f, 7.0f, 8.0f })
            {
                float actual = sdf.GetDistance(new Vector3(d, 0, 0));
                Assert.That(actual, Is.EqualTo(radius - d).Within(tolerance), $"outside d={d} res={resolution}");
            }
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        [TestCase(0.25f)]
        public void SDF_Sphere_RmsErrorScalesWithResolution(float resolution)
        {
            const float radius = 5.0f;
            var sdf = BuildSphereSdf(resolution, radius, out _);

            double sum = 0;
            int count = 0;
            for (float d = 0.5f; d <= 8.0f; d += 0.5f)
            {
                var samples = new[]
                {
                    new Vector3(d, 0, 0), new Vector3(-d, 0, 0),
                    new Vector3(0, d, 0), new Vector3(0, -d, 0),
                    new Vector3(0, 0, d), new Vector3(0, 0, -d),
                };
                foreach (var p in samples)
                {
                    double diff = sdf.GetDistance(p) - (radius - d);
                    sum += diff * diff;
                    count++;
                }
            }

            double rms = Math.Sqrt(sum / count);
            Assert.That(rms, Is.LessThanOrEqualTo(2.0 * resolution), $"RMS={rms:F4} res={resolution}");
        }

        [Test]
        public void SDF_GetDistanceAtVoxelCenter_IsNotHalfVoxelShifted()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var grid = new VoxelGrid(bbox, 1.0f);
            grid.RemoveVoxelsInSphere(Vector3.Zero, 3.0f);
            var sdf = SDFGrid.FromVoxelGrid(grid, narrowBandWidth: 10);

            var (sx, sy, sz) = sdf.Dimensions;
            for (int x = 1; x < sx - 1; x++)
                for (int y = 1; y < sy - 1; y++)
                    for (int z = 1; z < sz - 1; z++)
                    {
                        Vector3 center = bbox.Min + new Vector3(x + 0.5f, y + 0.5f, z + 0.5f);
                        Assert.That(sdf.GetDistance(center), Is.EqualTo(sdf.GetDistance(x, y, z)).Within(1e-4f),
                            $"voxel center ({x},{y},{z})");
                    }
        }

        [Test]
        public void SDF_IncrementalUpdate_MatchesFullRebuild()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));
            var grid = new VoxelGrid(bbox, 1.0f);
            grid.RemoveVoxelsInSphere(new Vector3(-8, 0, 0), 3.0f);

            var incremental = SDFGrid.FromVoxelGrid(grid, narrowBandWidth: 10);

            // Additional removal at world (5,0,0) radius 3 -> index region x:[22,28], y/z:[17,23].
            grid.RemoveVoxelsInSphere(new Vector3(5, 0, 0), 3.0f);
            incremental.UpdateRegionFromVoxelGrid(grid, 22, 17, 17, 28, 23, 23);

            var full = SDFGrid.FromVoxelGrid(grid, narrowBandWidth: 10);

            Assert.That(MaxDistanceDifference(incremental, full), Is.LessThanOrEqualTo(1e-3f),
                "Incremental SDF update must match a full rebuild");
        }

        [Test]
        public void SDF_BoundIncrementalUpdate_MatchesFullRebuild()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));
            var grid = new VoxelGrid(bbox, 1.0f);
            var incremental = SDFGrid.FromVoxelGrid(grid, narrowBandWidth: 10);
            incremental.BindToVoxelGrid(grid);

            grid.RemoveVoxelsInSphere(new Vector3(5, 0, 0), 3.0f);

            var full = SDFGrid.FromVoxelGrid(grid, narrowBandWidth: 10);

            Assert.That(MaxDistanceDifference(incremental, full), Is.LessThanOrEqualTo(1e-3f),
                "Event-driven incremental SDF update must match a full rebuild");
        }

        [Test]
        public void FiniteCylinder_DiffersFromCapsuleAtEndFace()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(30, 30, 30));

            var capsuleGrid = new SDFGrid(bbox, 1.0f, narrowBandWidth: 10);
            capsuleGrid.RemoveCapsule(new Vector3(-5, 0, 0), new Vector3(5, 0, 0), 1.0f);

            var cylinderGrid = new SDFGrid(bbox, 1.0f, narrowBandWidth: 10);
            cylinderGrid.RemoveFiniteCylinder(new Vector3(-5, 0, 0), new Vector3(5, 0, 0), 1.0f);

            var probe = new Vector3(5.5f, 0, 0);

            // Capsule: the probe lies inside the spherical end cap -> carved (positive).
            Assert.That(capsuleGrid.GetDistance(probe), Is.GreaterThan(0),
                "Capsule should remove material beyond the end face");

            // Finite cylinder: the probe is outside the flat end face -> still material (negative).
            Assert.That(cylinderGrid.GetDistance(probe), Is.LessThan(0),
                "Finite cylinder must keep material beyond the flat end face");
        }

        [Test]
        public void WorldToVoxel_NegativeOutsidePoint_RemainsOutside()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var grid = new VoxelGrid(bbox, 1.0f);

            Vector3 outside = bbox.Min - new Vector3(0.25f, 0, 0);

            // A point just below the minimum corner must not map to voxel index 0.
            Assert.That(grid.GetVoxelAtWorld(outside), Is.False);

            grid.SetVoxelAtWorld(outside, false);
            Assert.That(grid.GetVoxel(0, 5, 5), Is.True,
                "Writing outside the grid must not modify voxel index 0");
        }

        private static float BoxSignedDistance(Vector3 point, float halfSize)
        {
            Vector3 q = new Vector3(
                MathF.Abs(point.X) - halfSize,
                MathF.Abs(point.Y) - halfSize,
                MathF.Abs(point.Z) - halfSize);
            Vector3 outside = Vector3.Max(q, Vector3.Zero);
            return outside.Length() + MathF.Min(MathF.Max(q.X, MathF.Max(q.Y, q.Z)), 0f);
        }

        private static SDFGrid BuildBoxVoidSdf(float resolution, float halfSize, float narrowBand)
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(30, 30, 30));
            var grid = new VoxelGrid(bbox, resolution);
            var (sx, sy, sz) = grid.Dimensions;

            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                    {
                        Vector3 center = bbox.Min + new Vector3(
                            (x + 0.5f) * resolution,
                            (y + 0.5f) * resolution,
                            (z + 0.5f) * resolution);

                        if (Math.Abs(center.X) <= halfSize &&
                            Math.Abs(center.Y) <= halfSize &&
                            Math.Abs(center.Z) <= halfSize)
                        {
                            grid.SetVoxel(x, y, z, false);
                        }
                    }

            return SDFGrid.FromVoxelGrid(grid, narrowBandWidth: (int)MathF.Round(narrowBand / resolution));
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        [TestCase(0.25f)]
        public void SDF_Box_MatchesAnalyticDistance(float resolution)
        {
            const float halfSize = 4f;
            var sdf = BuildBoxVoidSdf(resolution, halfSize, narrowBand: 10f);

            double sumSquares = 0;
            int count = 0;
            float maxError = 0;

            // Sample along the axes at 0.5mm steps: the discrete surface is at x = +/-halfSize,
            // so the half-voxel correction makes these close to the analytic values.
            for (float d = 0.5f; d <= 9f; d += 0.5f)
            {
                var samples = new[]
                {
                    new Vector3(d, 0, 0), new Vector3(-d, 0, 0),
                    new Vector3(0, d, 0), new Vector3(0, -d, 0),
                    new Vector3(0, 0, d), new Vector3(0, 0, -d),
                };

                foreach (var p in samples)
                {
                    float expected = -BoxSignedDistance(p, halfSize);
                    float actual = sdf.GetDistance(p);
                    float error = MathF.Abs(actual - expected);

                    sumSquares += (double)error * error;
                    if (error > maxError) maxError = error;
                    count++;
                }
            }

            double rms = Math.Sqrt(sumSquares / count);
            Assert.That(rms, Is.LessThanOrEqualTo(1.5 * resolution), $"RMS error {rms:F4} at res={resolution}");
            Assert.That(maxError, Is.LessThanOrEqualTo(3.0 * resolution), $"max error {maxError:F4} at res={resolution}");

            // The surface on the +X axis must sit at the box face.
            Assert.That(sdf.GetDistance(new Vector3(halfSize, 0, 0)), Is.EqualTo(0f).Within(1.5f * resolution));
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void SDF_ContainsNoNaNOrInfinity(float resolution)
        {
            var sdf = BuildSphereSdf(resolution, radius: 5f, out _);

            var (sx, sy, sz) = sdf.Dimensions;
            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                    {
                        float value = sdf.GetDistance(x, y, z);
                        Assert.That(float.IsNaN(value) || float.IsInfinity(value), Is.False,
                            $"SDF value at ({x},{y},{z}) must be finite (was {value})");
                    }

            // Interpolated queries around the surface must also be finite.
            for (float d = -8f; d <= 8f; d += 0.5f)
            {
                float value = sdf.GetDistance(new Vector3(d, 0.25f, -0.25f));
                Assert.That(float.IsNaN(value) || float.IsInfinity(value), Is.False);
            }
        }

        [Test]
        public void SDF_Build_IsDeterministic()
        {
            var first = BuildSphereSdf(0.5f, 5f, out _);
            var second = BuildSphereSdf(0.5f, 5f, out _);

            var (sx, sy, sz) = first.Dimensions;
            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                    {
                        Assert.That(second.GetDistance(x, y, z), Is.EqualTo(first.GetDistance(x, y, z)),
                            $"Deterministic build mismatch at ({x},{y},{z})");
                    }
        }
    }
}
