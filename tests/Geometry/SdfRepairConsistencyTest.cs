using System;
using System.Numerics;
using MillSimSharp.Geometry;
using NUnit.Framework;

namespace MillSimSharp.Tests.Geometry
{
    /// <summary>
    /// V9 audit: consistency of the native SDF CSG / RepairDistances against an EDT reference built
    /// from the final sign pattern, plus order independence of cutting operations.
    /// </summary>
    [TestFixture]
    public class SdfRepairConsistencyTest
    {
        private static BoundingBox StockBounds =>
            BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(30, 30, 30));

        /// <summary>
        /// Builds an EDT-based reference SDF from the production sign pattern (occupancy snapshot).
        /// </summary>
        private static SDFGrid BuildEdtReference(SDFGrid production, int narrowBandWidth)
        {
            var grid = new VoxelGrid(production.Bounds, production.Resolution);
            var (sx, sy, sz) = grid.Dimensions;

            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                        if (production.GetDistance(x, y, z) >= 0f)
                            grid.SetVoxel(x, y, z, false);

            return SDFGrid.FromVoxelGrid(grid, narrowBandWidth);
        }

        private readonly struct Comparison
        {
            public Comparison(int signMismatch, float maxError, double meanError, double rmsError,
                int maxX, int maxY, int maxZ, float maxA, float maxB)
            {
                SignMismatch = signMismatch;
                MaxError = maxError;
                MeanError = meanError;
                RmsError = rmsError;
                MaxX = maxX;
                MaxY = maxY;
                MaxZ = maxZ;
                MaxA = maxA;
                MaxB = maxB;
            }

            public int SignMismatch { get; }
            public float MaxError { get; }
            public double MeanError { get; }
            public double RmsError { get; }
            public int MaxX { get; }
            public int MaxY { get; }
            public int MaxZ { get; }
            public float MaxA { get; }
            public float MaxB { get; }

            public override string ToString() =>
                $"signs={SignMismatch} max={MaxError:F4} mean={MeanError:F5} rms={RmsError:F5} " +
                $"at=({MaxX},{MaxY},{MaxZ}) a={MaxA:F4} b={MaxB:F4}";
        }

        private static Comparison Compare(SDFGrid production, SDFGrid reference)
        {
            var (sx, sy, sz) = production.Dimensions;
            float band = production.NarrowBandWidth;

            int signMismatch = 0;
            float maxError = 0;
            double sum = 0;
            double sumSquares = 0;
            int count = 0;
            int maxX = -1, maxY = -1, maxZ = -1;
            float maxA = 0, maxB = 0;

            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                    {
                        float a = production.GetDistance(x, y, z);
                        float b = reference.GetDistance(x, y, z);

                        if ((a < 0f) != (b < 0f)) signMismatch++;

                        if (MathF.Abs(a) <= band || MathF.Abs(b) <= band)
                        {
                            float error = MathF.Abs(a - b);
                            if (error > maxError)
                            {
                                maxError = error;
                                maxX = x;
                                maxY = y;
                                maxZ = z;
                                maxA = a;
                                maxB = b;
                            }

                            sum += error;
                            sumSquares += (double)error * error;
                            count++;
                        }
                    }

            return new Comparison(signMismatch, maxError, sum / count, Math.Sqrt(sumSquares / count),
                maxX, maxY, maxZ, maxA, maxB);
        }

        [Test]
        public void NativeSdf_CrossingCuts_MatchEdtReference()
        {
            const float resolution = 0.5f;
            var sdf = new SDFGrid(StockBounds, resolution, narrowBandWidth: 8);
            sdf.RemoveFiniteCylinder(new Vector3(-8, 0, 0), new Vector3(8, 0, 0), 2.5f);
            sdf.RemoveFiniteCylinder(new Vector3(0, -8, 0), new Vector3(0, 8, 0), 2.5f);

            var reference = BuildEdtReference(sdf, narrowBandWidth: 8);
            var comparison = Compare(sdf, reference);
            TestContext.Out.WriteLine($"crossing cuts (res 0.5): {comparison}");

            Assert.That(comparison.SignMismatch, Is.EqualTo(0), "The sign pattern must match the EDT reference");
            Assert.That(comparison.MaxError, Is.LessThanOrEqualTo(1.0f * resolution),
                $"Max narrow-band error {comparison.MaxError:F4}");
            Assert.That(comparison.RmsError, Is.LessThanOrEqualTo(0.5f * resolution),
                $"RMS narrow-band error {comparison.RmsError:F4}");
        }

        [Test]
        public void NativeSdf_OverlappingCuts_MatchEdtReference()
        {
            const float resolution = 0.5f;
            var sdf = new SDFGrid(StockBounds, resolution, narrowBandWidth: 8);

            // Overlapping tools, and the same cut applied twice.
            sdf.RemoveSphere(new Vector3(-1, 0, 0), 4f);
            sdf.RemoveSphere(new Vector3(1, 0, 0), 4f);
            sdf.RemoveSphere(new Vector3(-1, 0, 0), 4f);

            var reference = BuildEdtReference(sdf, narrowBandWidth: 8);
            var comparison = Compare(sdf, reference);
            TestContext.Out.WriteLine($"overlapping cuts (res 0.5): {comparison}");

            Assert.That(comparison.SignMismatch, Is.EqualTo(0), "The sign pattern must match the EDT reference");
            Assert.That(comparison.MaxError, Is.LessThanOrEqualTo(1.0f * resolution),
                $"Max narrow-band error {comparison.MaxError:F4}");
            Assert.That(comparison.RmsError, Is.LessThanOrEqualTo(0.5f * resolution),
                $"RMS narrow-band error {comparison.RmsError:F4}");
        }

        [Test]
        public void NativeSdf_OperationOrder_IsConsistent()
        {
            const float resolution = 0.5f;
            var forward = new SDFGrid(StockBounds, resolution, narrowBandWidth: 8);
            forward.RemoveSphere(new Vector3(-3, 0, 0), 4f);
            forward.RemoveFiniteCylinder(new Vector3(0, -6, 0), new Vector3(0, 6, 0), 2.5f);

            var backward = new SDFGrid(StockBounds, resolution, narrowBandWidth: 8);
            backward.RemoveFiniteCylinder(new Vector3(0, -6, 0), new Vector3(0, 6, 0), 2.5f);
            backward.RemoveSphere(new Vector3(-3, 0, 0), 4f);

            var comparison = Compare(forward, backward);
            TestContext.Out.WriteLine($"operation order (res 0.5): {comparison}");

            Assert.That(comparison.SignMismatch, Is.EqualTo(0),
                "A -> B and B -> A must produce the same sign pattern");
            Assert.That(comparison.MaxError, Is.LessThanOrEqualTo(1.0f * resolution),
                $"A -> B and B -> A must produce the same field within the narrow band (max {comparison.MaxError:F4})");
        }
    }
}
