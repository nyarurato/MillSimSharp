using System;
using System.Numerics;
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using NUnit.Framework;

namespace MillSimSharp.Tests.Simulation
{
    /// <summary>
    /// Accuracy of the tapered end mill signed distance against an independent capped-frustum
    /// reference (2D meridian cross-section distance). The reference is implemented in the test
    /// only and does not use production helpers.
    /// </summary>
    [TestFixture]
    public class TaperGeometryAccuracyTest
    {
        private const float TipRadius = 2f;
        private const float AngleDegrees = 10f;
        private const float Length = 20f;

        private static float TopRadius =>
            TipRadius + Length * MathF.Tan(AngleDegrees * MathF.PI / 180f);

        /// <summary>
        /// Independent reference: signed distance in the meridian half-plane to the convex
        /// quadrilateral (0,0), (R0,0), (R1,L), (0,L). For a solid of revolution this equals the
        /// 3D signed distance.
        /// </summary>
        private static float ReferenceFrustumSignedDistance(Vector3 localPoint)
        {
            float r = MathF.Sqrt(localPoint.X * localPoint.X + localPoint.Y * localPoint.Y);
            float z = localPoint.Z;

            float bottom = DistanceToSegment(r, z, 0f, 0f, TipRadius, 0f);
            float lateral = DistanceToSegment(r, z, TipRadius, 0f, TopRadius, Length);
            float top = DistanceToSegment(r, z, 0f, Length, TopRadius, Length);
            float boundary = MathF.Min(bottom, MathF.Min(lateral, top));

            float radiusAtZ = TipRadius + (TopRadius - TipRadius) * (z / Length);
            bool inside = z >= 0f && z <= Length && r <= radiusAtZ;
            return inside ? -boundary : boundary;
        }

        private static float DistanceToSegment(float px, float pz, float ax, float az, float bx, float bz)
        {
            float abx = bx - ax;
            float abz = bz - az;
            float apx = px - ax;
            float apz = pz - az;
            float denominator = abx * abx + abz * abz;
            float t = denominator > 1e-12f
                ? Math.Clamp((apx * abx + apz * abz) / denominator, 0f, 1f)
                : 0f;
            float dx = apx - abx * t;
            float dz = apz - abz * t;
            return MathF.Sqrt(dx * dx + dz * dz);
        }

        [Test]
        public void TaperedGeometry_SignedDistance_MatchesFrustumReference()
        {
            var geometry = new TaperedEndMillGeometry(TipRadius, AngleDegrees, Length);

            double sum = 0;
            double sumSquares = 0;
            int count = 0;
            float maxError = 0;
            Vector3 worst = default;

            for (float r = 0f; r <= 8f; r += 0.25f)
                for (float z = -2f; z <= 22f; z += 0.25f)
                {
                    var local = new Vector3(r, 0f, z);
                    float expected = ReferenceFrustumSignedDistance(local);
                    float error = MathF.Abs(geometry.SignedDistance(local) - expected);

                    sum += error;
                    sumSquares += (double)error * error;
                    count++;
                    if (error > maxError)
                    {
                        maxError = error;
                        worst = local;
                    }
                }

            double mean = sum / count;
            double rms = Math.Sqrt(sumSquares / count);
            TestContext.Out.WriteLine(
                $"Taper SDF error: max={maxError:F5} mean={mean:F5} rms={rms:F5} worst={(worst.X, worst.Z)}");

            // Analytic corner case: the bottom-cap x lateral edge. Along the outward bisector the
            // exact distance is the distance to the corner point.
            float cos = MathF.Cos(AngleDegrees * MathF.PI / 180f);
            float sin = MathF.Sin(AngleDegrees * MathF.PI / 180f);
            Vector3 bisector = Vector3.Normalize(new Vector3(cos, 0f, -(1f + sin)));
            Vector3 cornerProbe = new Vector3(TipRadius, 0f, 0f) + bisector * 1.5f;
            Assert.That(geometry.SignedDistance(cornerProbe), Is.EqualTo(1.5f).Within(1e-3f),
                "Distance along the bottom-cap x lateral corner bisector");

            Assert.That(maxError, Is.LessThanOrEqualTo(1e-4f),
                $"Tapered geometry must match the exact capped-frustum distance (worst at {worst})");
            Assert.That(rms, Is.LessThanOrEqualTo(1e-5f), $"RMS={rms:F6}");
        }

        [Test]
        public void TaperedSdfCarve_ZeroLevelDisplacement_IsSmall()
        {
            const float resolution = 0.5f;
            // The whole tool (z = 0..20) stays inside the grid, so the probes do not hit the
            // air boundary of the SDF grid.
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));
            var sdf = new SDFGrid(bbox, resolution, narrowBandWidth: 8);
            var tool = new TaperEndMill(tipDiameter: 2f * TipRadius, length: Length, taperAngleDegrees: AngleDegrees);
            new SDFCutterSimulator(sdf).CutPoint(Vector3.Zero, tool);

            float angle = AngleDegrees * MathF.PI / 180f;
            float normalR = MathF.Cos(angle);
            float normalZ = -MathF.Sin(angle);

            float maxDisplacement = 0;
            foreach (float z in new[] { 2f, 5f, 10f, 15f })
            {
                float radiusAtZ = TipRadius + (TopRadius - TipRadius) * (z / Length);

                // Walk along the outward surface normal and find the first zero crossing.
                float previousS = -2f;
                float previousValue = sdf.GetDistance(new Vector3(radiusAtZ + previousS * normalR, 0f, z + previousS * normalZ));
                float crossing = float.NaN;

                for (float s = -1.9f; s <= 2.01f; s += 0.05f)
                {
                    float value = sdf.GetDistance(new Vector3(radiusAtZ + s * normalR, 0f, z + s * normalZ));
                    if ((previousValue < 0f) != (value < 0f))
                    {
                        crossing = previousS + (s - previousS) * (0f - previousValue) / (value - previousValue);
                        break;
                    }

                    previousS = s;
                    previousValue = value;
                }

                Assert.That(float.IsNaN(crossing), Is.False, $"No zero crossing found at z={z}");
                if (MathF.Abs(crossing) > maxDisplacement) maxDisplacement = MathF.Abs(crossing);
            }

            TestContext.Out.WriteLine($"Taper SDF zero-level max displacement: {maxDisplacement:F4} mm");
            Assert.That(maxDisplacement, Is.LessThanOrEqualTo(0.25f * resolution),
                $"Zero level displacement {maxDisplacement:F4} exceeds a quarter voxel");
        }
    }
}
