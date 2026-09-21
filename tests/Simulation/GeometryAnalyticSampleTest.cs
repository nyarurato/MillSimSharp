using System;
using System.Collections.Generic;
using System.Numerics;
using System.Reflection;
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using NUnit.Framework;

namespace MillSimSharp.Tests.Simulation
{
    /// <summary>
    /// Analytic samples for the builtin tool geometries (known inside / outside / exact surface
    /// points, axisymmetry) and conservativeness of the internally derived rotation sweep radius.
    /// </summary>
    [TestFixture]
    public class GeometryAnalyticSampleTest
    {
        private static IToolGeometry CreateGeometry(string name)
        {
            return name switch
            {
                "Flat" => new FlatEndMillGeometry(5f, 30f),
                "Ball" => new BallEndMillGeometry(5f, 30f),
                "BullNose" => new BullNoseEndMillGeometry(5f, 2f, 30f),
                "Taper" => new TaperedEndMillGeometry(2f, 10f, 20f),
                "FlatLong" => new FlatEndMillGeometry(5f, 80f),
                "BullNoseLong" => new BullNoseEndMillGeometry(5f, 2f, 80f),
                "TaperWide" => new TaperedEndMillGeometry(2f, 20f, 60f),
                _ => throw new ArgumentException($"Unknown geometry {name}", nameof(name)),
            };
        }

        /// <summary>
        /// Per-geometry samples: (point, sign) where sign 0 = exact surface, -1 = inside, +1 = outside.
        /// </summary>
        private static IEnumerable<(Vector3 Point, int Sign)> Samples(string name)
        {
            switch (name)
            {
                case "Flat":
                case "FlatLong":
                    yield return (new Vector3(0, 0, 0), 0);       // bottom face
                    yield return (new Vector3(5, 0, 15), 0);      // side
                    yield return (new Vector3(0, 0, 30), 0);      // top face
                    yield return (new Vector3(0, 0, 15), -1);     // interior
                    yield return (new Vector3(4, 0, 1), -1);
                    yield return (new Vector3(0, 0, -1), 1);      // below tip
                    yield return (new Vector3(6, 0, 15), 1);      // outside radius
                    yield return (new Vector3(0, 0, 31), 1);      // above top
                    break;
                case "Ball":
                    yield return (new Vector3(0, 0, 0), 0);       // physical tip on the ball
                    yield return (new Vector3(5, 0, 5), 0);       // ball equator
                    yield return (new Vector3(3, 0, 1), 0);       // lower hemisphere (exposed)
                    yield return (new Vector3(5, 0, 30), 0);      // flute top rim
                    yield return (new Vector3(0, 0, 5), -1);      // ball center
                    yield return (new Vector3(3, 0, 8), -1);
                    yield return (new Vector3(0, 0, 10), -1);     // inside the flute cylinder
                    yield return (new Vector3(0, 0, -1), 1);
                    yield return (new Vector3(6, 0, 5), 1);
                    yield return (new Vector3(6, 0, 20), 1);
                    break;
                case "BullNose":
                case "BullNoseLong":
                    yield return (new Vector3(0, 0, 0), 0);       // tip
                    yield return (new Vector3(3, 0, 0), 0);       // flat-bottom / corner boundary
                    yield return (new Vector3(5, 0, 2), 0);       // torus tangent
                    yield return (new Vector3(5, 0, 15), 0);      // side
                    yield return (new Vector3(2, 0, 1), -1);
                    yield return (new Vector3(0, 0, 15), -1);
                    yield return (new Vector3(0, 0, -1), 1);
                    yield return (new Vector3(6, 0, 15), 1);
                    yield return (new Vector3(5.5f, 0, 1), 1);    // outside the toroidal corner
                    break;
                case "Taper":
                case "TaperWide":
                    float topRadius = name == "Taper"
                        ? 2f + 20f * MathF.Tan(10f * MathF.PI / 180f)
                        : 2f + 60f * MathF.Tan(20f * MathF.PI / 180f);
                    float radiusAt10 = name == "Taper"
                        ? 2f + 10f * MathF.Tan(10f * MathF.PI / 180f)
                        : 2f + 10f * MathF.Tan(20f * MathF.PI / 180f);
                    float length = name == "Taper" ? 20f : 60f;

                    yield return (new Vector3(0, 0, 0), 0);           // bottom center
                    yield return (new Vector3(2, 0, 0), 0);           // bottom rim
                    yield return (new Vector3(radiusAt10, 0, 10), 0); // lateral surface
                    yield return (new Vector3(0, 0, length), 0);      // top center
                    yield return (new Vector3(topRadius, 0, length), 0); // top rim
                    yield return (new Vector3(0, 0, 10), -1);
                    yield return (new Vector3(3, 0, 10), -1);
                    yield return (new Vector3(0, 0, -1), 1);
                    yield return (new Vector3(radiusAt10 + 1f, 0, 10), 1);
                    yield return (new Vector3(0, 0, length + 1f), 1);
                    break;
            }
        }

        [TestCase("Flat")]
        [TestCase("Ball")]
        [TestCase("BullNose")]
        [TestCase("Taper")]
        public void BuiltinGeometry_KnownInsideOutsideAndSurfaceSamples(string name)
        {
            IToolGeometry geometry = CreateGeometry(name);

            foreach (var (point, sign) in Samples(name))
            {
                float distance = geometry.SignedDistance(point);
                switch (sign)
                {
                    case 0:
                        Assert.That(MathF.Abs(distance), Is.LessThanOrEqualTo(1e-3f),
                            $"{name}: {point} must be on the surface (d={distance})");
                        break;
                    case -1:
                        Assert.That(distance, Is.LessThan(0f), $"{name}: {point} must be inside (d={distance})");
                        break;
                    default:
                        Assert.That(distance, Is.GreaterThan(0f), $"{name}: {point} must be outside (d={distance})");
                        break;
                }
            }
        }

        [TestCase("Flat")]
        [TestCase("Ball")]
        [TestCase("BullNose")]
        [TestCase("Taper")]
        [TestCase("TaperWide")]
        public void BuiltinGeometry_IsAxisymmetric(string name)
        {
            IToolGeometry geometry = CreateGeometry(name);

            foreach (var (radius, height) in new[] { (0f, 5f), (2f, 8f), (4.5f, 20f), (-3f, 12f) })
            {
                float alongX = geometry.SignedDistance(new Vector3(radius, 0f, height));
                float alongY = geometry.SignedDistance(new Vector3(0f, radius, height));
                float alongNegativeX = geometry.SignedDistance(new Vector3(-radius, 0f, height));

                // Float tolerance: the same scalar expression with permuted components.
                Assert.That(alongY, Is.EqualTo(alongX).Within(1e-5f), $"{name}: radial symmetry at r={radius}, z={height}");
                Assert.That(alongNegativeX, Is.EqualTo(alongX).Within(1e-5f), $"{name}: radial symmetry (-x) at r={radius}, z={height}");
            }
        }

        [TestCase("Flat")]
        [TestCase("Ball")]
        [TestCase("BullNose")]
        [TestCase("Taper")]
        [TestCase("FlatLong")]
        [TestCase("BullNoseLong")]
        [TestCase("TaperWide")]
        public void RotationSweepRadius_CoversLocalBoundsCorners(string name)
        {
            IToolGeometry geometry = CreateGeometry(name);
            float radius = InvokeGetRotationSweepRadius(geometry);

            BoundingBox bounds = geometry.LocalBounds;
            float maxCornerDistance = 0;
            foreach (float x in new[] { bounds.Min.X, bounds.Max.X })
                foreach (float y in new[] { bounds.Min.Y, bounds.Max.Y })
                    foreach (float z in new[] { bounds.Min.Z, bounds.Max.Z })
                        maxCornerDistance = MathF.Max(maxCornerDistance, new Vector3(x, y, z).Length());

            Assert.That(maxCornerDistance, Is.LessThanOrEqualTo(radius + 1e-4f),
                $"{name}: every LocalBounds corner must be inside the rotation radius");
            Assert.That(radius, Is.EqualTo(maxCornerDistance).Within(1e-4f),
                $"{name}: the internal radius is the farthest bounds corner distance");
        }

        private static float InvokeGetRotationSweepRadius(IToolGeometry geometry)
        {
            Type? type = typeof(VoxelGrid).Assembly.GetType("MillSimSharp.Simulation.ToolPoseMath");
            Assert.That(type, Is.Not.Null, "ToolPoseMath must exist");
            MethodInfo? method = type!.GetMethod("GetRotationSweepRadius", BindingFlags.Public | BindingFlags.Static);
            Assert.That(method, Is.Not.Null, "GetRotationSweepRadius must exist");
            return (float)method!.Invoke(null, new object[] { geometry })!;
        }
    }
}
